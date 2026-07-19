"""
angle2pnw-faithful2 — Ford CAN-FD path-angle-primary lateral control, ported FAITHFULLY (line by
line, constant by constant) from Alan Polk's BluePilot bp-7.0
opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py (repo sunny/bluepilot, branch bp-7.0,
tip 19858f2888).

This is a re-port. An earlier attempt on branch ``angle2pnw`` (first pass, opendbc-layer only) and
a second attempt on ``angle2pnw-faithful`` (built from a design doc that turned out to be wrong
about several mechanisms) both preceded this file and are NOT its ancestry — this is a fresh,
line-by-line transcription of his ACTUAL SHIPPING CODE, not of either prior port or of the
(partially incorrect) ``ALAN-POLK-SPEC.md`` design notes. See
drives/2026-07-18/lightning-angle-steering/ALAN-POLK-PORT-DEVIATIONS.md for the complete, audited
list of every remaining difference from his source and why each one is there — every difference not
listed there is a bug, not a deliberate choice.

THE CONTROL LAW (his line 451, verbatim, no negation inside the strategy):
  path_angle_calc = kappa_cmd * v_ego * self.curvature_factor
His module docstring claims ``path_angle = 1/2 * kappa * d_ref`` — that docstring is STALE and
contradicts his own code (``pscm_d_ref_m`` / ``d_ref`` is computed and kept for potential future use
but is NOT multiplied into path_angle anywhere in his file). The code wins; this port implements the
code, not the docstring, and the deviations doc records that this is not a deviation we introduced.

SIGN CONVENTION (see SIGN-CONVENTION-TRACE.md for the full frame-by-frame proof): there is NO
negation anywhere inside this file. ``self.bp_kappa_cmd`` is the un-negated, error-clipped
kappa_cmd. Negation happens ONLY at the two points his carcontroller.py negates:
  (1) all four LMC/LMC2 wire signals: -path_offset, -path_angle, -apply_curvature, -curvature_rate
  (2) shadow_curvature sent on the LKA message: -self.bp_kappa_cmd when angle mode is engaged
Both negations live in carcontroller.py, not here — see that file's angle-mode dispatch block.

Everything bp-7.0 does is kept, per the driver's explicit instruction that our PREVIOUS port
(``angle2pnw``) most likely failed on the road because it dropped several of these mechanisms:
  - Variable Lookup Time (VLT), including the direction-aware ``kappa_entering`` early/late split
  - the exit-biased blend collapse (b * 0.25 near a saturation/exit condition)
  - the current-curvature deviation clip (v_ego > 9, CarControllerParams.CURVATURE_ERROR)
  - the PSCM authority-limit clamp, including ``_PSCM_SAT_UNWIND_RATE``
  - the unconditional soft ROC (Python-side, tighter than panda's backstop)
  - the human-turn override (mode 0, all-zero wire, via the shared HumanTurnDetector)
  - BOTH blips: the reactive post-override stall blip AND the proactive hand-off press blip
  - the lane-change factor path (precision_type = 0 while it's active)

The only re-porting adaptations (see ALAN-POLK-PORT-DEVIATIONS.md for the full manifest with
categories and justifications):
  (a) import/module-path rewiring for this tree's naming (*_pnw, not *_ext) and its composition-
      based CarController (this file is NOT mixed into LateralCurvExt via multiple inheritance the
      way bp-7.0's is — it owns its own small SubMaster instead of sharing LateralCurvExt's).
  (b) no MADS in this tree — not applicable to this file (bp-7.0's lateral_angle_ext.py itself has
      no MADS references; this note exists only for completeness against the task's category list).
  (d) per-platform gain-table selection and the master enable gate go through
      ``opendbc.car.pnw_vehicle.PnwVehicle`` (capability view), never a ``carFingerprint`` check in
      this file — his ``update_angle_params``'s own ``getattr(self.CP, 'carFingerprint', '')``
      branch is replaced by ``PnwVehicle(CP).angle_gain``, computed once at construction.
  (e) his four user-tunable Params reads (``FordLowSpeedFactor_ang``, ``FordHighSpeedFactor_ang``,
      ``lane_change_factor_high_ang``, plus the blend ratio / VLT-extra-max he also allows tuning
      via params in his own file) do not exist as openpilot Params in this tree. They are served
      instead by the on-road JSON tuning overlay (``angle_tuning_pnw.py``), which defaults to his
      exact bp-7.0 constants and only ever narrows toward them — see that module's docstring. This
      is an EXTRA on-road-tuning path added on top of the faithful defaults, not a change to what
      those defaults are.
"""
import numpy as np
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.carlog import carlog
from opendbc.car.lateral import apply_std_steer_angle_limits
from opendbc.car.ford.values import CarControllerParams
from opendbc.car.ford.angle_tuning_pnw import AngleTuning, load_angle_tuning
from opendbc.car.ford.human_turn_pnw import HumanTurnDetector
from opendbc.car.ford.lateral_curv_pnw import LateralResult
from opendbc.car.ford.values_pnw import BP_ANGLE_LIMITS
from opendbc.car.pnw_vehicle import PnwVehicle

try:
  from openpilot.selfdrive.modeld.constants import ModelConstants
except ImportError:
  from selfdrive.modeld.constants import ModelConstants

import cereal.messaging as messaging


# DBC ``LatCtlPath_An_Actl`` (rad) — panda safety uses the same in ``ford.h``; PSCM enforces in firmware.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235


# PSCM d_ref (m) vs speed (m/s) — 6 points; above ~55.6 m/s use plateau + optional cap to 5 m.
# NOTE (faithful to his file, not a bug): computed by pscm_d_ref_m() below but NOT consumed by
# path_angle_calc anywhere in his source (see this file's module docstring re: the stale docstring
# in his file). Kept because his shipping code keeps it — possibly future/telemetry use on his side.
_PSCM_DREF_SPEEDS_MS = (0.0, 4.17, 27.78, 41.67, 50.0, 55.56)
_PSCM_DREF_M = (0.5, 0.95, 1.4, 2.075, 2.75, 3.875)

# Default blend ratio validated on F-150 fleet data (0.5s lookup time). Tunable via angle_tuning_pnw.
_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT = 0.50

# Variable lookup time (VLT): curvature_lookup_time adapts to speed and curvature magnitude.
# t_lookup = t_base + t_extra_max × speed_factor(v) × kappa_factor(|κ|)
# t_base = liveDelay.lateralDelay + DT_MDL — always matches the planner's pre-compensation floor.
# Extra lookahead collapses toward zero at high speed (PSCM responds faster)
# and at large curvature (prevents blend importing a "start unwinding" signal too early).
_DT_MDL = 0.05                       # model loop period (matches common/realtime.py)
_VLT_T_EXTRA_MAX = 0.10              # max extra lookahead above t_base. Tunable via angle_tuning_pnw.
_VLT_V_LOW_MS   = 25.0 * 0.44704    # 25 mph — full extra lookahead at or below this speed
_VLT_V_HIGH_MS  = 55.0 * 0.44704    # 55 mph — no extra lookahead at or above this speed
_VLT_KAPPA_FULL  = 0.005             # 1/m — full extra lookahead below this curvature (200m+ radius)
_VLT_KAPPA_TAPER = 0.020             # 1/m — no extra lookahead above this curvature (50m radius)

# Rate cap on path_angle magnitude DECREASE during PSCM LimitReached (rad/call = 0.40 rad/s).
# Both model and planner naturally drop path_angle ~0.36 rad/s at a sharp 90° apex, while the PSCM is
# physically pinned and cannot execute the rapidly falling desired angle. The resulting actual-vs-desired
# gap (up to 47° observed) causes a snap correction the moment the PSCM is released. This cap limits
# the desired-angle drop rate to what the PSCM can reasonably track, at the cost of holding the car
# slightly more in the curve during saturation.
# His comment (kept verbatim): this strategy runs once per STEER_STEP (CarControllerParams.STEER_STEP=5),
# i.e. once every 5th 100Hz control tick = 20Hz, not every tick — this tree's STEER_STEP is also 5
# (opendbc/car/ford/values.py), so the same 20Hz-native scaling applies unchanged.
_PSCM_SAT_UNWIND_RATE = 0.02        # rad/call (0.02 * 20Hz = 0.40 rad/s)

# Post-override stall blip. His road test 2026-07-14 (route 886240741b067740/000000bd--feb980680f)
# showed that after driver-touch episodes the Mach-E PSCM keeps reporting InProgress but honors
# path_angle at only ~0.56x (healthy hands-free delivery on the same route: ~0.95 median). The
# current-curvature deviation clip below then pins kappa_cmd at measured + CURVATURE_ERROR, so the
# command can never lead the car enough to overcome the attenuation -- a stall equilibrium the
# driver reads as "not engaging" (wire path_angle flat at ~4 deg for 4.5 s while desired kappa
# climbed to 3x measured, EPS motor current ~0 A). A short mode-0 pulse -- the identical
# panda-clean wire pattern the human-turn override sends, no ford.h involvement -- resets the
# PSCM's authority, after which path_angle ramps back in from zero through the soft ROC. Kept for
# fidelity even though our only driven platform (F-150 Lightning) has no on-road evidence yet of
# this specific Mach-E attenuation — the task brief is explicit that dropping mechanisms is the
# leading suspect for the previous port's on-road failure.
_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick (matches human_turn_pnw.py)
_STALL_GAP_MIN = 2.0 * CarControllerParams.CURVATURE_ERROR  # desired must lead measured by 2x the clip tolerance
_STALL_HOLD_S = 0.5          # accumulated clip-binding time before a pulse fires
_STALL_BLIP_FRAMES = 6       # mode-0 pulse length (6 frames @ 20 Hz = 300 ms; PSCM acked mode 0 in ~150 ms on-road)
_STALL_COOLDOWN_S = 2.0      # re-arm delay after a pulse (release ramp + PSCM response time)
_STALL_MAX_BLIPS = 3         # give up on a stuck episode; devLim telemetry keeps recording the stall
# Proactive hand-off blip: any sustained driver press attenuates the PSCM (his route 000000be seg 4:
# 3 s of sub-45-deg circle-exit steering left it at ~0x delivery, and the reactive detector's
# fire-after-the-stall-develops timing meant 2.4 s of dead-straight running into the next curve
# before the pulse landed). Firing the same pulse on the falling edge of a sustained press resets
# the PSCM while the car is straight and the command is small -- a 300 ms lateral gap right at
# hand-off, imperceptible, instead of a missed curve. The reactive detector above stays as backstop.
_PRESS_BLIP_MIN_S = 0.5      # press must last this long before its release earns a pulse

# angle2pnw-faithful2: how often (in units of this strategy's own 20Hz update() calls) to re-poll
# the JSON tuning overlay, mirroring his own params-read cadence (he re-reads Params every
# carcontroller frame) without adding a file-stat to every 20Hz control tick. ~5s at 20Hz.
_TUNING_RELOAD_CALLS = 100

# Deviation (a): his lane_change_factor_bp / lane_change_factor_low live on the shared
# LateralCurvExt mixin instance; owned locally here (unchanged values, [4.4, 40.23] m/s / 0.95)
# since this class has no such instance to share. See update()'s lane-change block below.
_LANE_CHANGE_FACTOR_BP = [4.4, 40.23]  # speed breakpoints (m/s)
_LANE_CHANGE_FACTOR_LOW = 0.95


def pscm_d_ref_m(v_ego_ms: float) -> float:
  v = max(float(v_ego_ms), 0.0)
  d = float(np.interp(v, _PSCM_DREF_SPEEDS_MS, _PSCM_DREF_M))
  if v > _PSCM_DREF_SPEEDS_MS[-1]:
    # His doc: d_ref table ends at 3.875 m; contribution saturates for high speed — cap at 5 m.
    d = min(5.0, d)
  return d


class LateralAngleExt:
  """angle2pnw-faithful2 angle-primary lateral strategy.

  Deviation (a): unlike bp-7.0's ``LateralAngleExt`` (mixed into ``CarController`` alongside
  ``LateralCurvExt`` via multiple inheritance, so it shares that class's SubMaster/model/state),
  this tree's ``CarController`` uses composition — only one of ``LateralCurvExt`` /
  ``LateralAngleExt`` is ever constructed (see carcontroller.py), so this class owns its own small
  SubMaster. It subscribes to exactly the messages this file's logic reads (``modelV2`` for
  predicted curvature / lane-change state, ``liveDelay`` for the VLT ``t_base`` floor) — the same
  data dependency his mixed-in version has, just not sharing the object instance.
  """

  def __init__(self, CP):
    self.sm = messaging.SubMaster(['modelV2', 'liveDelay'])
    self.model = None

    # Capability view (deviation d): platform gain pair, resolved once here, never re-checked via
    # carFingerprint anywhere else in this file (his own update_angle_params() re-derives it from
    # CP.carFingerprint every call; PnwVehicle.angle_gain is the single place that lookup lives in
    # this tree, matching every other pnw feature's capability-view rule).
    veh = PnwVehicle(CP)
    self._platform_gain = veh.angle_gain  # (low, high) — (0.95, 0.95) for FORD_F_150_LIGHTNING_MK1

    # angle2pnw-faithful2 (brief addendum 2026-07-19): on-road JSON tuning overlay. Defaults here
    # are Alan Polk's exact bp-7.0 constants; see angle_tuning_pnw.py's module docstring for the
    # "faithful defaults are law" guarantee. Loaded once now, and re-polled every
    # _TUNING_RELOAD_CALLS calls of update() (see below) — never touches disk on every 20Hz tick.
    self._tuning_defaults = AngleTuning(
      gain_lowC_highV=self._platform_gain[0],
      gain_highC_highV=self._platform_gain[1],
      low_speed_curv_factor=1.0,
      high_speed_curv_factor=1.0,
      lane_change_factor_high_ang=1.0,
      path_angle_blend_ratio=_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT,
      vlt_extra_max=_VLT_T_EXTRA_MAX,
      gain_speed_lo_ms=13.5,
      gain_speed_hi_ms=26.82,
      low_speed_boost=1.30,
      curvature_factor_bp_lo=0.0007,
      curvature_factor_bp_hi=0.001,
    )
    self._tuning_reload_calls_left = 0
    self._reload_tuning(log_prefix="init")

    # Predicted-curvature blend for path_angle: pred * b + desired * (1-b); b from tuning overlay
    # (his ``FordPathAngleBlendRatio`` param). Set by _apply_tuning() below.
    self.path_angle_blend_ratio = _FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT
    # Max extra VLT above t_base; from tuning overlay (his ``FordVLTExtraMax`` param).
    self.vlt_extra_max = _VLT_T_EXTRA_MAX
    # Telemetry: final path_angle (rad) after limits (see bp_card_publisher in his tree; unread in
    # this tree today, kept for fidelity/future UI consumption).
    self.bp_path_angle_final = 0.0
    # High-speed gain factors: set per-platform via PnwVehicle in update_angle_params (deviation d).
    self.path_angle_gain_lowC_highV = 1.0   # dampening at high speed, low curvature
    self.path_angle_gain_highC_highV = 1.0  # gain at high speed, high curvature
    self.bp_path_angle_gain_lowC_highV = 1.0
    self.bp_path_angle_gain_highC_highV = 1.0
    # User-tunable "feel" multipliers: from tuning overlay (his FordLowSpeedFactor_ang / etc).
    self.low_speed_curv_factor = 1.0
    self.high_speed_curv_factor = 1.0
    self.bp_low_speed_curv_factor = 1.0
    self.bp_high_speed_curv_factor = 1.0
    # His: angle mode's own lane-change scaling factor, independent of curvature mode's
    # lane_change_factor_high_curv -- angle needs a boost (>1) where curvature needs a cut (<1).
    self.lane_change_factor_high_ang = 1.0
    # Telemetry: variable curvature lookup time used this frame (s)
    self.bp_curvature_lookup_time = _VLT_T_EXTRA_MAX + 0.3725  # warm start at ~0.5s
    # His: error-clipped kappa path_angle was derived from -- carcontroller.py reads this as
    # shadow_curvature for ford.h's angle-mode deviation check. Actively consumed, not telemetry.
    self.bp_kappa_cmd = 0.0
    # His: rate-limit diagnostics (controllerStateBP in his tree; unread in this tree today).
    self.bp_angle_rate_limited = False      # path_angle soft-ROC clip actually bit this frame
    self.bp_curvature_rate_limited = False  # equivalent curvature would be rate-limited by curv-mode logic (sim)
    self.bp_curvature_deviation_limited = False  # current_curvature error-clip constrained kappa_cmd this frame
    self.sim_curvature_last = 0.0           # shadow curvature-mode last for the curvatureRateLimited sim
    # Exit detection: track previous desired curvature to sense when planner is actively reducing
    self._desired_curvature_last = 0.0

    # Human-turn override: while the driver manually turns, lateral is forced inactive (mode 0,
    # all-zero signals) instead of winding path_angle into a stale command the PSCM can't cleanly
    # reconcile on release (2-3 s re-engage dead time observed on his Mach-E). See module docstring.
    self.human_turn_detector = HumanTurnDetector()
    self.angle_human_turn_active = False  # read by carcontroller to force mode 0

    # Post-override stall blip state (see module constants). angle_stall_blip_active is read by
    # carcontroller to force mode 0, exactly like angle_human_turn_active.
    self.stall_blip_hold_s = 0.0      # accumulated deviation-clip-binding time toward a pulse
    self.stall_blip_frames_left = 0   # remaining pulse frames; > 0 -> mode 0 on the wire
    self.stall_blip_cooldown_s = 0.0  # re-arm delay after a pulse
    self.stall_blip_count = 0         # pulses fired this stall episode
    self.angle_stall_blip_active = False
    self.press_timer_s = 0.0          # continuous steeringPressed time, for the hand-off blip

    # Previous-frame path_angle / apply_curvature state (his ``self.path_angle_last`` /
    # ``self.apply_curvature_last``, initialized lazily via the CC.latActive==False branch in his
    # file; explicit here since this class has no shared-mixin lazy-init path to ride along on).
    self.path_angle_last = 0.0
    self.apply_curvature_last = 0.0
    self.precision_type = 1
    self.lane_change = False

    self._apply_tuning()

  # -- tuning overlay -------------------------------------------------------------------------
  def _apply_tuning(self):
    """Copy the currently-loaded AngleTuning onto the instance attributes his update() body reads,
    so the body below is otherwise an unmodified transcription of his file."""
    t = self._tuning
    self.path_angle_gain_lowC_highV = t.gain_lowC_highV
    self.path_angle_gain_highC_highV = t.gain_highC_highV
    self.low_speed_curv_factor = t.low_speed_curv_factor
    self.high_speed_curv_factor = t.high_speed_curv_factor
    self.lane_change_factor_high_ang = t.lane_change_factor_high_ang
    self.path_angle_blend_ratio = t.path_angle_blend_ratio
    self.vlt_extra_max = t.vlt_extra_max
    self._gain_speed_bp = [t.gain_speed_lo_ms, t.gain_speed_hi_ms]
    self._curvature_factor_bp = [t.curvature_factor_bp_lo, t.curvature_factor_bp_hi]
    self._low_speed_boost = t.low_speed_boost

  def _reload_tuning(self, log_prefix: str = "reload"):
    try:
      self._tuning = load_angle_tuning(self._tuning_defaults)
    except Exception as e:
      carlog.warning(f"angle_tuning_pnw: {log_prefix} reload failed ({e}) — keeping bp-7.0 defaults")
      self._tuning = self._tuning_defaults
    self._tuning_reload_calls_left = _TUNING_RELOAD_CALLS

  def update_angle_params(self, params=None):
    """Deviation (e): his ``update_angle_params(self, params)`` reads four BluePilot-only openpilot
    Params keys each carcontroller frame. Those keys don't exist in this tree's params registry, so
    this method instead re-polls the JSON tuning overlay on a throttled cadence (see
    _TUNING_RELOAD_CALLS) and re-applies it via _apply_tuning(). ``params`` is accepted (unused) so
    carcontroller.py's call site can mirror his call signature without extra branching."""
    self._tuning_reload_calls_left -= 1
    if self._tuning_reload_calls_left <= 0:
      self._reload_tuning()
      self._apply_tuning()

  def update_sm(self):
    self.sm.update(0)
    if self.sm.updated['modelV2']:
      self.model = self.sm['modelV2']

  def update(self, CC, CS, actuators, CP):
    """
    Curvature from planner (+ optional predicted blend) → path_angle via kappa * v_ego * gain.
    c0 (path_offset) is always zero on the wire -- no centering trim in angle mode. c2 and c3 are zero.
    Blended κ is not passed through Ford c2 rate / DBC limits (those target the curvature actuator).

    Deviation (a): his method is named ``update_angle_strategy(self, CC, CS, actuators, CP)``; this
    tree names it ``update`` to match ``LateralCurvExt.update``'s calling convention (see
    carcontroller.py's angle-mode dispatch, which mirrors the 4-signal dispatch immediately above it).
    """
    self.update_angle_params(None)

    v_ego = float(CS.out.vEgoRaw)
    d_ref = pscm_d_ref_m(v_ego)  # noqa: F841 — computed for fidelity; not consumed (see module docstring)

    curvature_rate = 0.0
    path_offset = 0.0
    path_angle = 0.0
    ramp_type = 0
    lateral_uncertainty = 0.0
    precision = 1

    if not CC.latActive:
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      self.bp_kappa_cmd = 0.0
      self.human_turn_detector.reset()
      self.angle_human_turn_active = False
      self.stall_blip_hold_s = 0.0
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.press_timer_s = 0.0
      self.precision_type = 1
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

    # Human-turn override: sustained driver press + large wheel angle → force lateral inactive
    # (carcontroller drops mode to 0; all signals are zero on the wire) so path_angle can't wind
    # into a stale command while the driver turns. Always on in angle mode (no param gate) -- the
    # curv-suffixed human-turn toggle belongs to curvature mode's reset strategy, and the Mach-E
    # PSCM re-engage stall this prevents is not something a user should be able to opt out of.
    # On release, no jump seed: path_angle_last is 0, so the normal flow below ramps the command
    # back in through the soft ROC -- generous at human-turn speeds, no panda bypass involved.
    self.angle_human_turn_active = self.human_turn_detector.update(
      True, CS.out.steeringPressed, CS.out.steeringAngleDeg)
    if self.angle_human_turn_active:
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      # Zero the shadow curvature on the wire during the override (mirrors the inactive path);
      # ford.h skips the deviation check while steer_control_enabled is 0 either way.
      self.bp_kappa_cmd = 0.0
      # Keep exit detection current so resume doesn't compare against a stale pre-turn value.
      self._desired_curvature_last = float(actuators.curvature)
      # A human turn ends any stall episode -- its own mode 0 does the PSCM reset job. That also
      # covers the press so far: only press time accumulated AFTER the latch releases should earn
      # a hand-off pulse.
      self.stall_blip_hold_s = 0.0
      self.stall_blip_frames_left = 0
      self.stall_blip_cooldown_s = 0.0
      self.stall_blip_count = 0
      self.angle_stall_blip_active = False
      self.press_timer_s = 0.0
      self.precision_type = 1
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )

    # Proactive hand-off blip: the falling edge of a sustained press earns an immediate mode-0
    # pulse (see _PRESS_BLIP_MIN_S) -- resets the PSCM's press-induced attenuation right at
    # hand-off, while the car is straight and the command small, instead of waiting for the
    # reactive stall detector below to watch the car miss the next curve first.
    if CS.out.steeringPressed:
      self.press_timer_s += _STEER_DT
    else:
      if (self.press_timer_s >= _PRESS_BLIP_MIN_S and self.stall_blip_cooldown_s <= 0.0
          and self.stall_blip_frames_left <= 0):
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
      self.press_timer_s = 0.0

    # Stall-blip pulse in progress: hold lateral inactive (mode 0, all-zero signals -- the same
    # wire pattern as the human-turn override, no ford.h involvement) for _STALL_BLIP_FRAMES so the
    # PSCM drops its post-override attenuation, then release; path_angle ramps back in from zero
    # through the soft ROC exactly like a human-turn release. Detection lives at the end of the
    # normal flow below.
    if self.stall_blip_frames_left > 0:
      self.stall_blip_frames_left -= 1
      self.angle_stall_blip_active = True
      self.path_angle_last = 0.0
      self.bp_path_angle_final = 0.0
      self.apply_curvature_last = 0.0
      self.bp_angle_rate_limited = False
      self.bp_curvature_rate_limited = False
      self.bp_curvature_deviation_limited = False
      self.sim_curvature_last = 0.0
      self.bp_kappa_cmd = 0.0
      self._desired_curvature_last = float(actuators.curvature)
      self.precision_type = 1
      if self.stall_blip_frames_left <= 0:
        self.stall_blip_cooldown_s = _STALL_COOLDOWN_S
      return LateralResult(
        apply_curvature=0.0,
        curvature_rate=0.0,
        path_offset=0.0,
        path_angle=0.0,
        ramp_type=0,
        precision_type=1,
        lateralUncertainty=0.0,
      )
    self.angle_stall_blip_active = False

    self.precision_type = 1
    precision = 1
    desired_curvature = float(actuators.curvature)

    # Variable lookup time: t_base tracks planner pre-compensation; extra tapers on high speed and large curves.
    # Cap liveDelay at 0.15s for VLT purposes. liveDelay can calibrate up to ~420ms on some runs, which inflates
    # VLT to 0.6s and pushes the model lookahead 5m into the curve. At that depth the model sees full peak
    # curvature, kappa_entering stays True, and the exit-biased blend is permanently disabled — causing the car
    # to command max path_angle through the entire apex. 0.15s gives t_base ≤ 0.20s and VLT ≤ 0.33s, restoring
    # the 2.8m lookahead that kept kappa_entering False at the apex in his earlier successful runs.
    _t_base = float(clip(self.sm['liveDelay'].lateralDelay, 0.1, 0.15)) + _DT_MDL
    _speed_factor = float(interp(v_ego, [_VLT_V_LOW_MS, _VLT_V_HIGH_MS], [1.0, 0.0]))
    # Direction-aware kappa factor: on curve ENTRY (model shows more curvature at t_base than planner now),
    # keep full lookahead so pre-steering begins early. On exit/apex, taper by magnitude to prevent unwind.
    _kappa_at_t_base = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      _curvatures_ref = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      _kappa_at_t_base = abs(float(interp(_t_base, ModelConstants.T_IDXS, _curvatures_ref)))
    _kappa_entering = _kappa_at_t_base > abs(desired_curvature)
    if _kappa_entering:
      _kappa_factor = 1.0  # curve deepening ahead: full extra lookahead for gradual entry
    else:
      _kappa_factor = float(interp(abs(desired_curvature), [_VLT_KAPPA_FULL, _VLT_KAPPA_TAPER], [1.0, 0.0]))
    curvature_lookup_time = _t_base + self.vlt_extra_max * _speed_factor * _kappa_factor
    self.bp_curvature_lookup_time = curvature_lookup_time

    predicted_curvature = 0.0
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      curvatures = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      predicted_curvature = float(
        interp(curvature_lookup_time, ModelConstants.T_IDXS, curvatures)
      )

    b = float(self.path_angle_blend_ratio)
    b = float(clip(b, 0.0, 1.0))

    # Exit-biased blend: near the PSCM authority limit or while the planner is actively
    # reducing curvature (exit detected), drop model prediction weight from 60% → ~15%.
    # This lets the planner's natural unwind dominate instead of being diluted by a model
    # prediction that still sees the curve (→ seg-14 slow unwind) or that snaps when its
    # lookahead window crosses the curve exit (→ seg-17 snap + reverse PSCM hit).
    # Normal gentle curves are unaffected: no PSCM limit, no falling desired → full b=0.60.
    # (His comment says 0.60/~15% verbatim; his actual constant default is 0.50, so the real
    # collapsed value is 0.125, not ~0.15. Kept his comment text unmodified for fidelity -- see
    # ALAN-POLK-PORT-DEVIATIONS.md's "issues found in his code, not fixed" section.)
    _pscm_lim = getattr(CS, 'lat_ctl_lim_stat', 0)
    # In angle mode, LatCtlLim_D_Stat (→ lat_ctl_lim_stat) does not fire.
    # His earlier version used angleState.saturated (CtrSat) as a proxy, but CtrSat fires whenever
    # the car lags the commanded path_angle by > 2.5° — which happens during any normal curve entry.
    # That caused a positive-feedback flat-line: under-steer → CtrSat → path_angle frozen → more under-steer.
    # Use DBC-limit proximity instead: only block when path_angle is already near the ±0.5 rad CAN limits,
    # which is the only condition where the anti-snap unwind rate cap makes physical sense.
    _dbc_sat = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * 0.90 or
                self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * 0.90)
    _in_hard_sat = _pscm_lim >= 2 or _dbc_sat
    # His per-call delta threshold, already 20Hz-native on this branch (his STEER_STEP is also 5).
    _desired_falling = abs(desired_curvature) < abs(self._desired_curvature_last) - 0.010
    _on_exit_near_limit = not _kappa_entering and (_pscm_lim >= 1 or _in_hard_sat or _desired_falling)
    b_blend = float(clip(b * 0.25, 0.0, 1.0)) if _on_exit_near_limit else b
    requested_curvature = predicted_curvature * b_blend + desired_curvature * (1.0 - b_blend)
    self._desired_curvature_last = desired_curvature

    if self.model is not None:
      self.lane_change = self.model.meta.laneChangeState in (1, 2, 3)
    else:
      self.lane_change = False

    # His lane_change_factor_bp / lane_change_factor_low are LateralCurvExt attributes shared via
    # the mixin; deviation (a) — this class has no LateralCurvExt instance to share, so the same
    # two constants (unchanged values: speed breakpoints [4.4, 40.23] m/s, low factor 0.95) are
    # owned locally instead of inherited.
    lane_change_factor = interp(
      v_ego, _LANE_CHANGE_FACTOR_BP, [_LANE_CHANGE_FACTOR_LOW, self.lane_change_factor_high_ang]
    )
    if self.lane_change and self.model is not None:
      if self.model.meta.laneChangeDirection == 1 and requested_curvature < 0:
        requested_curvature *= lane_change_factor
        precision = 0
      elif self.model.meta.laneChangeDirection == 2 and requested_curvature > 0:
        requested_curvature *= lane_change_factor
        precision = 0
    self.precision_type = precision

    # Use planner / predicted κ directly for the κ → path_angle map; we are not sending κ on CAN.
    kappa_cmd = float(requested_curvature)

    # His: clip kappa_cmd to current_curvature (measured, from yaw rate) +- CURVATURE_ERROR,
    # mirroring lateral_curv_pnw.py's apply_ford_curvature_limits_ext exactly (same formula, same
    # v_ego > 9 gate, same CarControllerParams.CURVATURE_ERROR tolerance). Without this, kappa_cmd
    # (and therefore path_angle, and the shadow_curvature sent to ford.h) can legitimately lead the
    # measured curvature by more than ford.h's angle-error tolerance during normal curve entry/exit
    # -- the shadow-curvature deviation check (ford_shadow_curvature_error_check) would then block
    # routinely, not just on genuine pothole/override divergence. Curvature mode has always clipped
    # here; this brings angle mode's actual steering intent in line with that proven behavior rather
    # than only clipping the value reported to panda (which would make the check a no-op).
    current_curvature = -CS.out.yawRate / max(v_ego, 0.1)
    self.bp_curvature_deviation_limited = False
    if v_ego > 9:
      _kappa_cmd_pre_error_clip = kappa_cmd
      kappa_cmd = float(clip(kappa_cmd, current_curvature - CarControllerParams.CURVATURE_ERROR,
                            current_curvature + CarControllerParams.CURVATURE_ERROR))
      # Did this clip actually constrain kappa_cmd this frame (deviation from measured, not
      # rate-of-change -- see carcontroller.py)?
      self.bp_curvature_deviation_limited = bool(abs(kappa_cmd - _kappa_cmd_pre_error_clip) > 1e-9)

    lateral_uncertainty = 0.0  # no curvature-limit ladder until angle-mode torque display is defined

    # Speed-interpolated gain: at low speed both curves use 1.0; at high speed the params take effect.
    # Breakpoints / low-speed boost come from the tuning overlay (defaults: [13.5, 26.82] m/s, 1.30).
    self.low_gain_calc = interp(v_ego, self._gain_speed_bp, [1.0, self.path_angle_gain_lowC_highV])
    self.high_gain_calc = interp(v_ego, self._gain_speed_bp,
                                 [(self._low_speed_boost * self.low_speed_curv_factor),
                                  (self.path_angle_gain_highC_highV * self.high_speed_curv_factor)])

    # As the curve gets bigger, we will need a little boost to the signal to to not understeer
    self.curvature_factor = interp(abs(kappa_cmd), self._curvature_factor_bp, [self.low_gain_calc, self.high_gain_calc])

    path_angle_calc = kappa_cmd * v_ego * self.curvature_factor
    path_angle = path_angle_calc

    # PSCM authority limit clamp.
    # On CANFD Fords in angle mode, LatCtlLim_D_Stat does not fire, so _pscm_lim stays 0.
    # _in_hard_sat (computed above) combines _pscm_lim >= 2 with _dbc_sat (path_angle near ±0.5 rad limit).
    # LimitClose (_pscm_lim >= 1 only): block magnitude increases — exit-biased blend provides unwind.
    # Hard saturation (_in_hard_sat): block increases AND rate-limit decreases to _PSCM_SAT_UNWIND_RATE.
    #   Without the decrease cap, model+planner drop path_angle at ~0.36 rad/s at a sharp apex,
    #   driving desired steering 30°+ ahead of actual while the PSCM is pinned, causing a snap when released.
    if _in_hard_sat:
      _last = self.path_angle_last
      _last_mag = abs(_last)
      _curr_mag = abs(path_angle)
      if _curr_mag > _last_mag:  # magnitude growing — block
        path_angle = _last
      elif _last_mag - _curr_mag > _PSCM_SAT_UNWIND_RATE:  # decreasing too fast — rate-limit
        _limited_mag = _last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = float(_limited_mag if _last >= 0 else -_limited_mag)
    elif _pscm_lim >= 1:  # LimitClose (F150/non-angle-mode only): block increases only
      path_angle = float(clip(path_angle, -abs(self.path_angle_last), abs(self.path_angle_last)))

    path_angle = min(FORD_DBC_PATH_ANGLE_MAX, max(FORD_DBC_PATH_ANGLE_MIN, path_angle))

    # Soft ROC limit — unconditional, slightly tighter than ford.h, applied before the
    # hardware bypass in ford.h is re-enabled.  Lets us observe whether the limit would
    # suppress control and tune it, while the PSCM still receives the clipped value.
    # NOT tunable via the JSON overlay (deliberately excluded, see angle_tuning_pnw.py's
    # docstring): this table is mirrored in panda's ford.h FORD_PATH_ANGLE_LIMITS_ANGLE, and
    # changing it from Python alone would desync the safety backstop.
    _soft_roc = float(interp(v_ego, [9., 10., 15., 25.], [0.055, 0.055, 0.0425, 0.009]))
    _path_angle_pre_roc = path_angle
    path_angle = float(clip(path_angle,
                            self.path_angle_last - _soft_roc,
                            self.path_angle_last + _soft_roc))
    # Did the soft ROC clip actually limit the path_angle we wanted to send this frame?
    self.bp_angle_rate_limited = bool(abs(path_angle - _path_angle_pre_roc) > 1e-9)

    # c0 always zero -- no centering trim in angle mode.
    path_offset = 0.0

    # Telemetry / state
    self.bp_path_angle_gain_lowC_highV = self.path_angle_gain_lowC_highV
    self.bp_path_angle_gain_highC_highV = self.path_angle_gain_highC_highV
    self.bp_low_speed_curv_factor = self.low_speed_curv_factor
    self.bp_high_speed_curv_factor = self.high_speed_curv_factor
    self.path_angle_last = path_angle
    self.bp_path_angle_final = path_angle
    self.apply_curvature_last = 0.0
    # His: the error-clipped kappa path_angle was derived from -- carcontroller.py reads this
    # as shadow_curvature for ford.h's angle-mode deviation check (see fordcan_pnw.create_lka_msg).
    # Not just telemetry: an actively-consumed value, unlike the removed *_kappa_cmd_raw stubs.
    self.bp_kappa_cmd = kappa_cmd

    # Would the equivalent curvature (kappa_cmd) have been rate-limited by curvature-mode's
    # ROC (apply_std_steer_angle_limits)? kappa_cmd is already error-clipped above (same clip
    # curvature mode applies), so only the rate-of-change portion remains to simulate here.
    _equiv_curv_rl = apply_std_steer_angle_limits(kappa_cmd, self.sim_curvature_last, v_ego,
                                                  CS.out.steeringAngleDeg, CC.latActive, BP_ANGLE_LIMITS)
    self.bp_curvature_rate_limited = bool(abs(_equiv_curv_rl - kappa_cmd) > 1e-9)
    self.sim_curvature_last = float(_equiv_curv_rl)

    # Post-override stall detection (mechanism in the module constants' comment above). Fires the
    # mode-0 blip when, hands-free, desired curvature has led measured by more than 2x the
    # deviation clip's tolerance while the clip was actually binding for _STALL_HOLD_S accumulated
    # seconds. devLim flickers mid-stall on his diagnosis route, so off frames hold the accumulator
    # rather than resetting it; a closed gap or driver press ends the episode.
    self.stall_blip_cooldown_s = max(0.0, self.stall_blip_cooldown_s - _STEER_DT)
    _stall_gap = desired_curvature - current_curvature
    _stalled = (not CS.out.steeringPressed and not self.lane_change and v_ego > 9.0
                and abs(_stall_gap) > _STALL_GAP_MIN
                and abs(desired_curvature) > abs(current_curvature))
    if _stalled:
      if self.bp_curvature_deviation_limited and self.stall_blip_cooldown_s <= 0.0:
        self.stall_blip_hold_s += _STEER_DT
      if self.stall_blip_hold_s >= _STALL_HOLD_S and self.stall_blip_count < _STALL_MAX_BLIPS:
        self.stall_blip_frames_left = _STALL_BLIP_FRAMES
        self.stall_blip_hold_s = 0.0
        self.stall_blip_count += 1
    else:
      self.stall_blip_hold_s = 0.0
      if CS.out.steeringPressed or abs(_stall_gap) < 0.5 * _STALL_GAP_MIN:
        self.stall_blip_count = 0  # episode over: the car is tracking again or the driver took it

    ramp_type = 2

    return LateralResult(
      apply_curvature=0.0,
      curvature_rate=curvature_rate,
      path_offset=path_offset,
      path_angle=path_angle,
      ramp_type=ramp_type,
      precision_type=self.precision_type,
      lateralUncertainty=lateral_uncertainty,
    )

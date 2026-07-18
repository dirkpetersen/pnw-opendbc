"""
angle2pnw — BluePilot (alan-polk) bp-7.0 Ford angle-primary lateral control, ported from
bluepilotdev/bp-7.0 opendbc_repo/opendbc/sunnypilot/car/ford/lateral_angle_ext.py.

FIRST PASS (2026-07-18): opendbc-layer build+test only. Not wired live — the master gate
(``PnwVehicle.angle_lat``) is hardcoded False in pnw_vehicle.py; nothing in this file runs on the
road until a later pass wires a runtime toggle and flips that capability. See
docs/pnw/ANGLE2PNW.md for the full port notes and what was deliberately trimmed.

Where this diverges from a fresh drive, an actual second look is needed before flipping the toggle.

Steering intent is c1 (path_angle) derived directly from planner/model curvature:
``path_angle = kappa_cmd * v_ego * curvature_factor``. Curvature (c2/c3) and path_offset (c0) stay
pinned at 0 on the wire — see FORDSAFETY2PNW.md's four-signal path for the curvature-primary
default this sits alongside (LateralCurvExt, untouched, still the default and the fallback).

Deliberately trimmed vs bp-7.0 (documented, not silently dropped — first-pass scope, Rule 2):
  - Variable Lookup Time (VLT): bp-7.0 adapts the predicted-curvature lookahead from
    ``liveDelay.lateralDelay`` plus a speed/curvature taper. This port uses lateral_curv_pnw's
    existing FIXED 0.2s lookup (matching carcontroller.py's PC_BLEND_LOOKUP_S) instead --
    one fewer cereal subscription, one fewer moving part, at the cost of the entry/exit lookahead
    tuning VLT provides. The exit-biased blend collapse (the safety-relevant half of VLT, which
    prevents the model from holding the blend open into a curve exit) IS ported.
  - Stall-blip / proactive hand-off blip: bp-7.0's fix for a Mach-E-specific PSCM authority
    attenuation after a driver touch (observed on that platform's fleet telemetry, not ours). Not
    ported -- no data yet that the Lightning's PSCM exhibits the same attenuation. The PSCM
    saturation clamp (which IS ported) covers the sharp-apex windup case that matters for us.
  - Telemetry-only diagnostics for BluePilot's ``controllerStateBP`` UI panel (rate-limited-sim,
    per-frame debug prints): not ported, no consumer in this tree.
  - Per-platform gain table: kept (see PnwVehicle.angle_gain), but selection now lives in
    pnw_vehicle.py per the capability-view rule, not a carFingerprint check in this file.

Everything else -- the kappa->path_angle mapping, the curvature-deviation clip (mirrors
apply_ford_curvature_limits_ext exactly), the PSCM saturation clamp, the soft ROC, the human-turn
override (ported to human_turn_pnw.HumanTurnDetector), and the shadow_curvature output panda's
ford.h cross-checks -- is a faithful port.
"""
import numpy as np
from numpy import clip, interp

from opendbc.car import DT_CTRL
from opendbc.car.ford.values import CarControllerParams
from opendbc.car.ford.human_turn_pnw import HumanTurnDetector
from opendbc.car.ford.lateral_curv_pnw import LateralResult
from opendbc.car.pnw_vehicle import PnwVehicle

try:
  from openpilot.selfdrive.modeld.constants import ModelConstants
except ImportError:
  from selfdrive.modeld.constants import ModelConstants

import cereal.messaging as messaging


# DBC ``LatCtlPath_An_Actl`` (rad) — panda safety uses the same range in ford.h's angle-mode value
# check, gated on the corroborated ``ford_bp_angle_mode_engaged`` flag (see FORD_Lane_Assist_Data1
# in ford.h). Outside angle mode the tight curvature-mode +-0.25 rad cap applies instead.
FORD_DBC_PATH_ANGLE_MIN = -0.5
FORD_DBC_PATH_ANGLE_MAX = 0.5235

_CURVATURE_LOOKUP_S = 0.2  # seconds into the model horizon (fixed; see module docstring re: VLT)
_STEER_DT = CarControllerParams.STEER_STEP * DT_CTRL  # 20 Hz lateral tick

# Rate cap on path_angle magnitude DECREASE during PSCM saturation (rad/call = 0.40 rad/s at this
# branch's 20Hz STEER_STEP cadence -- see ford.h's mirrored FORD_PATH_ANGLE_LIMITS_ANGLE comment).
# Both model and planner can drop desired path_angle faster than the PSCM can physically track at a
# sharp apex; without this cap the actual-vs-desired gap grows until the PSCM releases, producing a
# snap correction. This limits the desired-angle drop rate to what the PSCM can reasonably track.
_PSCM_SAT_UNWIND_RATE = 0.02  # rad/call (0.02 * 20Hz = 0.40 rad/s)

# Soft ROC limit -- unconditional Python-side rate cap, applied BEFORE panda's own (looser)
# backstop. Mirrors ford.h's FORD_PATH_ANGLE_LIMITS_ANGLE (2% tighter, see that struct's comment).
_SOFT_ROC_BP = [9., 10., 15., 25.]         # m/s
_SOFT_ROC_V = [0.055, 0.055, 0.0425, 0.009]  # rad/call


class LateralAngleExt:
  """angle2pnw angle-primary lateral strategy. Owns its own SubMaster (mutually exclusive with
  LateralCurvExt -- only one of the two lateral strategy objects is ever constructed by
  CarController, see carcontroller.py)."""

  def __init__(self, CP):
    self.sm = messaging.SubMaster(['modelV2'])
    self.model = None

    veh = PnwVehicle(CP)  # capability view -- gain table, no fingerprint check in this file
    self.gain_lowC_highV, self.gain_highC_highV = veh.angle_gain

    # angle mode has no user-tunable low/high speed "feel" factors wired in this pass (no UI
    # toggle yet -- see module docstring); both stay at BluePilot's neutral default of 1.0.
    self.low_speed_curv_factor = 1.0
    self.high_speed_curv_factor = 1.0
    self.lane_change_factor_high_ang = 1.0
    self.lane_change_factor_bp = [4.4, 40.23]  # speed breakpoints (m/s), matches curv mode
    self.lane_change_factor_low = 0.95

    self.path_angle_last = 0.0
    self.apply_curvature_last = 0.0
    self.precision_type = 1
    self.lane_change = False

    # angle-mode-only diagnostics; not wired to any UI in this tree yet, kept cheap.
    self.bp_angle_rate_limited = False
    self.bp_curvature_deviation_limited = False
    # Error-clipped kappa path_angle was derived from -- carcontroller reads this (negated, see
    # carcontroller.py's LKA send) as shadow_curvature for ford.h's angle-mode deviation check.
    # Actively consumed, not telemetry.
    self.bp_kappa_cmd = 0.0

    self._desired_curvature_last = 0.0

    # Human-turn override: while the driver manually turns, lateral is forced inactive (mode 0,
    # all-zero signals on the wire) instead of winding path_angle into a stale command the PSCM
    # has to reconcile on release. angle_human_turn_active is read by carcontroller to force
    # mode 0 (see carcontroller.py's angle-mode LMC/LMC2 dispatch).
    self.human_turn_detector = HumanTurnDetector()
    self.angle_human_turn_active = False

  def update_sm(self):
    self.sm.update(0)
    if self.sm.updated['modelV2']:
      self.model = self.sm['modelV2']

  def _predicted_curvature(self, v_ego: float) -> float:
    if self.model is not None and len(self.model.orientationRate.z) >= 17:
      curvatures = np.array(self.model.orientationRate.z) / max(0.01, v_ego)
      return float(interp(_CURVATURE_LOOKUP_S, ModelConstants.T_IDXS, curvatures))
    return 0.0

  def _zero_state(self):
    self.path_angle_last = 0.0
    self.apply_curvature_last = 0.0
    self.bp_angle_rate_limited = False
    self.bp_curvature_deviation_limited = False
    self.bp_kappa_cmd = 0.0

  def update(self, CC, CS, actuators, CP):
    """Compute angle-primary lateral signals for the current frame. Called at 20Hz from
    CarController.update() when inside the STEER_STEP block, mirroring LateralCurvExt.update()'s
    calling convention (see carcontroller.py)."""
    if not CC.latActive:
      self._zero_state()
      self.human_turn_detector.reset()
      self.angle_human_turn_active = False
      self.precision_type = 1
      return LateralResult(apply_curvature=0.0, curvature_rate=0.0, path_offset=0.0, path_angle=0.0,
                           ramp_type=0, precision_type=1, lateralUncertainty=0.0)

    v_ego = float(CS.out.vEgoRaw)

    # Human-turn override: sustained driver press + large wheel angle -> force lateral inactive.
    # Always on in angle mode (no param gate, matching BluePilot -- this is not something a user
    # should be able to opt out of; see human_turn_pnw.py docstring).
    self.angle_human_turn_active = self.human_turn_detector.update(
      True, CS.out.steeringPressed, CS.out.steeringAngleDeg)
    if self.angle_human_turn_active:
      self._zero_state()
      self._desired_curvature_last = float(actuators.curvature)
      self.precision_type = 1
      return LateralResult(apply_curvature=0.0, curvature_rate=0.0, path_offset=0.0, path_angle=0.0,
                           ramp_type=0, precision_type=1, lateralUncertainty=0.0)

    self.precision_type = 1
    precision = 1
    desired_curvature = float(actuators.curvature)
    predicted_curvature = self._predicted_curvature(v_ego)

    # Exit-biased blend: near the PSCM authority limit or while the planner is actively reducing
    # curvature (exit detected), drop model-prediction weight so the planner's natural unwind
    # dominates instead of a model prediction that still sees the curve, or a snap when the
    # model's lookahead crosses the curve exit. LatCtlLim_D_Stat does not fire in angle mode on
    # CAN-FD Fords, so the DBC-limit-proximity proxy (_dbc_sat) is the only saturation signal.
    _kappa_entering = abs(predicted_curvature) > abs(desired_curvature)
    _dbc_sat = (self.path_angle_last >= FORD_DBC_PATH_ANGLE_MAX * 0.90 or
                self.path_angle_last <= FORD_DBC_PATH_ANGLE_MIN * 0.90)
    _desired_falling = abs(desired_curvature) < abs(self._desired_curvature_last) - 0.010
    _on_exit_near_limit = not _kappa_entering and (_dbc_sat or _desired_falling)
    b = 0.15 if _on_exit_near_limit else 0.60  # BluePilot default full-blend weight is 0.60
    requested_curvature = predicted_curvature * b + desired_curvature * (1.0 - b)
    self._desired_curvature_last = desired_curvature

    if self.model is not None:
      self.lane_change = self.model.meta.laneChangeState in (1, 2, 3)
    else:
      self.lane_change = False

    lane_change_factor = interp(v_ego, self.lane_change_factor_bp,
                                [self.lane_change_factor_low, self.lane_change_factor_high_ang])
    if self.lane_change and self.model is not None:
      if self.model.meta.laneChangeDirection == 1 and requested_curvature < 0:
        requested_curvature *= lane_change_factor
        precision = 0
      elif self.model.meta.laneChangeDirection == 2 and requested_curvature > 0:
        requested_curvature *= lane_change_factor
        precision = 0
    self.precision_type = precision

    kappa_cmd = float(requested_curvature)

    # Curvature-deviation clip -- mirrors lateral_curv_pnw.apply_ford_curvature_limits_ext exactly
    # (same formula, same v_ego>9 gate, same CarControllerParams.CURVATURE_ERROR tolerance).
    # Without this, kappa_cmd (and the shadow_curvature sent to ford.h) can lead the measured
    # curvature by more than ford.h's angle-error tolerance during normal curve entry/exit, and
    # ford_shadow_curvature_error_check would then block routinely, not just on genuine divergence.
    current_curvature = -CS.out.yawRate / max(v_ego, 0.1)
    self.bp_curvature_deviation_limited = False
    if v_ego > 9:
      _pre_clip = kappa_cmd
      kappa_cmd = float(clip(kappa_cmd, current_curvature - CarControllerParams.CURVATURE_ERROR,
                            current_curvature + CarControllerParams.CURVATURE_ERROR))
      self.bp_curvature_deviation_limited = bool(abs(kappa_cmd - _pre_clip) > 1e-9)

    # Speed-interpolated gain: at low speed both curves use 1.0; at high speed platform gain +
    # the (currently-fixed-at-1.0) user feel factors take effect.
    low_gain = interp(v_ego, [13.5, 26.82], [1.0, self.gain_lowC_highV])
    high_gain = interp(v_ego, [13.5, 26.82],
                       [(1.30 * self.low_speed_curv_factor), (self.gain_highC_highV * self.high_speed_curv_factor)])
    curvature_factor = interp(abs(kappa_cmd), [0.0007, 0.001], [low_gain, high_gain])

    path_angle = kappa_cmd * v_ego * curvature_factor

    # PSCM authority limit clamp: block magnitude increases and rate-limit decreases while near
    # the DBC path_angle limit -- see _PSCM_SAT_UNWIND_RATE comment above.
    if _dbc_sat:
      _last = self.path_angle_last
      _last_mag = abs(_last)
      _curr_mag = abs(path_angle)
      if _curr_mag > _last_mag:
        path_angle = _last
      elif _last_mag - _curr_mag > _PSCM_SAT_UNWIND_RATE:
        _limited_mag = _last_mag - _PSCM_SAT_UNWIND_RATE
        path_angle = float(_limited_mag if _last >= 0 else -_limited_mag)

    path_angle = min(FORD_DBC_PATH_ANGLE_MAX, max(FORD_DBC_PATH_ANGLE_MIN, path_angle))

    # Soft ROC -- unconditional, tighter than ford.h's backstop (FORD_PATH_ANGLE_LIMITS_ANGLE).
    _soft_roc = float(interp(v_ego, _SOFT_ROC_BP, _SOFT_ROC_V))
    _pre_roc = path_angle
    path_angle = float(clip(path_angle, self.path_angle_last - _soft_roc, self.path_angle_last + _soft_roc))
    self.bp_angle_rate_limited = bool(abs(path_angle - _pre_roc) > 1e-9)

    self.path_angle_last = path_angle
    self.apply_curvature_last = 0.0
    self.bp_kappa_cmd = kappa_cmd

    return LateralResult(
      apply_curvature=0.0,       # curvature (c2) stays pinned inactive on the wire in angle mode
      curvature_rate=0.0,        # curvature_rate (c3) likewise
      path_offset=0.0,           # c0: no centering trim in angle mode (see module docstring)
      path_angle=path_angle,     # c1: the actuator
      ramp_type=2,
      precision_type=self.precision_type,
      lateralUncertainty=0.0,
    )

import json
import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.carlog import carlog
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.ford import fordcan
from opendbc.car.ford import fordcan_pnw  # fordsafety2pnw: BluePilot 4-signal lateral builders
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
# lightning-extra2pnw. Aliased with a leading underscore so the 100 Hz control loop does no import
# work per frame, and so these names cannot be confused with the cruise-button constants.
from opendbc.car.ford.lightning_extra_pnw import (PPO_ADDR as _PPO_ADDR, PPO_RATE_HZ as _PPO_RATE_HZ,
                                                  PpoInputs as _PpoInputs)
from opendbc.car.interfaces import CarControllerBase, V_CRUISE_MAX
from opendbc.car.pnw_vehicle import PnwVehicle

LongCtrlState = structs.CarControl.Actuators.LongControlState
VisualAlert = structs.CarControl.HUDControl.VisualAlert

# fordlat2pnw: predicted-curvature blend constants (BluePilot/alan-polk defaults: pc_blend_ratio
# low/high both 0.40, lookup 0.2 s). Tunable from drive telemetry (strAng excursions on turn exit).
PC_BLEND_LOOKUP_S = 0.2   # seconds into the model horizon for the predicted curvature
PC_BLEND_RATIO = 0.40     # predicted weight; (1 - ratio) = planner-desired weight
PC_BLEND_MIN_V = 9.0      # m/s — blend only where apply_ford_curvature_limits' current-curvature
                          # clip is ACTIVE (>9); below it, 1/v amplifies resting model-yaw noise and
                          # the clip is bypassed (Gemini catch) -> pure stock desired curvature

# CAN FD limits:
# Limit to average banked road since safety doesn't have the roll
AVERAGE_ROAD_ROLL = 0.06  # ~3.4 degrees, 6% superelevation. higher actual roll raises lateral acceleration
MAX_LATERAL_ACCEL = ISO_LATERAL_ACCEL - (ACCELERATION_DUE_TO_GRAVITY * AVERAGE_ROAD_ROLL)  # ~2.4 m/s^2


def anti_overshoot(apply_curvature, apply_curvature_last, v_ego):
  diff = 0.1
  tau = 5  # 5s smooths over the overshoot
  dt = DT_CTRL * CarControllerParams.STEER_STEP
  alpha = 1 - np.exp(-dt / tau)

  lataccel = apply_curvature * (v_ego ** 2)
  last_lataccel = apply_curvature_last * (v_ego ** 2)
  last_lataccel = apply_hysteresis(lataccel, last_lataccel, diff)
  last_lataccel = alpha * lataccel + (1 - alpha) * last_lataccel

  output_curvature = last_lataccel / (max(v_ego, 1) ** 2)

  return float(np.interp(v_ego, [5, 10], [apply_curvature, output_curvature]))


def apply_ford_curvature_limits(apply_curvature, apply_curvature_last, current_curvature, v_ego_raw, steering_angle, lat_active, CP):
  # No blending at low speed due to lack of torque wind-up and inaccurate current curvature
  if v_ego_raw > 9:
    apply_curvature = np.clip(apply_curvature, current_curvature - CarControllerParams.CURVATURE_ERROR,
                              current_curvature + CarControllerParams.CURVATURE_ERROR)

  # Curvature rate limit after driver torque limit
  apply_curvature = apply_std_steer_angle_limits(apply_curvature, apply_curvature_last, v_ego_raw, steering_angle, lat_active, CarControllerParams.ANGLE_LIMITS)

  # Ford Q4/CAN FD has more torque available compared to Q3/CAN so we limit it based on lateral acceleration.
  # Safety is not aware of the road roll so we subtract a conservative amount at all times
  if CP.flags & FordFlags.CANFD:
    # Limit curvature to conservative max lateral acceleration
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(v_ego_raw, 1) ** 2)
    apply_curvature = float(np.clip(apply_curvature, -curvature_accel_limit, curvature_accel_limit))

  return apply_curvature


def apply_creep_compensation(accel: float, v_ego: float) -> float:
  creep_accel = np.interp(v_ego, [1., 3.], [0.6, 0.])
  creep_accel = np.interp(accel, [0., 0.2], [creep_accel, 0.])
  accel -= creep_accel
  return float(accel)


# fordregen2pnw Fix B: EV regen-bite compensation for the F-150 Lightning. The truck's PCM turns a
# gentle gas/coast request into much STRONGER regen deceleration than an ICE would (live-measured:
# openpilot commands ~-0.3 m/s^2, the truck delivers ~-1.25), producing the op-long "deadband-then-
# bite" jerk that stock ACC (tuned to its own regen) doesn't have. This adds a small, bounded POSITIVE
# bias to the GAS (throttle/coast) request in the coast band so a gentle command doesn't invoke the
# hard regen bite.
#
# SAFETY (this is why it cannot reduce braking authority):
#   * It only ever touches `gas` (the throttle/coast request, AccPrpl_A_Rq). It NEVER touches `accel`,
#     which is the separate BRAKE signal, nor the brake_request / precharge bits.
#   * It only fires while gas is inside (MIN_GAS, REGEN_COAST_HI) and TAPERS TO 0 at both edges. Below
#     MIN_GAS the gas request is INACTIVE and braking runs entirely via `accel`/brake_request — a hard
#     stop or lead-braking command lives there, gets ZERO bias, and keeps full authority.
#   * The bias is POSITIVE only (never adds regen) and hard-clamped to a small envelope, whatever the
#     tuning file says.
# Magnitude is tunable at /data/pnw/regen.json (only ONE clean bite data point exists so far, so ship a
# conservative default and refine on-road — the file reloads without a rebuild, like rain.json/dm.json).
# Ford-only (this is the ford carcontroller); Tesla is untouched.
REGEN_CFG_PATH = "/data/pnw/regen.json"
_REGEN_DEFAULTS = {"coast_bias": 0.15, "coast_hi": 0.0}   # coast_bias=0.0 disables it; band = (MIN_GAS, 0]


def _load_regen_config() -> dict:
  cfg = dict(_REGEN_DEFAULTS)
  try:
    with open(REGEN_CFG_PATH) as f:
      raw = json.load(f)
    for k in _REGEN_DEFAULTS:
      if k in raw:
        cfg[k] = float(raw[k])
  except Exception:
    pass
  # hard safety envelope regardless of the file: never negative (never ADD regen), never large.
  cfg["coast_bias"] = min(max(cfg["coast_bias"], 0.0), 0.4)
  cfg["coast_hi"] = min(max(cfg["coast_hi"], 0.0), 0.3)
  return cfg


def _regen_gas_bias(gas: float, cfg: dict) -> float:
  """Positive bias for a GAS request in the coast band (MIN_GAS, coast_hi), triangular so it is 0 at
  both edges — 0 at MIN_GAS (continuous into the untouched brake region) and 0 at coast_hi."""
  peak = cfg["coast_bias"]
  lo = CarControllerParams.MIN_GAS   # below this, gas is INACTIVE and braking uses `accel` -> no bias
  hi = cfg["coast_hi"]
  if peak <= 0.0 or hi <= lo or not (lo < gas < hi):
    return 0.0
  mid = 0.5 * (lo + hi)
  half = 0.5 * (hi - lo)
  bias = peak * max(0.0, 1.0 - abs(gas - mid) / half)
  # HARD GUARD (Gemini #6): never invert a decel request into an accel request. Cap the bias so the
  # biased gas can only rise toward 0, never cross into positive (throttle). So even at max tuning the
  # worst case is "no regen", never "the truck accelerates when the planner asked it to slow".
  return min(bias, max(0.0, -gas))


class CarController(CarControllerBase):
  def __init__(self, dbc_names, CP):
    super().__init__(dbc_names, CP)
    self.packer = CANPacker(dbc_names[Bus.pt])
    self.CAN = fordcan.CanBus(CP)

    self.apply_curvature_last = 0
    self.anti_overshoot_curvature_last = 0
    self.accel = 0.0
    self.gas = 0.0
    self.brake_request = False
    self.main_on_last = False
    self._regen_cfg = _load_regen_config()   # fordregen2pnw Fix B: EV regen-bite gas compensation
    self.lkas_enabled_last = False
    self.steer_alert_last = False
    self.lead_distance_bars_last = None
    self.distance_bar_frame = 0

    # fordlat2pnw: predicted-curvature blend for the F-150 Lightning — port of BluePilot's
    # (alan-polk) pc_blend mechanism, CURVATURE-SIGNAL-ONLY so it stays entirely within stock Ford
    # panda safety (no flash; the 4-signal LateralCurvExt is deliberately NOT ported). The model's
    # predicted curvature LEADS the planner's desired curvature, so blending it in relaxes steering
    # earlier on turn exit — fixes the post-curve wind-up that threw the truck left onto straights
    # (driver report 2026-07-11). Guarded imports: bare opendbc checkout -> blend off, pure stock.
    veh = PnwVehicle(CP)  # capability view — no fingerprint checks in feature code (driver directive)

    # fordsafety2pnw: BluePilot's (alan-polk) full 4-signal lateral control (LateralCurvExt).
    # REQUIRES the matching 4-signal ford.h panda safety from this branch — with STOCK ford safety
    # the nonzero curvature_rate would be blocked on the bus and lateral would go dead, so this
    # capability must only ship together with the panda rebuild. Guarded imports: on a bare opendbc
    # checkout (no cereal / modeld constants) construction fails and we fall back to the stock
    # curvature-only path below. When active, LateralCurvExt OWNS lateral: it contains its own
    # predicted-curvature blend and human-turn reset, so the standalone pc_blend/ht_reset helpers
    # below are bypassed to avoid double-applying them.
    # angle2pnw-faithful2: Alan Polk's BluePilot bp-7.0 angle-primary lateral strategy
    # (LateralAngleExt) — see lateral_angle_pnw.py's module docstring for the full design and
    # drives/2026-07-18/lightning-angle-steering/ALAN-POLK-PORT-DEVIATIONS.md for the audited
    # deviation manifest. Mutually exclusive with the 4-signal curvature path (self._latext) --
    # angle mode drives 100% of the steering through path_angle and pins curvature/curvature_rate
    # at zero, so there is nothing for the two strategies to combine. veh.angle_lat is an EXPLICIT
    # driver opt-in (FordAngleLateral toggle, default OFF) and therefore takes precedence over
    # veh.four_signal_lat below -- four_signal_lat is an always-on capability for the Lightning
    # today (not a toggle), so without this precedence the 4-signal path would construct first
    # every time and permanently starve angle mode of ever running, even with the toggle on. With
    # the toggle off (the default), veh.angle_lat is False, construction never runs, and the
    # STEER_STEP/LKA_STEP dispatch below falls through to the untouched 4-signal/pc-blend/stock
    # path exactly as before -- runtime behavior is byte-identical to today's curvature lateral.
    self._latext_angle = None
    if veh.angle_lat:
      try:
        from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
        self._latext_angle = LateralAngleExt(CP)
      except Exception:
        self._latext_angle = None

    self._latext = None
    if veh.four_signal_lat and self._latext_angle is None:
      try:
        from opendbc.car.ford.lateral_curv_pnw import LateralCurvExt
        self._latext = LateralCurvExt(CP)
      except Exception:
        self._latext = None

    self._pcblend_enabled = veh.pc_blend and self._latext is None and self._latext_angle is None
    # angle2pnw-faithful2: LKA-message state for ford.h's angle-mode corroboration channel (see
    # fordcan_pnw.create_lka_msg). Stay False/0.0 whenever _latext_angle is None -- the LKA send
    # site below only calls the pnw builder when _latext_angle is not None, so these values are
    # unread (and the wire is byte-identical to stock) whenever angle mode is off.
    self._angle_mode_engaged = False
    self._shadow_curvature = 0.0

    # fordlong2pnw: BluePilot highway follow control. Needs the radarState SubMaster that
    # LateralCurvExt owns, so it activates only alongside a live 4-signal path; falls back to
    # the stock ACC message on any failure (same never-kill-card policy as lateral).
    self._longext = None
    if veh.bp_long_follow and self._latext is not None:
      try:
        from opendbc.car.ford.longitudinal_ext_pnw import LongitudinalExt
        self._longext = LongitudinalExt()
      except Exception:
        self._longext = None
    # fordlat_pnw human-turn reset (see fordlat_pnw.py) — guarded like everything else
    self._htreset = None
    if veh.ht_reset and self._latext is None:
      try:
        from opendbc.car.ford.fordlat_pnw import HumanTurnHold
        self._htreset = HumanTurnHold()
      except Exception:
        self._htreset = None
    self._pcblend_sm = None
    self._pcblend_tidxs = None
    if self._pcblend_enabled:
      try:
        import cereal.messaging as messaging
        try:
          from openpilot.selfdrive.modeld.constants import ModelConstants
        except ImportError:
          from selfdrive.modeld.constants import ModelConstants
        self._pcblend_sm = messaging.SubMaster(['modelV2'])
        self._pcblend_tidxs = list(ModelConstants.T_IDXS)
      except Exception:
        self._pcblend_enabled = False

    # fordlatui2pnw: publish which lateral path is live (4-signal / pc-blend / stock) so the UI
    # overlay can WARN if the alan-polk 4-signal path silently fell back to stock (e.g. LateralCurvExt
    # failed to construct). Independent /dev/shm handle — the 4-signal runs in BOTH Lightning long
    # modes (not only when ICBM is enabled). Display-only; fully guarded so it can never affect control.
    self._latstat_params = None
    try:
      from openpilot.common.params import Params as _P
      self._latstat_params = _P("/dev/shm/params")
    except Exception:
      self._latstat_params = None

    # icbm2pnw / speedadjust-exec2pnw: ONE generic stock-ACC button-management executor for the
    # F-150 Lightning (Tier 1, no op-long), gated on the car-agnostic `veh.button_management`
    # capability (true today only for the Lightning's stock-ACC buttons; NOT a fingerprint check
    # here — see opendbc/car/pnw_vehicle.py). Any number of car-agnostic pnw brains can publish a
    # {target, ceiling, ts, dir?} mem-param and get slowdowns for free on any car that declares this
    # capability and implements the (necessarily per-brand) tap executor below — today: icbm2pnw
    # (curve slow-downs, ces_pnw.py -> IcbmTarget) and speedadjust2pnw (police-ahead / lower-speed-
    # limit reduce-only cap, speedadjust_controller.py -> SpeedAdjustTarget). arbitrate() (icbm_pnw.py)
    # reduces every brain's command to the ONE unified button-management target every poll, before
    # decide_press() runs — see its docstring for the rule. Params import is runtime-only and guarded:
    # on a bare opendbc checkout this simply stays off.
    self._icbm_enabled = veh.button_management
    self._icbm_governor = None
    self._icbm_guard = None
    self._icbm_cmd = None
    self._sa_cmd = None          # speedadjust-exec2pnw: the SpeedAdjustTarget-derived command
    # restore2pnw-hardening: oscillation debounce -- last time ANY fresh dec won the shared bus (see
    # icbm_pnw.arbitrate's last_dec_ts).
    self._last_dec_ts = None
    self._icbm_params = None
    if self._icbm_enabled:
      try:
        from openpilot.common.params import Params
        from opendbc.car.ford.icbm_pnw import PressGovernor, RestoreGuard
        self._icbm_params = Params("/dev/shm/params")
        self._icbm_governor = PressGovernor()
        self._icbm_guard = RestoreGuard()   # icbmrestore2pnw: human-detection latch for the inc path
      except Exception:
        self._icbm_enabled = False

    # madsresume2pnw: the RESUME tap. Deliberately its OWN flag, its OWN mem-param, its OWN parser
    # and its OWN one-shot press latch -- it shares nothing with the SET+/- path above except the
    # 0x083 frame, because its cruise precondition is the exact OPPOSITE (cruise OFF, not ON).
    # Also gated on the MADS capability: without the flashed MADS panda there is no brake-induced
    # lateral-only state for it to complete, and the brain would never publish anyway.
    self._resume_cmd = None
    self._resume_press = None
    self._resume_get_fail = 0
    self._resume_lat_block = 0     # consecutive frames a SET offer was held back by latActive
    # lightning-extra2pnw: re-arm Pro Power Onboard once per ignition. Constructed unconditionally
    # (it is inert on any truck that never reports the state) and never allowed to raise into the
    # control path -- see the call site.
    self._ppo_armer = None
    self._ppo_fail = 0
    # 0x455 frames handed to pandad this ignition. NOT proof the panda accepted them -- see the
    # log wording at the call site.
    self._ppo_tx = 0
    # CAPABILITY, never a fingerprint test in feature code (pnw/CLAUDE.md). The gate is load-bearing:
    # 0x455 is `eCall_Info` in ford_cgea1_2_ptcan_2011.dbc, so on some other Ford this ID is an
    # EMERGENCY-CALL frame, and every Ford shares ford_lincoln_base_pt (Fable review 2026-09-07).
    if veh.pro_power_onboard:
      try:
        from opendbc.car.ford.lightning_extra_pnw import ProPowerArmer
        self._ppo_armer = ProPowerArmer()
      except Exception:
        carlog.exception("lightning-extra2pnw: ProPowerArmer construction FAILED -- feature INERT")
    self._resume_enabled = veh.mads_resume and self._icbm_params is not None
    if veh.mads_resume and self._icbm_params is None:
      # Fable S1: this car HAS the capability but the mem-param store never came up, so the executor
      # is dead while the brain keeps publishing offers. Silence here looks exactly like "the gates
      # refused" in the drive log. Say it once, loudly.
      carlog.error("madsresume2pnw: capability present but /dev/shm params unavailable -- auto-resume executor is INERT")
    if self._resume_enabled:
      try:
        from opendbc.car.ford.icbm_pnw import ResumePress
        self._resume_press = ResumePress()
      except Exception:
        carlog.exception("madsresume2pnw: ResumePress import/construction FAILED -- auto-resume executor is INERT")
        self._resume_enabled = False

  def _resume_button(self, CS, lat_active: bool) -> bool:
    """madsresume2pnw: poll MadsResumeTarget at 4 Hz, gate it against the real car state at 100 Hz,
    and assert RESUME for exactly one press per offer. Returns True on the frames the button should
    be asserted. NEVER raises into the control path.

    The gate list lives in icbm_pnw.decide_resume() (pure, unit-tested). The one that matters most
    here: `cruise_enabled` must be FALSE -- RES while ACC is engaged is a SET+ on Ford, and raising
    the driver's set speed is the single thing this feature must never do."""
    import time
    from opendbc.car.ford.icbm_pnw import SET_DIR, decide_resume, parse_resume_cmd
    # 4 Hz while idle (nothing to act on), but EVERY FRAME once a command is in hand.
    # Gemini review 2026-09-06: at a flat 4 Hz this held a CACHED command for up to 250 ms after the
    # brain cleared the mem-param, and since the cached `ts` was still inside RESUME_STALE_LIMIT_S
    # it would have gone on pressing RESUME a quarter of a second after the brain aborted for a lead
    # cutting in. The brain's fast-abort is only real if the withdrawal is read at the control rate.
    if self._resume_cmd is not None or (self.frame % 25) == 0:
      try:
        self._resume_cmd = parse_resume_cmd(self._icbm_params.get("MadsResumeTarget"))
        self._resume_get_fail = 0
      except Exception:
        # Fable S1: fail closed, but not silently. Throttled -- this can run at 100 Hz.
        self._resume_cmd = None
        self._resume_get_fail += 1
        if self._resume_get_fail == 1 or self._resume_get_fail % 1000 == 0:
          carlog.exception(f"madsresume2pnw: MadsResumeTarget read FAILING ({self._resume_get_fail} consecutive) -- auto-resume executor is blind")
    # MONOTONIC, matching the clock the brain stamps `ts` with -- see RESUME_STALE_LIMIT_S.
    ok = decide_resume(self._resume_cmd, time.monotonic(),
                       bool(CS.out.cruiseState.enabled), bool(CS.out.cruiseState.available),
                       bool(CS.out.gasPressed or CS.out.brakePressed),
                       float(CS.out.cruiseState.speed))
    # The SET path's latActive check is folded in HERE, ahead of the eid latch, never applied to its
    # result (Fable review 2026-09-07 round 3, finding D -- reproduced). ResumePress latches
    # `_used_eid` on the first frame it returns True, one press per eid ever. Checking latActive
    # AFTERWARDS therefore BURNED the eid while sending zero taps: no press, no record, the offer
    # unusable forever, and the brain reporting `fire` and then `verify: noCruise` 10 s later.
    # Folded in, a False simply means no frame goes out and the offer survives until latActive
    # returns inside the offer window.
    if self._resume_cmd is None:
      # No offer standing: clear the blocked-frame counter. Without this it is left orphaned above
      # zero when a blocked SET offer simply expires, and the NEXT blocked episode never hits the
      # `== 1` branch -- so its first warning is swallowed and the throttle reports nothing until
      # frame 500 (Gemini verification pass 2026-09-07, finding A).
      self._resume_lat_block = 0
    elif self._resume_cmd.mode == SET_DIR and not lat_active:
      ok = False
      # Rule 2: an executor-side refusal the brain cannot see must not be silent.
      self._resume_lat_block += 1
      if self._resume_lat_block == 1 or self._resume_lat_block % 500 == 0:
        carlog.warning(f"madsresume2pnw: SET offer standing but CC.latActive is False -- not pressing (frames: {self._resume_lat_block})")
    elif ok:
      self._resume_lat_block = 0
    return self._resume_press.update(self.frame, self._resume_cmd, ok)

  def _parse_button_cmd(self, raw):
    """speedadjust-exec2pnw: shared parser for both IcbmTarget and SpeedAdjustTarget — both mem-params
    use the identical {target, ceiling, ts, dir?} JSON shape. Returns an IcbmCommand or None; NEVER
    raises (fail-closed: any malformed/missing/unknown-dir/non-finite payload -> None, no press).
    restore2pnw-hardening: `json.loads` happily parses NaN/Infinity (non-standard but accepted by
    Python's json module) -- a non-finite target/ceiling/ts could poison a downstream min()/comparison,
    so every numeric field is explicitly finite-checked before accepting the command."""
    from opendbc.car.ford.icbm_pnw import IcbmCommand
    try:
      if isinstance(raw, (bytes, str)) and raw:
        raw = json.loads(raw)
      # params_pyx returns a dict for JSON keys; require all fields or stand down
      if isinstance(raw, dict) and all(k in raw for k in ("target", "ceiling", "ts")):
        # icbmrestore2pnw: optional "dir" marks a guarded restore ("inc"); anything else is a cap.
        # An unknown value stands down entirely (fail-closed on protocol drift).
        d = str(raw.get("dir", "dec"))
        if d in ("dec", "inc"):
          target, ceiling, ts = float(raw["target"]), float(raw["ceiling"]), float(raw["ts"])
          if math.isfinite(target) and math.isfinite(ceiling) and math.isfinite(ts):
            return IcbmCommand(target_ms=target, ceiling_ms=ceiling, ts=ts, dir=d)
    except Exception:
      pass
    return None

  def _icbm_buttons(self, CS) -> str | None:
    """Poll both brains' targets at ~4 Hz, arbitrate, run the executor at 100 Hz. Returns 'dec'/'inc'/None."""
    import time
    from opendbc.car.ford.icbm_pnw import arbitrate, decide_press
    if (self.frame % 25) == 0:  # 4 Hz mem-param read
      try:
        self._icbm_cmd = self._parse_button_cmd(self._icbm_params.get("IcbmTarget"))
      except Exception:
        self._icbm_cmd = None
      try:
        self._sa_cmd = self._parse_button_cmd(self._icbm_params.get("SpeedAdjustTarget"))
      except Exception:
        self._sa_cmd = None
    driver_override = bool(CS.out.gasPressed or CS.out.brakePressed)
    now = time.time()
    stock_set = float(CS.out.cruiseState.speed)
    # reduce every brain's command to the ONE unified button-management target before deciding what
    # to press — see arbitrate()'s docstring (DEC always wins; most-restrictive dec wins across
    # sources, with a debounce before a fresh inc after a dec). Extensible: any future brain just adds
    # its command to this list.
    cmd = arbitrate([self._icbm_cmd, self._sa_cmd], now, self._last_dec_ts)
    if cmd is not None and getattr(cmd, "dir", "dec") == "dec":
      self._last_dec_ts = now
    intent = decide_press(stock_set, cmd, now,
                          bool(CS.out.cruiseState.enabled), driver_override)
    # restore2pnw-hardening (cross-brain dec-interlude fix): the guard's veto latch must survive a
    # brief dec interlude from a DIFFERENT brain while THIS restore episode is still being offered
    # underneath — key `restoring`/`ceiling` to whether ANY brain still has a fresh inc command
    # pending (re-arbitrated over inc-only candidates), not just whichever command WON the bus this
    # exact tick. A dec that's actually the offering brain's OWN cap (cancelling its own restore) is
    # handled naturally: that brain stops publishing "inc" at all, so it drops out of `pending_inc` too.
    pending_inc = arbitrate([c for c in (self._icbm_cmd, self._sa_cmd)
                            if c is not None and getattr(c, "dir", "dec") == "inc"], now)
    restoring = pending_inc is not None
    ceiling = pending_inc.ceiling_ms if pending_inc is not None else None
    # Fable fail-safe fix: tell the guard whether THIS tick's arbitrated winner (`cmd`, not `intent` —
    # `cmd` reflects who owns the bus even on ticks decide_press stays silent, e.g. between taps) is a
    # dec, so it freezes movement-judgment rather than mistaking a dec interlude's own SET- taps for a
    # driver SET- and falsely latching the restore veto — see RestoreGuard.filter's docstring.
    dec_owns_bus = cmd is not None and getattr(cmd, "dir", "dec") == "dec"
    intent = self._icbm_guard.filter(intent, stock_set, now, restoring, ceiling, dec_owns_bus)
    return self._icbm_governor.update(self.frame, intent)

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # fordsafety2pnw: BluePilot updates SubMaster (modelV2/liveParameters/selfdriveState/radarState)
    # and the vehicle model every frame, before the lateral step
    if self._latext is not None:
      self._latext.update_sm()
    elif self._latext_angle is not None:
      self._latext_angle.update_sm()

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### lightning-extra2pnw: Pro Power Onboard re-arm ###
    # Wholly separate from everything below: its own frame (0x455), its own bounded state machine,
    # and no interaction with cruise. Runs before the acc-button chain purely so a failure here
    # cannot skip it. Every bound lives in ProPowerArmer; this only transmits what it returns.
    #
    # NOT A CONTROL PATH. The worst case is a body-comfort setting not being re-armed.
    if self._ppo_armer is not None:
      try:
        # Match the frame's own 10 Hz cadence rather than spraying at the 100 Hz control rate --
        # measured on the truck, the real sender transmits 0x455 at exactly 10.0 Hz.
        if (self.frame % int(100 / _PPO_RATE_HZ)) == 0:
          was = self._ppo_armer.phase
          # DELIBERATELY a local alias, not the module `time`. update() already does a bare
          # `import time` further down (the FordLatStatus block), which makes `time` a LOCAL name
          # for this entire function -- so a module-level import would leave this line raising
          # UnboundLocalError and crash card on the first tick the armer runs. Caught by ruff F823
          # when this was "helpfully" hoisted; declining Fable's N2 for that reason.
          import time as _time
          payload = self._ppo_armer.update(_PpoInputs(
            now=_time.monotonic(),
            standstill=bool(CS.out.standstill),
            # CLAUDE.md rule 3: the GEAR is the truth source for "parked", not IsOnroad and not
            # standstill alone -- a red light is a standstill too.
            #
            # COMPARE THE ENUM, NEVER str(). `CS.out` is a **capnp** struct here, and capnp renders
            # an enum as the bare member name: str(gearShifter) is 'park', NOT 'GearShifter.park'.
            # The original `str(...) == "GearShifter.park"` was therefore ALWAYS False, `parked`
            # never went true, the armer never left IDLE, and -- because the only log line fired on
            # a phase change -- the whole feature was silently inert for its entire first outing
            # (2026-09-07; found by Fable review, not by the tests, which feed `parked` directly to
            # the pure module). `card.py:488` already does this correctly; match it.
            parked=CS.out.gearShifter == structs.CarState.GearShifter.park,
            state_valid=bool(getattr(CS, "ppo_valid", False)),
            ppo_on=bool(getattr(CS, "ppo_on", False)),
            enabled=True,
            # Stale CAN keeps its last decoded values and `ppo_valid` is a forever-latch, so without
            # this the armer can burn all three presses into a silent bus and then report a
            # confident failure (Fable review 2026-09-07).
            can_valid=bool(CS.out.canValid),
          ))
          if payload is not None:
            can_sends.append((_PPO_ADDR, payload, self.CAN.main))
            self._ppo_tx += 1
          # Rule 2: log whatever the armer has to say, phase change or NOT. Logging only on a phase
          # change is what made the dead `parked` test above look exactly like a healthy feature
          # with nothing to do. "handed to pandad" is deliberate -- the panda may still refuse the
          # frame, so this count is an upper bound, not a delivery receipt.
          note = self._ppo_armer.take_note()
          if note:
            log = carlog.error if self._ppo_armer.phase == "failed" else carlog.warning
            log("lightning-extra2pnw: %s -> %s: %s (%d frames handed to pandad)",
                was, self._ppo_armer.phase, note, self._ppo_tx)
      except Exception:
        self._ppo_fail += 1
        if self._ppo_fail == 1 or self._ppo_fail % 1000 == 0:
          carlog.exception("lightning-extra2pnw: armer step FAILED (%d) -- feature inert this frame", self._ppo_fail)

    ### acc buttons ###
    # madsresume2pnw: evaluated EVERY frame (not inside the elif chain below) so its one-shot press
    # latch sees a continuous frame count and can abort mid-press the instant a gate stops holding.
    # Mutually exclusive with the SET+/- taps by construction -- decide_press requires cruise ON,
    # decide_resume requires cruise OFF -- but the branch below is ordered ahead of them anyway.
    resume_btn = False
    set_btn = False
    if self._resume_enabled:
      try:
        # gasset2pnw: same one-shot latch, two different buttons. "res" hands back the driver's
        # remembered set speed (RESUME); "set" establishes the speed they just chose with the
        # accelerator. SET- is used for the latter because it is the tap ICBM already proves on this
        # truck, and because in the impossible case that ACC were somehow engaged it moves the set
        # speed DOWN, never up. The SET latActive gate lives inside _resume_button, ahead of the eid
        # latch -- see there for what that check is and is not worth.
        from opendbc.car.ford.icbm_pnw import SET_DIR
        if self._resume_button(CS, bool(CC.latActive)):
          if self._resume_cmd is not None and self._resume_cmd.mode == SET_DIR:
            set_btn = True
          else:
            resume_btn = True
      except Exception:
        carlog.exception("madsresume2pnw: _resume_button failed -- no auto-resume this frame")
        resume_btn = False
        set_btn = False

    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # madsresume2pnw: openpilot taps RESUME once, on the driver's behalf, to give back the speed
    # THEY had already set -- only out of the brake-induced MADS lateral-only state, only inside the
    # bounded window after the brake is fully released, only once per brake event, and never with a
    # close lead. Same 0x083 frame + `resume=True` bit as the CC.cruiseControl.resume branch above
    # (the panda's ford.h resume gate now accepts lateral authority for exactly this).
    elif resume_btn and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
    # gasset2pnw: SET at the speed the driver just chose with the accelerator. Same 0x083 frame; the
    # SET- bit is NOT panda-gated (ford.h only validates the cancel and resume bits), so this needs
    # no panda change -- exactly as ICBM's SET+/- taps do not.
    elif set_btn and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, set_dec=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, set_dec=True))
    # if stock lane centering isn't off, send a button press to toggle it off
    # the stock system checks for steering pressed, and eventually disengages cruise control
    elif CS.acc_tja_status_stock_values["Tja_D_Stat"] != 0 and (self.frame % CarControllerParams.ACC_UI_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, tja_toggle=True))
    # icbm2pnw: steer the STOCK ACC set speed toward the brain's target via SET +/- taps (Lightning
    # only, never with op-long, never engages/resumes ACC — full envelope in icbm_pnw.py). Sent at
    # the SCCM 10 Hz cadence pattern to camera+main like cancel/resume above.
    elif self._icbm_enabled:
      btn = self._icbm_buttons(CS)
      if btn == "dec" and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
        # Caps stay DEC-ONLY (see icbm_pnw.py)
        can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, set_dec=True))
        can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, set_dec=True))
      elif btn == "inc" and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
        # icbmrestore2pnw: restore-only SET+ path — reachable ONLY for a brain command explicitly
        # marked dir="inc" (return to the driver's OWN latched ceiling), ceiling-clamped in
        # decide_press, human-latched by RestoreGuard, same 0x083 message the safety code already
        # TX-allows (cancel/resume bits are the only checked signals) — no panda change.
        can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, set_inc=True))
        can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, set_inc=True))

    ### lateral control ###
    # send steer msg at 20Hz
    if (self.frame % CarControllerParams.STEER_STEP) == 0 and self._latext is not None:
      # fordsafety2pnw: BluePilot 4-signal lateral path (curvature, curvature_rate, path_offset,
      # path_angle) — LateralCurvExt OWNS lateral here (own predicted blend + human-turn reset;
      # the standalone pc_blend/ht_reset helpers are disabled in __init__ when this is active).
      # BluePilot: do not run apply_ford_curvature_limits here or overwrite apply_curvature_last
      # before LateralCurvExt.update. Panda rate-checks desired_curvature vs the last TX on the
      # bus; that must match the prior frame's lat.apply_curvature only (not an intermediate
      # stock-limited value).
      # fordsafety2pnw resilience: a LateralCurvExt failure must NEVER kill card (today's numpy
      # cast crash was this exact blast radius — card died on every drive). On any exception,
      # log once and permanently fall back to the stock curvature-only path for this drive:
      # steering assist stays alive, the driver just loses the 4-signal extras.
      try:
        lat = self._latext.update(CC, CS, actuators, self.apply_curvature_last, self.CP)
      except Exception:
        carlog.exception("LateralCurvExt failed — falling back to stock lateral for this drive")
        self._latext = None
        lat = None
      if lat is None:
        pass  # one 20Hz frame without a lat msg; stock path resumes next STEER_STEP
      else:
        self.apply_curvature_last = lat.apply_curvature

        if self.CP.flags & FordFlags.CANFD:
          mode = 1 if CC.latActive else 0
          counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
          can_sends.append(fordcan_pnw.create_lat_ctl2_msg(
            self.packer, self.CAN, mode, lat.ramp_type, lat.precision_type,
            -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate, counter
          ))
        else:
          can_sends.append(fordcan_pnw.create_lat_ctl_msg(
            self.packer, self.CAN, CC.latActive, lat.ramp_type, lat.precision_type,
            -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate
          ))

    elif (self.frame % CarControllerParams.STEER_STEP) == 0 and self._latext_angle is not None:
      # angle2pnw-faithful2: Alan Polk's bp-7.0 angle-primary lateral path (LateralAngleExt) --
      # path_angle (c1) is the actuator; curvature/curvature_rate/path_offset stay pinned inactive
      # on the wire (see lateral_angle_pnw.py). Same never-kill-card fallback discipline as the
      # 4-signal path above: on any exception, log once and permanently fall back to the stock
      # curvature-only path for this drive.
      try:
        lat = self._latext_angle.update(CC, CS, actuators, self.CP)
      except Exception:
        carlog.exception("LateralAngleExt failed — falling back to stock lateral for this drive")
        self._latext_angle = None
        lat = None
        # The wire is curvature again from here on -- the LKA corroboration bit and shadow must
        # drop with it, or ford.h would keep judging the stock curvature frames under angle-mode
        # rules (blocked lateral). Part of the never-kill-card fallback deviation (manifest c).
        self._angle_mode_engaged = False
        self._shadow_curvature = 0.0
      if lat is None:
        pass  # one 20Hz frame without a lat msg; stock path resumes next STEER_STEP
      else:
        self.apply_curvature_last = lat.apply_curvature  # always 0.0 in angle mode

        # Human-turn override / stall-blip: force lateral inactive (mode 0, all-zero signals) for
        # the duration of a sustained manual turn or a post-override PSCM-reset pulse, instead of
        # winding path_angle into a stale command (see lateral_angle_pnw.py's module docstring).
        lat_active = CC.latActive and not (self._latext_angle.angle_human_turn_active
                                            or self._latext_angle.angle_stall_blip_active)
        # Alan Polk's semantics (bp-7.0 carcontroller.py line 223, review finding M1): the LKA
        # corroboration bit asserts whenever angle MODE is selected -- NOT only while lateral is
        # active. During human-turn/stall-blip mode-0 frames the bit stays set; bp_kappa_cmd there
        # carries the measured curvature (see lateral_angle_pnw.get_current_curvature, ported from
        # bp-dev 699c17d9fd), so ford.h validates the shadow against fresh reality -- matching his
        # fork -- rather than a stale zero. Our previous narrowing to lat_active was an unlisted deviation.
        self._angle_mode_engaged = True
        # SIGN CONVENTION (see SIGN-CONVENTION-TRACE.md): negated here to match the sign
        # convention path_angle/apply_curvature use on the wire (see the -lat.* sends just below,
        # and in the 4-signal branch above -- both negate all four LMC/LMC2 signals). ford.h's
        # angle_meas (measured curvature, from raw yaw rate, no negation) is calibrated against
        # that wire convention, not bp_kappa_cmd's internal (un-negated) one -- this is Alan Polk's
        # own bp-7.0 carcontroller.py convention (his file's comment on this exact line, verbatim):
        # "un-negated, shadow_curvature and angle_meas were consistently opposite-signed, so the
        # deviation check found a 'divergence' on every frame once speed crossed
        # angle_error_min_speed."
        self._shadow_curvature = -self._latext_angle.bp_kappa_cmd if self._angle_mode_engaged else 0.0

        if self.CP.flags & FordFlags.CANFD:
          mode = 1 if lat_active else 0
          counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
          can_sends.append(fordcan_pnw.create_lat_ctl2_msg(
            self.packer, self.CAN, mode, lat.ramp_type, lat.precision_type,
            -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate, counter
          ))
        else:
          can_sends.append(fordcan_pnw.create_lat_ctl_msg(
            self.packer, self.CAN, lat_active, lat.ramp_type, lat.precision_type,
            -lat.path_offset, -lat.path_angle, -lat.apply_curvature, -lat.curvature_rate
          ))

    elif (self.frame % CarControllerParams.STEER_STEP) == 0:
      # fordlat2pnw: blend the model's PREDICTED curvature (0.2 s lookahead, leads the planner) into
      # the desired curvature at BluePilot's default 40/60 ratio. The blended value flows through the
      # UNCHANGED stock pipeline below (anti_overshoot n/a for Lightning, curvature-error clip, rate
      # limits, CAN-FD lat-accel cap), so every stock safety property is preserved. Freshness-guarded:
      # a stale/absent model falls back to the pure desired curvature.
      desired_curvature = actuators.curvature
      if self._pcblend_enabled and CC.latActive and CS.out.vEgoRaw > PC_BLEND_MIN_V:
        try:
          self._pcblend_sm.update(0)
          model = self._pcblend_sm['modelV2']
          lane_changing = model.meta.laneChangeState in (1, 2, 3)  # preLaneChange/Starting/Finishing
          if (self._pcblend_sm.alive['modelV2'] and not lane_changing
              and len(model.orientationRate.z) >= 17):
            curvatures = np.array(model.orientationRate.z) / CS.out.vEgoRaw
            predicted = float(np.interp(PC_BLEND_LOOKUP_S, self._pcblend_tidxs, curvatures))
            desired_curvature = predicted * PC_BLEND_RATIO + desired_curvature * (1.0 - PC_BLEND_RATIO)
        except Exception:
          pass

      # fordlat_pnw human-turn reset: during a sustained manual turn, flush the COMMANDED curvature
      # to 0 through the normal rate limiter — on release it ramps back from ~0 instead of slamming
      # in from the value accumulated while fighting the driver (the other-lane release lurch).
      if self._htreset is not None and self._htreset.tick(CS.out.steeringPressed, CS.out.steeringAngleDeg):
        desired_curvature = 0.0

      # Bronco and some other cars consistently overshoot curv requests
      # Apply some deadzone + smoothing convergence to avoid oscillations
      if self.CP.carFingerprint in (CAR.FORD_BRONCO_SPORT_MK1, CAR.FORD_F_150_MK14):
        self.anti_overshoot_curvature_last = anti_overshoot(desired_curvature, self.anti_overshoot_curvature_last, CS.out.vEgoRaw)
        apply_curvature = self.anti_overshoot_curvature_last
      else:
        apply_curvature = desired_curvature

      # apply rate limits, curvature error limit, and clip to signal range
      current_curvature = -CS.out.yawRate / max(CS.out.vEgoRaw, 0.1)

      self.apply_curvature_last = apply_ford_curvature_limits(apply_curvature, self.apply_curvature_last, current_curvature,
                                                              CS.out.vEgoRaw, 0., CC.latActive, self.CP)

      if self.CP.flags & FordFlags.CANFD:
        # TODO: extended mode
        # Ford uses four individual signals to dictate how to drive to the car. Curvature alone (limited to 0.02m/s^2)
        # can actuate the steering for a large portion of any lateral movements. However, in order to get further control on
        # steer actuation, the other three signals are necessary. Ford controls vehicles differently than most other makes.
        # A detailed explanation on ford control can be found here:
        # https://www.f150gen14.com/forum/threads/introducing-bluepilot-a-ford-specific-fork-for-comma3x-openpilot.24241/#post-457706
        mode = 1 if CC.latActive else 0
        counter = (self.frame // CarControllerParams.STEER_STEP) % 0x10
        can_sends.append(fordcan.create_lat_ctl2_msg(self.packer, self.CAN, mode, 0., 0., -self.apply_curvature_last, 0., counter))
      else:
        can_sends.append(fordcan.create_lat_ctl_msg(self.packer, self.CAN, CC.latActive, 0., 0., -self.apply_curvature_last, 0.))

    # send lka msg at 33Hz
    if (self.frame % CarControllerParams.LKA_STEP) == 0:
      if self._latext_angle is not None:
        # angle2pnw-faithful2: tell ford.h whether angle mode is engaged, out-of-band from
        # LMC/LMC2, packed into Lane_Assist_Data1's unused bits (see fordcan_pnw.create_lka_msg).
        # self._angle_mode_engaged / self._shadow_curvature are set by the angle-mode STEER_STEP
        # branch above and persist between LKA_STEP ticks (LKA_STEP=33Hz does not align with
        # STEER_STEP=20Hz).
        can_sends.append(fordcan_pnw.create_lka_msg(
          self.packer, self.CAN, CC.latActive, hud_control,
          self._angle_mode_engaged, self._shadow_curvature))
      else:
        # Unchanged stock call -- byte-identical output while angle mode is off (which is always,
        # unless the driver has flipped FordAngleLateral on a Lightning; see
        # fordcan_pnw.create_lka_msg's docstring for why the two calls are equivalent then).
        can_sends.append(fordcan.create_lka_msg(self.packer, self.CAN))

    ### longitudinal control ###
    # send acc msg at 50Hz
    if self.CP.openpilotLongitudinalControl and (self.frame % CarControllerParams.ACC_CONTROL_STEP) == 0:
      accel = actuators.accel
      gas = accel

      if CC.longActive:
        # Compensate for engine creep at low speed.
        # Either the ABS does not account for engine creep, or the correction is very slow
        # TODO: verify this applies to EV/hybrid
        accel = apply_creep_compensation(accel, CS.out.vEgo)

        # The stock system has been seen rate limiting the brake accel to 5 m/s^3,
        # however even 3.5 m/s^3 causes some overshoot with a step response.
        accel = max(accel, self.accel - (3.5 * CarControllerParams.ACC_CONTROL_STEP * DT_CTRL))

      accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
      gas = float(np.clip(gas, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))

      # Both gas and accel are in m/s^2, accel is used solely for braking
      if not CC.longActive or gas < CarControllerParams.MIN_GAS:
        gas = CarControllerParams.INACTIVE_GAS

      # PCM applies pitch compensation to gas/accel, but we need to compensate for the brake/pre-charge bits
      accel_due_to_pitch = 0.0
      if len(CC.orientationNED) == 3:
        accel_due_to_pitch = math.sin(CC.orientationNED[1]) * ACCELERATION_DUE_TO_GRAVITY

      accel_pitch_compensated = accel + accel_due_to_pitch
      if accel_pitch_compensated > 0.3 or not CC.longActive:
        self.brake_request = False
      elif accel_pitch_compensated < 0.0:
        self.brake_request = True

      stopping = CC.actuators.longControlState == LongCtrlState.stopping

      lng = None
      if self._longext is not None and self._latext is not None:
        # fordlong2pnw: BP follow control on top of the stock-processed accel/gas. Downhill
        # clamp applied here per BP (disable_downhill_comp_UI default): negative pitch -> 0.
        try:
          pitch_for_bp = accel_due_to_pitch
          if self._longext.disable_downhill_comp_UI and pitch_for_bp < 0:
            pitch_for_bp = 0.0
          lng = self._longext.update(CC, CS, self._latext.sm, accel, gas, pitch_for_bp,
                                     CS.out.vEgo * 2.23694, stopping, V_CRUISE_MAX)
        except Exception:
          carlog.exception("LongitudinalExt failed — falling back to stock long for this drive")
          self._longext = None
          lng = None

      # fordlong2pnw hop fix (field incident 2026-07-12, city stop-and-go "hopping like a horse"):
      # ONLY use the BP acc message (narrow -0.14/-0.06 brake hysteresis + precharge split) when BP
      # follow control is ACTUALLY applied (bp_long_used: >50mph deadband etc.). Below the deadband
      # the BP path previously still owned the brake bit with its narrow band -> the request
      # flapped around gentle city decels and pulsed the brakes. Stock path = stock 0.0/0.3
      # hysteresis, byte-identical to pre-port city behavior.
      # fordregen2pnw Fix B: reload the regen tuning at ~1 Hz so on-road refinement of
      # /data/pnw/regen.json takes effect without a rebuild (negligible I/O at 1 Hz).
      if (self.frame % 100) == 0:
        self._regen_cfg = _load_regen_config()

      if lng is not None and lng.bp_long_used:
        # apply the regen-bite gas bias to whichever gas actually goes on the wire (BP path, >50 mph).
        bp_gas = lng.gas + (_regen_gas_bias(lng.gas, self._regen_cfg) if CC.longActive else 0.0)
        can_sends.append(fordcan_pnw.create_acc_msg(
          self.packer, self.CAN, CC.longActive, bp_gas, lng.accel, lng.accel_pred_send,
          lng.stopping, lng.brake_actuate, lng.precharge_actuate, v_ego_kph=lng.target_speed))
        self.accel = lng.accel
        self.gas = bp_gas
      else:
        # TODO: look into using the actuators packet to send the desired speed
        st_gas = gas + (_regen_gas_bias(gas, self._regen_cfg) if CC.longActive else 0.0)
        can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, st_gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

        self.accel = accel
        self.gas = st_gas

    ### ui ###
    send_ui = (self.main_on_last != main_on) or (self.lkas_enabled_last != CC.latActive) or (self.steer_alert_last != steer_alert)
    # send lkas ui msg at 1Hz or if ui state changes
    if (self.frame % CarControllerParams.LKAS_UI_STEP) == 0 or send_ui:
      can_sends.append(fordcan.create_lkas_ui_msg(self.packer, self.CAN, main_on, CC.latActive, steer_alert, hud_control, CS.lkas_status_stock_values))

    # send acc ui msg at 5Hz or if ui state changes
    if hud_control.leadDistanceBars != self.lead_distance_bars_last:
      send_ui = True
      self.distance_bar_frame = self.frame

    if (self.frame % CarControllerParams.ACC_UI_STEP) == 0 or send_ui:
      show_distance_bars = self.frame - self.distance_bar_frame < 400
      if self._latext is not None:
        # fordlat2pnw/hud: 4-signal path -> BluePilot rich cluster messaging. BlueCruise blue display
        # + DM-state-driven TJA warning/text (Resume Control / Cancelled / hands prompts / lane
        # departure) computed from selfdriveState.alertType (already subscribed by LateralCurvExt).
        # Display-only, ACCDATA_3 (0x18A) is TX-allowlisted -> no safety surface.
        standstill = CS.out.cruiseState.standstill
        alert_type = self._latext.ss.alertType if getattr(self._latext, 'ss', None) is not None else ""
        _tja_msg, _tja_warn, _hands = fordcan_pnw.compute_dm_msg_values(
          alert_type, hud_control, True, main_on, standstill)
        can_sends.append(fordcan_pnw.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on,
          CC.latActive, fcw_alert, standstill, hud_control, CS.acc_tja_status_stock_values,
          True, send_ui, show_distance_bars, _tja_warn, _tja_msg))
      else:
        can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                   fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                   hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    # fordsafety2pnw: float() casts are BluePilot's, not optional — the 4-signal LateralCurvExt
    # math returns numpy.float64 (np.clip/np.interp), and capnp setters reject numpy types
    # (KjException "unsupported type"). Without the cast, card dies the moment lateral engages.
    new_actuators.curvature = float(self.apply_curvature_last)
    new_actuators.accel = float(self.accel)
    new_actuators.gas = float(self.gas)

    # fordlatui2pnw: ~4 Hz lateral-path status for the UI overlay. "4sig" = alan-polk LateralCurvExt
    # owns lateral; "angle" = angle2pnw-faithful2 LateralAngleExt owns lateral; "pc" = predicted-
    # curvature blend fallback; "stock" = plain curvature. Display-only, fully guarded (a param
    # hiccup here must never touch the actuators returned above).
    if self._latstat_params is not None and (self.frame % 25) == 0:
      try:
        import time
        _mode = ("4sig" if self._latext is not None else
                "angle" if self._latext_angle is not None else
                "pc" if self._pcblend_enabled else "stock")
        self._latstat_params.put_nonblocking("FordLatStatus", {"mode": _mode, "ts": round(time.time(), 2)})
      except Exception:
        pass

    self.frame += 1
    return new_actuators, can_sends

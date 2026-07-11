import math
import numpy as np
from opendbc.can import CANPacker
from opendbc.car import ACCELERATION_DUE_TO_GRAVITY, Bus, DT_CTRL, apply_hysteresis, structs
from opendbc.car.lateral import ISO_LATERAL_ACCEL, apply_std_steer_angle_limits
from opendbc.car.ford import fordcan
from opendbc.car.ford import fordcan_pnw  # fordsafety2pnw: BluePilot 4-signal lateral builders
from opendbc.car.ford.values import CarControllerParams, FordFlags, CAR
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
    self._latext = None
    if veh.four_signal_lat:
      try:
        from opendbc.car.ford.lateral_curv_pnw import LateralCurvExt
        self._latext = LateralCurvExt(CP)
      except Exception:
        self._latext = None

    self._pcblend_enabled = veh.pc_blend and self._latext is None
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

    # icbm2pnw: stock-ACC set-speed steering for the F-150 Lightning (Tier 1, no op-long). The brain
    # (target selection from CES/VTSC curve logic) runs in the pnw layer and publishes the IcbmTarget
    # mem-param; this side is only the closed-loop executor (see icbm_pnw.py for the safety envelope).
    # Params import is runtime-only and guarded: on a bare opendbc checkout ICBM simply stays off.
    self._icbm_enabled = veh.icbm
    self._icbm_governor = None
    self._icbm_cmd = None
    self._icbm_params = None
    if self._icbm_enabled:
      try:
        from openpilot.common.params import Params
        from opendbc.car.ford.icbm_pnw import PressGovernor
        self._icbm_params = Params("/dev/shm/params")
        self._icbm_governor = PressGovernor()
      except Exception:
        self._icbm_enabled = False

  def _icbm_buttons(self, CS) -> str | None:
    """Poll the brain's target at ~4 Hz, run the executor at 100 Hz. Returns 'dec'/'inc'/None."""
    import json
    import time
    from opendbc.car.ford.icbm_pnw import IcbmCommand, decide_press
    if (self.frame % 25) == 0:  # 4 Hz mem-param read
      try:
        raw = self._icbm_params.get("IcbmTarget")
        if isinstance(raw, (bytes, str)) and raw:
          raw = json.loads(raw)
        # params_pyx returns a dict for JSON keys; require all fields or stand down
        if isinstance(raw, dict) and all(k in raw for k in ("target", "ceiling", "ts")):
          self._icbm_cmd = IcbmCommand(target_ms=float(raw["target"]), ceiling_ms=float(raw["ceiling"]), ts=float(raw["ts"]))
        else:
          self._icbm_cmd = None
      except Exception:
        self._icbm_cmd = None
    driver_override = bool(CS.out.gasPressed or CS.out.brakePressed)
    intent = decide_press(float(CS.out.cruiseState.speed), self._icbm_cmd, time.time(),
                          bool(CS.out.cruiseState.enabled), driver_override)
    return self._icbm_governor.update(self.frame, intent)

  def update(self, CC, CS, now_nanos):
    can_sends = []

    # fordsafety2pnw: BluePilot updates SubMaster (modelV2/liveParameters/selfdriveState/radarState)
    # and the vehicle model every frame, before the lateral step
    if self._latext is not None:
      self._latext.update_sm()

    actuators = CC.actuators
    hud_control = CC.hudControl

    main_on = CS.out.cruiseState.available
    steer_alert = hud_control.visualAlert in (VisualAlert.steerRequired, VisualAlert.ldw)
    fcw_alert = hud_control.visualAlert == VisualAlert.fcw

    ### acc buttons ###
    if CC.cruiseControl.cancel:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, cancel=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, cancel=True))
    elif CC.cruiseControl.resume and (self.frame % CarControllerParams.BUTTONS_STEP) == 0:
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, resume=True))
      can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, resume=True))
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
        # DEC-ONLY by design (see icbm_pnw.py) — there is deliberately no set_inc send path here
        can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.camera, CS.buttons_stock_values, set_dec=True))
        can_sends.append(fordcan.create_button_msg(self.packer, self.CAN.main, CS.buttons_stock_values, set_dec=True))

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
      lat = self._latext.update(CC, CS, actuators, self.apply_curvature_last, self.CP)
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
      # TODO: look into using the actuators packet to send the desired speed
      can_sends.append(fordcan.create_acc_msg(self.packer, self.CAN, CC.longActive, gas, accel, stopping, self.brake_request, v_ego_kph=V_CRUISE_MAX))

      self.accel = accel
      self.gas = gas

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
      can_sends.append(fordcan.create_acc_ui_msg(self.packer, self.CAN, self.CP, main_on, CC.latActive,
                                                 fcw_alert, CS.out.cruiseState.standstill, show_distance_bars,
                                                 hud_control, CS.acc_tja_status_stock_values))

    self.main_on_last = main_on
    self.lkas_enabled_last = CC.latActive
    self.steer_alert_last = steer_alert
    self.lead_distance_bars_last = hud_control.leadDistanceBars

    new_actuators = actuators.as_builder()
    new_actuators.curvature = self.apply_curvature_last
    new_actuators.accel = self.accel
    new_actuators.gas = self.gas

    self.frame += 1
    return new_actuators, can_sends

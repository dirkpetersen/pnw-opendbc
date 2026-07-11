"""
fordlong2pnw — BluePilot Ford longitudinal follow control, ported from BluePilot (alan-polk),
bluepilotdev/bp-dev opendbc_repo/opendbc/sunnypilot/car/ford/longitudinal_ext.py.

Smoother highway following on top of stock op-long: classifies the lead (gaining / pacing /
trailing) and shapes gas/accel per state, rate-limits downward accel changes (dampens the initial
brake hit), and splits brake into brake + precharge with independent hysteresis for smoother decel.

Port adaptations (behavior-identical to BP defaults):
  - Composition instead of mixin: the pnw carcontroller owns an instance and passes the
    radarState-bearing SubMaster explicitly (BP shares it implicitly via multiple inheritance).
  - No CP_SP (sunnypilot CarParams extension does not exist in this tree).
  - No UI Params reads: BP's __init__ defaults are kept as constants — BP long ENABLED
    (disable_BP_long_UI=False) and downhill compensation DISABLED (disable_downhill_comp_UI=True,
    i.e. negative pitch clamped to 0 by the caller). Only reachable with op-long (Alpha Long)
    on a bp_long_follow-capable car, so the whole feature is inert until that opt-in.
  - Safety envelope unchanged: outputs are clipped to the same CarControllerParams
    ACCEL_MIN/ACCEL_MAX/MIN_GAS bounds the stock path uses; panda ford.h ACCDATA checks apply
    identically. Attribution: mechanism by alan-polk (BluePilot).

Key behaviors (all BP-faithful):
  - Speed deadband: engages above 50 mph, disengages below 45 mph (self.bpSpeedAllow latch)
  - Gaining lead within 1.5 s -> zero gas; pacing -> gas capped at 0.2+pitch; trailing -> stock
  - No lead -> only MILD decel suppressed (pnw divergence, see inline comment: genuine
    VTSC/curve/speed-drop braking passes through; BP zeroed all lead-less braking)
  - Downward accel rate limit 0.002/scan unless TTC < 8 s or gap < 0.5 s (emergency bypass)
  - Driver gas/brake press or lead slower than 40 mph -> stock values pass through
  - Mutual exclusion: brake_actuate forces INACTIVE_GAS
"""

from collections import namedtuple

import numpy as np

from opendbc.car.ford.values import CarControllerParams

LongitudinalResult = namedtuple('LongitudinalResult', [
  'accel',
  'gas',
  'brake_actuate',
  'precharge_actuate',
  'accel_pred_send',
  'stopping',
  'target_speed',
  'bp_long_used',
])


class LongitudinalExt:
  def __init__(self):
    self._bp_long_active_last = False
    self.bp_gas_last = 0.0
    self.bp_accel_last = 0.0
    self.bpSpeedAllow = False

    self.MAX_URBAN_SPEED_MPH = 45.0
    self.following_accel_ROC = 0.002  # max accel change per scan in following mode

    self.brake_actuate_target = -0.14   # engage brakes below this accel
    self.brake_actuate_release = -0.06  # release brakes above this accel
    self.precharge_actuate_target = -0.12
    self.precharge_actuate_release = -0.06
    self.op_brake_actuate_last = False
    self.bp_brake_actuate_last = False
    self.bp_precharge_actuate_last = False

    # BP __init__ defaults, frozen (no UI params in this tree — see module docstring)
    self.disable_BP_long_UI = False
    self.disable_downhill_comp_UI = True

  def update(self, CC, CS, sm, op_accel, op_gas, accel_due_to_pitch, v_ego_mph, stopping, target_speed):
    """Apply BP follow control on top of stock op_accel/op_gas (50 Hz, inside ACC_CONTROL_STEP).

    sm: the radarState-bearing SubMaster (owned by LateralCurvExt in the pnw carcontroller).
    All other args match BP: op_* are the stock values after creep compensation + rate limiting;
    accel_due_to_pitch must already have the downhill clamp applied by the caller.
    """
    # Op brake actuate hysteresis (replaces the stock brake_request when this path is active)
    accel_pitch_compensated = op_accel + accel_due_to_pitch
    op_brake_actuate = self.op_brake_actuate_last
    if accel_pitch_compensated > self.brake_actuate_release or not CC.longActive:
      op_brake_actuate = False
    elif accel_pitch_compensated < self.brake_actuate_target:
      op_brake_actuate = True

    # Speed deadband: engage above 50 mph, disallow below 45 mph
    if v_ego_mph > self.MAX_URBAN_SPEED_MPH + 5:
      self.bpSpeedAllow = True
    if v_ego_mph < self.MAX_URBAN_SPEED_MPH:
      self.bpSpeedAllow = False

    if not self.disable_BP_long_UI:
      v_ego = max(CS.out.vEgo, 0.5)
      lead_time_sec = 999.0
      lead = None
      v_rel = 0.0
      v_lead = 0.0

      if sm is not None and sm.valid['radarState']:
        rs = sm['radarState']
        lead = getattr(rs, 'leadOne', None)
        if lead is not None and getattr(lead, 'status', False) is not True:
          lead = None
        if lead:
          d_rel = float(getattr(lead, 'dRel', 0))
          v_rel = float(getattr(lead, 'vRel', 0))
          v_lead = float(getattr(lead, 'vLead', 0))
          if d_rel > 0:
            lead_time_sec = d_rel / v_ego

      lead_time_sec = float(np.clip(lead_time_sec, 0.0, 999.0))
      v_lead_mph = v_lead * 2.23694

      # Time to collision
      ttc_sec = 120.0
      if lead:
        d_rel = float(getattr(lead, 'dRel', 0))
        v_rel = float(getattr(lead, 'vRel', 0))
        if d_rel > 0 and v_rel < 0:
          ttc_sec = d_rel / (-v_rel)
        else:
          ttc_sec = 60.0
      ttc_sec = float(np.clip(ttc_sec, 0.2, 120.0))

      # Classify lead state
      gaining = False
      pacing = False
      trailing = False
      max_follow_gas = op_gas
      min_follow_gas = op_gas
      max_follow_accel = op_accel
      min_follow_accel = op_accel
      # pnw fix of a BP defect (Gemini finding): BP re-inits these False every scan, so inside
      # the hysteresis deadband (-0.14..-0.06) the brake bit CHATTERS instead of latching.
      # Latch across scans like the op_brake_actuate path does.
      bp_brake_actuate = self.bp_brake_actuate_last
      bp_precharge_actuate = self.bp_precharge_actuate_last

      if lead:
        if v_rel < -0.1:
          gaining = True
        elif v_rel > 0.1:
          trailing = True
        else:
          pacing = True

      if gaining:
        if lead_time_sec < 1.5:
          max_follow_gas = 0.0  # within 1.5 s and closing — no gas
          min_follow_gas = 0.0

      if pacing:
        max_follow_gas = 0.2 + accel_due_to_pitch  # cap gas when pacing
        min_follow_gas = 0.0

      if lead is None:
        # pnw divergence from BP (Gemini finding, 2026-07-11): BP zeroes accel entirely with no
        # lead (phantom-brake-pulse suppression on empty highway) — but OUR stack brakes lead-less
        # on purpose (VTSC/CES curve slowdowns, speed-limit drops). Suppress only MILD decel
        # (> -0.75 m/s^2 = the phantom-pulse band); genuine braking passes through untouched.
        if op_accel > -0.75:
          max_follow_accel = 0
          min_follow_accel = 0

      bp_gas = float(np.clip(op_gas, min_follow_gas, max_follow_gas))
      bp_accel = float(np.clip(op_accel, min_follow_accel, max_follow_accel))

      # Rate limit downward accel changes unless imminent-collision bypass.
      # pnw: FOLLOW-mode only (lead present) — in BP no-lead accel was always 0 so this never
      # mattered; with our mild-decel-only divergence, rate-limiting lead-less braking would slew
      # VTSC/curve decel at 0.1 m/s^2/s (unusable). No lead -> no follow ROC.
      if lead is not None and ttc_sec > 8.0 and lead_time_sec > 0.5:
        bp_accel = float(np.clip(bp_accel, self.bp_accel_last - self.following_accel_ROC, 999))

      # BP brake/precharge hysteresis
      if bp_accel < self.brake_actuate_target:
        bp_brake_actuate = True
      if bp_accel > self.brake_actuate_release:
        bp_brake_actuate = False
      if bp_accel < self.precharge_actuate_target:
        bp_precharge_actuate = True
      if bp_accel > self.precharge_actuate_release:
        bp_precharge_actuate = False

      apply_bp_long = (self.bpSpeedAllow and
                       not CS.out.gasPressed and not CS.out.brakePressed and
                       (lead is None or v_lead_mph > 40.0))

      if apply_bp_long and CC.longActive:
        accel = bp_accel
        gas = bp_gas
        brake_actuate = bp_brake_actuate
        precharge_actuate = bp_precharge_actuate
      else:
        accel = op_accel
        gas = op_gas
        brake_actuate = op_brake_actuate
        precharge_actuate = op_brake_actuate

      self.bp_gas_last = bp_gas
      self.bp_accel_last = bp_accel
      self.bp_brake_actuate_last = bp_brake_actuate
      self.bp_precharge_actuate_last = bp_precharge_actuate
      bp_long_used = apply_bp_long
    else:
      accel = op_accel
      gas = op_gas
      brake_actuate = op_brake_actuate
      precharge_actuate = op_brake_actuate
      bp_long_used = False

    # Mutual exclusion: no brake and gas at the same time
    if brake_actuate:
      gas = CarControllerParams.INACTIVE_GAS

    # Clip to the same bounds the stock path uses (panda ford.h ACCDATA limits apply on top)
    accel = float(np.clip(accel, CarControllerParams.ACCEL_MIN, CarControllerParams.ACCEL_MAX))
    if gas != CarControllerParams.INACTIVE_GAS:
      gas = float(np.clip(gas, CarControllerParams.MIN_GAS, CarControllerParams.ACCEL_MAX))
    accel_pred_send = CarControllerParams.INACTIVE_GAS

    self._bp_long_active_last = bp_long_used
    self.op_brake_actuate_last = op_brake_actuate

    return LongitudinalResult(
      accel=accel,
      gas=gas,
      brake_actuate=bool(brake_actuate),
      precharge_actuate=bool(precharge_actuate),
      accel_pred_send=float(accel_pred_send),
      stopping=bool(stopping),
      target_speed=float(target_speed),
      bp_long_used=bool(bp_long_used),
    )

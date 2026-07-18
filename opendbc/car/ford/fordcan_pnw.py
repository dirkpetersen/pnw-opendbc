"""
fordsafety2pnw — Ford CAN message builder extensions, ported from BluePilot (alan-polk),
bluepilotdev/bp-dev opendbc_repo/opendbc/sunnypilot/car/ford/fordcan_ext.py.

Extended versions of the stock fordcan.py lateral builders with dynamic ramp_type and
precision_type (stock hardcodes ramp_type=0 Slow, precision_type=1 Precise), used by the
4-signal LateralCurvExt path. create_acc_ui_msg is ported for fidelity but NOT wired into
the pnw carcontroller (it needs BP's DM-state/BlueCruise inputs); the stock acc_ui path is
unchanged.

Only the builders the 4-signal port needs are ported — BP's acc/button/lkas_ui extensions
belong to its LongitudinalExt/HudExt/ICBM features, which live elsewhere in this tree or are
not ported.
"""

from opendbc.car.ford.fordcan import CanBus, calculate_lat_ctl2_checksum

# angle2pnw: shadow_curvature packing scale, matches ford.h's FORD_BP_SHADOW_CURVATURE_TO_CAN decode.
_BP_LKA_SHADOW_CURVATURE_SCALE = 1e-6  # 1/meter per raw unit


def create_lka_msg(packer, CAN: CanBus, lat_active: bool, hud_control,
                   angle_mode_engaged: bool = False, shadow_curvature: float = 0.0):
  """
  Creates a CAN message for the Ford LKA Command (Lane_Assist_Data1), ported from BluePilot
  (alan-polk), bluepilotdev/bp-7.0 opendbc_repo/opendbc/sunnypilot/car/ford/fordcan_ext.py.

  angle2pnw: also carries angle_mode_engaged + shadow_curvature packed into bits with no DBC
  signal mapped to them (confirmed unused -- always 0, no cabana signal -- on real F-150 dashcam
  routes per BluePilot's own investigation and FORDSAFETY2PNW's prior 0x5F0 dead-end). This
  message is one openpilot itself originates every cycle, and ford_tx_hook already reads other
  fields (LkaActvStats_D2_Req) directly out of these same bytes synchronously in the same
  tx_hook call -- no separate CAN ID, no RX round-trip (panda does not self-receive its own TX).

  With angle_mode_engaged=False and shadow_curvature=0.0 (the defaults, and the ONLY values ever
  passed while PnwVehicle.angle_lat is off) this produces byte-identical output to stock
  fordcan.create_lka_msg — `dat[4] |= 0` is a no-op and dat[5]/dat[6] are already 0 from the same
  underlying empty-values packer.make_can_msg call stock uses.

  Byte layout (bits not covered by any Lane_Assist_Data1 DBC signal):
    byte 4 bit 0:     angle_mode_engaged
    byte 4 bits 1-4:  reserved (future bools)
    byte 5-6:         shadow_curvature (int16, scale 1e-6 1/m)
    byte 7:           reserved (future value)
  Must match the decode in ford.h's FORD_Lane_Assist_Data1 tx_hook check exactly.

  Frequency is 33Hz.
  """
  addr, dat, bus = packer.make_can_msg("Lane_Assist_Data1", CAN.main, {})
  dat = bytearray(dat)

  shadow_curvature_raw = int(round(shadow_curvature / _BP_LKA_SHADOW_CURVATURE_SCALE))
  shadow_curvature_raw = max(-32768, min(32767, shadow_curvature_raw)) & 0xFFFF

  dat[4] |= 1 if angle_mode_engaged else 0
  dat[5] = (shadow_curvature_raw >> 8) & 0xFF
  dat[6] = shadow_curvature_raw & 0xFF

  return addr, bytes(dat), bus


def create_lat_ctl_msg(packer, CAN: CanBus, lat_active: bool, ramp_type: int, precision_type: int,
                       path_offset: float, path_angle: float, curvature: float, curvature_rate: float):
  """
  Creates a CAN message for the Ford TJA/LCA Command (non-CAN FD).

  BluePilot extension: dynamic ramp_type and precision_type parameters.
  Stock hardcodes ramp_type=0 (Slow) and precision_type=1 (Precise).

  Ford lane centering uses a third-order polynomial to describe the road centerline:
    c0 (path_offset): lateral offset between vehicle and centerline (positive is right)
    c1 (path_angle): heading angle between vehicle and centerline (positive is right)
    c2 (curvature): curvature of the centerline (positive is left)
    c3 (curvature_rate): rate of change of curvature

  Frequency is 20Hz.
  """
  values = {
    "LatCtlRng_L_Max": 0,                       # Unknown [0|126] meter
    "HandsOffCnfm_B_Rq": 0,                     # Unknown: 0=Inactive, 1=Active [0|1]
    "LatCtl_D_Rq": 1 if lat_active else 0,      # Mode: 0=None, 1=ContinuousPathFollowing, 2=InterventionLeft,
                                                 #       3=InterventionRight, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # Ramp speed: 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
    "LatCtlPrecision_D_Rq": precision_type,     # Precision: 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
    "LatCtlPathOffst_L_Actl": path_offset,      # Path offset [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # Path angle [-0.5|0.5235] radians
    "LatCtlCurv_NoRate_Actl": curvature_rate,   # Curvature rate [-0.001024|0.00102375] 1/meter^2
    "LatCtlCurv_No_Actl": curvature,            # Curvature [-0.02|0.02094] 1/meter
  }
  return packer.make_can_msg("LateralMotionControl", CAN.main, values)


def create_lat_ctl2_msg(packer, CAN: CanBus, mode: int, ramp_type: int, precision_type: int,
                        path_offset: float, path_angle: float, curvature: float,
                        curvature_rate: float, counter: int):
  """
  Creates a CAN message for the Ford Lane Centering command (CAN FD).

  BluePilot extension: dynamic ramp_type and precision_type parameters.
  Stock hardcodes ramp_type=0 (Slow) and precision_type=1 (Precise).

  This message replaces LateralMotionControl on CAN FD platforms and includes
  counter and checksum fields.

  Frequency is 20Hz.
  """
  values = {
    "LatCtl_D2_Rq": mode,                       # Mode: 0=None, 1=PathFollowingLimitedMode, 2=PathFollowingExtendedMode,
                                                 #       3=SafeRampOut, 4-7=NotUsed [0|7]
    "LatCtlRampType_D_Rq": ramp_type,           # 0=Slow, 1=Medium, 2=Fast, 3=Immediate [0|3]
    "LatCtlPrecision_D_Rq": precision_type,     # 0=Comfortable, 1=Precise, 2/3=NotUsed [0|3]
    "LatCtlPathOffst_L_Actl": path_offset,      # [-5.12|5.11] meter
    "LatCtlPath_An_Actl": path_angle,           # [-0.5|0.5235] radians
    "LatCtlCurv_No_Actl": curvature,            # [-0.02|0.02094] 1/meter
    "LatCtlCrv_NoRate2_Actl": curvature_rate,   # [-0.001024|0.001023] 1/meter^2
    "HandsOffCnfm_B_Rq": 0,                     # 0=Inactive, 1=Active [0|1]
    "LatCtlPath_No_Cnt": counter,               # [0|15]
    "LatCtlPath_No_Cs": 0,                      # [0|255]
  }

  # Calculate checksum (reuse stock function)
  dat = packer.make_can_msg("LateralMotionControl2", 0, values)[1]
  values["LatCtlPath_No_Cs"] = calculate_lat_ctl2_checksum(mode, counter, dat)

  return packer.make_can_msg("LateralMotionControl2", CAN.main, values)


# fordlat2pnw/hud (BluePilot alan-polk compute_dm_msg_values + get_dm_state, ported 1:1) — maps
# selfdriveState.alertType to the Ford cluster's TJA warning/message signals so BlueCruise-mode
# alerts ("Resume Control", "Cancelled", hands-on prompts, lane-departure) render in the cluster.
# Display-only (ACCDATA_3 0x18A, TX-allowlisted). Returns (tja_msg, tja_warn, hands).
def get_dm_state(d_state, main_on):
  e = str(d_state or "").split("/")
  if main_on:
    return e[0], e[-1]
  return "none", "none"


def compute_dm_msg_values(alert_type, hud_control, send_hands_free_cluster_msg, main, standstill=False):
  tja_msg = 0
  tja_warn = 0
  hands = 0
  driverState, disableState = get_dm_state(alert_type, main)

  if send_hands_free_cluster_msg:
    if disableState == "noEntry":
      tja_msg = 1
    elif (driverState in ("driverDistracted", "driverUnresponsive") or
          disableState in ("softDisable", "immediateDisable")):
      tja_warn = 3   # Resume Control
    elif disableState == "userDisable":
      tja_warn = 1   # Cancelled
    elif driverState == "preDriverDistracted":
      hands = 1
    elif driverState == "promptDriverDistracted":
      hands = 2 if not standstill else 1
    elif driverState == "preDriverUnresponsive":
      hands = 1
    elif driverState == "promptDriverUnresponsive":
      hands = 2 if not standstill else 1
    elif hud_control.leftLaneDepart:
      tja_warn = 5
    elif hud_control.rightLaneDepart:
      tja_warn = 4
  else:
    if disableState == "noEntry":
      tja_msg = 1
    elif (driverState in ("driverDistracted", "driverUnresponsive") or
          disableState in ("softDisable", "immediateDisable")):
      tja_warn = 3
    elif disableState == "userDisable":
      tja_warn = 1
    elif driverState in ("preDriverDistracted", "preDriverUnresponsive"):
      hands = 1
    elif driverState in ("promptDriverDistracted", "promptDriverUnresponsive"):
      hands = 2 if not standstill else 1

  return tja_msg, tja_warn, hands


def create_acc_ui_msg(packer, CAN: CanBus, CP, main_on: bool, enabled: bool, fcw_alert: bool,
                      standstill: bool, hud_control, stock_values: dict, send_hands_free_msg: bool,
                      send_ui: bool, send_bars: bool, tja_warn: int, tja_msg: int):
  """
  Creates a CAN message for the Ford IPC adaptive cruise, FCW and TJA status.

  BluePilot extension: replaces stock show_distance_bars with explicit send_ui,
  send_bars, and TJA parameters. Adds BlueCruise status 7 for hands-free cluster
  UI. TJA warn/msg are set from DM state computation rather than stock passthrough.

  pnw: ported for fidelity, NOT wired in — the pnw carcontroller keeps the stock
  fordcan.create_acc_ui_msg path (no BP DM-state/BlueCruise inputs in this tree).

  Frequency is 5Hz.
  """

  # Tja_D_Stat: TJA status for cluster display
  if enabled:
    if hud_control.leftLaneDepart:
      status = 3  # ActiveInterventionLeft
    elif hud_control.rightLaneDepart:
      status = 4  # ActiveInterventionRight
    elif send_hands_free_msg:
      status = 7  # BlueCruise UI in the cluster
    else:
      status = 2  # Active
  elif main_on:
    if hud_control.leftLaneDepart:
      status = 5  # ActiveWarningLeft
    elif hud_control.rightLaneDepart:
      status = 6  # ActiveWarningRight
    else:
      status = 1  # Standby
  elif standstill:
    status = 0  # Off
  else:
    status = 1  # Standby

  values = {s: stock_values[s] for s in [
    "HaDsply_No_Cs",
    "HaDsply_No_Cnt",
    "AccStopStat_D_Dsply",       # ACC stopped status message
    "AccTrgDist2_D_Dsply",       # ACC target distance
    "AccStopRes_B_Dsply",
    # TjaWarn_D_Rq and TjaMsgTxt_D_Dsply are set explicitly below, not passed through
    "IaccLamp_D_Rq",             # iACC status icon
    "AccMsgTxt_D2_Rq",           # ACC text
    "FcwDeny_B_Dsply",           # FCW disabled
    "FcwMemStat_B_Actl",         # FCW enabled setting
    "AccTGap_B_Dsply",           # ACC time gap display setting
    "CadsAlignIncplt_B_Actl",
    "AccFllwMde_B_Dsply",        # ACC follow mode display setting
    "CadsRadrBlck_B_Actl",
    "CmbbPostEvnt_B_Dsply",      # AEB event status
    "AccStopMde_B_Dsply",        # ACC stop mode display setting
    "FcwMemSens_D_Actl",         # FCW sensitivity setting
    "FcwMsgTxt_D_Rq",            # FCW text
    "AccWarn_D_Dsply",           # ACC warning
    "FcwVisblWarn_B_Rq",         # FCW visible alert
    "FcwAudioWarn_B_Rq",         # FCW audio alert
    "AccTGap_D_Dsply",           # ACC time gap
    "AccMemEnbl_B_RqDrv",        # ACC adaptive/normal setting
    "FdaMem_B_Stat",             # FDA enabled setting
  ]}

  values.update({
    "Tja_D_Stat": status,         # TJA status
    "TjaWarn_D_Rq": tja_warn,    # TJA warning (from DM state, not stock passthrough)
    "TjaMsgTxt_D_Dsply": tja_msg, # TJA text (from DM state, not stock passthrough)
  })

  if CP.openpilotLongitudinalControl:
    values.update({
      "AccStopStat_D_Dsply": 2 if standstill else 0,              # Stopping status text
      "AccMsgTxt_D2_Rq": 0,                                       # ACC text
      "AccTGap_B_Dsply": 1 if send_bars else 0,                   # Show time gap control UI
      "AccFllwMde_B_Dsply": 1 if hud_control.leadVisible else 0,  # Lead indicator
      "AccStopMde_B_Dsply": 1 if standstill else 0,
      "AccWarn_D_Dsply": 0,                                        # ACC warning
      "AccTGap_D_Dsply": hud_control.leadDistanceBars,            # Time gap
    })

  # Forward FCW alert from IPMA
  if fcw_alert:
    values["FcwVisblWarn_B_Rq"] = 1  # FCW visible alert
    values["FcwAudioWarn_B_Rq"] = 1  # FCW audio alert

  return packer.make_can_msg("ACCDATA_3", CAN.main, values)


def create_acc_msg(packer, CAN: CanBus, long_active: bool, gas: float, accel: float, accel_pred: float,
                   stopping: bool, brake_actuate: bool, precharge_actuate: bool, v_ego_kph: float):
  """
  Creates a CAN message for the Ford ACC Command (BluePilot extension, ported 1:1).

  vs stock create_acc_msg: brake control split into brake_actuate and precharge_actuate
  (independent hysteresis, precharge engages slightly before full brake for smoother initial
  decel) and accel_pred passed in instead of the stock hardcoded -5.0.

  Frequency is 50Hz.
  """
  values = {
    "AccBrkTot_A_Rq": accel,                           # Brake total accel request: [-20|11.9449] m/s^2
    "Cmbb_B_Enbl": 1 if long_active else 0,            # Enabled: 0=No, 1=Yes
    "AccPrpl_A_Rq": gas,                               # Acceleration request: [-5|5.23] m/s^2
    "AccPrpl_A_Pred": accel_pred,                      # Predicted accel (parameter, not hardcoded)
    "AccResumEnbl_B_Rq": 1 if long_active else 0,
    "AccVeh_V_Trg": v_ego_kph,                         # Target speed: [0|255] km/h
    "AccBrkPrchg_B_Rq": 1 if precharge_actuate else 0, # Pre-charge brake request
    "AccBrkDecel_B_Rq": 1 if brake_actuate else 0,     # Deceleration request
    "AccStopStat_B_Rq": 1 if stopping else 0,
  }
  return packer.make_can_msg("ACCDATA", CAN.main, values)

import copy
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, structs
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.interfaces import CarStateBase
from opendbc.car.tesla.teslacan import get_steer_ctrl_type
from opendbc.car.tesla.values import DBC, CANBUS, GEAR_MAP, STEER_THRESHOLD, TeslaFlags, TeslaLegacyParams, CAR, LEGACY_CARS

ButtonType = structs.CarState.ButtonEvent.Type


# steerdebounce2pnw: separate the EPS's engage-handover artifact from a real steering inhibit.
#
# The Tesla EPS reports EAC_INHIBITED for a few frames while it hands the rack over from
# EAC_AVAILABLE to EAC_ACTIVE at engage. carstate mapped ANY EAC_INHIBITED to steerFaultTemporary,
# which is a SOFT_DISABLE event -- so that artifact produced a "Steering Temporarily Unavailable"
# alert and an enabled->softDisabling->enabled flip at engage.
#
# The discriminator is the ERROR CODE, not the duration. Surveyed over ~18,600 EPAS frames across
# four segments (2026-09-03, archived under drives/2026-09-03/hotspot-drive-tesla/, decoder
# eac_runs.py): the only inhibited run near an engage was 4 frames carrying EAC_ERROR_IDLE -- the
# EPS saying "inhibited, no reason". Every real inhibit observed (2026-08-31 seg58, driver override)
# ran 19-20 frames and carried EAC_ERROR_HANDS_ON.
#
# So: a CODED inhibit is a real fault and is reported INSTANTLY, with no debounce and no added
# latency -- strictly better than delaying every inhibit as the first cut did. Only the UNCODED
# (IDLE) class, which is the artifact class, is filtered.
#
# The uncoded filter is a leaky counter rather than a consecutive count, on review: a consecutive
# count that resets on one clean frame means an uncoded inhibit flapping faster than the threshold
# is NEVER reported, however long it lasts. Rising 3 / decaying 1 makes anything above ~25% duty
# ratchet up instead of hiding, and holds the fault ~300 ms after the last inhibit rather than
# clearing on the first healthy sample (fail-safe for a fault flag is to hold, not to clear).
# 10 consecutive uncoded frames still reports at frame 10, i.e. ~100 ms.
EAC_IDLE_RISE = 3            # per uncoded-inhibit tick
EAC_IDLE_DECAY = 1           # per healthy tick
EAC_IDLE_REPORT = 30         # -> 10 consecutive uncoded ticks (~100 ms at the 100 Hz carState rate)
EAC_IDLE_CAP = 60            # bounds the hold to ~300 ms after the last uncoded inhibit


def next_steer_fault_temporary(count: int, inhibited: bool, err_idle: bool) -> tuple[bool, int]:
  """(report_fault, new_count) for steerFaultTemporary.

  `inhibited` = eacStatus is EAC_INHIBITED. `err_idle` = eacErrorCode is EAC_ERROR_IDLE (no reason
  given). A coded inhibit reports immediately and does not touch the counter.
  """
  if inhibited and not err_idle:
    return True, 0
  count = min(count + EAC_IDLE_RISE, EAC_IDLE_CAP) if inhibited else max(count - EAC_IDLE_DECAY, 0)
  return count >= EAC_IDLE_REPORT, count


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    self.can_define = CANDefine(DBC[CP.carFingerprint][Bus.party])

    if self.CP.carFingerprint in LEGACY_CARS:
      if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
        CANBUS.chassis = 1
        CANBUS.radar = 5
      elif self.CP.carFingerprint in (CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_X_HW1, ):
        CANBUS.powertrain = CANBUS.party
        CANBUS.autopilot_powertrain = CANBUS.autopilot_party

      self.can_define_party = CANDefine(DBC[CP.carFingerprint][Bus.party])
      self.can_define_pt = CANDefine(DBC[CP.carFingerprint][Bus.pt])
      self.can_define_chassis = CANDefine(DBC[CP.carFingerprint][Bus.chassis])
      self.can_defines = {
        **self.can_define_party.dv,
        **self.can_define_pt.dv,
        **self.can_define_chassis.dv,
      }
      self.shifter_values = self.can_defines["DI_torque2"]["DI_gear"]
    else:
      self.shifter_values = self.can_define.dv["DI_systemStatus"]["DI_gear"]

    self.autopark = False
    self.autopark_prev = False
    self.cruise_enabled_prev = False
    self.fsd14_error_logged = False
    self.suspected_fsd14 = False

    self.hands_on_level = 0
    self.eac_idle_cnt = 0        # steerdebounce2pnw: leaky counter for UNCODED inhibits only
    self.das_control = None

  def update_autopark_state(self, autopark_state: str, cruise_enabled: bool):
    autopark_now = autopark_state in ("ACTIVE", "COMPLETE", "SELFPARK_STARTED")
    if autopark_now and not self.autopark_prev and not self.cruise_enabled_prev:
      self.autopark = True
    if not autopark_now:
      self.autopark = False
    self.autopark_prev = autopark_now
    self.cruise_enabled_prev = cruise_enabled

  def update(self, can_parsers) -> structs.CarState:
    if self.CP.carFingerprint in LEGACY_CARS:
      return self.update_legacy(can_parsers)

    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_party.vl["DI_speed"]["DI_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_party.vl["DI_systemStatus"]["DI_accelPedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_party.vl["ESP_status"]["ESP_driverBrakeApply"] == 2

    # Steering wheel
    epas_status = cp_party.vl["EPAS3S_sysStatus"]
    self.hands_on_level = epas_status["EPAS3S_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS3S_internalSAS"]
    ret.steeringRateDeg = -cp_ap_party.vl["SCCM_steeringAngleSensor"]["SCCM_steeringAngleSpeed"]
    ret.steeringTorque = -epas_status["EPAS3S_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacStatus"].get(int(epas_status["EPAS3S_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    eac_error_code = self.can_define.dv["EPAS3S_sysStatus"]["EPAS3S_eacErrorCode"].get(int(epas_status["EPAS3S_eacErrorCode"]), None)
    # steerdebounce2pnw: a CODED inhibit is a real fault -> reported instantly, no added latency.
    # Only the UNCODED (EAC_ERROR_IDLE) class -- the engage-handover artifact -- is filtered.
    # steerFaultPermanent (EAC_FAULT) and steeringDisengage below stay instant: the first is a real
    # rack fault, the second is the panda-mirrored hard override. Neither may be delayed.
    ret.steerFaultTemporary, self.eac_idle_cnt = next_steer_fault_temporary(
      self.eac_idle_cnt, eac_status == "EAC_INHIBITED", eac_error_code == "EAC_ERROR_IDLE")

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_define.dv["DI_state"]["DI_cruiseState"].get(int(cp_party.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_define.dv["DI_state"]["DI_speedUnits"].get(int(cp_party.vl["DI_state"]["DI_speedUnits"]), None)

    autopark_state = self.can_define.dv["DI_state"]["DI_autoparkState"].get(int(cp_party.vl["DI_state"]["DI_autoparkState"]), None)
    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")
    self.update_autopark_state(autopark_state, cruise_enabled)

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled and not self.autopark
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_party.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cp_party.vl["ESP_B"]["ESP_vehicleStandstillSts"] == 1
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_define.dv["DI_systemStatus"]["DI_gear"].get(int(cp_party.vl["DI_systemStatus"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    ret.doorOpen = cp_party.vl["UI_warning"]["anyDoorOpen"] == 1

    # Blinkers
    ret.leftBlinker = cp_party.vl["UI_warning"]["leftBlinkerBlinking"] in (1, 2)
    ret.rightBlinker = cp_party.vl["UI_warning"]["rightBlinkerBlinking"] in (1, 2)

    # Seatbelt
    ret.seatbeltUnlatched = cp_party.vl["UI_warning"]["buckleStatus"] != 1

    # Blindspot
    ret.leftBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearLeft"] != 0
    ret.rightBlindspot = cp_ap_party.vl["DAS_status"]["DAS_blindSpotRearRight"] != 0

    # AEB
    ret.stockAeb = cp_ap_party.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    # On FSD 14+, ANGLE_CONTROL behavior changed to allow user winddown while actuating.
    # FSD switched from using ANGLE_CONTROL to LANE_KEEP_ASSIST to likely keep the old steering override disengage logic.
    # LKAS switched from LANE_KEEP_ASSIST to ANGLE_CONTROL to likely allow overriding LKAS events smoothly
    lkas_ctrl_type = get_steer_ctrl_type(self.CP.flags, 2)
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == lkas_ctrl_type  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # TODO: find for TESLA_MODEL_X and HW2.5 vehicles
    if not (self.CP.flags & TeslaFlags.MISSING_DAS_SETTINGS):
      ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

      # Because we don't have FSD 14 detection outside of a set of FW, we should check if this FW is accidentally missing from FSD_14_FW
      # 1. If in Autosteer or FSD, already caught by invalidLkasSetting
      # 2. If in TACC and DAS ever sends ANGLE_CONTROL (1), we can infer it's trying to do LKAS on FSD 14+
      angle_control = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 1  # ANGLE_CONTROL
      if not ret.invalidLkasSetting and angle_control and not self.CP.flags & TeslaFlags.FSD_14:
        self.suspected_fsd14 = True

      if self.suspected_fsd14:
        ret.invalidLkasSetting = True
        if not self.fsd14_error_logged:
          carlog.error("FSD 14 detected, but FW not in FSD_14_FW set")
          self.fsd14_error_logged = True

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_party.vl["DAS_control"])

    return ret

  def update_legacy(self, can_parsers) -> structs.CarState:
    cp_party = can_parsers[Bus.party]
    cp_ap_party = can_parsers[Bus.ap_party]
    cp_pt = can_parsers[Bus.pt]
    cp_ap_pt = can_parsers[Bus.ap_pt]
    cp_chassis = can_parsers[Bus.chassis]
    ret = structs.CarState()

    # Vehicle speed
    ret.vEgoRaw = cp_chassis.vl["ESP_B"]["ESP_vehicleSpeed"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)

    # Gas pedal
    ret.gasPressed = cp_pt.vl["DI_torque1"]["DI_pedalPos"] > 0

    # Brake pedal
    ret.brake = 0
    ret.brakePressed = cp_chassis.vl["BrakeMessage"]["driverBrakeStatus"] == 2

    # Steering wheel
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      epas_status = cp_party.vl["EPAS_sysStatus"]
    else:
      epas_status = cp_chassis.vl["EPAS_sysStatus"]
    self.hands_on_level = epas_status["EPAS_handsOnLevel"]
    ret.steeringAngleDeg = -epas_status["EPAS_internalSAS"]
    ret.steeringRateDeg = -cp_chassis.vl["STW_ANGLHP_STAT"]["StW_AnglHP_Spd"]
    ret.steeringTorque = -epas_status["EPAS_torsionBarTorque"]

    # stock handsOnLevel uses >0.5 for 0.25s, but is too slow
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > STEER_THRESHOLD, 5)

    eac_status = self.can_defines["EPAS_sysStatus"]["EPAS_eacStatus"].get(int(epas_status["EPAS_eacStatus"]), None)
    ret.steerFaultPermanent = eac_status == "EAC_FAULT"
    eac_error_code = self.can_defines["EPAS_sysStatus"]["EPAS_eacErrorCode"].get(int(epas_status["EPAS_eacErrorCode"]), None)
    # steerdebounce2pnw: a CODED inhibit is a real fault -> reported instantly, no added latency.
    # Only the UNCODED (EAC_ERROR_IDLE) class -- the engage-handover artifact -- is filtered.
    # steerFaultPermanent (EAC_FAULT) and steeringDisengage below stay instant: the first is a real
    # rack fault, the second is the panda-mirrored hard override. Neither may be delayed.
    ret.steerFaultTemporary, self.eac_idle_cnt = next_steer_fault_temporary(
      self.eac_idle_cnt, eac_status == "EAC_INHIBITED", eac_error_code == "EAC_ERROR_IDLE")

    # FSD disengages using union of handsOnLevel (slow overrides) and high angle rate faults (fast overrides, high speed)
    ret.steeringDisengage = self.hands_on_level >= 3 or (eac_status == "EAC_INHIBITED" and
                                                         eac_error_code == "EAC_ERROR_HIGH_ANGLE_RATE_SAFETY")

    # Cruise state
    cruise_state = self.can_defines["DI_state"]["DI_cruiseState"].get(int(cp_chassis.vl["DI_state"]["DI_cruiseState"]), None)
    speed_units = self.can_defines["DI_state"]["DI_speedUnits"].get(int(cp_chassis.vl["DI_state"]["DI_speedUnits"]), None)

    cruise_enabled = cruise_state in ("ENABLED", "STANDSTILL", "OVERRIDE", "PRE_FAULT", "PRE_CANCEL")

    # Match panda safety cruise engaged logic
    ret.cruiseState.enabled = cruise_enabled
    if speed_units == "KPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.KPH_TO_MS, 1e-3)
    elif speed_units == "MPH":
      ret.cruiseState.speed = max(cp_chassis.vl["DI_state"]["DI_digitalSpeed"] * CV.MPH_TO_MS, 1e-3)
    ret.cruiseState.available = cruise_state == "STANDBY" or ret.cruiseState.enabled
    ret.cruiseState.standstill = False  # This needs to be false, since we can resume from stop without sending anything special
    ret.standstill = cruise_state == "STANDSTILL"
    ret.accFaulted = cruise_state == "FAULT"

    # Gear
    ret.gearShifter = GEAR_MAP[self.can_defines["DI_torque2"]["DI_gear"].get(int(cp_chassis.vl["DI_torque2"]["DI_gear"]), "DI_GEAR_INVALID")]

    # Doors
    DOORS = ["DOOR_STATE_FL", "DOOR_STATE_FR", "DOOR_STATE_RL", "DOOR_STATE_RR", "DOOR_STATE_FrontTrunk", "BOOT_STATE"]
    ret.doorOpen = any((self.can_defines["GTW_carState"][door].get(int(cp_chassis.vl["GTW_carState"][door]), "OPEN") == "OPEN") for door in DOORS)

    # Blinkers
    if self.CP.carFingerprint == CAR.TESLA_MODEL_X_HW1:
      ret.leftBlinker = cp_chassis.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] == 1
      ret.rightBlinker = cp_chassis.vl["STW_ACTN_RQ"]["TurnIndLvr_Stat"] == 2
    else:
      ret.leftBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorLStatus"] == 1
      ret.rightBlinker = cp_chassis.vl["GTW_carState"]["BC_indicatorRStatus"] == 1

    # Seatbelt
    if self.CP.flags & TeslaLegacyParams.NO_SDM1:
      ret.seatbeltUnlatched = cp_chassis.vl["RCM_status"]["RCM_buckleDriverStatus"] != 1
    else:
      ret.seatbeltUnlatched = cp_chassis.vl["SDM1"]["SDM_bcklDrivStatus"] != 1

    # Blindspot (bsm2pnw) — Tesla's own rear blind-spot warning (computed by the Autopilot computer
    # from the rear repeater cameras + ultrasonics), broadcast as AutopilotStatus (0x399) on the
    # chassis bus. 2-bit fields: 0=NO_WARNING, 1/2=WARNING, 3=SNA. Only an actual warning counts as
    # occupied; 0/SNA read as clear, so behaviour degrades to the pre-BSM baseline if the car stops
    # emitting warnings. The message is EXPLICITLY subscribed in get_can_parsers (HW3 only), so a
    # total loss of 0x399 surfaces as a CAN error instead of a silent always-clear — a silent clear
    # would defeat the DesireHelper lane-change gate while looking protected.
    if self.CP.carFingerprint == CAR.TESLA_MODEL_S_HW3:
      bsm = cp_chassis.vl["AutopilotStatus"]
      ret.leftBlindspot = bsm["DAS_blindSpotRearLeft"] in (1, 2)
      ret.rightBlindspot = bsm["DAS_blindSpotRearRight"] in (1, 2)

    # AEB
    ret.stockAeb = cp_ap_pt.vl["DAS_control"]["DAS_aebEvent"] == 1

    # LKAS
    ret.stockLkas = cp_ap_party.vl["DAS_steeringControl"]["DAS_steeringControlType"] == 2  # LANE_KEEP_ASSIST

    # Stock Autosteer should be off (includes FSD)
    # ret.invalidLkasSetting = cp_ap_party.vl["DAS_settings"]["DAS_autosteerEnabled"] != 0

    # Buttons # ToDo: add Gap adjust button

    # Messages needed by carcontroller
    self.das_control = copy.copy(cp_ap_pt.vl["DAS_control"])

    return ret

  @staticmethod
  def get_can_parsers(CP):
    if CP.carFingerprint in LEGACY_CARS:
      return {
        Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
        Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party),
        Bus.pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.powertrain),
        Bus.ap_pt: CANParser(DBC[CP.carFingerprint][Bus.pt], [], CANBUS.autopilot_powertrain),
        # bsm2pnw: explicitly subscribe AutopilotStatus (0x399, blind-spot) on the HW3 Raven so its
        # absence trips can_valid (CAN error) instead of silently reading as "no blindspot". freq=0
        # keeps the parser's learned-frequency timeout (same as every other legacy message).
        Bus.chassis: CANParser(DBC[CP.carFingerprint][Bus.chassis],
                               [("AutopilotStatus", 0)] if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else [],
                               CANBUS.chassis if CP.carFingerprint == CAR.TESLA_MODEL_S_HW3 else CANBUS.party),
      }

    return {
      Bus.party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.party),
      Bus.ap_party: CANParser(DBC[CP.carFingerprint][Bus.party], [], CANBUS.autopilot_party)
    }

import time
from opendbc.can import CANDefine, CANParser
from opendbc.car import Bus, create_button_events, structs
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford.fordcan import CanBus
from opendbc.car.ford.values import DBC, CarControllerParams, FordFlags
from opendbc.car.carlog import carlog
from opendbc.car.interfaces import CarStateBase

ButtonType = structs.CarState.ButtonEvent.Type
GearShifter = structs.CarState.GearShifter
TransmissionType = structs.CarParams.TransmissionType


class CarState(CarStateBase):
  def __init__(self, CP):
    super().__init__(CP)
    can_define = CANDefine(DBC[CP.carFingerprint][Bus.pt])
    if CP.transmissionType == TransmissionType.automatic:
      self.shifter_values = can_define.dv["PowertrainData_10"]["TrnRng_D_Rq"]

    self.distance_button = 0
    self.lc_button = 0
    # onebutton2pnw: the ACC master ON/OFF button on the wheel. Surfaced as a mainCruise button
    # event because openpilot cannot infer the press from CcStat_D_Actl alone -- see the rising-edge
    # comment at its use site below.
    self.main_button = 0
    # onebutton2pnw: RES / SET+ / SET- presses. Ford emitted NO cruise ButtonTypes at all before
    # this (only gapAdjustCruise and lkas), so anything upstream or downstream that keys on
    # accelCruise/decelCruise/resumeCruise silently never fired on this brand -- including the
    # off-request latch's "the driver changed their mind" clear, which was dead code until now.
    self.res_button = 0
    self.set_inc_button = 0
    self.set_dec_button = 0

    # cargps2pnw: /dev/shm handle for publishing the truck's own GPS fix, plus a decimator. Same
    # pattern as fordlatui2pnw's FordLatStatus: an independent mem-param handle, fully guarded, so a
    # params failure can never touch the car path. None => the feature is simply off.
    self._cargps_params = None
    try:
      from openpilot.common.params import Params as _P
      self._cargps_params = _P("/dev/shm/params")
    except Exception:
      self._cargps_params = None
    self._cargps_decim = 0
    self._cargps_err = 0

  def update(self, can_parsers) -> structs.CarState:
    cp = can_parsers[Bus.pt]
    cp_cam = can_parsers[Bus.cam]

    ret = structs.CarState()

    # Occasionally on startup, the ABS module recalibrates the steering pinion offset, so we need to block engagement
    # The vehicle usually recovers out of this state within a minute of normal driving
    ret.vehicleSensorsInvalid = cp.vl["SteeringPinion_Data"]["StePinCompAnEst_D_Qf"] != 3

    # car speed
    ret.vEgoRaw = cp.vl["BrakeSysFeatures"]["Veh_V_ActlBrk"] * CV.KPH_TO_MS
    ret.vEgo, ret.aEgo = self.update_speed_kf(ret.vEgoRaw)
    ret.yawRate = cp.vl["Yaw_Data_FD1"]["VehYaw_W_Actl"]
    ret.standstill = cp.vl["DesiredTorqBrk"]["VehStop_D_Stat"] == 1

    # gas pedal
    ret.gasPressed = cp.vl["EngVehicleSpThrottle"]["ApedPos_Pc_ActlArb"] / 100. > 1e-6

    # brake pedal
    ret.brake = cp.vl["BrakeSnData_4"]["BrkTot_Tq_Actl"] / 32756.  # torque in Nm
    ret.brakePressed = cp.vl["EngBrakeData"]["BpedDrvAppl_D_Actl"] == 2
    ret.parkingBrake = cp.vl["DesiredTorqBrk"]["PrkBrkStatus"] in (1, 2)

    # steering wheel
    ret.steeringAngleDeg = cp.vl["SteeringPinion_Data"]["StePinComp_An_Est"]
    ret.steeringTorque = cp.vl["EPAS_INFO"]["SteeringColumnTorque"]
    ret.steeringPressed = self.update_steering_pressed(abs(ret.steeringTorque) > CarControllerParams.STEER_DRIVER_ALLOWANCE, 5)
    ret.steerFaultTemporary = cp.vl["EPAS_INFO"]["EPAS_Failure"] == 1
    ret.steerFaultPermanent = cp.vl["EPAS_INFO"]["EPAS_Failure"] in (2, 3)
    ret.espDisabled = cp.vl["Cluster_Info1_FD1"]["DrvSlipCtlMde_D_Rq"] != 0  # 0 is default mode

    if self.CP.flags & FordFlags.CANFD:
      # this signal is always 0 on non-CAN FD cars
      ret.steerFaultTemporary |= cp.vl["Lane_Assist_Data3_FD1"]["LatCtlSte_D_Stat"] not in (1, 2, 3)

    # cruise state
    is_metric = cp.vl["INSTRUMENT_PANEL"]["METRIC_UNITS"] == 1 if not self.CP.flags & FordFlags.CANFD else False
    ret.cruiseState.speed = cp.vl["EngBrakeData"]["Veh_V_DsplyCcSet"] * (CV.KPH_TO_MS if is_metric else CV.MPH_TO_MS)
    ret.cruiseState.enabled = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (4, 5)
    ret.cruiseState.available = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (3, 4, 5)
    ret.cruiseState.nonAdaptive = cp.vl["Cluster_Info1_FD1"]["AccEnbl_B_RqDrv"] == 0
    ret.cruiseState.standstill = cp.vl["EngBrakeData"]["AccStopMde_D_Rq"] == 3
    ret.accFaulted = cp.vl["EngBrakeData"]["CcStat_D_Actl"] in (1, 2)
    if not self.CP.openpilotLongitudinalControl:
      ret.accFaulted = ret.accFaulted or cp_cam.vl["ACCDATA"]["CmbbDeny_B_Actl"] == 1

    # gear
    if self.CP.transmissionType == TransmissionType.automatic:
      gear = self.shifter_values.get(cp.vl["PowertrainData_10"]["TrnRng_D_Rq"])
      ret.gearShifter = self.parse_gear_shifter(gear)
    elif self.CP.transmissionType == TransmissionType.manual:
      if bool(cp.vl["BCM_Lamp_Stat_FD1"]["RvrseLghtOn_B_Stat"]):
        ret.gearShifter = GearShifter.reverse
      else:
        ret.gearShifter = GearShifter.drive

    # safety
    ret.stockFcw = bool(cp_cam.vl["ACCDATA_3"]["FcwVisblWarn_B_Rq"])
    ret.stockAeb = bool(cp_cam.vl["ACCDATA_2"]["CmbbBrkDecel_B_Rq"])

    # button presses
    ret.leftBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 1
    ret.rightBlinker = cp.vl["Steering_Data_FD1"]["TurnLghtSwtch_D_Stat"] == 2
    # TODO: block this going to the camera otherwise it will enable stock TJA
    ret.genericToggle = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])
    prev_distance_button = self.distance_button
    prev_lc_button = self.lc_button
    prev_main_button = self.main_button
    prev_res_button = self.res_button
    prev_set_inc_button = self.set_inc_button
    prev_set_dec_button = self.set_dec_button
    self.distance_button = cp.vl["Steering_Data_FD1"]["AccButtnGapTogglePress"]
    self.lc_button = bool(cp.vl["Steering_Data_FD1"]["TjaButtnOnOffPress"])
    self.main_button = int(cp.vl["Steering_Data_FD1"]["CcButtnOnOffPress"])
    self.res_button = int(cp.vl["Steering_Data_FD1"]["CcAsllButtnResPress"])
    self.set_inc_button = int(cp.vl["Steering_Data_FD1"]["CcAslButtnSetIncPress"])
    self.set_dec_button = int(cp.vl["Steering_Data_FD1"]["CcAslButtnSetDecPress"])

    # lock info
    ret.doorOpen = any([cp.vl["BodyInfo_3_FD1"]["DrStatDrv_B_Actl"], cp.vl["BodyInfo_3_FD1"]["DrStatPsngr_B_Actl"],
                        cp.vl["BodyInfo_3_FD1"]["DrStatRl_B_Actl"], cp.vl["BodyInfo_3_FD1"]["DrStatRr_B_Actl"]])
    ret.seatbeltUnlatched = cp.vl["RCMStatusMessage2_FD1"]["FirstRowBuckleDriver"] == 2

    # blindspot sensors
    if self.CP.enableBsm:
      cp_bsm = cp_cam if self.CP.flags & FordFlags.CANFD else cp
      ret.leftBlindspot = cp_bsm.vl["Side_Detect_L_Stat"]["SodDetctLeft_D_Stat"] != 0
      ret.rightBlindspot = cp_bsm.vl["Side_Detect_R_Stat"]["SodDetctRight_D_Stat"] != 0

    # Stock steering buttons so that we can passthru blinkers etc.
    self.buttons_stock_values = cp.vl["Steering_Data_FD1"]
    # Stock values from IPMA so that we can retain some stock functionality
    self.acc_tja_status_stock_values = cp_cam.vl["ACCDATA_3"]
    self.lkas_status_stock_values = cp_cam.vl["IPMA_Data"]

    ret.buttonEvents = [
      *create_button_events(self.distance_button, prev_distance_button, {1: ButtonType.gapAdjustCruise}),
      *create_button_events(self.lc_button, prev_lc_button, {1: ButtonType.lkas}),
      # onebutton2pnw. MEASURED 2026-09-07 (drives/2026-09-07/lightning-onoff-button/): the ACC
      # state alone CANNOT tell you the driver asked for off. From Standby (CcStat 3) -- which is
      # exactly where the truck sits after a brake drops cruise while MADS keeps steering -- this
      # button NEVER reaches Off. Four presses observed from Standby: two did nothing at all, two
      # turned the system ON, zero produced Off. From Off and from Active it is a clean toggle.
      # So `cruiseState.available` going false is not a usable signal in the state that matters,
      # and the PRESS itself has to be surfaced. Nothing else in openpilot acts on mainCruise
      # (only Hyundai's own interface lists it), so this is inert for every other consumer.
      *create_button_events(self.main_button, prev_main_button, {1: ButtonType.mainCruise}),
      # The driver's own ENGAGE presses. Read from bus 0, which carries the SCCM's frames only --
      # openpilot's own RES/SET taps go out on sendcan and do NOT come back as RX here (verified
      # 2026-09-07: a RESUME tapped at t=80.2 appears in sendcan and in NO bus-0 button event), so
      # these cannot be self-triggered by our own presses.
      *create_button_events(self.res_button, prev_res_button, {1: ButtonType.resumeCruise}),
      *create_button_events(self.set_inc_button, prev_set_inc_button, {1: ButtonType.accelCruise}),
      *create_button_events(self.set_dec_button, prev_set_dec_button, {1: ButtonType.decelCruise}),
    ]

    self._publish_car_gps(cp)
    return ret

  def _publish_car_gps(self, cp) -> None:
    """cargps2pnw: publish the truck's own GPS fix to /dev/shm CarGps for the ces_events log.

    TELEMETRY ONLY -- nothing reads this for control, and every failure path is a silent no-op that
    leaves the car untouched. Decimated to ~1 Hz because the source is 1 Hz; update() runs at 100 Hz.

    DECODE GOTCHA, easy to get wrong: GPS_Longitude_Degrees is scaled (1,-179), so in the western
    hemisphere it arrives ALREADY NEGATIVE (e.g. -122.0) while GPS_Longitude_Minutes/_Min_dec are
    UNSIGNED magnitudes. Adding them naively yields -121.635 for a true -122.365 -- about 57 km east,
    and plausible enough to believe. Combine sign-first (see _dm_to_deg).

    BUS GOTCHA, cost a whole drive of frozen telemetry (2026-09-05): APIMGPS originates on the
    POWERTRAIN bus (0), not the camera bus (2). The panda RELAYS it onto bus 2, but a relayed frame
    is reported with the TX flag set (src = 2 + 128 = 130), and CANParser.update() skips anything
    where `src != self.bus` -- so a bus-2 parser NEVER sees it. Registered on cp (Bus.pt) for that
    reason. Symptom if this regresses: `age` climbs without bound while lat/lon hold their last
    value. (ACCDATA_3 is the mirror image: it originates on bus 2 and is relayed to bus 0 as 128.)
    """
    if self._cargps_params is None:
      return
    self._cargps_decim += 1
    if self._cargps_decim % 100:          # ~1 Hz against a 100 Hz update()
      return
    # Fable 2026-09-05: `cp_cam.vl` is a VLDict whose __getitem__ LAZILY registers an unknown
    # message with freq=None -- i.e. ALIVE-CHECKED. If the DBC probe in get_can_parsers ever failed
    # while the DBC did have the message, this indexing would register it checked, can_valid would
    # go False, and interfaces.py would set ret.canValid=False -> THE CAR BECOMES UNDRIVEABLE. The
    # defensive branch failed OPEN into exactly the outcome the nan-frequency registration exists to
    # prevent. `in` uses dict.__contains__, which does NOT lazily add, so this fails closed.
    if "APIMGPS_Data_Nav_1_FD1" not in cp.vl or "APIMGPS_Data_Nav_3_FD1" not in cp.vl:
      return
    try:
      nav1 = cp.vl["APIMGPS_Data_Nav_1_FD1"]
      nav3 = cp.vl["APIMGPS_Data_Nav_3_FD1"]
      # Oldest of the two messages: hdg/spd/sats/hdop come from Nav_3, so tracking only Nav_1 would
      # let a frozen Nav_3 keep reading fresh (Fable + Gemini, 2026-09-05).
      last_ns = min(int(cp.ts_nanos["APIMGPS_Data_Nav_1_FD1"]["GPS_Latitude_Degrees"]),
                    int(cp.ts_nanos["APIMGPS_Data_Nav_3_FD1"]["GPS_Heading"]))
      lat_deg = float(nav1["GPS_Latitude_Degrees"])
      lon_deg = float(nav1["GPS_Longitude_Degrees"])
      if lat_deg == 0.0 and lon_deg == 0.0:
        return                            # no fix yet -- publish nothing rather than 0,0
      self._cargps_params.put_nonblocking("CarGps", {
        "lat": round(_dm_to_deg(lat_deg, nav1["GPS_Latitude_Minutes"], nav1["GPS_Latitude_Min_dec"]), 6),
        "lon": round(_dm_to_deg(lon_deg, nav1["GPS_Longitude_Minutes"], nav1["GPS_Longitude_Min_dec"]), 6),
        "hdg": round(float(nav3["GPS_Heading"]), 1),
        "spd": round(float(nav3["GPS_Speed"]), 1),          # MPH, as the DBC defines it
        "sats": int(nav3["GPS_Sat_num_in_view"]),
        "hdop": round(float(nav3["GPS_Hdop"]), 1),
        "ts": round(time.time(), 2),
        # Rule 2: `ts` is when we PUBLISHED, which advances even when the decode is frozen -- that is
        # exactly what hid the wrong-bus bug above for a whole drive. `age` is seconds since the CAN
        # frame was actually received, so a stale fix is visible in the log instead of masquerading
        # as live. Differenced against the parser's OWN clock (_last_update_nanos, the logMonoTime of
        # the batch we just consumed) rather than time.monotonic_ns(): ts_nanos is stamped from
        # nanos_since_boot() (CLOCK_BOOTTIME) while monotonic_ns() is CLOCK_MONOTONIC, and it keeps
        # the number meaningful under REPLAY. NOT clamped at zero -- a negative age means the clocks
        # disagree, and clamping would turn that into a permanent "age 0.0", i.e. the exact
        # reads-as-live failure this commit exists to fix (Fable, 2026-09-05).
        "age": round((cp._last_update_nanos - last_ns) / 1e9, 2),
      })
    except Exception:
      # Rule 2: silence here is what let the wrong-bus bug run a whole drive. The catch itself has to
      # stay -- an exception escaping CarState.update() would take down `card` and the truck for a
      # telemetry field -- but it must not be invisible. Rate-limited so a persistent failure costs
      # one line a minute, not 100 a second.
      self._cargps_err += 1
      if self._cargps_err == 1 or self._cargps_err % 6000 == 0:
        carlog.exception("cargps2pnw: publish failed (%d so far)", self._cargps_err)

  # cargps2pnw: the truck's OWN GPS fix, broadcast by the GWM on the POWERTRAIN bus at 1 Hz.
  # Confirmed on-vehicle 2026-09-05 (F-150 Lightning, route 000000dc--c844257700 seg 8): decoded
  # position agreed with the comma's own GPS to 5.4 m, with 31 satellites and HDOP 0.4 -- a better
  # fix than the device gets behind a windshield. Telemetry only; nothing consumes it for control.
  #
  # REGISTERED WITH float("nan") FREQUENCY, WHICH IS LOAD-BEARING. CANParser sets
  # ignore_alive = isnan(freq) (opendbc/can/parser.py), and MessageState.valid() returns True
  # immediately when ignore_alive. Without that, a missing APIMGPS message would make this parser
  # can_valid False, and interfaces.py does `ret.canValid = all(cp.can_valid ...)` -- i.e. a Ford
  # with no SYNC nav, an asleep APIM, or a GPS fault would render the CAR UNUSABLE for a telemetry
  # field. nan makes that impossible: the messages are decoded when present and simply absent
  # otherwise.
  GPS_MSGS = ("APIMGPS_Data_Nav_1_FD1", "APIMGPS_Data_Nav_3_FD1")

  @staticmethod
  def get_can_parsers(CP):
    dbc_name = DBC[CP.carFingerprint][Bus.pt]
    # Not every Ford DBC carries the APIMGPS messages, and CANParser RAISES on an unknown message
    # name (parser.py: "could not find message ..."), which would be a hard failure at car start.
    # Probe the DBC first and only register what it actually has.
    pt_msgs = []
    try:
      from opendbc.can.parser import DBC as _DBC
      known = _DBC(dbc_name).name_to_msg
      pt_msgs = [(m, float("nan")) for m in CarState.GPS_MSGS if m in known]
    except Exception:
      pt_msgs = []
    return {
      Bus.pt: CANParser(dbc_name, pt_msgs, CanBus(CP).main),
      Bus.cam: CANParser(dbc_name, [], CanBus(CP).camera),
    }


def _dm_to_deg(deg: float, minutes: float, min_dec: float) -> float:
  """Degrees + UNSIGNED minutes -> signed decimal degrees, SIGN FIRST (see _publish_car_gps)."""
  sign = -1.0 if deg < 0 else 1.0
  return sign * (abs(deg) + (float(minutes) + float(min_dec)) / 60.0)

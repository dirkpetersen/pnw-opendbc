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
SpeedUnit = structs.CarState.CruiseState.SpeedUnit
TransmissionType = structs.CarParams.TransmissionType

# gearunknown2pnw: PowertrainData_10 is a 10 Hz frame (measured: 600 of 6001 bus-0 batches in a parked
# rlog), and card's CarState starts after fingerprinting, when the bus is already awake. 10 s without one
# is not a slow start.
GEAR_MISSING_LOG_S = 10.0
# truckdecode2pnw: the cluster-unit decode says so if it is still unknown this long after carState started, and
# otherwise logs a change at most once per this many seconds (Cluster_Info1_FD1 arrives at ~10 Hz, so a flapping
# bit must not become 10 lines a second).
UNIT_LOG_S = 10.0


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
    # lightning-extra2pnw: latched Pro Power Onboard state. `ppo_valid` stays False until at least
    # ONE of the two signals has actually been received (they are ORed, not ANDed), so "not yet
    # read" can never be mistaken for "off" -- the armer refuses to act on an unread state rather
    # than guessing.
    self.ppo_on = False
    self.ppo_valid = False
    self._ppo_err = 0
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

    # gearunknown2pnw: Rule 2 bookkeeping for the gear message (see _gear_from_can).
    self._gear_wait_start_nanos: int | None = None
    self._gear_seen_logged = False
    self._gear_missing_logged = False

    # truckdecode2pnw: Rule 2 bookkeeping for the cluster-unit decode (see _cluster_unit).
    self._unit_start_nanos: int | None = None
    self._unit_logged = None               # last LOGGED unit; None = nothing logged yet
    self._unit_log_nanos: int | None = None
    self._unit_err = 0

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
    if self.CP.flags & FordFlags.CANFD:
      # units2pnw: on CAN FD, Veh_V_DsplyCcSet is in the cluster's unit, which MetricActv_B_Actl gives (proven on the
      # owner's truck after its cluster switched to km/h; see _cluster_unit). Upstream hardcoded mph here, so every
      # consumer read a km/h set 1.609x high. `unknown` (Cluster_Info1_FD1 never received) keeps that mph assumption
      # and the decode logs it.
      unit = self._cluster_unit(cp, cp_cam)
      ret.cruiseState.speedClusterUnit = unit
      is_metric = unit == SpeedUnit.kph
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
      ret.gearShifter = self._gear_from_can(cp)
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

    self._read_pro_power(cp)
    self._publish_car_gps(cp)
    return ret

  def _gear_from_can(self, cp) -> structs.CarState.GearShifter:
    """gearunknown2pnw: the automatic's gear, or `unknown` until PowertrainData_10 has been received.

    The parser starts every signal at 0, and TrnRng_D_Rq 0 means "Park". Decoding it before a frame
    has arrived therefore reported a confident Park from a bus that had said nothing at all -- a dead
    powertrain bus, or the start of every card session. Consumers that trust the gear without also
    checking canValid (loggerd's parked-video/thin-rlog gate, card's GearPark writer) could not tell
    that apart from a real Park.

    PRESENCE IS TESTED WITH ts_nanos, as in _read_pro_power: it stays 0 until a frame has been parsed
    successfully and never returns to 0 afterwards. Once the message has been seen, the decode is
    exactly what it was before this change, including holding the last value if the message later
    goes quiet (that case is reported by canValid, which this message's alive check makes False).
    """
    # ORDER MATTERS: indexing cp.vl is what registers the message with the parser (alive-checked, as it
    # always was); cp.ts_nanos is a plain dict and raises KeyError for a message not yet registered.
    trn_rng = cp.vl["PowertrainData_10"]["TrnRng_D_Rq"]
    seen = cp.ts_nanos["PowertrainData_10"]["TrnRng_D_Rq"] != 0
    self._log_gear_presence(cp, seen)
    if not seen:
      return GearShifter.unknown
    return self.parse_gear_shifter(self.shifter_values.get(trn_rng))

  def _log_gear_presence(self, cp, seen: bool) -> None:
    """Rule 2: say once when the gear becomes known, and once if it still is not after GEAR_MISSING_LOG_S.

    card only runs with ignition on, so the wait is measured from this CarState's first update, on the
    parser's own clock (logMonoTime of the CAN batches), which also keeps it meaningful under replay."""
    if self._gear_seen_logged:
      return
    now = cp._last_update_nanos
    if self._gear_wait_start_nanos is None:
      self._gear_wait_start_nanos = now
    waited = (now - self._gear_wait_start_nanos) / 1e9
    if seen:
      self._gear_seen_logged = True
      carlog.warning(f"gearunknown2pnw: PowertrainData_10 first received {waited:.1f} s after carState started; gearShifter decoded from here on")
    elif not self._gear_missing_logged and waited >= GEAR_MISSING_LOG_S:
      self._gear_missing_logged = True
      bus = "the powertrain bus has traffic" if cp.last_nonempty_nanos != 0 else "NO powertrain bus traffic at all"
      carlog.error(f"gearunknown2pnw: PowertrainData_10 not received {waited:.0f} s after carState started ({bus}); " +
                   "gearShifter stays unknown, so Park cannot be confirmed and the device keeps recording")

  def _cluster_unit(self, cp, cp_cam) -> structs.CarState.CruiseState.SpeedUnit:
    """units2pnw: the unit the CAN FD cluster shows the set speed (Veh_V_DsplyCcSet) in, or `unknown`.

    THE SOURCE IS Cluster_Info1_FD1 (0x430, bus 0, sent by the GWM) MetricActv_B_Actl, DBC "0 =Inactive(English),
    1=Active(Metric)": 1 -> kph, 0 -> mph, and `unknown` until a Cluster_Info1_FD1 frame has been RECEIVED (a
    never-seen message is not mph; the gearunknown2pnw pattern).

    PROVEN on the owner's truck, whose cluster switched to km/h on Sun 2026-09-13 21:06-21:08 PT
    (drives/2026-09-14/units-kmh/DRIVE_REPORT.md, "Which signal Veh_V_DsplyCcSet follows"):
      * Veh_V_RqCcSet (0x202, DBC unit kph) / Veh_V_DsplyCcSet while ACC is engaged: 1.571-1.587 on every segment
        with MetricActv_B_Actl 0 (09-11/09-12, 17 segments), 0.976-0.978 on every segment with it 1 (09-13 21:09 on).
        1.609 x 0.977 = 1.572: the same set speed, read in mph and then in km/h.
      * held speed, engaged with no lead: MetricActv 0 -> vEgo within -0.3..-0.8 mph of the set (+19..+27 if km/h);
        MetricActv 1 -> set 42 held at 41.1 km/h (-16.5 if mph); the 21:16 Corvallis set 55 held 53.7-54.0 km/h.
    NOT IPMA_Data2 (0x3D9, camera) IsaVLimUnit_D_Rq: it read 2 "Mph" in BOTH regimes, so it does not follow the set
    speed (an earlier version of this decode required it to agree, which would have reported mph on the km/h truck).
    It is decoded for the log line only and never selects the unit. Mc_VehUntTrpCoUsrSel_St (0x2FD) also stayed 1.
    Not separable from this data: Traffic_RecognitnData (0x3CD, camera) TsrVlUnitMsgTxt_D_Rq flipped with
    MetricActv_B_Actl in every segment. MetricActv_B_Actl is used because it is the cluster's own state, on bus 0.

    update() converts cruiseState.speed with this unit (kph -> KPH_TO_MS; mph and unknown -> MPH_TO_MS), so every
    consumer gets true m/s. speedClusterUnit stays published for what needs the cluster's own step: one SET tap moves
    the set by one unit, 1 km/h on a kph cluster (the ICBM executor).

    PRESENCE IS TESTED WITH ts_nanos (see _read_pro_power). Cluster_Info1_FD1 is already read (lazily registered,
    alive-checked) by upstream's espDisabled and nonAdaptive lines, so this adds no canValid dependency. IPMA_Data2 is
    registered ignore_alive in get_can_parsers and indexed only after the `in` check, which never lazily registers
    (see _publish_car_gps), so it can never make canValid false. Never raises: a car path exception would take down
    card; a decode failure reports unknown and logs.
    """
    unit, isa, metric = SpeedUnit.unknown, None, None
    try:
      if cp.ts_nanos["Cluster_Info1_FD1"]["MetricActv_B_Actl"] != 0:
        metric = int(cp.vl["Cluster_Info1_FD1"]["MetricActv_B_Actl"])
      if metric == 1:
        unit = SpeedUnit.kph
      elif metric == 0:
        unit = SpeedUnit.mph
      # telemetry only: carried in the log line, never part of the decision above
      if "IPMA_Data2" in cp_cam.vl and cp_cam.ts_nanos["IPMA_Data2"]["IsaVLimUnit_D_Rq"] != 0:
        isa = int(cp_cam.vl["IPMA_Data2"]["IsaVLimUnit_D_Rq"])
      self._log_cluster_unit(cp, unit, isa, metric)
    except Exception:
      self._unit_err += 1
      if self._unit_err == 1 or self._unit_err % 6000 == 0:
        carlog.exception("units2pnw: cluster unit decode failed (%d so far); reporting unknown", self._unit_err)
      unit = SpeedUnit.unknown
    return unit

  def _log_cluster_unit(self, cp, unit, isa, metric) -> None:
    """Rule 2: say what the unit is once it is known, say so if it is still unknown UNIT_LOG_S after carState started,
    and log every later change -- at most one line per UNIT_LOG_S, always carrying the raw values."""
    now = cp._last_update_nanos
    if self._unit_start_nanos is None:
      self._unit_start_nanos = now
    if unit == self._unit_logged:
      return
    if unit == SpeedUnit.unknown and self._unit_logged is None and (now - self._unit_start_nanos) / 1e9 < UNIT_LOG_S:
      return                        # the first frames have not arrived yet: not a finding
    if self._unit_log_nanos is not None and (now - self._unit_log_nanos) / 1e9 < UNIT_LOG_S:
      return
    self._unit_logged, self._unit_log_nanos = unit, now
    name = {SpeedUnit.mph: "mph", SpeedUnit.kph: "kph"}.get(unit, "unknown")   # the builder enum is a bare int
    line = (f"units2pnw: cluster set-speed unit {name} (Cluster_Info1_FD1.MetricActv_B_Actl={metric}; " +
            f"IPMA_Data2.IsaVLimUnit_D_Rq={isa}, telemetry only; None = never received)")
    if unit == SpeedUnit.unknown:
      carlog.warning(line + " -- cruiseState.speed ASSUMES mph (Veh_V_DsplyCcSet x MPH_TO_MS)")
    else:
      carlog.warning(line)

  def _read_pro_power(self, cp) -> None:
    """lightning-extra2pnw: latch the Pro Power Onboard state from the two messages that carry it.

    Both were verified to decode correctly against the REAL captured payloads (0x44A
    `0000000000bf0000` / `...bf1000`, 0x480 `4800808200000000` / `...8300000000`), not just derived
    from bit arithmetic. They are ORed rather than ANDed: either module reporting armed is enough,
    and the only consequence of a disagreement is that the armer does nothing, which is the
    fail-safe direction for a feature that can only ever turn the setting ON.

    Telemetry/convenience only -- nothing here reaches a control path. Never raises: a Ford whose
    DBC lacks these messages simply leaves `ppo_valid` False forever and the feature stays inert.
    """
    try:
      # PRESENCE IS TESTED WITH ts_nanos, NOT WITH THE VALUE. Verified empirically 2026-09-07:
      # `cp.vl["X"].get("sig")` returns **0.0**, not None, for a message that has NEVER been
      # received -- the registered message is lazily populated with defaults. An `is None` test
      # therefore never fires, and `"X" not in cp.vl` is False too (the message IS registered), so
      # neither of the obvious guards works.
      #
      # Left unguarded this was a real hazard on OTHER Fords, not a cosmetic bug: any Ford on this
      # DBC that never transmits these messages would have read `ppo_valid=True, ppo_on=False`, and
      # the armer would then have spoofed 0x455 on a truck where that address may mean something
      # else entirely. (Gemini review 2026-09-07 found this; its proposed `not in cp.vl` fix does
      # not fire, so this uses ts_nanos, which is 0 until a frame actually arrives.)
      seen_a = cp.ts_nanos["EffDrvModeData"]["PnwProPwrOnbd_B_Stat"] != 0
      seen_b = cp.ts_nanos["HEV_Powertrain_Data6"]["PnwProPwrOnbd_B_Stat2"] != 0
      if not (seen_a or seen_b):
        return                                   # never received -> stays invalid, feature inert
      a = cp.vl["EffDrvModeData"]["PnwProPwrOnbd_B_Stat"] if seen_a else 0
      b = cp.vl["HEV_Powertrain_Data6"]["PnwProPwrOnbd_B_Stat2"] if seen_b else 0
      self.ppo_on = bool(a) or bool(b)
      self.ppo_valid = True
    except Exception:
      # Rule 2: NOT `pass`. An absent message is expected and handled by the ts_nanos test above, so
      # reaching here means something else went wrong -- and a silently-inert feature is exactly
      # what this project keeps getting bitten by. Rate-limited, mirroring the _cargps_err pattern.
      self._ppo_err += 1
      if self._ppo_err == 1 or self._ppo_err % 6000 == 0:
        carlog.exception("lightning-extra2pnw: Pro Power state read failed (%d so far)", self._ppo_err)

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
      # truckdecode2pnw: the GWM's own position-source flag, 0x463 GPS_Actual_vs_Infer_pos (1 =
      # "Inferred_Position", i.e. dead reckoning; 0 = "Actual_Postition"). Telemetry only. Measured on all
      # 24 local Lightning rlog segments (drives/2026-09-12/central-oregon-weekend/TRUCK_DECODE.md): 1 on all
      # 243 frames of the Sat 09-12 06:24 PT cold start (HDOP 3.8-5.4), 0 on all 1,140 frames of 22
      # normal-driving segments, and one 1 -> 0 transition, parked (Sat 14:18:45 PT, HDOP already 1.0).
      # OPTIONAL: a missing 0x463 must never stop the position publish above, so it is not in the guard;
      # `dr` is None until a frame has arrived (ts_nanos stays 0 until then, see _read_pro_power), and
      # `drAge` is the frame's own age on the parser clock, like `age`, so the consumer can tell a stale
      # flag from a live one (the wrong-bus lesson below).
      dr = dr_age = None
      if "APIMGPS_Data_Nav_2_FD1" in cp.vl:
        nav2_ns = int(cp.ts_nanos["APIMGPS_Data_Nav_2_FD1"]["GPS_Actual_vs_Infer_pos"])
        if nav2_ns != 0:
          dr = int(cp.vl["APIMGPS_Data_Nav_2_FD1"]["GPS_Actual_vs_Infer_pos"])
          dr_age = round((cp._last_update_nanos - nav2_ns) / 1e9, 2)
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
        "dr": dr,
        "drAge": dr_age,
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
  # truckdecode2pnw: Nav_2 (0x463) carries the dead-reckoning flag. Same bus (measured: 60 frames per 60 s
  # segment on src 0 in all 24 local Lightning rlog segments), same nan registration, same DBC probe.
  GPS_MSGS = ("APIMGPS_Data_Nav_1_FD1", "APIMGPS_Data_Nav_3_FD1", "APIMGPS_Data_Nav_2_FD1")

  # lightning-extra2pnw: the two messages that carry the Pro Power Onboard state, found by CAN diff
  # while the driver toggled the setting ten times (drives/2026-09-07/propower-toggle-scan/). Both
  # agree, and both are registered with nan frequency for the SAME reason as GPS_MSGS above: a Ford
  # without them must not be rendered unusable over a body-comfort feature.
  PPO_MSGS = ("EffDrvModeData", "HEV_Powertrain_Data6")

  # truckdecode2pnw: the camera's IPMA_Data2 (0x3D9) carries IsaVLimUnit_D_Rq, logged next to the cluster-unit decode
  # (_cluster_unit) as telemetry only (units2pnw: it does not follow the set-speed unit). It originates on bus 2 and
  # is relayed onto bus 0 with the TX flag (src 128; measured 1,000 frames per 60 s on src 2 and src 128), so only the
  # CAMERA parser can see it. nan frequency for the same reason as GPS_MSGS: a missing message must not make the car
  # undriveable. Registered on CAN FD only, where the decode runs.
  UNIT_CAM_MSGS = ("IPMA_Data2",)

  @staticmethod
  def get_can_parsers(CP):
    dbc_name = DBC[CP.carFingerprint][Bus.pt]
    # Not every Ford DBC carries the APIMGPS messages, and CANParser RAISES on an unknown message
    # name (parser.py: "could not find message ..."), which would be a hard failure at car start.
    # Probe the DBC first and only register what it actually has.
    pt_msgs = []
    cam_msgs = []
    try:
      from opendbc.can.parser import DBC as _DBC
      known = _DBC(dbc_name).name_to_msg
      pt_msgs = [(m, float("nan")) for m in CarState.GPS_MSGS + CarState.PPO_MSGS if m in known]
      if CP.flags & FordFlags.CANFD:
        cam_msgs = [(m, float("nan")) for m in CarState.UNIT_CAM_MSGS if m in known]
    except Exception:
      pt_msgs = []
      cam_msgs = []
    return {
      Bus.pt: CANParser(dbc_name, pt_msgs, CanBus(CP).main),
      Bus.cam: CANParser(dbc_name, cam_msgs, CanBus(CP).camera),
    }


def _dm_to_deg(deg: float, minutes: float, min_dec: float) -> float:
  """Degrees + UNSIGNED minutes -> signed decimal degrees, SIGN FIRST (see _publish_car_gps)."""
  sign = -1.0 if deg < 0 else 1.0
  return sign * (abs(deg) + (float(minutes) + float(min_dec)) / 60.0)

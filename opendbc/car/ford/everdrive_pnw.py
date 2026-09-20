"""everdrive2pnw: telemetry snapshot of the AFTERMARKET EverDrive auxiliary AC charger + the truck's
own energy state, for a read-only onroad UI box ("ED:1.4kw,112mi->117mi").

DISPLAY ONLY. Nothing here reaches a control path, an actuator, panda, or safety code. Every failure
path is a no-op that leaves the car untouched, and the module is inert unless an EverDrive is
actually broadcasting.

WHY IT IS A SEPARATE FILE: carstate.py gets exactly three added lines (construct, call, register).
All of the logic, the constants and the Rule-2 bookkeeping live here.

SOURCES (all measured on VIN 1FT6W3L78SWG05094, 2026-09-19 -- see
CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md):
  0x2A7 EverDrive_AC_Meter_FD1  AFTERMARKET, added to ford_lincoln_base_pt.dbc by this branch. 1 Hz.
  0x442 VehElRnge_L_Dsply       the DASH range, driver-confirmed 180.2 km = 112 mi. Already in the DBC.
  0x36D VehElEffAvg_No_Dsply    10 Wh/km, offset -100. Already in the DBC.
  0x24C BattTracSoc2_Pc_Actl    0.01 %/bit, RAW pack SoC (not the dash's usable SoC). Already in the DBC.

⚠️ HONESTY NOTE FOR WHOEVER BUILDS THE "->117mi" PROJECTION: `effWhKm` is NOT measured consumption.
ENERGY-RANGE-SIGNALS.md §3 measured it constant at raw 42 (320 Wh/km) for an entire 13-minute drive,
and §2 shows RngPerChrgAvg x it = 127.2 kWh against a 131 kWh pack -- i.e. it is the truck's
REFERENCE efficiency used for range estimation, not a rolling average. CORRECTED 2026-09-20: an
earlier draft called it "~the EPA figure" and that is WRONG. 320 Wh/km = 515 Wh/mi = 1.94 mi/kWh,
against an EPA-equivalent ~2.44 mi/kWh (320 mi / 131 kWh) and a MEASURED 3.8 mi/kWh on the city drive.
So it is ~26% more pessimistic than EPA and about half the measured figure -- it behaves like the
truck's own LONG-RUN average (unmoved off raw 42 across a drive and a full day). A projection built on
it is "range this supply buys at the truck's own long-run rate", which is the conservative direction.
That document also records (§3) that NO broadcast signal gives real vehicle consumption.
"""
import time
from collections import deque

from opendbc.car.carlog import carlog

# everdrive2pnw: the messages this feature reads. Registered EXPLICITLY with float("nan") frequency by
# carstate.get_can_parsers -- see ENERGY_MSGS' use site there. This is load-bearing and is NOT
# optional: CANParser's VLDict lazily registers an unindexed message with freq=None, which means
# ignore_alive=False and a 10 s alive timeout, so a car that never transmits one of these would go
# can_valid False -> ret.canValid False -> THE CAR BECOMES UNDRIVEABLE over a display box. Measured
# 2026-09-19 (see the report for this branch): lazy-registered + never received => can_valid False
# after 15 s; nan-registered + never received => can_valid True. Same reason as CarState.GPS_MSGS /
# PPO_MSGS, which is why this tuple is fed through the same DBC probe.
AC_MSG = "EverDrive_AC_Meter_FD1"
RANGE_MSG = "MtrTrac_Data2_FD1"
EFF_MSG = "HEV_Powertrain_Data7_FD1"
SOC_MSG = "Battery_Traction_4_FD1"
# everdrive2pnw (2026-09-20): the truck's UNADJUSTED full-charge range. Paired with VehElEffAvg it
# yields the pack's usable capacity without hardcoding one -- see CAP_* below.
RPC_MSG = "Cluster_HEV_Data10_FD1"
ENERGY_MSGS = (AC_MSG, RANGE_MSG, EFF_MSG, SOC_MSG, RPC_MSG)

PARAM_KEY = "EverDriveStatus"

# everdrive2pnw: 5 Hz publish while an EverDrive is live, per spec. CarState.update() runs at 100 Hz.
PUBLISH_S = 0.2

# everdrive2pnw: THE ABSENCE RE-CHECK INTERVAL, and also how long 0x2A7 may go quiet before we call
# the module gone. 3.0 s, for one reason that covers both uses: 0x2A7 is a 1 Hz frame, so 3 s is
# THREE consecutive missed frames -- short enough that plugging the charger in mid-drive shows up
# within ~3 s (imperceptible next to the time a charger takes to start delivering power), long enough
# that a single dropped frame on a busy CAN FD bus can never flap the box off.
#
# Deliberately NOT a permanent latch (owner requirement 2026-09-19): the charger can be plugged in
# mid-session and must start working. The cost of not latching is one time.monotonic() and one float
# compare per call -- ~1 real check per 300 calls of update() -- which is the whole steady-state cost
# on a truck with no EverDrive fitted.
AC_QUIET_S = 3.0
AC_QUIET_NS = int(AC_QUIET_S * 1e9)

# everdrive2pnw: how often a persistent problem is allowed to produce a log line.
LOG_EVERY_S = 60.0

# everdrive2pnw: plausibility bands. A value outside its band is reported as None ("no usable
# value"), never as a number. These are NOT cosmetic -- each signal has DBC-declared sentinels that
# decode to a physically plausible-looking figure:
#   VehElRnge_L_Dsply   raw 4094 NoDataExists / 4095 Fault -> 409.4 / 409.5 km = 254 / 254.5 mi
#   BattTracSoc2_Pc_Actl raw 16382 NoDataExists / 16383 Faulty -> 163.82 / 163.83 %
#   VehElEffAvg_No_Dsply raw 0 = the -100 Wh/km ENCODING FLOOR ("not available", NOT a measurement);
#                        raw 126 NoDataExists / 127 Faulty -> 1160 / 1170 Wh/km
# Shipping 254 mi of range because the BECM said NoDataExists is exactly the "plausible number that
# is wrong" failure this project keeps paying for.
# 409.2, not 409.3. 4093 is the DBC's own GenSigStartValue for BOTH VehElRnge_L_Dsply (dbc:8128) and
# RngPerChrgAvg_L_Dsply (dbc:9542) -- the value these signals carry BEFORE the ECU has an estimate,
# which _usable()'s inclusive compare would otherwise accept. It is not a sentinel in the VAL_ table,
# so nothing else rejects it, and it decodes to a completely plausible number: 409.3 km = 254 mi of
# range, or 409.3 x 320 Wh/km = 131.0 kWh -- landing exactly on Ford's headline capacity, which is
# the one wrong answer nobody would question. 4092 is the largest raw that is neither a start value
# nor a sentinel. (Fable review 2026-09-20; the sibling RngPerChrgInst was observed saturating at
# raw 4093 in 759 of 1547 samples on this truck, so this family does put 4093 on the wire.)
RANGE_KM_BAND = (0.0, 409.25)
SOC_PCT_BAND = (0.0, 100.0)       # a real SoC cannot exceed 100 %
# RngPerChrgAvg band: 409.3 is the DBC's own max, below its sentinels. POSITIVE lower bound because
# this is a load-bearing input to the derived capacity, and a zero or negative would poison it.
RPC_KM_BAND = (0.1, 409.25)
# The efficiency lower bound is POSITIVE, not -99.9. `effOk` means "this is a real measurement", and a
# negative average Wh/km is not one -- raws 1..9 decode to -90..-10 Wh/km, which are inside the signal's
# declared range and are not sentinels, so nothing else would reject them. The consumer also DIVIDES by
# this (the gain rate) and MULTIPLIES by it (assumed power), so a negative arrives as a confidently
# signed wrong answer rather than an obviously broken one.
EFF_WH_KM_BAND = (0.1, 1159.9)    # exclusive of the -100 floor, of negatives, and of both sentinels

# everdrive2pnw: PLAUSIBILITY BAND ON THE AFTERMARKET AC METER. Unlike the three truck signals above,
# 0x2A7 comes from a third-party module, and its DBC entry has NO counter and NO checksum -- so a
# garbled frame is accepted by the parser as readily as a good one. Unbanded, the decode ceiling is
# 409.6 A x 8191.9 V = 3.3 MW, which would (a) be published as fact and (b) overflow the UI's
# fixed-width box and clip. Bands are PHYSICAL rather than an arbitrary kW cap:
#   current  0 .. 100 A   -- beyond any plausible EVSE; 0 is required for the genuine unplugged frame
#   voltage  0 .. 277 V   -- single-phase AC mains ceiling (277 V line-to-neutral on a 480Y system);
#                            0 likewise, since the unplugged frame is all zeros and that is a REAL zero
# Measured on this truck at L1: 12.50 A x 109.0 V. The implied ceiling, 27.7 kW, also bounds the UI's
# gain-rate digits to the two the fixed-width exemplar allows.
AC_I_BAND = (0.0, 100.0)
AC_U_BAND = (0.0, 277.0)

# everdrive2pnw (2026-09-20): ROLLING PACK CONSUMPTION -- the input to the UI's "range at the speed
# you are doing right now" figure. There is still NO broadcast signal for vehicle power on this truck
# (ENERGY-RANGE-SIGNALS.md §3 ruled out all eight candidates over 13 minutes of real driving), so it
# is differenced out of the pack SoC, which IS verified. Two measurements made that viable on
# 2026-09-20, both read over UDS and neither re-derived here:
#
#   0x224848 Energy  (HVB Energy to Empty) = 59.350 kWh
#   0x224801 HvbSoc  (true SoC)            = 47.120 %   ==  broadcast 0x24C BattTracSoc2_Pc_Actl
#
#   -> usable capacity = 59.350 / 0.4712 = 125.96 kWh, MEASURED. The derived capKwh below
#      (RngPerChrgAvg x VehElEffAvg) read 127.26 kWh at the same moment -- 1.0 % off, so it is now a
#      VALIDATED derivation rather than a guess, and it stays derived because it tracks the truck.
#   -> the BROADCAST SoC is the true SoC, so remaining energy needs no UDS at all.
#
# ---- MOVING_MS: accumulate only while actually driving (driver's explicit spec, 2026-09-20) -------
# Below this the truck draws accessories with no distance, which is not what a consumption figure
# used to predict range at a speed means. 10 mph is also the floor below which `energy / grossKw x
# speed` stops meaning anything: the window's average power was measured at road speed, and the truck
# does not draw it at walking pace, so extrapolating from it would be arithmetic, not a prediction.
MOVING_MPH = 10.0
MOVING_MS = MOVING_MPH / 2.23694          # 4.470 m/s
#
# ---- MIN_DSOC_PCT: the noise floor, in the units the quantisation actually lives in ---------------
# BattTracSoc2_Pc_Actl is 0.01 %/bit, so at the measured 125.96 kWh ONE LSB IS 12.6 Wh. Differencing
# two quantised readings carries up to +/-1 LSB of quantisation error regardless of how far apart they
# are, so the accumulated drop is what sets the accuracy:
#
#     accumulated drop | LSBs | worst-case quantisation error
#          0.05 %      |   5  |  +/-20 %      <- a figure built on 2-3 LSBs is noise wearing a number
#          0.10 %      |  10  |  +/-10 %
#          0.20 %      |  20  |   +/-5 %      <- CHOSEN
#          0.50 %      |  50  |   +/-2 %      <- would need ~2 min even at highway power
#
# 0.20 % = 0.252 kWh at 125.96 kWh. +/-5 % on the consumption is +/-5 % on the printed range, which is
# smaller than the spread between the truck's own three range estimates. Below the floor the producer
# publishes None -- NEVER a provisional value, because a provisional one is indistinguishable from a
# settled one on the screen.
MIN_DSOC_PCT = 0.20
#
# ---- WINDOW_MIN_S / WINDOW_MAX_S: long enough for the floor, short enough to still be "now" -------
# Time to accumulate 0.252 kWh is 907 / P seconds:
#
#       P = 40 kW (~70 mph highway)   ->  23 s
#       P = 20 kW (~45 mph arterial)  ->  45 s
#       P = 10 kW (~25 mph city)      ->  91 s
#       P =  6 kW (~12 mph crawl)     -> 151 s     (1.2 kW measured accessory load + traction)
#
# MAX 180 s covers the whole range down to ~5 kW, which is about the least this truck can draw while
# moving above 10 mph. Past 3 minutes the average stops describing the road you are on, and it is
# multiplied by the CURRENT speed, so a stale average is a wrong prediction rather than an old one.
# The cap is on WALL-CLOCK age precisely so that a long stop ages the window out.
#
# MIN 60 s is NOT the floor restated -- the window is shortened toward it whenever the floor still
# clears, so 60 s is what the figure settles to at normal road power. At 20 kW a 60 s window moves
# 0.2646 % = 26 LSB = +/-3.8 % quantisation: inside the +/-5 % the floor promises, and short enough to
# follow a change of road within a minute. Shortening further would peg the figure at the quantisation
# limit and make the printed range visibly jitter; not shortening at all would leave it 3 minutes
# behind reality at highway speed. Below ~15 kW the floor binds and the window grows past 60 s on its
# own, up to the 180 s cap.
WINDOW_MIN_S = 60.0
WINDOW_MAX_S = 180.0
#
# ---- SAMPLE_GAP_MAX_S: consecutive samples are PUBLISH_S apart; 5x that tolerates dropped cycles --
# but not a gap we did not watch (the module going quiet, a garbled-frame skip). An unwatched gap
# would fold energy spent at an unknown speed into the window.
SAMPLE_GAP_MAX_S = 1.0
#
# ---- NET_KW_MAX: the same plausibility discipline every other published field here gets ------------
# The BECM's SoC estimate can STEP -- it is an estimate, not a coulomb counter. A 5 % step down injects
# 6.3 kWh into the window and comes out as ~378 kW, which the UI would turn into a 9-mile range while
# the truck is perfectly healthy. 250 kW is above anything this truck sustains for a whole minute
# (peak output is ~430 kW, but that is a few-second burst; towing up a long grade at speed is ~100 kW),
# so a rolling average above it is a stepped estimate, not a measurement. Reported as None and logged.
NET_KW_MAX = 250.0


def _usable(seen: bool, value: float, band: tuple[float, float], nd: int) -> float | None:
  """everdrive2pnw: the decoded value rounded to its OWN resolution, or None if the message was never
  received or the value is a sentinel / encoding floor. None (JSON null) is the established in-repo
  "never received" marker -- same convention as cargps2pnw's `dr` / `drAge`. A 0.0 here would be a
  fabricated reading, because CANParser pre-fills every signal to 0.0 before the message has ever
  arrived.

  `nd` is the signal's DBC resolution, not a cosmetic choice: 1802 x 0.1 is 180.20000000000002 in
  binary floating point, and shipping that into a JSON mem-param claims precision the signal does
  not have (it is a 0.1 km/bit field)."""
  if not seen:
    return None
  return round(value, nd) if band[0] <= value <= band[1] else None


class EverDrive:
  """everdrive2pnw: publishes `EverDriveStatus` to /dev/shm while an EverDrive is broadcasting.

  THE KEY IS ABSENT WHEN NO EVERDRIVE IS FITTED. That is the Rule-2 contract with the UI, and it is
  deliberately not "present with acSeen false":

    key absent                      -> 0x2A7 has never been received (or has gone quiet): no module.
    key present, acKw 0.0, acSeen   -> 0x2A7 IS arriving and reports a genuine zero: module fitted,
                                       charger unplugged or idle. THIS IS A REAL MEASUREMENT.

  Collapsing those two is the exact failure mode ENERGY-RANGE-SIGNALS.md was written to avoid, and
  CANParser makes it easy to hit: `cp.vl[msg][sig]` returns 0.0 for a message that has NEVER been
  received, so the value alone cannot tell them apart. `cp.ts_nanos[msg][sig]` stays 0 until a frame
  has actually been parsed and then latches the receipt time, so it can -- and it is also what makes
  staleness computable. (Verified empirically 2026-09-19; `vl_all` is NOT an alternative, it is
  cleared at the top of every update() and so answers "arrived in THIS batch", not "this session".)

  `acSeen` is therefore INVARIANTLY True in any published payload. The key is kept because the spec
  asks for it and because it lets the UI assert the contract rather than assume it.
  """

  def __init__(self):
    # Nothing expensive, nothing imported, nothing registered globally. On a car with no EverDrive
    # this object costs one attribute check + one time.monotonic() per CarState.update().
    self._off = False                  # hard-off: the DBC has no 0x2A7, or Params is unavailable
    self._next_mono = 0.0              # monotonic deadline; the whole steady-state cost is one compare
    self._params = None
    self._live = False                 # is an EverDrive currently broadcasting?
    self._live_logged = False
    self._truck_gap_logged = False
    self._last_log_mono = -LOG_EVERY_S
    self._err = 0
    # everdrive2pnw: the rolling-consumption window. Cumulative snapshots (wall_mono, moving_s,
    # drop_kWh) so the window sums are two subtractions rather than a scan; `maxlen` is a hard memory
    # bound only -- samples arrive no faster than PUBLISH_S, so the wall-clock trim always bites first.
    # NOTHING HERE SURVIVES AN IGNITION CYCLE: `card` is only_onroad, so CarState -- and this object --
    # are rebuilt on every onroad transition, which is the ignition-change reset.
    self._win: deque[tuple[float, float, float]] = deque(maxlen=int(WINDOW_MAX_S / PUBLISH_S) + 2)
    self._cum_s = 0.0                  # seconds accumulated ABOVE MOVING_MS only
    self._cum_kwh = 0.0                # pack energy DROP over those seconds (may go down: regen)
    self._last_sample: tuple[float, float] | None = None   # (mono, socPct) of the previous sample

  def update(self, cp, v_ego: float) -> None:
    """Called once per CarState.update() (100 Hz) with the POWERTRAIN parser. Never raises."""
    if self._off:
      return
    now = time.monotonic()
    if now < self._next_mono:
      return
    # Past this point we do real work. Assume absent and come back slowly; the live path below
    # shortens the deadline to PUBLISH_S. Set FIRST so that every early return is also throttled.
    self._next_mono = now + AC_QUIET_S

    try:
      # cp.ts_nanos is a plain dict and raises KeyError for a message the parser does not carry --
      # i.e. a Ford on a DBC without 0x2A7. That can never change within a session, so it is the one
      # case it is correct to latch off permanently.
      ac_ns = cp.ts_nanos[AC_MSG]["EvrDrvAc_I_Actl"]
    except KeyError:
      self._off = True
      carlog.warning("everdrive2pnw: %s is not in this car's DBC; EverDriveStatus will never be " +
                     "published (feature off for this session)", AC_MSG)
      return
    except Exception:
      self._log_err()
      return

    # "Live" means a 0x2A7 frame arrived within AC_QUIET_S, NOT merely "arrived at some point". If
    # the module is unplugged mid-drive it stops transmitting and cp.vl freezes at its last value --
    # which would read as a live 1.4 kW forever. Differenced against the parser's OWN clock
    # (_last_update_nanos, the logMonoTime of the batch just consumed) because ts_nanos is stamped
    # from the same CLOCK_BOOTTIME source and it keeps this meaningful under REPLAY.
    # EVERYTHING FROM HERE IS INSIDE THE try (Fable review 2026-09-19). The docstring promises this
    # method never raises, and `card` dies -- taking the car with it -- if that promise is broken. The
    # liveness line below dereferences `cp._last_update_nanos`, a PRIVATE CANParser attribute: an
    # upstream rename would be an AttributeError escaping straight into CarState.update(). The publish
    # call was already guarded; the guard now starts one step earlier so the promise is actually true
    # rather than nearly true.
    try:
      live = ac_ns != 0 and (cp._last_update_nanos - ac_ns) <= AC_QUIET_NS
      if not live:
        if self._live:
          self._live = False
          self._log(("everdrive2pnw: 0x2A7 quiet for >%.0f s (last seen %.1f s ago) -- EverDrive " +
                     "stopped broadcasting; EverDriveStatus is no longer published")
                    % (AC_QUIET_S, (cp._last_update_nanos - ac_ns) / 1e9))
        return                         # PUBLISH NOTHING: an absent key is "no module fitted"

      if self._params is None and not self._open_params():
        return

      self._next_mono = now + PUBLISH_S
      self._publish(cp, v_ego, now)
    except Exception:
      self._log_err()

  def _publish(self, cp, v_ego: float, now: float) -> None:
    ac = cp.vl[AC_MSG]
    amps, volts = ac["EvrDrvAc_I_Actl"], ac["EvrDrvAc_U_Actl"]

    # everdrive2pnw: Rule 2 -- an implausible reading is NOT published as fact. 0x2A7 is aftermarket
    # with no counter and no checksum, so a garbled frame decodes silently into a huge number. Skip
    # this cycle instead: the key simply goes stale, the UI hides the box after its own _STALE_S, and
    # the next good frame republishes. Logged (rate-limited) so a persistently garbled module is
    # visible rather than looking like "the charger is unplugged".
    if not (AC_I_BAND[0] <= amps <= AC_I_BAND[1] and AC_U_BAND[0] <= volts <= AC_U_BAND[1]):
      self._log(("everdrive2pnw: 0x2A7 outside the physical band (%.3f A, %.1f V) -- frame " +
                 "ignored, EverDriveStatus not updated this cycle") % (amps, volts))
      return

    kw = (amps * volts) / 1000.0

    # ORDER IS LOAD-BEARING, DO NOT REORDER THESE BELOW THE cp.vl READS. cp.ts_nanos is a plain dict
    # and RAISES KeyError for an unregistered message; cp.vl is a VLDict whose __getitem__ LAZILY
    # REGISTERS one -- with freq=None, i.e. ALIVE-CHECKED, which is precisely how a display feature
    # turns into can_valid False and an undriveable car. Touching ts_nanos first means a message the
    # DBC probe skipped fails loudly into _log_err() instead of silently arming that hazard.
    ts_range = cp.ts_nanos[RANGE_MSG]["VehElRnge_L_Dsply"]
    ts_rpc = cp.ts_nanos[RPC_MSG]["RngPerChrgAvg_L_Dsply"]
    ts_eff = cp.ts_nanos[EFF_MSG]["VehElEffAvg_No_Dsply"]
    ts_soc = cp.ts_nanos[SOC_MSG]["BattTracSoc2_Pc_Actl"]

    # rounded to each signal's own DBC resolution: 0.1 km/bit, 10 Wh/km, 0.01 %/bit
    range_km = _usable(ts_range != 0, cp.vl[RANGE_MSG]["VehElRnge_L_Dsply"], RANGE_KM_BAND, 1)
    rpc_km = _usable(ts_rpc != 0, cp.vl[RPC_MSG]["RngPerChrgAvg_L_Dsply"], RPC_KM_BAND, 1)
    eff_wh_km = _usable(ts_eff != 0, cp.vl[EFF_MSG]["VehElEffAvg_No_Dsply"], EFF_WH_KM_BAND, 0)
    soc_pct = _usable(ts_soc != 0, cp.vl[SOC_MSG]["BattTracSoc2_Pc_Actl"], SOC_PCT_BAND, 2)

    # everdrive2pnw (2026-09-20): the pack's usable capacity, and the energy actually left in it.
    # See the CAP/MIN_DSOC block above for why the capacity is derived rather than hardcoded, and for
    # the 2026-09-20 UDS measurement that validated it to 1.0 %.
    cap_kwh = None if (rpc_km is None or eff_wh_km is None) else round(rpc_km * eff_wh_km / 1000.0, 2)
    energy_kwh = None if (soc_pct is None or cap_kwh is None) else round(soc_pct * cap_kwh / 100.0, 2)
    # ORDER: the window must be stepped on EVERY publish, including the ones where it reports None,
    # or it would only ever advance while it already had an answer.
    gross_kw = self._gross_kw(now, soc_pct, cap_kwh, v_ego, kw)

    self._params.put_nonblocking(PARAM_KEY, {
      # Wall clock, for the UI's staleness check. Rule 2: this advances even if the DECODE is frozen,
      # so it is a heartbeat for "the publisher is running", NOT evidence the numbers are fresh. The
      # freshness of the numbers is what the AC_QUIET_S gate above guarantees: this payload is only
      # written while 0x2A7 has arrived within the last 3 s.
      "ts": round(time.time(), 2),
      "acKw": round(kw, 3),
      # Invariantly True -- see the class docstring. Absence of the key is how "no module" is said.
      "acSeen": True,
      # None = that message has never been received, or reported a sentinel / encoding floor.
      # NEVER 0.0-as-a-guess; see _usable().
      "rangeKm": range_km,
      "effWhKm": eff_wh_km,
      # False whenever effWhKm is not a real measurement: never received, the raw-0 = -100 Wh/km
      # encoding FLOOR, or a NoDataExists / Faulty sentinel. (And even when True, it is the truck's
      # REFERENCE efficiency, not measured consumption -- see the module docstring.)
      "effOk": eff_wh_km is not None,
      "socPct": soc_pct,
      # everdrive2pnw (2026-09-20): the pack's usable capacity in kWh, DERIVED from the truck's own
      # two numbers -- RngPerChrgAvg (unadjusted full-charge range) x VehElEffAvg -- rather than a
      # hardcoded constant. Measured 2026-09-20: 396.9 km x 320 Wh/km = 127.0 kWh, against Ford's
      # stated 131 kWh usable (3% apart). Deriving it means the figure follows if the truck revises
      # its own estimate, and it stays self-consistent with the range shown beside it.
      #
      # VALIDATED 2026-09-20 to 1.0 % against a UDS read of 0x224848 Energy / 0x224801 HvbSoc
      # (59.350 kWh at 47.120 % => 125.96 kWh usable, against 127.26 kWh derived at the same moment).
      # It is still a 10 Wh/km-quantised product -- see test_one_lsb_of_efficiency_spans_fords_stated
      # _capacity -- so a 126-vs-131 "gap" is quantisation, not a degraded pack.
      "capKwh": cap_kwh,
      # everdrive2pnw (2026-09-20): energy actually left in the pack, kWh. The broadcast SoC IS the
      # true SoC (0x24C read 47.12 % against UDS HvbSoc 47.120 % on 2026-09-20), so this needs no UDS.
      # None whenever socPct or capKwh is None -- never a 0.0-as-a-guess.
      "energyKwh": energy_kwh,
      # everdrive2pnw (2026-09-20): rolling pack consumption with the EverDrive input ADDED BACK, kW.
      # GROSS, not net, and that is load-bearing: a SoC-derived figure is already net of whatever the
      # charger is feeding in, so handing the UI a net figure and then letting it apply its
      # `range * P/(P - acKw)` projection would count the charger TWICE. Gross is "what the truck
      # would be drawing with no EverDrive fitted", which is what that projection has always meant.
      # None until the window clears its floor -- see _gross_kw and the MIN_DSOC_PCT block.
      "grossKw": gross_kw,
      "vMs": round(float(v_ego), 2),
    })

    if not self._live:
      self._live = True
      if not self._live_logged:
        self._live_logged = True
        carlog.warning("everdrive2pnw: EverDrive live on 0x2A7 -- %.3f kW (%.2f A x %.1f V), " +
                       "rangeKm=%s effWhKm=%s socPct=%s (None = never received or sentinel)",
                       kw, ac["EvrDrvAc_I_Actl"], ac["EvrDrvAc_U_Actl"], range_km, eff_wh_km, soc_pct)

    # Rule 2: an EverDrive that IS talking on this bus means we are on the Lightning, so the truck's
    # own energy messages must be there too. If one is not, the DBC/bus assumption is wrong and that
    # has to be visible rather than showing up as a silently missing part of the display box.
    # RngPerChrgAvg is included (2026-09-20) because capKwh depends on it: without it the consumer
    # simply drops the kWh figure, which would otherwise be an invisible degradation.
    if not self._truck_gap_logged and (range_km is None or eff_wh_km is None or soc_pct is None
                                       or rpc_km is None):
      self._truck_gap_logged = True
      carlog.error("everdrive2pnw: 0x2A7 is live but a truck energy signal is not usable " +
                   "(rangeKm=%s ts=%d, effWhKm=%s ts=%d, socPct=%s ts=%d, rngPerChrgKm=%s ts=%d; " +
                   "ts 0 = NEVER RECEIVED, non-zero ts with a None value = sentinel or floor)",
                   range_km, ts_range, eff_wh_km, ts_eff, soc_pct, ts_soc, rpc_km, ts_rpc)

  def _gross_kw(self, now: float, soc_pct: float | None, cap_kwh: float | None,
                v_ego: float, ac_kw: float) -> float | None:
    """everdrive2pnw: rolling pack consumption in kW, GROSS of the EverDrive input, or None.

    Stepped on every publish (~5 Hz). Returns a number only when the truck is moving above MOVING_MS
    AND the window has accumulated a credible drop; otherwise None, which the UI reads as "fall back
    to the truck's own range". None is never a provisional value: a half-built window would print a
    number indistinguishable from a settled one.

    THE INCREMENT IS BUILT FROM dSoC x capacity, NOT from d(SoC x capacity). That is not a rearranged
    formula, it is the difference between a measurement and a fabricated spike: capKwh is a quantised
    product, and ONE LSB of VehElEffAvg (10 Wh/km) moves it by ~4 kWh. Differencing the product would
    turn a routine revision of the truck's own efficiency average into ~2 kWh of "consumption" in a
    single 0.2 s step -- about 100 kW of pure artifact, inside NET_KW_MAX and therefore published as
    fact. Differencing the SoC alone means a capacity revision changes only the SCALE of subsequent
    increments, which is what it actually is.
    """
    if soc_pct is None or cap_kwh is None:
      # Rule 2: a window whose inputs went away is not a current window. Drop it rather than let it
      # be reported later as though it had been measured continuously.
      self._win.clear()
      self._cum_s = self._cum_kwh = 0.0
      self._last_sample = None
      return None

    moving = v_ego >= MOVING_MS
    if moving:
      if self._last_sample is not None:
        dt = now - self._last_sample[0]
        if 0.0 < dt <= SAMPLE_GAP_MAX_S:
          # SoC RISES under regen on a long descent, and while an EverDrive is charging a moving
          # truck. That is a genuine negative increment and it belongs in the average -- it is only
          # the WINDOW TOTAL that must stay positive, which the floor check below enforces. Letting a
          # negative total through would put a negative kW into the UI's division.
          self._cum_s += dt
          self._cum_kwh += (self._last_sample[1] - soc_pct) / 100.0 * cap_kwh
          self._win.append((now, self._cum_s, self._cum_kwh))
      else:
        self._win.append((now, self._cum_s, self._cum_kwh))   # baseline for the next increment
      self._last_sample = (now, soc_pct)
    else:
      # Below MOVING_MS we stop sampling entirely, so the stopped interval contributes neither time
      # nor energy. Clearing _last_sample is what makes the resumption start a fresh increment
      # instead of one that silently spans the stop.
      self._last_sample = None

    # Hard cap on WALL-CLOCK age: this is also what ages the whole window out over a long stop.
    while self._win and (now - self._win[0][0]) > WINDOW_MAX_S:
      self._win.popleft()

    if not moving or not self._win:
      return None
    # Pick the NEWEST baseline that still clears both constraints -- i.e. the shortest window that is
    # long enough -- WITHOUT discarding the older snapshots. Popping them instead (the obvious
    # implementation, and the first one written here) makes the shortening IRREVERSIBLE: the window
    # settles at the 60 s boundary, and the moment consumption dips the floor no longer clears and it
    # cannot grow back into history it has thrown away. Replayed against route 000001b8--a46fe398b3
    # that produced a figure appearing and vanishing in 10 blocks of median 14 s across 14 minutes,
    # which on screen is the first number flickering between two different quantities. Keeping the
    # history costs a bounded forward scan and settles it: 5 blocks of median 72 s, and coverage up
    # from 23 % to 37 % of that route -- against a ceiling of 71 %, which is simply how much of a
    # stop-and-go city drive was spent above 10 mph at all.
    # Conservative where regen makes cum_kwh non-monotonic: the scan stops at the first baseline that
    # fails, so it may use a longer window than strictly necessary. Longer is the safe direction.
    #
    # ⚠️ NOT COVERED BY A TEST, AND SAID SO RATHER THAN QUIETLY LEFT (mutation P14, 2026-09-20).
    # Replacing this scan with `popleft()` leaves all 87 tests in test_everdrive_pnw.py green. The two
    # forms are output-identical on every synthetic profile tried -- constant power, power steps,
    # regen bursts, coasts, stop-and-go cycles, capacity revisions -- because popping only hurts once
    # the discarded history is needed AGAIN, and on a clean profile the next samples always restore
    # the margin first. It separates only on the real, noisy article. Instrumented at the first
    # divergence on route 000001b8--a46fe398b3 (t = 195.2 s):
    #     baseline: 692 snapshots, oldest 180.0 s, window 137.6 s / 0.6812 kWh  -> 15.42 kW
    #     popping : 303 snapshots, oldest  60.4 s, window  60.4 s / 0.2522 kWh  -> None (floor
    #                                                                             0.2522, under by eps)
    # i.e. popping pins the window EXACTLY on the floor and then has no margin for the next dip.
    # Reproduce with the replay harness, not with pytest. If you are tempted to "simplify" this back
    # to a popleft, run the route first.
    floor_kwh = MIN_DSOC_PCT / 100.0 * cap_kwh
    i = 0
    while (i + 1 < len(self._win) and (self._cum_s - self._win[i + 1][1]) >= WINDOW_MIN_S
           and (self._cum_kwh - self._win[i + 1][2]) >= floor_kwh):
      i += 1
    win_s = self._cum_s - self._win[i][1]
    win_kwh = self._cum_kwh - self._win[i][2]
    if win_s < WINDOW_MIN_S or win_kwh < floor_kwh:
      return None                      # includes the SoC-went-UP case: win_kwh <= 0 < floor_kwh

    net_kw = win_kwh * 3600.0 / win_s
    if net_kw > NET_KW_MAX:
      self._log(("everdrive2pnw: rolling consumption %.0f kW over %.0f s exceeds the %.0f kW " +
                 "plausibility ceiling -- the pack SoC estimate stepped; no consumption reported")
                % (net_kw, win_s, NET_KW_MAX))
      return None
    return round(net_kw + ac_kw, 2)

  def _open_params(self) -> bool:
    """everdrive2pnw: build the /dev/shm handle ON FIRST PRESENCE, not at construction, so a truck
    with no EverDrive never opens one at all. Runtime import, so a bare opendbc checkout (no
    openpilot on the path) simply turns the feature off instead of failing to import."""
    try:
      from openpilot.common.params import Params
      self._params = Params("/dev/shm/params")
      return True
    except Exception:
      # Permanent: if openpilot.common.params cannot be imported or /dev/shm/params cannot be opened
      # now, it will not start working later, and retrying at 5 Hz would be the per-frame cost this
      # design exists to avoid.
      self._off = True
      carlog.exception("everdrive2pnw: could not open /dev/shm params; EverDrive telemetry is OFF " +
                       "for this session (the car is unaffected)")
      return False

  def _log(self, line: str) -> None:
    now = time.monotonic()
    if now - self._last_log_mono < LOG_EVERY_S:
      return
    self._last_log_mono = now
    carlog.warning(line)

  def _log_err(self) -> None:
    """Rule 2: the catch has to stay -- an exception escaping CarState.update() would take down
    `card` and the truck for a display box -- but it must NOT be silent. This project has already
    shipped one telemetry feature that published nothing at all behind an `except: pass` and showed
    no error anywhere. Rate-limited, mirroring carstate's _cargps_err / _ppo_err."""
    self._err += 1
    if self._err == 1 or self._err % 300 == 0:
      carlog.exception("everdrive2pnw: telemetry publish failed (%d so far); EverDriveStatus is " +
                       "stale or absent. The car is unaffected.", self._err)

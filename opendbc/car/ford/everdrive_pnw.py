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
REFERENCE efficiency used for range estimation (~the EPA figure), not a rolling average. A projection
built on it is "range this supply buys at the EPA rate", not "at the rate you are actually driving".
That document also records (§3) that NO broadcast signal gives real vehicle consumption.
"""
import time

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
ENERGY_MSGS = (AC_MSG, RANGE_MSG, EFF_MSG, SOC_MSG)

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
RANGE_KM_BAND = (0.0, 409.3)      # 409.3 = the DBC's own stated max, below the two sentinels
SOC_PCT_BAND = (0.0, 100.0)       # a real SoC cannot exceed 100 %
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
      self._publish(cp, v_ego)
    except Exception:
      self._log_err()

  def _publish(self, cp, v_ego: float) -> None:
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
    ts_eff = cp.ts_nanos[EFF_MSG]["VehElEffAvg_No_Dsply"]
    ts_soc = cp.ts_nanos[SOC_MSG]["BattTracSoc2_Pc_Actl"]

    # rounded to each signal's own DBC resolution: 0.1 km/bit, 10 Wh/km, 0.01 %/bit
    range_km = _usable(ts_range != 0, cp.vl[RANGE_MSG]["VehElRnge_L_Dsply"], RANGE_KM_BAND, 1)
    eff_wh_km = _usable(ts_eff != 0, cp.vl[EFF_MSG]["VehElEffAvg_No_Dsply"], EFF_WH_KM_BAND, 0)
    soc_pct = _usable(ts_soc != 0, cp.vl[SOC_MSG]["BattTracSoc2_Pc_Actl"], SOC_PCT_BAND, 2)

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
    # own three energy messages must be there too. If one is not, the DBC/bus assumption is wrong and
    # that has to be visible rather than showing up as a silently missing half of the display box.
    if not self._truck_gap_logged and (range_km is None or eff_wh_km is None or soc_pct is None):
      self._truck_gap_logged = True
      carlog.error("everdrive2pnw: 0x2A7 is live but a truck energy signal is not usable " +
                   "(rangeKm=%s ts=%d, effWhKm=%s ts=%d, socPct=%s ts=%d; ts 0 = NEVER RECEIVED, " +
                   "non-zero ts with a None value = sentinel or encoding floor)",
                   range_km, ts_range, eff_wh_km, ts_eff, soc_pct, ts_soc)

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

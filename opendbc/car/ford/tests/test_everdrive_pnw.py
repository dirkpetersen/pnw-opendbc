"""everdrive2pnw: the AFTERMARKET EverDrive AC meter (0x2A7) + the truck's own energy snapshot.

Display-only telemetry published to /dev/shm as `EverDriveStatus`. These tests exist because a
DISPLAY feature in the car port has exactly two ways to hurt somebody, and both are silent:

  1. it can make the car UNDRIVEABLE (can_valid -> ret.canValid -> offroad) by registering a message
     the car never transmits -- see TestCanValidIsNeverRiskedForADisplayBox, the most important test
     in this file;
  2. it can print a CONFIDENT WRONG NUMBER, because every failure mode of this data decodes to
     something physically plausible -- a never-received CANParser signal reads 0.0, a NoDataExists
     sentinel reads 254 mi, an encoding floor reads -100 Wh/km, and a module that stopped
     transmitting reads its last value forever.

Every frame here goes through the real DBC, the real CANPacker/CANParser and (where it matters) the
real Ford CarInterface.update -- the same path `card` runs.

Measured ground truth (VIN 1FT6W3L78SWG05094, 2026-09-19, see
CANbus/ford/f-150/lightning/2024-25/ENERGY-RANGE-SIGNALS.md):
  0x2A7 payload 07 D0 00 00 03 68 00 00 while charging = 12.50 A / 109.0 V = 1.3625 kW
  0x2A7 payload 00 00 00 00 00 00 00 00 x 865 consecutive frames while unplugged = a REAL zero
"""
import math

import pytest

from opendbc.can import CANPacker
from opendbc.can.dbc import DBC as DbcFile, Signal
from opendbc.can.parser import CANParser, get_raw_value
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.carlog import carlog
from opendbc.car.ford import everdrive_pnw as ed
from opendbc.car.ford.carstate import CarState
from opendbc.car.ford.values import CAR, DBC, FordFlags

LIGHTNING = CAR.FORD_F_150_LIGHTNING_MK1
DBC_NAME = DBC[LIGHTNING][Bus.pt]
DT = 10_000_000                  # 100 Hz, as card sees it

AC_ADDR = 0x2A7
RANGE_ADDR = 0x442
EFF_ADDR = 0x36D
SOC_ADDR = 0x24C

# The LITERAL payloads off the truck. Not reconstructed from a packer -- the bytes as captured.
CHARGING = bytes.fromhex("07d0000003680000")
UNPLUGGED = bytes(8)


# ---------------------------------------------------------------- harness


class Capture:
  """Stands in for Params("/dev/shm/params"). Injected BEFORE the first update so _open_params --
  and therefore the real openpilot import -- is never reached."""

  def __init__(self, raises: Exception | None = None):
    self.blobs: list[dict] = []
    self.raises = raises

  def put_nonblocking(self, key, val):
    assert key == "EverDriveStatus", key
    if self.raises is not None:
      raise self.raises
    self.blobs.append(dict(val))


class Clock:
  """everdrive_pnw gates itself on time.monotonic()/time.time(), but these tests drive SIMULATED CAN
  time. Without a clock the publisher would fire once and then sit out the whole test behind its
  3 s deadline while 20 s of CAN time went by. Advanced in lockstep with the CAN timestamps."""

  def __init__(self, t0: float = 1_000_000.0):
    self.t = t0

  def monotonic(self) -> float:
    return self.t

  def time(self) -> float:
    return self.t


class Truck:
  """A real Lightning CarInterface, a real 100 Hz Ford bus, and a captured EverDrive publisher."""

  def __init__(self, monkeypatch, params=None):
    self.clock = Clock()
    monkeypatch.setattr(ed, "time", self.clock)
    CarInterface = interfaces[LIGHTNING]
    fp = gen_empty_fingerprint()
    fp[0][0x5A] = 8                     # Gear_Shift_by_Wire_FD1 -> the automatic branch
    self.CI = CarInterface(CarInterface.get_params(LIGHTNING, fp, [], False, False, False))
    self.cap = params if params is not None else Capture()
    self.CI.CS._everdrive._params = self.cap
    self.packer = CANPacker(DBC_NAME)
    self.t = 0
    self.i = 0
    self.cs = self.CI.update([(self.t, [])])      # first update lazily registers the normal messages
    self.pt = self.CI.can_parsers[Bus.pt]
    self.cam = self.CI.can_parsers[Bus.cam]
    self.stream = self._alive_stream()

  @property
  def ed(self) -> ed.EverDrive:
    return self.CI.CS._everdrive

  def _alive_stream(self) -> list[CanData]:
    """Realistic Ford traffic: every message the parsers actually ALIVE-CHECK, on its own bus.

    Derived from the parsers rather than hardcoded, so it cannot drift away from what CarState
    reads. The four energy messages are registered ignore_alive and are therefore excluded BY
    CONSTRUCTION -- asserted explicitly below, because "the stream happens to be missing them" is
    the whole premise of TestCanValidIsNeverRiskedForADisplayBox."""
    frames = []
    for parser, bus in ((self.pt, 0), (self.cam, 2)):
      for addr, state in parser.message_states.items():
        if state.ignore_alive:
          continue
        a, dat, b = self.packer.make_can_msg(parser.dbc.addr_to_msg[addr].name, bus, {})
        frames.append(CanData(a, dat, b))
    return frames

  def frame(self, msg: str, values: dict, bus: int = 0) -> CanData:
    return CanData(*self.packer.make_can_msg(msg, bus, values))

  def energy(self, range_km=None, eff_wh_km=None, soc_pct=None) -> list[CanData]:
    """The truck's own three energy frames, packed from PHYSICAL values (the packer converts to raw
    with the DBC's own factor/offset, so e.g. 409.4 km == raw 4094 == NoDataExists)."""
    out = []
    if range_km is not None:
      out.append(self.frame("MtrTrac_Data2_FD1", {"VehElRnge_L_Dsply": range_km}))
    if eff_wh_km is not None:
      out.append(self.frame("HEV_Powertrain_Data7_FD1", {"VehElEffAvg_No_Dsply": eff_wh_km}))
    if soc_pct is not None:
      out.append(self.frame("Battery_Traction_4_FD1", {"BattTracSoc2_Pc_Actl": soc_pct}))
    return out

  def tick(self, *extra: CanData):
    self.t += DT
    self.clock.t += DT / 1e9
    self.i += 1
    self.cs = self.CI.update([(self.t, self.stream + list(extra))])
    return self.cs

  def run(self, seconds: float, ac: bytes | None = None, energy: list[CanData] | None = None,
          speed_kph: float | None = None):
    """`ac` is the 0x2A7 payload, sent at its real 1 Hz. `energy` frames go at 1 Hz too."""
    slow = list(energy or [])
    if ac is not None:
      slow.append(CanData(AC_ADDR, ac, 0))
    fast = []
    if speed_kph is not None:
      fast.append(self.frame("BrakeSysFeatures", {"Veh_V_ActlBrk": speed_kph}))
    for _ in range(int(round(seconds * 100))):
      self.tick(*(fast + (slow if self.i % 100 == 0 else [])))
    return self.cs


@pytest.fixture
def carlogs(monkeypatch):
  """Only this feature's lines. The parser's own rate-limited "not valid" warnings share carlog."""
  seen = []

  def capture(level):
    def log(msg, *a, **kw):
      if "everdrive2pnw" in str(msg):
        seen.append((level, str(msg), a))
    return log
  for level in ("warning", "error", "exception"):
    monkeypatch.setattr(carlog, level, capture(level))
  return seen


# ---------------------------------------------------------------- T1


class TestCanValidIsNeverRiskedForADisplayBox:
  """THE test. A Ford that never transmits 0x2A7 must stay driveable.

  CANParser's VLDict.__getitem__ LAZILY REGISTERS an unindexed message (parser.py `_add_message`,
  reached from `VLDict.__getitem__`) with freq=None. `_add_message` computes
  `ignore_alive = freq is not None and math.isnan(freq)`, so freq=None gives ignore_alive=False and
  `timeout_threshold = (1e9 / 1) * 10` -- a 10 s alive check on a message the car does not have.
  MessageState.valid() then returns False forever ("if not self.timestamps: return False"),
  can_valid goes False, interfaces.py ANDs it into ret.canValid, and the truck is offroad because of
  a display box.

  The fix is registering the four messages through carstate.get_can_parsers' existing float("nan")
  probe, which is the ONLY input that sets ignore_alive=True."""

  @staticmethod
  def _CP():
    CP = structs.CarParams()
    CP.carFingerprint = LIGHTNING
    CP.safetyConfigs = [structs.CarParams.SafetyConfig()]
    CP.flags = int(FordFlags.CANFD)
    return CP

  def test_all_four_energy_messages_are_registered_ignore_alive(self):
    """Structural: the nan probe is what makes them ignore_alive, and nothing else does."""
    pt = CarState.get_can_parsers(self._CP())[Bus.pt]
    assert set(ed.ENERGY_MSGS) == {ed.AC_MSG, ed.RANGE_MSG, ed.EFF_MSG, ed.SOC_MSG}
    assert [pt.dbc.name_to_msg[m].address for m in ed.ENERGY_MSGS] == \
           [AC_ADDR, RANGE_ADDR, EFF_ADDR, SOC_ADDR]        # 0x2A7 / 0x442 / 0x36D / 0x24C
    for name in ed.ENERGY_MSGS:
      addr = pt.dbc.name_to_msg[name].address
      assert addr in pt.addresses, f"{name} must be registered up front, not lazily"
      state = pt.message_states[addr]
      assert state.ignore_alive, \
        f"{name} is ALIVE-CHECKED: a Ford that never transmits it goes can_valid False -> UNDRIVEABLE"
      assert math.isnan(float("nan")) and state.timeout_threshold > 0   # freq came from the nan probe

  def test_twenty_seconds_of_real_ford_traffic_with_no_energy_messages_stays_valid(self, monkeypatch, carlogs):
    """End to end through the real CarInterface: > 15 s (the 10 s lazy timeout plus margin) of a
    realistic Ford bus that contains NONE of the four energy messages."""
    truck = Truck(monkeypatch)
    energy_addrs = {truck.pt.dbc.name_to_msg[m].address for m in ed.ENERGY_MSGS}
    sent = {f.address for f in truck.stream}
    assert not (sent & energy_addrs), "the stream must contain none of the four energy messages"
    assert len(sent) >= 15, f"a 'realistic Ford stream' of {len(sent)} messages is not realistic"

    for _ in range(2000):                                   # 20 s at 100 Hz
      cs = truck.tick()
      assert cs.canValid, f"an absent EverDrive took the truck offroad at t={truck.i / 100:.2f} s"
    assert truck.cap.blobs == [], "nothing may be published when 0x2A7 has never been received"
    assert truck.ed._off is False, "the feature must stay armed (the charger can be plugged in later)"

  def test_control_the_same_silence_DOES_invalidate_a_lazily_registered_parser(self, monkeypatch):
    """Without this control the test above proves nothing -- it would pass on a parser that cannot
    go invalid at all. This is the PRE-FIX shape: the probe registers only GPS+PPO, and a naive
    display feature reads cp.vl["EverDrive_AC_Meter_FD1"], which lazily registers it."""
    truck = Truck(monkeypatch)
    known = DbcFile(DBC_NAME).name_to_msg
    pre_fix = [(m, float("nan")) for m in CarState.GPS_MSGS + CarState.PPO_MSGS if m in known]
    naive = CANParser(DBC_NAME, pre_fix, 0)
    assert naive.can_valid, "positive control: a parser of only nan messages starts valid"

    _ = naive.vl[ed.AC_MSG]                                 # exactly what the naive display code did
    addr = naive.dbc.name_to_msg[ed.AC_MSG].address
    assert addr in naive.addresses, "cp.vl[...] must have lazily registered it"
    assert not naive.message_states[addr].ignore_alive, "lazy registration is ALIVE-CHECKED"

    last = True
    t = 0
    for _ in range(2000):                                   # the same 20 s of the same traffic
      t += DT
      naive.update([[t, [(f.address, f.dat, f.src) for f in truck.stream]]])
      last = naive.can_valid
    assert not last, "control failed: the lazily-registered message must invalidate the parser"


# ---------------------------------------------------------------- T2


class TestPresenceIsDecidedByTimestampNeverByValue:
  """CANParser PRE-FILLS every signal to 0.0 before the message has ever arrived, so the VALUE
  cannot tell "no module fitted" from "module fitted, charger idle". cp.ts_nanos stays 0 until a
  frame is actually parsed. Three states, never to be collapsed:

    never received      -> the param key is NEVER WRITTEN
    all-zero payload    -> key written, acKw 0.0, acSeen True   <- A REAL MEASUREMENT
    charging payload    -> key written, acKw 1.3625
  """

  def test_never_received_publishes_nothing_at_all(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    truck.run(10.0, ac=None, energy=truck.energy(180.2, 320.0, 49.99))
    assert truck.cap.blobs == [], "no 0x2A7 => the key must be absent entirely, not acSeen: false"
    assert carlogs == [], "an absent EverDrive is the NORMAL steady state and must not log"

  def test_never_received_never_even_opens_a_dev_shm_handle(self, monkeypatch, carlogs):
    """The driver's zero-cost requirement: a truck with no module must not open Params at all."""
    truck = Truck(monkeypatch)
    truck.ed._params = None
    calls = []
    monkeypatch.setattr(ed.EverDrive, "_open_params", lambda self: calls.append(1) or False)
    truck.run(10.0, ac=None)
    assert calls == [], "_open_params must not run until an EverDrive frame has been received"

  def test_all_zero_payload_is_a_real_measured_zero_and_IS_published(self, monkeypatch, carlogs):
    """865 consecutive all-zero frames were captured with the charger unplugged. The meter is live
    and reads zero: that is data, and it must reach the UI as acKw 0.0 / acSeen True."""
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=UNPLUGGED, energy=truck.energy(180.2, 320.0, 49.99))
    assert truck.cap.blobs, "an all-zero 0x2A7 is a MEASUREMENT -- it must still publish"
    last = truck.cap.blobs[-1]
    assert last["acKw"] == 0.0
    assert last["acSeen"] is True

  def test_charging_payload_publishes_the_measured_power(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(180.2, 320.0, 49.99))
    assert truck.cap.blobs
    last = truck.cap.blobs[-1]
    assert last["acKw"] == pytest.approx(1.3625, abs=5e-4)
    assert last["acSeen"] is True

  def test_the_three_states_are_all_distinguishable(self, monkeypatch, carlogs):
    """The one assertion the whole design rests on: absent != zero != charging."""
    absent = Truck(monkeypatch)
    absent.run(6.0, ac=None)
    idle = Truck(monkeypatch)
    idle.run(6.0, ac=UNPLUGGED)
    live = Truck(monkeypatch)
    live.run(6.0, ac=CHARGING)

    assert absent.cap.blobs == []
    assert idle.cap.blobs and idle.cap.blobs[-1]["acKw"] == 0.0
    assert live.cap.blobs and live.cap.blobs[-1]["acKw"] > 1.0
    # and the value alone cannot tell the first two apart -- which is why ts_nanos is used
    assert absent.pt.vl[ed.AC_MSG]["EvrDrvAc_I_Actl"] == idle.pt.vl[ed.AC_MSG]["EvrDrvAc_I_Actl"] == 0.0
    assert absent.pt.ts_nanos[ed.AC_MSG]["EvrDrvAc_I_Actl"] == 0
    assert idle.pt.ts_nanos[ed.AC_MSG]["EvrDrvAc_I_Actl"] != 0

  def test_publishes_at_five_hz_while_live(self, monkeypatch, carlogs):
    """PUBLISH_S = 0.2. Pinned because a publisher that ran at CarState.update's 100 Hz would be a
    real cost, and one that ran at AC_QUIET_S would make the box visibly laggy."""
    truck = Truck(monkeypatch)
    truck.run(13.0, ac=CHARGING)                           # ~3 s to arm, then ~10 s of publishing
    assert 45 <= len(truck.cap.blobs) <= 55, f"expected ~5 Hz, got {len(truck.cap.blobs)}"
    gaps = {round(b["ts"] - a["ts"], 3) for a, b in zip(truck.cap.blobs, truck.cap.blobs[1:], strict=False)}
    assert gaps == {0.2}, f"publish spacing must be exactly PUBLISH_S, saw {sorted(gaps)}"

  def test_vms_is_the_filtered_vEgo_the_car_reported(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    cs = truck.run(20.0, ac=CHARGING, speed_kph=100.0)
    assert cs.vEgo > 20.0, "positive control: the truck must actually be moving"
    assert truck.cap.blobs[-1]["vMs"] == pytest.approx(cs.vEgo, abs=0.01)


# ---------------------------------------------------------------- T3


class TestLivenessNotEverSeen:
  """cp.vl FREEZES at the last decoded value when a message stops arriving. An EverDrive unplugged
  mid-drive would otherwise republish 12.5 A / 109.0 V -- a confident, live-looking 1.4 kW --
  forever. Presence must mean "arrived within AC_QUIET_S", not "arrived at some point"."""

  def test_publishing_stops_after_the_module_goes_quiet(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING)
    assert truck.cap.blobs, "positive control: it was publishing while the module talked"
    frozen = truck.pt.vl[ed.AC_MSG]["EvrDrvAc_I_Actl"]
    assert frozen == pytest.approx(12.5), "the parser holds the last value, which is the hazard"

    n = len(truck.cap.blobs)
    truck.run(10.0, ac=None)                                # unplugged: 0x2A7 stops
    grew = len(truck.cap.blobs) - n
    assert truck.pt.vl[ed.AC_MSG]["EvrDrvAc_I_Actl"] == frozen, "cp.vl still reads 12.5 A"
    # AC_QUIET_S of grace at 5 Hz is at most ~15 more payloads; after that, silence.
    assert grew <= 20, f"{grew} payloads published from a module that stopped transmitting"
    tail = truck.cap.blobs[-1]["ts"]
    truck.run(10.0, ac=None)
    assert truck.cap.blobs[-1]["ts"] == tail, "a quiet EverDrive must publish NOTHING further"

  def test_going_quiet_is_logged_once(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING)
    truck.run(20.0, ac=None)
    quiet = [m for lvl, m, a in carlogs if "stopped broadcasting" in m]
    assert len(quiet) == 1, f"expected exactly one 'stopped broadcasting' line, got {len(quiet)}"

  def test_absence_does_not_latch_and_the_box_comes_back(self, monkeypatch, carlogs):
    """The charger can be unplugged and plugged back in mid-drive."""
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING)
    truck.run(10.0, ac=None)
    stalled = len(truck.cap.blobs)
    truck.run(6.0, ac=CHARGING)
    assert len(truck.cap.blobs) > stalled + 10, "publishing must RESUME when frames come back"
    assert truck.cap.blobs[-1]["acKw"] == pytest.approx(1.3625, abs=5e-4)

  def test_a_charger_plugged_in_mid_drive_appears_within_a_few_seconds(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    truck.run(30.0, ac=None)
    assert truck.cap.blobs == []
    truck.run(5.0, ac=CHARGING)
    assert truck.cap.blobs, "a module that starts talking mid-session must be picked up"


# ---------------------------------------------------------------- T4


class TestSentinelsAndEncodingFloorsAreNoneNeverNumbers:
  """Each of these decodes to a physically plausible but WRONG number:
       VehElRnge_L_Dsply    raw 4094 NoDataExists -> 409.4 km = 254 mi of range that does not exist
       BattTracSoc2_Pc_Actl raw 16382 NoDataExists -> 163.82 %
       VehElEffAvg_No_Dsply raw 0 -> -100 Wh/km, the ENCODING FLOOR ("not available")
  Shipping any of them as a number is the "plausible value that is wrong" failure this project keeps
  paying for. They must come back None (JSON null), and effOk must go False."""

  def _publish(self, monkeypatch, **energy):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(**energy))
    assert truck.cap.blobs, "nothing published -- the rest of this test would be vacuous"
    return truck.cap.blobs[-1]

  def test_normal_values_round_trip(self, monkeypatch, carlogs):
    """The positive control. Without it "everything is None" would pass this whole class."""
    last = self._publish(monkeypatch, range_km=180.2, eff_wh_km=320.0, soc_pct=49.99)
    assert last["rangeKm"] == pytest.approx(180.2)        # the driver-confirmed 112 mi dash reading
    assert last["effWhKm"] == pytest.approx(320.0)
    assert last["effOk"] is True
    assert last["socPct"] == pytest.approx(49.99)

  def test_range_NoDataExists_is_None_not_254_miles(self, monkeypatch, carlogs):
    last = self._publish(monkeypatch, range_km=409.4, eff_wh_km=320.0, soc_pct=49.99)
    assert 409.4 / 1.60934 == pytest.approx(254.4, abs=0.1)     # what it WOULD have printed
    assert last["rangeKm"] is None
    assert last["effWhKm"] == pytest.approx(320.0), "one bad signal must not poison the others"

  def test_range_Fault_is_None(self, monkeypatch, carlogs):
    assert self._publish(monkeypatch, range_km=409.5, eff_wh_km=320.0, soc_pct=49.99)["rangeKm"] is None

  def test_range_at_the_declared_maximum_is_still_a_real_reading(self, monkeypatch, carlogs):
    """409.3 is the DBC's own stated max, one bit below the first sentinel. Not a sentinel."""
    assert self._publish(monkeypatch, range_km=409.3, eff_wh_km=320.0,
                         soc_pct=49.99)["rangeKm"] == pytest.approx(409.3)

  def test_soc_NoDataExists_is_None_not_163_percent(self, monkeypatch, carlogs):
    last = self._publish(monkeypatch, range_km=180.2, eff_wh_km=320.0, soc_pct=163.82)
    assert last["socPct"] is None
    assert last["rangeKm"] == pytest.approx(180.2)

  def test_soc_Faulty_is_None(self, monkeypatch, carlogs):
    assert self._publish(monkeypatch, range_km=180.2, eff_wh_km=320.0, soc_pct=163.83)["socPct"] is None

  def test_soc_at_one_hundred_percent_is_a_real_reading(self, monkeypatch, carlogs):
    assert self._publish(monkeypatch, range_km=180.2, eff_wh_km=320.0,
                         soc_pct=100.0)["socPct"] == pytest.approx(100.0)

  def test_efficiency_encoding_floor_is_None_and_effOk_is_False(self, monkeypatch, carlogs):
    """raw 0 decodes to -100 Wh/km through (10, -100). That is "not available", not a measurement,
    and a UI that believed it would divide by a negative efficiency."""
    last = self._publish(monkeypatch, range_km=180.2, eff_wh_km=-100.0, soc_pct=49.99)
    assert last["effWhKm"] is None
    assert last["effOk"] is False
    assert last["rangeKm"] == pytest.approx(180.2), "the range half of the box must survive"

  def test_efficiency_NoDataExists_is_None_and_effOk_is_False(self, monkeypatch, carlogs):
    last = self._publish(monkeypatch, range_km=180.2, eff_wh_km=1160.0, soc_pct=49.99)   # raw 126
    assert last["effWhKm"] is None and last["effOk"] is False

  @pytest.mark.parametrize("eff_wh_km", [-90.0, -50.0, -10.0])
  def test_a_NEGATIVE_efficiency_is_None_and_effOk_is_False(self, monkeypatch, carlogs, eff_wh_km):
    """Added 2026-09-19 to kill mutation M10f, which survived the first pass.

    VehElEffAvg_No_Dsply has offset -100, so raws 1..9 decode to -90..-10 Wh/km: inside the signal's
    declared range, not a sentinel, and (before this band) reported as a real measurement. `effOk`
    means "this is a usable measurement", and a negative average Wh/km is not one. It also reaches
    the consumer as BOTH a divisor (the gain rate) and a multiplier (assumed power), so a negative
    would come back as a confidently-signed wrong answer rather than an obvious one.

    The UI rejects eff <= 0 independently, so this is defence in depth -- but the producer is where
    the claim "this is a measurement" is made, so it is where the band belongs."""
    last = self._publish(monkeypatch, range_km=180.2, eff_wh_km=eff_wh_km, soc_pct=49.99)
    assert last["effWhKm"] is None, f"{eff_wh_km} Wh/km was published as a real efficiency"
    assert last["effOk"] is False
    assert last["rangeKm"] == pytest.approx(180.2), "the other fields must be unaffected"

  def test_a_never_received_truck_message_is_None_not_zero(self, monkeypatch, carlogs):
    """A CANParser signal that has never arrived reads a PRE-FILLED 0.0 -- not its decoded floor,
    the literal 0.0 (parser.py `_add_message`: `{s: 0.0 for s in signal_names}`). A never-received SoC
    therefore reads 0 %, and a never-received range reads 0 km, BOTH OF WHICH ARE INSIDE THEIR
    PLAUSIBILITY BANDS. The bands cannot catch this; only ts_nanos can.

    (EFF_WH_KM_BAND's lower bound was tightened to +0.1 on 2026-09-19 so `effOk` cannot be True for a
    negative average Wh/km, which incidentally also excludes the 0.0 pre-fill. That is a HAPPY
    ACCIDENT and must not be mistaken for the protection: SoC and range still admit 0.0, and mutation
    M4d -- deleting `_usable`'s `seen` gate -- is what proves the ts_nanos check is load-bearing.)"""
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(range_km=180.2))   # eff + soc never transmitted
    last = truck.cap.blobs[-1]
    assert truck.pt.vl[ed.EFF_MSG]["VehElEffAvg_No_Dsply"] == 0.0     # the pre-fill that lies ...
    assert truck.pt.vl[ed.SOC_MSG]["BattTracSoc2_Pc_Actl"] == 0.0
    assert ed.SOC_PCT_BAND[0] <= 0.0 <= ed.SOC_PCT_BAND[1]            # ... and its band accepts it
    assert ed.RANGE_KM_BAND[0] <= 0.0 <= ed.RANGE_KM_BAND[1]          # ... as does range's
    assert truck.pt.ts_nanos[ed.EFF_MSG]["VehElEffAvg_No_Dsply"] == 0
    assert last["rangeKm"] == pytest.approx(180.2)
    assert last["effWhKm"] is None and last["effOk"] is False
    assert last["socPct"] is None, "a never-received SoC must be None, never a confident 0 %"

  def test_a_missing_truck_signal_is_logged_loudly(self, monkeypatch, carlogs):
    """Rule 2: if 0x2A7 is live we are on the Lightning, so the truck's own energy messages must be
    there too. A gap means the bus/DBC assumption is wrong and has to be visible."""
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(range_km=180.2))
    gaps = [m for lvl, m, a in carlogs if lvl == "error" and "truck energy signal is not usable" in m]
    assert len(gaps) == 1, f"expected exactly one gap report, got {len(gaps)}"

  def test_values_are_rounded_to_their_own_dbc_resolution(self, monkeypatch, carlogs):
    """0.1 km/bit: 1802 * 0.1 is 180.20000000000002 in binary floating point. Shipping that claims
    precision the signal does not have."""
    last = self._publish(monkeypatch, range_km=180.2, eff_wh_km=320.0, soc_pct=49.99)
    assert repr(last["rangeKm"]) == "180.2"
    assert last["socPct"] == 49.99


# ---------------------------------------------------------------- T5


class TestDbcDecodeIsExact:
  """The start bits are the one thing here with no safety net: every neighbouring byte offset also
  decodes, also without raising, and also to a number a human would accept."""

  PAYLOAD = CHARGING          # 07 D0 00 00 03 68 00 00, captured while charging

  def test_the_measured_payload_decodes_to_exactly_12_50_A_and_109_0_V(self):
    cp = CANParser(DBC_NAME, [(ed.AC_MSG, float("nan"))], 0)
    cp.update([[1_000, [(AC_ADDR, self.PAYLOAD, 0)]]])
    amps = cp.vl[ed.AC_MSG]["EvrDrvAc_I_Actl"]
    volts = cp.vl[ed.AC_MSG]["EvrDrvAc_U_Actl"]
    assert amps == 12.50, f"expected exactly 12.50 A, got {amps}"
    assert volts == 109.0, f"expected exactly 109.0 V, got {volts}"
    assert amps * volts / 1000.0 == pytest.approx(1.3625, abs=1e-9)

  def test_an_all_zero_payload_decodes_to_a_clean_zero(self):
    cp = CANParser(DBC_NAME, [(ed.AC_MSG, float("nan"))], 0)
    cp.update([[1_000, [(AC_ADDR, UNPLUGGED, 0)]]])
    assert cp.vl[ed.AC_MSG]["EvrDrvAc_I_Actl"] == 0.0
    assert cp.vl[ed.AC_MSG]["EvrDrvAc_U_Actl"] == 0.0
    assert cp.ts_nanos[ed.AC_MSG]["EvrDrvAc_I_Actl"] == 1_000, "and it is still RECEIVED"

  @staticmethod
  def _amps_at(start_bit: int) -> float:
    """The same 16-bit big-endian current signal read at a different start bit, decoded by the real
    get_raw_value with the DBC's own msb/lsb convention."""
    be_bits = [j + i * 8 for i in range(64) for j in range(7, -1, -1)]
    idx = be_bits.index(start_bit)
    sig = Signal("x", start_bit, start_bit, be_bits[idx + 15], 16, False, 0.00625, 0.0, False)
    return get_raw_value(CHARGING, sig) * sig.factor

  def test_negative_control_neighbouring_start_bits_are_all_plausible_and_all_wrong(self):
    """Nothing raises and nothing looks absurd, so ONLY exact equality distinguishes a correct start
    bit from a wrong one. (Cross-check that the harness is faithful: bit 7 reproduces the shipped
    12.50 A, and bit 39 reproduces the voltage field read as if it were current.)"""
    assert self._amps_at(7) == 12.50                       # D0:D1 -- what the DBC actually says
    assert self._amps_at(15) == 332.8                      # D1:D2 -- a big-but-believable charger
    assert self._amps_at(39) == 5.45                       # D4:D5 -- the VOLTAGE field, as current
    assert self._amps_at(47) == 166.4                      # D5:D6
    for bit in (15, 39, 47):
      assert self._amps_at(bit) != 12.50

  def test_the_dbc_signal_definition_itself(self):
    """Pins the shipped DBC line, so a later edit to 0x2A7 has to come past this test."""
    msg = DbcFile(DBC_NAME).name_to_msg[ed.AC_MSG]
    assert msg.address == AC_ADDR == 679 and msg.size == 8
    i, u = msg.sigs["EvrDrvAc_I_Actl"], msg.sigs["EvrDrvAc_U_Actl"]
    assert (i.start_bit, i.size, i.factor, i.offset, i.is_little_endian, i.is_signed) == \
           (7, 16, 0.00625, 0.0, False, False)             # 1/160 A per bit
    assert (u.start_bit, u.size, u.factor, u.offset, u.is_little_endian, u.is_signed) == \
           (39, 16, 0.125, 0.0, False, False)              # 1/8 V per bit


# ---------------------------------------------------------------- T6


class TestFeatureTurnsItselfOffLoudlyWhenTheMessageIsNotInTheDbc:
  """A Ford whose DBC has no 0x2A7 never gets it registered (get_can_parsers filters on
  `if m in known`), so cp.ts_nanos -- a plain dict -- raises KeyError. That must turn the feature
  off with a log line, never propagate into CarState.update, and never be retried per frame."""

  class CountingTs(dict):
    def __init__(self, inner):
      super().__init__(inner)
      self.n = 0

    def __getitem__(self, key):
      self.n += 1
      return super().__getitem__(key)

  def _parser_without_everdrive(self):
    # exactly what the probe produces on a DBC lacking the message: it is simply not registered
    cp = CANParser(DBC_NAME, [("BrakeSnData_4", 50)], 0)
    assert ed.AC_MSG not in cp.ts_nanos
    cp.ts_nanos = self.CountingTs(cp.ts_nanos)
    return cp

  def test_no_exception_escapes_and_the_feature_says_so(self, monkeypatch, carlogs):
    clock = Clock()
    monkeypatch.setattr(ed, "time", clock)
    dev = ed.EverDrive()
    dev._params = Capture()
    cp = self._parser_without_everdrive()

    dev.update(cp, 0.0)                                    # must not raise
    assert dev._off is True, "a DBC that cannot carry the message is the one case it is right to latch"
    warns = [m for lvl, m, a in carlogs if "not in this car's DBC" in m]
    assert len(warns) == 1, "Rule 2: turning a feature off silently is the failure, not the fix"
    assert dev._params.blobs == []

  def test_it_latches_off_instead_of_retrying_every_frame(self, monkeypatch, carlogs):
    clock = Clock()
    monkeypatch.setattr(ed, "time", clock)
    dev = ed.EverDrive()
    dev._params = Capture()
    cp = self._parser_without_everdrive()
    for _ in range(10_000):                                # 100 s of CarState.update at 100 Hz
      clock.t += DT / 1e9
      dev.update(cp, 0.0)
    assert cp.ts_nanos.n == 1, f"probed the parser {cp.ts_nanos.n} times; it must latch after one"
    assert len([m for lvl, m, a in carlogs if "not in this car's DBC" in m]) == 1

  def test_the_real_car_state_update_survives_it(self, monkeypatch, carlogs):
    """Belt and braces at the level that matters: the exception would take down `card`. The DBC-less
    car is reproduced by dropping 0x2A7 from the parser's ts_nanos, exactly the shape the probe's
    `if m in known` filter leaves behind."""
    truck = Truck(monkeypatch)
    truck.pt.ts_nanos = {k: v for k, v in truck.pt.ts_nanos.items() if k not in (ed.AC_MSG, AC_ADDR)}
    cs = truck.run(5.0)                                    # no 0x2A7 on the wire either, as on such a car
    assert cs.canValid
    assert truck.ed._off is True
    assert truck.cap.blobs == []
    assert len([m for lvl, m, a in carlogs if "not in this car's DBC" in m]) == 1


# ---------------------------------------------------------------- T7


class TestNoExceptionCanEscapeIntoTheCar:
  """The catch has to stay -- an exception out of CarState.update takes down `card` and the truck for
  a display box. But it must NOT be silent: this project has already shipped a telemetry feature
  (waysel2pnw) that published nothing at all behind an `except: pass`, with all eight fields reading
  null on the car and no error anywhere."""

  def test_a_failing_publish_is_swallowed_AND_logged(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch, params=Capture(raises=RuntimeError("put_nonblocking exploded")))
    cs = truck.run(6.0, ac=CHARGING)                       # must not raise
    assert cs.canValid, "the car is unaffected"
    logged = [m for lvl, m, a in carlogs if lvl == "exception" and "telemetry publish failed" in m]
    assert logged, "Rule 2: a publisher that silently does nothing is worse than one that crashes"

  def test_the_failure_log_is_rate_limited_not_five_times_a_second(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch, params=Capture(raises=RuntimeError("boom")))
    truck.run(120.0, ac=CHARGING)                          # ~600 failed publishes
    logged = [m for lvl, m, a in carlogs if lvl == "exception" and "telemetry publish failed" in m]
    assert 1 <= len(logged) <= 5, f"{len(logged)} log lines from a persistent fault is a flood"
    assert truck.ed._err > 100, "positive control: it really did keep failing"

  def test_a_NON_KeyError_from_the_presence_probe_is_swallowed_AND_logged(self, monkeypatch, carlogs):
    """The KeyError handler is deliberately specific (it means "this DBC has no 0x2A7" and latches
    the feature off permanently). Everything ELSE reaching the same probe -- a parser object that is
    not what we assumed, a ts_nanos that is not a dict -- must be caught too, logged, and NOT
    latched. Found by mutation testing: without this, deleting that generic handler survived."""
    clock = Clock()
    monkeypatch.setattr(ed, "time", clock)
    dev = ed.EverDrive()
    dev._params = Capture()

    class Hostile:
      _last_update_nanos = 10**12

      @property
      def ts_nanos(self):
        raise TypeError("ts_nanos is not a dict on this object")

    clock.t += 10.0
    dev.update(Hostile(), 0.0)                             # must not raise
    logged = [m for lvl, m, a in carlogs if lvl == "exception" and "telemetry publish failed" in m]
    assert len(logged) == 1, "Rule 2: the generic catch must not be silent either"
    assert dev._off is False, "only a DBC that cannot carry the message is a permanent latch"
    assert dev._params.blobs == []

  def test_a_failure_in_the_decode_path_is_swallowed_AND_logged(self, monkeypatch, carlogs):
    """Not just the param write -- anything inside the try."""
    clock = Clock()
    monkeypatch.setattr(ed, "time", clock)
    dev = ed.EverDrive()
    dev._params = Capture()

    class Exploding:
      _last_update_nanos = 10**12
      ts_nanos = {ed.AC_MSG: {"EvrDrvAc_I_Actl": 10**12}}

      @property
      def vl(self):
        raise ValueError("decode exploded")

    clock.t += 10.0
    dev.update(Exploding(), 0.0)                           # must not raise
    logged = [m for lvl, m, a in carlogs if lvl == "exception" and "telemetry publish failed" in m]
    assert len(logged) == 1
    assert dev._off is False, "a transient decode fault must not permanently kill the feature"

  def test_an_unavailable_params_store_turns_the_feature_off_loudly(self, monkeypatch, carlogs):
    """_open_params failing is permanent by design -- and must say so."""
    clock = Clock()
    monkeypatch.setattr(ed, "time", clock)
    dev = ed.EverDrive()
    cp = CANParser(DBC_NAME, [(ed.AC_MSG, float("nan"))], 0)
    cp.update([[10**12, [(AC_ADDR, CHARGING, 0)]]])
    clock.t += 10.0
    dev.update(cp, 0.0)                                    # no openpilot on the path in a bare opendbc
    assert dev._off is True
    assert [m for lvl, m, a in carlogs if "could not open /dev/shm params" in m]


# ---------------------------------------------------------------- T10
class TestAcMeterPlausibilityBand:
  """everdrive2pnw (added 2026-09-19 during Opus verification, closing two findings from the test pass):

  `acKw` was the ONLY published number with no plausibility band. Unlike the three truck signals it
  comes from an AFTERMARKET module, and its DBC entry has no counter and no checksum -- so a garbled
  0x2A7 is accepted by the parser exactly as readily as a good one, and decodes to as much as
  409.6 A x 8191.9 V = 3.3 MW. That would be (a) published as fact and (b) turned by the UI into a
  gain-rate term wide enough to overflow the fixed-width box and CLIP the whole line.

  The band is PHYSICAL, not an arbitrary kW cap: 0..100 A is beyond any plausible EVSE, and 0..277 V
  is the single-phase AC mains ceiling (277 V line-to-neutral on a 480Y system). Zero is IN band in
  both, because the unplugged frame is all zeros and that is a REAL measured zero -- the one case
  that must keep publishing."""

  # raw 20000 -> 125.0 A (over) with the measured 109.0 V; raw 4000 -> 500.0 V (over) with 12.5 A
  OVER_CURRENT = bytes.fromhex("4e20000003680000")
  OVER_VOLTAGE = bytes.fromhex("07d000000fa00000")   # 12.5 A x raw 4000 = 500.0 V (over)
  ALL_ONES = b"\xff" * 8                       # 409.59375 A x 8191.875 V = 3.35 MW
  AT_LIMIT = bytes.fromhex("3e80000008a80000")  # exactly 100.0 A x 277.0 V -- the inclusive boundary

  def _run(self, monkeypatch, ac):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=ac, energy=truck.energy(range_km=180.2, eff_wh_km=320.0, soc_pct=49.99))
    return truck

  @pytest.mark.parametrize("name", ["OVER_CURRENT", "OVER_VOLTAGE", "ALL_ONES"])
  def test_an_out_of_band_frame_is_not_published_and_is_logged(self, monkeypatch, carlogs, name):
    """Rule 2: skipped, and SAID SO. Silently dropping it would look identical to 'unplugged'."""
    truck = self._run(monkeypatch, getattr(self, name))
    assert truck.cap.blobs == [], f"{name} was published as fact"
    assert [m for lvl, m, a in carlogs if "outside the physical band" in m], f"{name} dropped silently"

  def test_the_inclusive_boundary_still_publishes(self, monkeypatch, carlogs):
    """The positive control. Without it, a band of (0,0) would pass every test above."""
    truck = self._run(monkeypatch, self.AT_LIMIT)
    assert truck.cap.blobs, "a frame exactly at the band limits must still publish"
    assert truck.cap.blobs[-1]["acKw"] == pytest.approx(100.0 * 277.0 / 1000.0)

  def test_the_real_charging_frame_is_unaffected(self, monkeypatch, carlogs):
    """The measured frame must sail through untouched -- the band must not cost us the feature."""
    truck = self._run(monkeypatch, CHARGING)
    assert truck.cap.blobs[-1]["acKw"] == pytest.approx(1.363)   # producer rounds to 3 dp
    assert not [m for lvl, m, a in carlogs if "outside the physical band" in m]

  def test_the_unplugged_all_zero_frame_still_publishes_a_real_zero(self, monkeypatch, carlogs):
    """0 A / 0 V is IN band on purpose: it is a measurement, not a fault. Collapsing it into the
    out-of-band path would destroy the very distinction this feature exists to make."""
    truck = self._run(monkeypatch, UNPLUGGED)
    assert truck.cap.blobs, "the unplugged frame must still publish"
    assert truck.cap.blobs[-1]["acKw"] == 0.0
    assert truck.cap.blobs[-1]["acSeen"] is True

  def test_a_garbled_frame_does_not_latch_the_feature_off(self, monkeypatch, carlogs):
    """One bad frame must not end the session: the next good frame republishes."""
    truck = Truck(monkeypatch)
    energy = truck.energy(range_km=180.2, eff_wh_km=320.0, soc_pct=49.99)
    truck.run(6.0, ac=self.ALL_ONES, energy=energy)
    assert truck.cap.blobs == []
    truck.run(6.0, ac=CHARGING, energy=energy)
    assert truck.cap.blobs, "a good frame after a garbled one must republish"
    assert truck.cap.blobs[-1]["acKw"] == pytest.approx(1.363)   # producer rounds to 3 dp

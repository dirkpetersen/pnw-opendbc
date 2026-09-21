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
RPC_ADDR = 0x471

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

  def energy(self, range_km=None, eff_wh_km=None, soc_pct=None, rpc_km=None) -> list[CanData]:
    """The truck's own three energy frames, packed from PHYSICAL values (the packer converts to raw
    with the DBC's own factor/offset, so e.g. 409.4 km == raw 4094 == NoDataExists)."""
    out = []
    if range_km is not None:
      out.append(self.frame("MtrTrac_Data2_FD1", {"VehElRnge_L_Dsply": range_km}))
    if eff_wh_km is not None:
      out.append(self.frame("HEV_Powertrain_Data7_FD1", {"VehElEffAvg_No_Dsply": eff_wh_km}))
    if soc_pct is not None:
      out.append(self.frame("Battery_Traction_4_FD1", {"BattTracSoc2_Pc_Actl": soc_pct}))
    if rpc_km is not None:
      out.append(self.frame("Cluster_HEV_Data10_FD1", {"RngPerChrgAvg_L_Dsply": rpc_km}))
    return out

  def tick(self, *extra: CanData, dt: int = DT):
    # `dt` defaults to card's real 100 Hz; the one caller that overrides it says why at its use site.
    self.t += dt
    self.clock.t += dt / 1e9
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

  def test_every_energy_message_is_registered_ignore_alive(self):
    """Structural: the nan probe is what makes them ignore_alive, and nothing else does.

    The exact-set assertion below is DELIBERATE and must be updated by hand when a message is added
    (it caught RngPerChrgAvg on 2026-09-20). That is the point: a new message that reaches `cp.vl`
    WITHOUT going through the nan probe is alive-checked, and on a Ford that never transmits it that
    is can_valid False -> ret.canValid False -> an undriveable car. Failing here is cheap; the
    alternative is not."""
    pt = CarState.get_can_parsers(self._CP())[Bus.pt]
    assert set(ed.ENERGY_MSGS) == {ed.AC_MSG, ed.RANGE_MSG, ed.EFF_MSG, ed.SOC_MSG, ed.RPC_MSG}
    assert [pt.dbc.name_to_msg[m].address for m in ed.ENERGY_MSGS] == \
           [AC_ADDR, RANGE_ADDR, EFF_ADDR, SOC_ADDR, RPC_ADDR]   # 0x2A7/0x442/0x36D/0x24C/0x471
    for name in ed.ENERGY_MSGS:
      addr = pt.dbc.name_to_msg[name].address
      assert addr in pt.addresses, f"{name} must be registered up front, not lazily"
      state = pt.message_states[addr]
      assert state.ignore_alive, \
        f"{name} is ALIVE-CHECKED: a Ford that never transmits it goes can_valid False -> UNDRIVEABLE"
      assert state.timeout_threshold > 0   # nan freq still gets the 1 Hz fallback threshold   # freq came from the nan probe

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
    """CORRECTED 2026-09-20 (Fable). This test used to assert that 409.3 IS a real reading, on the
    reasoning that it is the DBC's stated max and one bit below the first sentinel. That reasoning
    checked the VAL_ sentinel table and missed `GenSigStartValue SG_ 1090 VehElRnge_L_Dsply 4093`
    (dbc:8128) -- 409.3 is what the signal carries BEFORE the ECU has an estimate, and it decodes to
    254 mi of range that does not exist. The band now stops below it.

    The largest genuinely-real value is raw 4092 = 409.2 km, which must still pass -- that is what
    this test now pins. See TestDbcStartValuesAreRejected for the rejection side."""
    assert self._publish(monkeypatch, range_km=409.2, eff_wh_km=320.0,
                         soc_pct=49.99)["rangeKm"] == pytest.approx(409.2)
    assert self._publish(monkeypatch, range_km=409.3, eff_wh_km=320.0,
                         soc_pct=49.99)["rangeKm"] is None, "the start value is not a reading"

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

  def test_a_failure_in_the_LIVENESS_path_is_swallowed_AND_logged(self, monkeypatch, carlogs):
    """Added after the Fable review 2026-09-19. The liveness line dereferences
    `cp._last_update_nanos` -- a PRIVATE CANParser attribute this feature has no contract over. It
    used to sit OUTSIDE the try, so an upstream rename would have been an AttributeError escaping
    into CarState.update(), killing `card` and taking the car offroad for a display box.

    The presence probe must SUCCEED here (a real ts_nanos) so the failure lands in the liveness
    block specifically, not in the probe's own handler tested above."""
    clock = Clock()
    monkeypatch.setattr(ed, "time", clock)
    dev = ed.EverDrive()
    dev._params = Capture()

    class NoClock:
      ts_nanos = {ed.AC_MSG: {"EvrDrvAc_I_Actl": 10**12}}   # probe succeeds: a frame WAS seen

      def __getattr__(self, name):                          # ._last_update_nanos is gone
        raise AttributeError(f"CANParser has no attribute {name!r}")

    clock.t += 10.0
    dev.update(NoClock(), 0.0)                              # must not raise
    logged = [m for lvl, m, a in carlogs if lvl == "exception" and "telemetry publish failed" in m]
    assert len(logged) == 1, "Rule 2: swallowed is not enough, it must also be logged"
    assert dev._off is False, "a transient parser fault is not a permanent latch"
    assert dev._params.blobs == [], "nothing may be published from a failed liveness check"

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


# ---------------------------------------------------------------- T11
class TestDerivedPackCapacity:
  """everdrive2pnw (2026-09-20): `capKwh` is the pack's usable energy, DERIVED from the truck's own
  two numbers (RngPerChrgAvg x VehElEffAvg) instead of a hardcoded constant, so it follows if the
  truck revises and stays self-consistent with the range shown beside it.

  ⚠️ It is NOT a measurement of pack health. VehElEffAvg has 10 Wh/km resolution, so ONE LSB moves
  the answer from 123.0 to 131.0 kWh at the observed 396.9 km -- Ford's stated 131 kWh usable sits
  exactly on the next LSB up. Anyone reading a 127-vs-131 gap as degradation is reading quantisation.
  test_one_lsb_of_efficiency_spans_fords_stated_capacity pins that so the claim cannot be lost."""

  def _pub(self, monkeypatch, **energy):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(**energy))
    assert truck.cap.blobs, "nothing published -- the rest would be vacuous"
    return truck.cap.blobs[-1]

  def test_capacity_is_the_product_of_the_trucks_own_two_numbers(self, monkeypatch, carlogs):
    last = self._pub(monkeypatch, range_km=156.1, eff_wh_km=320.0, soc_pct=43.85, rpc_km=396.9)
    assert last["capKwh"] == pytest.approx(396.9 * 320.0 / 1000.0, abs=0.01)

  @pytest.mark.parametrize("missing", ["rpc_km", "eff_wh_km"])
  def test_capacity_is_None_when_either_input_is_missing(self, monkeypatch, carlogs, missing):
    """Rule 2: half an input must not produce a confident number. NEVER a 0.0-as-a-guess."""
    kw = dict(range_km=156.1, eff_wh_km=320.0, soc_pct=43.85, rpc_km=396.9)
    kw.pop(missing)
    assert self._pub(monkeypatch, **kw)["capKwh"] is None, f"missing {missing} must give None"

  def test_a_sentinel_RngPerChrgAvg_is_None_not_a_capacity(self, monkeypatch, carlogs):
    """raw 4094 NoDataExists decodes to 409.4 km -- which would imply a plausible 131 kWh."""
    last = self._pub(monkeypatch, range_km=156.1, eff_wh_km=320.0, soc_pct=43.85, rpc_km=409.4)
    assert last["capKwh"] is None

  def test_a_missing_RngPerChrgAvg_is_logged_loudly(self, monkeypatch, carlogs):
    """Without this the kWh figure just silently vanishes from the box -- an invisible degradation."""
    self._pub(monkeypatch, range_km=156.1, eff_wh_km=320.0, soc_pct=43.85)
    assert [m for lvl, m, a in carlogs if lvl == "error" and "rngPerChrgKm" in m]

  def test_one_lsb_of_efficiency_spans_fords_stated_capacity(self, monkeypatch, carlogs):
    """THE anti-misreading test. VehElEffAvg is 10 Wh/km per bit. At the observed 396.9 km one LSB
    moves the derived capacity across Ford's entire stated figure, so a 127-vs-131 'gap' is
    quantisation, not a degraded pack."""
    caps = {}
    for eff in (310.0, 320.0, 330.0):
      caps[eff] = self._pub(monkeypatch, range_km=156.1, eff_wh_km=eff,
                            soc_pct=43.85, rpc_km=396.9)["capKwh"]
    assert caps[310.0] == pytest.approx(123.0, abs=0.05)
    assert caps[320.0] == pytest.approx(127.0, abs=0.05)
    assert caps[330.0] == pytest.approx(131.0, abs=0.05), "one LSB up IS Ford's 131 kWh"
    assert caps[330.0] - caps[310.0] > 7.5, "the +/-1 LSB band must straddle 131 kWh"


class TestDbcStartValuesAreRejected:
  """Fable review 2026-09-20. `GenSigStartValue` is what a signal carries BEFORE its ECU has a real
  estimate. For both 12-bit range signals that value is **4093**:

      dbc:8128  VehElRnge_L_Dsply      GenSigStartValue 4093   -> 409.3 km = 254 mi of range
      dbc:9542  RngPerChrgAvg_L_Dsply  GenSigStartValue 4093   -> 409.3 km -> 131.0 kWh capacity

  Neither is in the VAL_ sentinel table, so nothing else rejects them, and both decode to a number a
  human would accept without blinking -- 131.0 kWh is Ford's own headline capacity. The bands stop at
  409.2 (4092, the largest raw that is neither a start value nor a sentinel) precisely for this.

  The sibling RngPerChrgInst_L_Dsply -- same PCM_HEV, same 12-bit encoding -- was observed saturating
  at raw 4093 in 759 of 1547 samples on this truck, so this signal family does put 4093 on the wire."""

  def _pub(self, monkeypatch, **energy):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(**energy))
    assert truck.cap.blobs, "nothing published -- the rest would be vacuous"
    return truck.cap.blobs[-1]

  def test_RngPerChrgAvg_start_value_4093_is_not_a_131kWh_pack(self, monkeypatch, carlogs):
    last = self._pub(monkeypatch, range_km=156.1, eff_wh_km=320.0, soc_pct=43.85, rpc_km=409.3)
    assert last["capKwh"] is None, "409.3 km x 320 Wh/km = 131.0 kWh, indistinguishable from the truth"

  def test_VehElRnge_start_value_4093_is_not_254_miles_of_range(self, monkeypatch, carlogs):
    last = self._pub(monkeypatch, range_km=409.3, eff_wh_km=320.0, soc_pct=43.85, rpc_km=396.9)
    assert last["rangeKm"] is None, "409.3 km = 254 mi -- a range the driver would plan a trip on"

  def test_the_largest_REAL_value_still_passes(self, monkeypatch, carlogs):
    """Positive control: the band must reject 4093, not everything near it."""
    last = self._pub(monkeypatch, range_km=409.2, eff_wh_km=320.0, soc_pct=43.85, rpc_km=409.2)
    assert last["rangeKm"] == pytest.approx(409.2)
    assert last["capKwh"] == pytest.approx(409.2 * 320.0 / 1000.0, abs=0.01)

  def test_a_zero_RngPerChrgAvg_is_None_not_a_zero_capacity(self, monkeypatch, carlogs):
    """Kills mutation P7 (lower bound 0.1 -> 0.0), which survived the author's own pass. A zero
    full-charge range would publish capKwh 0.0, and the UI would then print '(0.000kwh)' -- an empty
    pack, from a signal that simply had not populated."""
    last = self._pub(monkeypatch, range_km=156.1, eff_wh_km=320.0, soc_pct=43.85, rpc_km=0.0)
    assert last["capKwh"] is None

  def test_a_capacity_that_rounds_to_zero_is_None_not_a_zero_capacity(self, monkeypatch, carlogs):
    """Fable review 2026-09-20. Both inputs can sit AT their band floors and still be 'usable':
    0.1 km x 10 Wh/km = 0.001 kWh, which round(.., 2) takes to 0.00. That is in-band, so the
    rpc_km == 0.0 guard above does not catch it. A zero capacity makes floor_kwh zero, and the
    `win_kwh >= floor_kwh` test in _gross_kw then passes on ANY window -- publishing acKw itself
    as 'consumption', a plausible wrong number rather than no answer (Rule 2)."""
    last = self._pub(monkeypatch, range_km=156.1, eff_wh_km=10.0, soc_pct=43.85, rpc_km=0.1)
    assert last["capKwh"] is None, "0.1 km x 10 Wh/km rounds to 0.00 kWh -- must be None"
    assert last["energyKwh"] is None, "energy derives from capacity; it cannot survive it"
    assert last["grossKw"] is None, "a zero floor must not let the window report"


# ---------------------------------------------------------------- T12
def synth_drive(dev, *, kw, mph, seconds, cap=126.0, soc=50.0, ac_kw=0.0, t0=0.0,
                dt=ed.PUBLISH_S, quantise=True):
  """everdrive2pnw: feed `_gross_kw` a synthetic constant-power drive at the real publish cadence.

  SoC is QUANTISED to its true 0.01 %/bit before being handed over, because the whole point of
  MIN_DSOC_PCT is the quantisation: an un-quantised ramp would clear any floor instantly and every
  test below would be measuring a resolution the signal does not have.

  `kw` is the NET pack drain (what SoC actually sees). A charging EverDrive therefore shows up as a
  smaller `kw` plus a non-zero `ac_kw`, which is exactly the situation grossKw has to undo.
  Returns [(t, soc, gross_or_None), ...]."""
  out = []
  t = t0
  end = t0 + seconds
  v_ego = mph / 2.23694
  while t < end - 1e-9:
    s = soc - kw * (t - t0) / 3600.0 / cap * 100.0
    if quantise:
      s = int(s * 100.0) / 100.0        # the BECM puts whole LSBs on the wire, not a real number
    out.append((t, s, dev._gross_kw(t, s, cap, v_ego, ac_kw)))
    t += dt
  return out


class TestRollingConsumption:
  """everdrive2pnw (2026-09-20): the rolling pack-consumption window behind the UI's "range at the
  speed you are doing right now".

  It is differenced out of pack SoC because ENERGY-RANGE-SIGNALS.md §3 ruled out every broadcast
  power signal on this truck. That makes it a QUANTISED measurement -- 0.01 %/bit is 12.6 Wh at the
  125.96 kWh measured over UDS on 2026-09-20 -- so most of what is tested here is the difference
  between a number and a number-shaped piece of noise."""

  def test_a_steady_drive_recovers_the_power_that_produced_it(self):
    """The positive control the rest of the class depends on. 20 kW at 60 mph for 5 minutes."""
    dev = ed.EverDrive()
    got = [g for _t, _s, g in synth_drive(dev, kw=20.0, mph=60.0, seconds=300.0)]
    live = [g for g in got if g is not None]
    assert live, "a 5-minute steady drive must produce a consumption figure"
    # +/-5 % is exactly what MIN_DSOC_PCT promises; assert it rather than a loose band.
    assert all(19.0 <= g <= 21.0 for g in live), f"{min(live)}..{max(live)} kW for a 20 kW drive"

  def test_nothing_is_reported_until_the_floor_is_met(self):
    """Rule 2: a provisional figure is indistinguishable from a settled one once it is on screen.
    WINDOW_MIN_S of moving time is required, so the first ~60 s must be None -- not a guess."""
    dev = ed.EverDrive()
    got = synth_drive(dev, kw=20.0, mph=60.0, seconds=120.0)
    early = [g for t, _s, g in got if t < ed.WINDOW_MIN_S]
    assert set(early) == {None}, "reported before WINDOW_MIN_S of moving time had elapsed"
    assert any(g is not None for _t, _s, g in got), "positive control: it must start eventually"

  def test_a_low_power_drive_waits_for_the_floor_instead_of_reporting_noise(self):
    """3 kW at 15 mph: a 60 s window moves 0.04 % = 4 LSB, i.e. +/-25 % quantisation error. The floor
    has to hold it back until 0.20 % has accumulated, which at 3 kW takes 302 s -- longer than
    WINDOW_MAX_S, so on this input it must NEVER report."""
    dev = ed.EverDrive()
    got = [g for _t, _s, g in synth_drive(dev, kw=3.0, mph=15.0, seconds=600.0)]
    assert set(got) == {None}, "0.04 %/min of SoC is noise; it must not be published as consumption"

  def test_the_floor_is_the_stated_number_of_LSBs(self):
    """Pins the arithmetic in the constant's comment: 0.20 % at 126 kWh is 0.252 kWh = 20 LSB."""
    assert ed.MIN_DSOC_PCT == 0.20
    assert ed.MIN_DSOC_PCT / 100.0 * 126.0 == pytest.approx(0.252)
    assert ed.MIN_DSOC_PCT / 0.01 == 20               # LSBs -> +/-5 % worst-case quantisation

  def test_grossKw_adds_the_everdrive_input_back(self):
    """THE reason grossKw exists. SoC-derived consumption is already NET of whatever the charger is
    feeding in, so a truck drawing 21.4 kW while taking 1.4 kW from the EverDrive drains the pack at
    20 kW. Handing the UI that 20 kW and letting it apply `range * P/(P - acKw)` would count the
    charger TWICE. grossKw must read 21.4."""
    net = ed.EverDrive()
    gross = ed.EverDrive()
    n = [g for _t, _s, g in synth_drive(net, kw=20.0, mph=60.0, seconds=300.0, ac_kw=0.0)]
    g = [x for _t, _s, x in synth_drive(gross, kw=20.0, mph=60.0, seconds=300.0, ac_kw=1.4)]
    nl = [x for x in n if x is not None]
    gl = [x for x in g if x is not None]
    assert nl and gl and len(nl) == len(gl)
    assert all(b - a == pytest.approx(1.4, abs=0.011) for a, b in zip(nl, gl, strict=True))
    assert all(20.4 <= x <= 22.4 for x in gl), f"{min(gl)}..{max(gl)} for 20 kW net + 1.4 kW in"

  def test_below_ten_mph_nothing_accumulates_at_all(self):
    """Driver's explicit spec 2026-09-20. Below MOVING_MS the truck is drawing accessories with no
    distance, which is not what this number means -- and `energy / grossKw x speed` would then be
    extrapolating a road-speed average down to walking pace."""
    dev = ed.EverDrive()
    got = synth_drive(dev, kw=20.0, mph=9.9, seconds=600.0)
    assert {g for _t, _s, g in got} == {None}, "accumulated below the moving threshold"
    assert dev._cum_s == 0.0 and dev._cum_kwh == 0.0 and not dev._win
    # and the boundary is inclusive on the moving side -- the positive control for the band
    ok = ed.EverDrive()
    assert any(g is not None for _t, _s, g in synth_drive(ok, kw=20.0, mph=10.1, seconds=300.0))

  def test_a_stop_contributes_neither_time_nor_energy(self):
    """A red light burns ~1.2 kW of accessory load over no distance. If the stop were folded in, the
    consumption would read high and the range low. Drive, stop with the pack STILL DRAINING, drive
    again: the figure must stay the moving figure."""
    dev = ed.EverDrive()
    a = synth_drive(dev, kw=20.0, mph=60.0, seconds=120.0)
    soc_a = a[-1][1]
    # 40 s stopped, SoC still falling at 1.2 kW -- and 10 samples of it are fed in at 0 mph
    stopped = synth_drive(dev, kw=1.2, mph=0.0, seconds=40.0, soc=soc_a, t0=120.0)
    assert {g for _t, _s, g in stopped} == {None}, "a stopped truck must not report"
    b = synth_drive(dev, kw=20.0, mph=60.0, seconds=120.0, soc=stopped[-1][1], t0=160.0)
    live = [g for _t, _s, g in b if g is not None]
    assert live, "it must come back once moving again"
    assert all(19.0 <= g <= 21.0 for g in live), \
      f"the stop leaked into the average: {min(live)}..{max(live)} kW for a 20 kW drive"

  def test_a_long_stop_ages_the_window_out_instead_of_reporting_it_later(self):
    """Rule 2: a stale window must not be reported as current. After WINDOW_MAX_S parked, the whole
    history is gone and the figure has to be rebuilt from scratch."""
    dev = ed.EverDrive()
    synth_drive(dev, kw=20.0, mph=60.0, seconds=180.0)
    assert dev._win, "positive control: there was a window before the stop"
    synth_drive(dev, kw=1.2, mph=0.0, seconds=ed.WINDOW_MAX_S + 5.0, soc=49.0, t0=180.0)
    assert not dev._win, "a window older than WINDOW_MAX_S is not current and must be discarded"
    back = synth_drive(dev, kw=20.0, mph=60.0, seconds=40.0, soc=49.0, t0=180.0 + ed.WINDOW_MAX_S + 5.0)
    assert {g for _t, _s, g in back} == {None}, "40 s is below WINDOW_MIN_S -- it must rebuild, not resume"

  def test_a_rising_soc_reports_nothing_rather_than_a_negative(self):
    """SoC RISES on a long descent and while an EverDrive charges a moving truck. A negative window
    total must not reach the UI, which DIVIDES by this number -- a negative kW is a negative range."""
    dev = ed.EverDrive()
    got = [g for _t, _s, g in synth_drive(dev, kw=-15.0, mph=60.0, seconds=300.0)]
    assert set(got) == {None}, "a rising SoC produced a consumption figure"
    assert dev._cum_kwh < 0.0, "positive control: the window really did accumulate a NEGATIVE drop"

  def test_regen_inside_a_net_discharge_is_kept_not_discarded(self):
    """The counterpart: an individual negative increment is real and belongs in the average. Only the
    window TOTAL has to be positive. A drive that regenerates for a third of its length must report a
    LOWER consumption, not no consumption."""
    flat = ed.EverDrive()
    synth_drive(flat, kw=30.0, mph=60.0, seconds=200.0)
    steady = [g for _t, _s, g in synth_drive(flat, kw=30.0, mph=60.0, seconds=100.0, soc=48.0, t0=200.0)
              if g is not None]
    hilly = ed.EverDrive()
    a = synth_drive(hilly, kw=30.0, mph=60.0, seconds=200.0)
    b = synth_drive(hilly, kw=-20.0, mph=60.0, seconds=100.0, soc=a[-1][1], t0=200.0)
    live = [g for _t, _s, g in b if g is not None]
    assert steady and live
    assert min(live) < min(steady), "the regen stretch must pull the average DOWN, not be dropped"
    assert min(live) > 0.0, "and the reported figure must still be positive"

  @pytest.mark.parametrize("cap_before,cap_after", [(127.0, 131.0), (131.0, 127.0)])
  def test_a_capacity_revision_is_not_consumption(self, cap_before, cap_after):
    """THE reason the increment is dSoC x capacity and not d(SoC x capacity). VehElEffAvg is
    10 Wh/km per bit, so ONE LSB moves capKwh by ~4 kWh -- at 50 % SoC that is ~2 kWh of apparent
    energy appearing in a single 0.2 s step, which differencing the PRODUCT would publish as ~100 kW
    of consumption (inside NET_KW_MAX, so nothing else would catch it).

    BOTH DIRECTIONS matter and they fail differently: a revision UP makes the window total negative
    and the figure vanish; a revision DOWN manufactures the ~100 kW spike. Only the downward case is
    dangerous, and only the upward case is obvious, so both are pinned."""
    dev = ed.EverDrive()
    a = synth_drive(dev, kw=20.0, mph=60.0, seconds=150.0, cap=cap_before)
    before = [g for _t, _s, g in a if g is not None][-1]
    # the truck revises its long-run efficiency by one LSB: 396.9 km x 330 Wh/km instead of x 320
    b = synth_drive(dev, kw=20.0, mph=60.0, seconds=60.0, cap=cap_after, soc=a[-1][1], t0=150.0)
    live = [g for _t, _s, g in b if g is not None]
    assert live, "the revision must not stop the figure either"
    assert max(live) < 30.0, f"a capacity revision was published as {max(live)} kW of consumption"
    assert abs(max(live) - before) < 5.0, "the step must not move the figure by more than the rescale"

  def test_a_dip_in_consumption_does_not_throw_the_figure_away(self):
    """The window is shortened toward WINDOW_MIN_S by CHOOSING a newer baseline, not by discarding
    the older snapshots. Discarding them (the first implementation here) makes the shortening
    irreversible: the window settles at 60 s, consumption dips, the floor stops clearing and there is
    no history left to grow back into. Replayed on route 000001b8--a46fe398b3 that flickered the
    first number on and off in 10 blocks of median 14 s.

    25 kW for 200 s (a 60 s window clears the floor with 1.65x margin), then 12 kW, at which a 60 s
    window holds only 0.20 kWh against a 0.252 kWh floor. It must keep reporting throughout."""
    dev = ed.EverDrive()
    a = synth_drive(dev, kw=25.0, mph=60.0, seconds=200.0)
    assert [g for _t, _s, g in a if g is not None], "positive control: it was reporting"
    assert 12.0 * 60.0 / 3600.0 < ed.MIN_DSOC_PCT / 100.0 * 126.0, \
      "premise: 12 kW over WINDOW_MIN_S must NOT clear the floor on its own"
    b = synth_drive(dev, kw=12.0, mph=40.0, seconds=120.0, soc=a[-1][1], t0=200.0)
    gaps = [g for _t, _s, g in b if g is None]
    assert not gaps, f"the figure was lost for {len(gaps) * ed.PUBLISH_S:.1f} s of a 12 kW stretch"

  def test_an_implausible_rolling_power_is_None_and_is_logged(self, monkeypatch, carlogs):
    """The BECM's SoC is an ESTIMATE and can step. A 5 % step down is 6.3 kWh appearing at once; over
    a 60 s window that is ~378 kW, and the UI would turn it into a 9-mile range on a healthy truck."""
    monkeypatch.setattr(ed, "time", Clock())
    dev = ed.EverDrive()
    a = synth_drive(dev, kw=20.0, mph=60.0, seconds=150.0)
    assert [g for _t, _s, g in a if g is not None], "positive control: it was reporting"
    stepped = a[-1][1] - 5.0                      # the BECM re-estimates 5 points lower
    got = [g for _t, _s, g in synth_drive(dev, kw=20.0, mph=60.0, seconds=30.0,
                                          soc=stepped, t0=150.0)]
    assert set(got) == {None}, "a stepped SoC estimate was published as consumption"
    assert [m for lvl, m, _a in carlogs if "plausibility ceiling" in m], \
      "Rule 2: skipping it silently looks exactly like 'the window is not full yet'"

  def test_missing_inputs_drop_the_window_rather_than_pausing_it(self):
    """Rule 2: if socPct or capKwh goes away the window stops being a continuous measurement. It must
    be discarded, not resumed later as though nothing had happened."""
    dev = ed.EverDrive()
    synth_drive(dev, kw=20.0, mph=60.0, seconds=150.0)
    assert dev._win and dev._cum_s > 0.0
    assert dev._gross_kw(150.0, None, 126.0, 30.0, 0.0) is None          # socPct gone
    assert not dev._win and dev._cum_s == 0.0 and dev._last_sample is None
    synth_drive(dev, kw=20.0, mph=60.0, seconds=150.0, soc=49.0, t0=200.0)
    assert dev._win, "and it rebuilds afterwards"
    assert dev._gross_kw(400.0, 49.0, None, 30.0, 0.0) is None           # capKwh gone
    assert not dev._win

  def test_an_unwatched_gap_is_not_folded_into_the_window(self):
    """Samples arrive every PUBLISH_S. A gap longer than SAMPLE_GAP_MAX_S means we were NOT watching
    -- the module went quiet, or a garbled frame was skipped -- so the energy spent across it was
    spent at a speed nobody measured. It must not become an increment.

    The gap below is a LITERAL 3.0 s, not `SAMPLE_GAP_MAX_S + something`. Writing it in terms of the
    constant makes the test scale with the constant and therefore prove nothing: mutation P7 raised
    SAMPLE_GAP_MAX_S to 1e9 and the original version of this test moved its gap to 1e9 + 0.5 and
    passed. The constant is asserted separately, once."""
    assert ed.SAMPLE_GAP_MAX_S == 1.0, "5x PUBLISH_S -- tolerates dropped cycles, not a stop"
    dev = ed.EverDrive()
    dev._gross_kw(0.0, 50.0, 126.0, 30.0, 0.0)
    dev._gross_kw(3.0, 49.0, 126.0, 30.0, 0.0)                           # 1 % "consumed" unseen
    assert dev._cum_kwh == 0.0, "energy from an unwatched gap entered the window"
    assert dev._cum_s == 0.0

  def test_a_brief_dip_below_the_threshold_breaks_the_increment_chain(self):
    """A dip below MOVING_MS SHORTER than SAMPLE_GAP_MAX_S -- a speed bump, a tight turn -- is the
    one case where the moving gate is the only thing protecting the window: the gap guard cannot see
    it, because the samples either side are less than a second apart.

    2.0 s at 60 mph (9 increments = 1.8 s), 0.6 s below 10 mph, 2.0 s at 60 mph. The dip must
    contribute NOTHING, and the first sample after it must start a FRESH increment rather than one
    spanning it -- which is worth exactly 0.8 s of moving time that was never moving."""
    dev = ed.EverDrive()
    a = synth_drive(dev, kw=20.0, mph=60.0, seconds=2.0)
    assert dev._cum_s == pytest.approx(1.8, abs=1e-9), "premise: 9 increments of PUBLISH_S"
    b = synth_drive(dev, kw=20.0, mph=5.0, seconds=0.6, soc=a[-1][1], t0=2.0)
    assert dev._cum_s == pytest.approx(1.8, abs=1e-9), "a sub-threshold sample accumulated"
    synth_drive(dev, kw=20.0, mph=60.0, seconds=2.0, soc=b[-1][1], t0=2.6)
    assert dev._cum_s == pytest.approx(3.6, abs=1e-9), \
      "the increment after the dip spanned it -- 0.8 s of not-moving became moving time"

  def test_a_fresh_object_carries_nothing_over_an_ignition_cycle(self):
    """card is only_onroad, so CarState -- and this object -- are rebuilt on every onroad transition.
    That IS the ignition reset, and it only works because no window state is global or class-level."""
    dev = ed.EverDrive()
    synth_drive(dev, kw=20.0, mph=60.0, seconds=200.0)
    assert dev._win and dev._cum_s > 0.0
    fresh = ed.EverDrive()
    assert not fresh._win and fresh._cum_s == 0.0 and fresh._cum_kwh == 0.0
    assert fresh._last_sample is None
    assert {g for _t, _s, g in synth_drive(fresh, kw=20.0, mph=60.0, seconds=40.0)} == {None}

  def test_the_window_never_grows_without_bound(self):
    """Two hours of driving must not leave two hours of samples in memory."""
    dev = ed.EverDrive()
    synth_drive(dev, kw=20.0, mph=60.0, seconds=1200.0)
    assert len(dev._win) <= int(ed.WINDOW_MAX_S / ed.PUBLISH_S) + 2
    assert (dev._win[-1][0] - dev._win[0][0]) <= ed.WINDOW_MAX_S + ed.PUBLISH_S

  def test_the_figure_tracks_a_change_of_road_within_the_window(self):
    """"Short enough to track reality": after a sustained step from 20 kW to 45 kW the reported
    figure must converge on the new number, not stay on the old one."""
    dev = ed.EverDrive()
    a = synth_drive(dev, kw=20.0, mph=60.0, seconds=200.0)
    b = synth_drive(dev, kw=45.0, mph=70.0, seconds=ed.WINDOW_MAX_S, soc=a[-1][1], t0=200.0)
    live = [g for _t, _s, g in b if g is not None]
    assert live[-1] == pytest.approx(45.0, abs=2.5), f"still reading {live[-1]} kW after the step"


class TestPublishedEnergyAndConsumption:
  """The two new payload keys, end to end through the real Lightning CarInterface and the real DBC --
  the same path `card` runs. The maths itself is unit-tested above; this is the WIRING."""

  def test_energyKwh_is_socPct_times_capKwh(self, monkeypatch, carlogs):
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(range_km=156.1, eff_wh_km=320.0,
                                                    soc_pct=43.85, rpc_km=396.9))
    last = truck.cap.blobs[-1]
    assert last["capKwh"] == pytest.approx(127.01, abs=0.01)
    assert last["energyKwh"] == pytest.approx(43.85 / 100.0 * 127.01, abs=0.01)

  @pytest.mark.parametrize("missing", ["soc_pct", "rpc_km", "eff_wh_km"])
  def test_energyKwh_is_None_when_an_input_is_missing_never_zero(self, monkeypatch, carlogs, missing):
    """Rule 2. A 0.0 here would print as an empty pack and as a zero range."""
    kw = dict(range_km=156.1, eff_wh_km=320.0, soc_pct=43.85, rpc_km=396.9)
    kw.pop(missing)
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(**kw))
    assert truck.cap.blobs[-1]["energyKwh"] is None, f"missing {missing} must give None"

  def test_grossKw_is_published_and_is_None_on_a_short_parked_session(self, monkeypatch, carlogs):
    """The key must EXIST (the UI reads it every poll) and must be None until earned. A parked truck
    with the charger plugged in is exactly the state the device sits in for hours."""
    truck = Truck(monkeypatch)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(range_km=180.2, eff_wh_km=320.0,
                                                    soc_pct=49.99, rpc_km=396.9))
    last = truck.cap.blobs[-1]
    assert "grossKw" in last, "the UI polls this key every cycle; it must always be present"
    assert last["grossKw"] is None

  def test_the_window_is_stepped_on_EVERY_publish_not_only_when_it_has_an_answer(self, monkeypatch,
                                                                                 carlogs):
    """Kills mutation P18. A window that is only advanced once it already has an answer can never
    acquire one -- it would be a feature that quietly does nothing forever, with every other test in
    this file still green because they all assert None on short runs."""
    truck = Truck(monkeypatch)
    calls = []
    real = ed.EverDrive._gross_kw
    monkeypatch.setattr(ed.EverDrive, "_gross_kw",
                        lambda self, *a: (calls.append(a), real(self, *a))[1])
    truck.run(6.0, ac=CHARGING, energy=truck.energy(range_km=180.2, eff_wh_km=320.0,
                                                    soc_pct=49.99, rpc_km=396.9))
    assert len(calls) == len(truck.cap.blobs), "one window step per published payload, always"
    assert len(calls) > 10, "positive control: it really did publish repeatedly"
    assert all(a[1] == pytest.approx(49.99) and a[2] is not None for a in calls), \
      "the window must be handed the decoded SoC and capacity, not None"

  def test_a_real_number_reaches_the_payload_and_is_rounded(self, monkeypatch, carlogs):
    """The other half of P18: whatever the window computes has to actually land in `grossKw`. The
    window's own maths is unit-tested above; this pins the WIRING, which no other test reaches
    because a credible window needs a minute of driving that the 100 Hz harness cannot afford."""
    truck = Truck(monkeypatch)
    monkeypatch.setattr(ed.EverDrive, "_gross_kw", lambda self, *a: 21.4)
    truck.run(6.0, ac=CHARGING, energy=truck.energy(range_km=180.2, eff_wh_km=320.0,
                                                    soc_pct=49.99, rpc_km=396.9))
    assert truck.cap.blobs[-1]["grossKw"] == 21.4

  def test_end_to_end_a_minute_of_real_driving_produces_a_real_consumption(self, monkeypatch,
                                                                           carlogs):
    """THE integration test: the real Lightning CarInterface, the real DBC, the real CANPacker, a
    falling SoC and a moving truck -- and a number out the far end. Everything in between (the nan
    probe, the AC liveness gate, the 5 Hz publish, the window) has to work for this to pass.

    30 kW at 62 mph for 75 s: 0.625 kWh = 0.49 % of SoC, comfortably over the 0.20 % floor, and
    75 s is WINDOW_MIN_S plus the ~3 s the AC meter takes to arm.

    Ticked at 20 Hz rather than card's 100 Hz, purely for runtime -- 75 s of simulated driving is
    1500 CarInterface.update calls instead of 7500. Nothing under test depends on the tick rate
    (the publisher gates on wall time, and every message still arrives far inside its alive
    threshold, which `canValid` below asserts); the real 5 Hz publish cadence is pinned separately
    by test_publishes_at_five_hz_while_live."""
    truck = Truck(monkeypatch)
    cap = 396.9 * 320.0 / 1000.0                       # what the producer will derive: 127.01 kWh
    soc0, kw = 49.99, 30.0
    hz, secs = 20, 75
    for i in range(hz * secs):
      slow = []
      if i % hz == 0:
        soc = soc0 - kw * (i / hz) / 3600.0 / cap * 100.0
        slow = truck.energy(range_km=180.2, eff_wh_km=320.0, soc_pct=round(soc, 2), rpc_km=396.9)
        slow.append(CanData(AC_ADDR, CHARGING, 0))
      truck.tick(truck.frame("BrakeSysFeatures", {"Veh_V_ActlBrk": 100.0}), *slow,
                 dt=int(1e9 / hz))
      assert truck.cs.canValid, "the bus went invalid -- this test would then prove nothing"
    assert truck.cs.vEgo > 25.0, "positive control: the truck must actually be moving"
    gross = [b["grossKw"] for b in truck.cap.blobs if b["grossKw"] is not None]
    assert gross, "75 s of 30 kW driving produced no consumption figure at all"
    # 30 kW net + the measured 1.3625 kW of EverDrive input added back
    assert all(29.0 <= g <= 33.0 for g in gross), f"{min(gross)}..{max(gross)} kW for a 30 kW drive"
    assert truck.cap.blobs[-1]["energyKwh"] == pytest.approx(49.5 / 100.0 * cap, abs=0.4)

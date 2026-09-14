"""gearunknown2pnw: the Lightning reports gearShifter=unknown until PowertrainData_10 has been received.

Every frame here goes through the real DBC, the real CANPacker and CANParser, and the real Ford
CarInterface.update -- the same path card runs.

The bug this pins: the parser starts every signal at 0 and TrnRng_D_Rq 0 is "Park", so before this a
powertrain bus that had said nothing decoded a confident Park.
"""
import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.carlog import carlog
from opendbc.car.ford import carstate as ford_carstate
from opendbc.car.ford.values import CAR

GearShifter = structs.CarState.GearShifter
LIGHTNING = CAR.FORD_F_150_LIGHTNING_MK1
DBC = "ford_lincoln_base_pt"
DT = 10_000_000          # 100 Hz, as card sees it


@pytest.fixture
def logs(monkeypatch):
  # only this feature's lines: the parser's own rate-limited "not valid" warnings also go to carlog
  seen = []

  def capture(level):
    def log(msg, *a, **kw):
      if "gearunknown2pnw" in str(msg):
        seen.append((level, msg))
    return log
  monkeypatch.setattr(carlog, "warning", capture("warning"))
  monkeypatch.setattr(carlog, "error", capture("error"))
  return seen


class Truck:
  """A Lightning CarInterface plus a clock. The gear decode needs the automatic branch, which the
  interface picks from Gear_Shift_by_Wire_FD1 (0x5A) in the fingerprint."""

  def __init__(self, transmission_addr=0x5A):
    CarInterface = interfaces[LIGHTNING]
    fp = gen_empty_fingerprint()
    if transmission_addr is not None:
      fp[0][transmission_addr] = 8
    self.CI = CarInterface(CarInterface.get_params(LIGHTNING, fp, [], False, False, False))
    self.packer = CANPacker(DBC)
    self.t = 0
    self.cs = self.CI.update([(self.t, [])])      # first update registers the messages, as in card

  def frame(self, msg, values):
    addr, dat, bus = self.packer.make_can_msg(msg, 0, values)
    return CanData(addr, dat, bus)

  def tick(self, *frames):
    self.t += DT
    self.cs = self.CI.update([(self.t, list(frames))])
    return self.cs

  def gear(self, trn_rng):
    return self.frame("PowertrainData_10", {"TrnRng_D_Rq": trn_rng})

  def other_traffic(self):
    # a real powertrain frame that is not the gear message: the bus is alive, the PCM is not talking
    return self.frame("Yaw_Data_FD1", {"VehYaw_W_Actl": 0.0})

  def run(self, seconds, *frames):
    for _ in range(int(round(seconds * 100))):
      self.tick(*frames)
    return self.cs


def test_lightning_takes_the_automatic_branch():
  assert Truck().CI.CP.transmissionType == structs.CarParams.TransmissionType.automatic


class TestNeverReceived:
  def test_dead_bus_is_unknown_not_park(self, logs):
    truck = Truck()
    cs = truck.run(30)
    assert cs.gearShifter == GearShifter.unknown
    assert cs.gearShifter != GearShifter.park
    assert not cs.canValid

  def test_alive_bus_without_the_gear_message_is_unknown(self, logs):
    truck = Truck()
    cs = truck.run(30, truck.other_traffic())
    assert cs.gearShifter == GearShifter.unknown
    assert not cs.canValid

  def test_the_parser_really_holds_zero_park_underneath(self, logs):
    # the trap is real, not hypothetical: the raw value this change refuses to decode is "Park"
    truck = Truck()
    truck.run(1)
    cp = truck.CI.can_parsers[list(truck.CI.can_parsers)[0]]
    assert cp.vl["PowertrainData_10"]["TrnRng_D_Rq"] == 0
    assert truck.CI.CS.shifter_values[0].upper() == "PARK"


class TestOnceReceived:
  @pytest.mark.parametrize("trn_rng", range(16))
  def test_every_value_decodes_exactly_as_before(self, logs, trn_rng):
    truck = Truck()
    cs = truck.run(0.5, truck.gear(trn_rng))
    before = truck.CI.CS.parse_gear_shifter(truck.CI.CS.shifter_values.get(trn_rng))   # the pre-change expression
    assert cs.gearShifter == before

  def test_named_gears(self, logs):
    truck = Truck()
    assert truck.run(0.5, truck.gear(0)).gearShifter == GearShifter.park
    assert truck.run(0.5, truck.gear(1)).gearShifter == GearShifter.reverse
    assert truck.run(0.5, truck.gear(2)).gearShifter == GearShifter.neutral
    assert truck.run(0.5, truck.gear(3)).gearShifter == GearShifter.drive
    assert truck.run(0.5, truck.gear(14)).gearShifter == GearShifter.unknown      # Unknown_Position

  def test_first_frame_is_used_immediately(self, logs):
    truck = Truck()
    truck.run(2)
    assert truck.tick(truck.gear(3)).gearShifter == GearShifter.drive

  def test_message_going_quiet_holds_the_last_gear_as_before(self, logs):
    truck = Truck()
    truck.run(1, truck.gear(3))
    cs = truck.run(5, truck.other_traffic())
    assert cs.gearShifter == GearShifter.drive
    assert not cs.canValid                        # the staleness is reported where it always was

  def test_park_from_a_real_frame_is_park(self, logs):
    truck = Truck()
    truck.run(1, truck.gear(3))
    assert truck.run(1, truck.gear(0)).gearShifter == GearShifter.park


class TestLogging:
  def test_first_received_logged_once(self, logs):
    truck = Truck()
    truck.run(2.5, truck.other_traffic())
    truck.run(5, truck.gear(0))
    truck.run(5, truck.gear(3))
    firsts = [m for lvl, m in logs if "first received" in m]
    assert len(firsts) == 1 and "2.5 s" in firsts[0]
    assert [m for lvl, m in logs if "not received" in m] == []

  def test_missing_logged_once_after_the_threshold(self, logs):
    truck = Truck()
    truck.run(ford_carstate.GEAR_MISSING_LOG_S - 0.5, truck.other_traffic())
    assert logs == []
    truck.run(120, truck.other_traffic())
    missing = [(lvl, m) for lvl, m in logs if "not received" in m]
    assert len(missing) == 1
    assert missing[0][0] == "error"
    assert "has traffic" in missing[0][1]

  def test_missing_names_a_silent_bus(self, logs):
    truck = Truck()
    truck.run(ford_carstate.GEAR_MISSING_LOG_S + 1)
    missing = [m for lvl, m in logs if "not received" in m]
    assert len(missing) == 1 and "NO powertrain bus traffic" in missing[0]

  def test_late_arrival_after_the_error_is_also_logged(self, logs):
    truck = Truck()
    truck.run(ford_carstate.GEAR_MISSING_LOG_S + 5, truck.other_traffic())
    truck.run(1, truck.gear(0))
    assert [lvl for lvl, m in logs] == ["error", "warning"]
    assert truck.cs.gearShifter == GearShifter.park

  def test_normal_start_is_one_line(self, logs):
    truck = Truck()
    truck.run(60, truck.gear(3), truck.other_traffic())
    assert len(logs) == 1 and logs[0][0] == "warning"


def test_manual_transmission_branch_is_untouched(logs):
  # no 0x5A in the fingerprint and no 0x732 ECU: the interface picks manual, which never reads PowertrainData_10
  truck = Truck(transmission_addr=None)
  if truck.CI.CP.transmissionType != structs.CarParams.TransmissionType.manual:
    pytest.skip(f"interface chose {truck.CI.CP.transmissionType} without 0x5A")
  assert truck.run(1).gearShifter == GearShifter.drive
  assert logs == []

"""fsdguard2pnw: carState.teslaAutopilot on the Raven (TESLA_MODEL_S_HW3), through the real DBC, CANParser and
Tesla CarInterface.update -- the path card runs. 0x399 AutopilotStatus on the chassis bus, stock 0x488
DAS_steeringControl on the autopilot-party bus."""
import pytest

from opendbc.can import CANPacker
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.carstate import tesla_autopilot_state
from opendbc.car.tesla.values import CANBUS, CAR

TA = structs.CarState.TeslaAutopilot
DT = 10_000_000   # 100 Hz


@pytest.fixture
def canbus(monkeypatch):
  for k in ("party", "vehicle", "radar", "autopilot_party", "powertrain", "chassis", "autopilot_powertrain"):
    monkeypatch.setattr(CANBUS, k, getattr(CANBUS, k))


class Car:
  def __init__(self, platform=CAR.TESLA_MODEL_S_HW3):
    CarInterface = interfaces[platform]
    self.CI = CarInterface(CarInterface.get_params(platform, gen_empty_fingerprint(), [], False, False, False))
    self.packer = CANPacker("tesla_can")
    self.t = 0
    self.cnt = 0
    self.cs = self.CI.update([(self.t, [])])

  def tick(self, status=None, steer_type=None):
    """One 10 ms step. None = that frame is not sent this step."""
    self.t += DT
    frames = []
    if status is not None:
      frames.append(CanData(0x399, bytes([status & 0xF]) + bytes(7), self.CI.can_parsers["chassis"].bus))
    if steer_type is not None:
      _, dat, _ = self.packer.make_can_msg("DAS_steeringControl", 0, {
        "DAS_steeringControlType": steer_type, "DAS_steeringControlCounter": self.cnt % 16})
      frames.append(CanData(0x488, dat, self.CI.can_parsers["ap_party"].bus))   # the packer fills the checksum
      self.cnt += 1
    self.cs = self.CI.update([(self.t, frames)])
    return self.cs.teslaAutopilot

  def run(self, status, steer_type, steps=200):
    for _ in range(steps):
      out = self.tick(status, steer_type)
    return out


@pytest.mark.parametrize("status, steer_type, expected", [
  (1, 0, TA.taccOnly),
  (0, 0, TA.selected),
  (2, 0, TA.selected),
  (3, 0, TA.stockSteering),
  (6, 0, TA.stockSteering),       # ACTIVE_6_UNKNOWN: seen while stock steered
  (15, 0, TA.stockSteering),      # not in the DBC at all
  (1, 1, TA.stockSteering),       # stock ANGLE_CONTROL while the status still says 1
  (2, 1, TA.stockSteering),
  (1, 2, TA.stockSteering),
  (1, 3, TA.stockSteering),
])
def test_pure_rule(status, steer_type, expected):
  assert tesla_autopilot_state(status, steer_type, True) == expected


@pytest.mark.parametrize("status, steer_type", [(1, 0), (6, 1), (2, 0)])
def test_not_fresh_is_missing_whatever_the_values(status, steer_type):
  assert tesla_autopilot_state(status, steer_type, False) == TA.missing


@pytest.mark.parametrize("status, steer_type, expected", [
  (1, 0, TA.taccOnly), (2, 0, TA.selected), (0, 0, TA.selected), (6, 1, TA.stockSteering), (1, 1, TA.stockSteering),
])
def test_through_the_real_parser(canbus, status, steer_type, expected):
  assert Car().run(status, steer_type) == expected


def test_before_any_frame_it_is_missing(canbus):
  car = Car()
  assert car.tick() == TA.missing


@pytest.mark.parametrize("drop", ["status", "steer"])
def test_a_frame_that_stops_goes_missing(canbus, drop):
  car = Car()
  assert car.run(1, 0) == TA.taccOnly
  # 0x488 learns ~100 Hz here (timeout 0.1 s); 0x399 learns ~100 Hz too. 3 s of silence is past either timeout.
  kwargs = {"status": None if drop == "status" else 1, "steer_type": None if drop == "steer" else 0}
  assert car.run(steps=300, **kwargs) == TA.missing


def test_lightning_is_unavailable():
  CarInterface = interfaces["FORD_F_150_LIGHTNING_MK1"]
  CI = CarInterface(CarInterface.get_params("FORD_F_150_LIGHTNING_MK1", gen_empty_fingerprint(), [], False, False, False))
  assert CI.update([(0, [])]).teslaAutopilot == TA.unavailable

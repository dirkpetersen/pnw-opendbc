"""teslayaw2pnw: the Raven (TESLA_MODEL_S_HW3) reports carState.yawRate from BrakeMessage (0x20a, chassis bus).

Every frame goes through the real DBC, the real CANParser and the real Tesla CarInterface.update -- the path
card runs. The two "real" frames are byte-for-byte captures from drives/2026-09-07/tesla-coopsteer-shadow-first/
rlogs/seg0.zst, with what the car and the device IMU said at the same moment.
"""
import math

import pytest

from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.tesla.values import CANBUS, CAR

DT = 10_000_000   # 100 Hz, as card sees it

# (frame bytes, steeringAngleDeg, livePose.angularVelocityDevice.z) captured together on the Raven.
# The device frame is z-DOWN, so a LEFT turn reads NEGATIVE z; the vehicle convention is its negation.
REAL_LEFT = (bytes.fromhex("062230ed71180000"), 217.2, -0.1593)    # raw 2161 -> +0.161 rad/s
REAL_RIGHT = (bytes.fromhex("05c9303938f70000"), -86.3, +0.1065)   # raw 1848 -> -0.152 rad/s


def brake_frame(raw_yaw: int, counter: int = 0, brake_bits: int = 0) -> bytes:
  """A BrakeMessage built by hand -- NOT through CANPacker, so a wrong DBC line cannot round-trip itself into
  a passing test. raw_yaw sits in bits 32..43 (LE); the upper nibble of byte 5 is the car's rolling counter."""
  assert 0 <= raw_yaw < 4096 and 0 <= counter < 16
  dat = bytearray(8)
  dat[0] = (brake_bits & 0x3) << 2
  dat[4] = raw_yaw & 0xFF
  dat[5] = ((raw_yaw >> 8) & 0x0F) | (counter << 4)
  return bytes(dat)


@pytest.fixture
def canbus(monkeypatch):
  # the Tesla CarState rewrites these CLASS attributes for legacy platforms; keep that from leaking into
  # whatever test runs next in this process.
  for k in ("party", "vehicle", "radar", "autopilot_party", "powertrain", "chassis", "autopilot_powertrain"):
    monkeypatch.setattr(CANBUS, k, getattr(CANBUS, k))


class Car:
  def __init__(self, platform):
    CarInterface = interfaces[platform]
    self.CI = CarInterface(CarInterface.get_params(platform, gen_empty_fingerprint(), [], False, False, False))
    self.bus = self.CI.can_parsers["chassis"].bus
    self.t = 0
    self.cs = self.CI.update([(self.t, [])])     # first update registers the messages, as in card

  def tick(self, dat: bytes | None):
    self.t += DT
    frames = [CanData(0x20a, dat, self.bus)] if dat is not None else []
    self.cs = self.CI.update([(self.t, frames)])
    return self.cs


def test_raven_decodes_on_the_chassis_bus(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  assert car.bus == 1, "the Raven's BrakeMessage is 0x20a on bus 1 (the frame brakePressed already reads)"


@pytest.mark.parametrize("raw, expected", [
  (2000, 0.0),        # the zero point
  (2161, 0.161),      # left
  (1848, -0.152),     # right
  (2471, 0.471),      # past bit 11: the 27 deg/s parking-lot turn of 2026-09-07 -- proves 12 bits, not 11
  (1677, -0.323),
  (4095, 2.095),      # full scale
  (0, -2.0),
])
def test_scale_and_offset(canbus, raw, expected):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  cs = car.tick(brake_frame(raw))
  assert cs.yawRate == pytest.approx(expected, abs=1e-6)


def test_the_counter_nibble_is_not_part_of_the_value(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  vals = {car.tick(brake_frame(2100, counter=c)).yawRate for c in range(16)}
  assert len(vals) == 1 and vals.pop() == pytest.approx(0.1, abs=1e-6)


def test_brake_bit_still_decodes_alongside(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  cs = car.tick(brake_frame(2050, brake_bits=2))
  assert cs.brakePressed and cs.yawRate == pytest.approx(0.05, abs=1e-6)


@pytest.mark.parametrize("frame", [REAL_LEFT, REAL_RIGHT], ids=["left", "right"])
def test_sign_on_real_frames(canbus, frame):
  """POSITIVE = LEFT, the convention controlsd's kActl expects (Ford's VehYaw_W_Actl is positive on a left-hander).
  Two independent witnesses from the same instant: the steering wheel (positive = left in openpilot) and the IMU
  (device frame z-down, so vehicle yaw = -z)."""
  dat, steer_deg, pose_z = frame
  car = Car(CAR.TESLA_MODEL_S_HW3)
  yaw = car.tick(dat).yawRate
  assert math.copysign(1.0, yaw) == math.copysign(1.0, steer_deg)
  assert math.copysign(1.0, yaw) == math.copysign(1.0, -pose_z)
  assert yaw == pytest.approx(-pose_z, abs=0.05)


def test_no_frame_yet_reads_no_yaw_and_can_is_not_valid(canbus):
  """Before the first BrakeMessage the parser's value is its 0.0 initialiser -- NOT a reading. That must not
  pass as a healthy straight road: the message is alive-checked (brakePressed reads it too), so canValid is
  False for exactly as long as the 0.0 is fake."""
  car = Car(CAR.TESLA_MODEL_S_HW3)
  for _ in range(200):
    cs = car.tick(None)
  assert cs.yawRate == 0.0
  assert not cs.canValid


@pytest.mark.parametrize("platform", [CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_X_HW1,
                                      CAR.TESLA_MODEL_X_HW2])
def test_unverified_legacy_platforms_are_not_decoded(canbus, platform):
  """Only the HW3 Raven was measured. The older platforms share this DBC message, but nobody has checked that
  their 0x20a carries yaw in the same place -- so they keep the default rather than a guessed decode."""
  car = Car(platform)
  frame = brake_frame(2300)
  car.CI.update([(car.t + DT, [CanData(0x20a, frame, car.bus)])])
  cs = car.CI.update([(car.t + 2 * DT, [CanData(0x20a, frame, car.bus)])])
  assert cs.yawRate == 0.0

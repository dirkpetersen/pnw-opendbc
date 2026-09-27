"""fordtsr2pnw: carState.cruiseState.speedLimitSign / speedLimitSignStatus -- the Lightning camera's traffic-sign limit.

Source: Traffic_RecognitnData (0x3CD) TsrVLim1MsgTxt_D_Rq on the CAMERA bus (src 2), 1 Hz plus a frame on every change
(drives/2026-09-24/ford-tsr-measure/DRIVE_REPORT.md). The number is the sign's own value: TsrVlUnitMsgTxt_D_Rq follows the
CLUSTER's display unit, not the sign, so it is never used to convert.

Every frame goes through the real DBC and the real Ford CarInterface.update, the path card runs. ALL payloads are REAL
frames (drives/2026-09-24/ford-tsr-measure/extract.jsonl.gz and out_kph_era_extract.jsonl):
  TSR_55        000001f5--b6253e9b1d seg 5   (2026-09-24, mph cluster)  value 55, unit 2 "Mph", Vl1Stat 2 Reliable
  TSR_60_CHG    00000200--ee5be1a798 seg 16  (2026-09-24)               value 60, unit 2, Vl1Stat 1 LimitChanged
  TSR_NOLIMIT   00000200--ee5be1a798 seg 107 (2026-09-24)               value 255 "NoLimit", Vl1Stat 3 LimitOutdated
  TSR_35_KPH    0000013e--c7ec081219 seg 15  (2026-09-13, KM/H cluster) value 35, unit 1 "Kph" -- a US 35 mph sign
"""
import pytest

from opendbc.can.parser import CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.carlog import carlog
from opendbc.car.ford import carstate as ford_carstate
from opendbc.car.ford.values import CAR

SignStatus = structs.CarState.CruiseState.SpeedLimitSignStatus
DBC = "ford_lincoln_base_pt"
DT = 10_000_000          # 100 Hz card tick, ns
ADDR = 0x3CD

TSR_55 = "172a6137ffc90220"
TSR_60_CHG = "172a513cffc90220"
TSR_NOLIMIT = "172a71ffffc80220"
TSR_35_KPH = "172a6123ffc50220"


def _decode(sig, hexdat):
  p = CANParser(DBC, [("Traffic_RecognitnData", float("nan"))], 2)
  p.update([(DT, [(ADDR, bytes.fromhex(hexdat), 2)])])
  return p.vl["Traffic_RecognitnData"][sig]


def test_the_payloads_mean_what_the_names_say():
  assert (_decode("TsrVLim1MsgTxt_D_Rq", TSR_55), _decode("TsrVlUnitMsgTxt_D_Rq", TSR_55)) == (55, 2)
  assert (_decode("TsrVLim1MsgTxt_D_Rq", TSR_60_CHG), _decode("TsrVl1StatMsgTxt_D_Rq", TSR_60_CHG)) == (60, 1)
  assert _decode("TsrVLim1MsgTxt_D_Rq", TSR_NOLIMIT) == 255
  assert (_decode("TsrVLim1MsgTxt_D_Rq", TSR_35_KPH), _decode("TsrVlUnitMsgTxt_D_Rq", TSR_35_KPH)) == (35, 1)


@pytest.fixture
def logs(monkeypatch):
  seen = []

  def capture(level):
    def log(msg, *a, **kw):
      if "fordtsr2pnw" in str(msg):
        seen.append((level, str(msg)))
    return log
  monkeypatch.setattr(carlog, "warning", capture("warning"))
  monkeypatch.setattr(carlog, "error", capture("error"))
  monkeypatch.setattr(carlog, "exception", capture("exception"))
  return seen


class Car:
  def __init__(self, platform=CAR.FORD_F_150_LIGHTNING_MK1):
    CarInterface = interfaces[platform]
    fp = gen_empty_fingerprint()
    fp[0][0x5A] = 8
    self.CI = CarInterface(CarInterface.get_params(platform, fp, [], False, False, False))
    self.t = 0
    self.tick = 0
    self.cs = self.CI.update([(self.t, [])])

  def run(self, seconds, hexdat=None, src=2, every=100):
    """100 Hz card ticks; the 0x3CD frame is sent on `src` every `every` ticks (100 = the module's 1 Hz)."""
    for _ in range(int(round(seconds * 100))):
      self.t += DT
      self.tick += 1
      frames = [(ADDR, bytes.fromhex(hexdat), src)] if hexdat and self.tick % every == 1 % every else []
      self.cs = self.CI.update([(self.t, frames)])
    return self.cs

  def sign(self):
    return self.cs.cruiseState.speedLimitSign, self.cs.cruiseState.speedLimitSignStatus


class TestDecode:
  def test_real_55_frame_is_55_valid(self, logs):
    car = Car()
    car.run(3.0, TSR_55)
    assert car.sign() == (55.0, SignStatus.valid)

  def test_unit_trap_kmh_cluster_frame_is_the_mph_sign_number(self, logs):
    """The 2026-09-13 km/h-cluster frame: unit flag 1 "Kph", number 35 -- where mapd said 35 MPH. Converting by the
    flag (FrogPilot's way) would give 35 km/h = 21.7 mph. The number must come out as the sign's 35, untouched."""
    car = Car()
    car.run(3.0, TSR_35_KPH)
    assert car.sign() == (35.0, SignStatus.valid)
    assert car.sign()[0] != pytest.approx(35 / 1.609344, abs=0.5)

  def test_no_limit_255_is_nolimit_zero(self, logs):
    car = Car()
    car.run(3.0, TSR_NOLIMIT)
    assert car.sign() == (0.0, SignStatus.noLimit)

  def test_never_received_is_stale_not_unavailable(self, logs):
    car = Car()
    car.run(1.0)
    assert car.sign() == (0.0, SignStatus.stale)

  def test_1hz_cadence_stays_valid_between_frames(self, logs):
    """The module sends at 1 Hz: the 99 ticks between two frames must not read stale."""
    car = Car()
    for _ in range(300):
      car.run(0.01, TSR_55)
      if car.tick > 1:
        assert car.sign() == (55.0, SignStatus.valid), car.tick

  def test_silence_over_2s_goes_stale(self, logs):
    car2 = Car()
    car2.run(0.02, TSR_55)                         # frame at tick 1
    car2.run(ford_carstate.SIGN_STALE_S - 0.05)   # 1.97 s after it
    assert car2.sign() == (55.0, SignStatus.valid)
    car2.run(0.1)                                  # 2.07 s
    assert car2.sign() == (0.0, SignStatus.stale)

  def test_follows_a_change(self, logs):
    car = Car()
    car.run(2.0, TSR_55)
    car.run(0.02, TSR_60_CHG)
    assert car.sign() == (60.0, SignStatus.valid)

  @pytest.mark.parametrize("src", [0, 128])
  def test_the_relayed_copy_does_not_count(self, logs, src):
    """Only the camera bus carries the module's own frame; the bus-0 / TX-echo copies are not read."""
    car = Car()
    car.run(3.0, TSR_55, src=src)
    assert car.sign() == (0.0, SignStatus.stale)

  def test_non_canfd_ford_is_unavailable(self, logs):
    car = Car(CAR.FORD_EXPLORER_MK6)
    car.run(ford_carstate.UNIT_LOG_S + 1, TSR_55)
    assert car.sign() == (0.0, SignStatus.unavailable)
    assert logs == []

  def test_tesla_is_unavailable(self):
    from opendbc.car.tesla.values import CAR as TESLA
    CarInterface = interfaces[TESLA.TESLA_MODEL_S_HW3]
    CI = CarInterface(CarInterface.get_params(TESLA.TESLA_MODEL_S_HW3, gen_empty_fingerprint(), [], False, False, False))
    cs = CI.update([(DT, [])])
    assert (cs.cruiseState.speedLimitSign, cs.cruiseState.speedLimitSignStatus) == (0.0, SignStatus.unavailable)


class TestRegistration:
  def test_on_the_camera_parser_and_ignore_alive(self):
    cam = Car().CI.can_parsers[Bus.cam]
    assert ADDR in cam.message_states
    assert cam.message_states[ADDR].ignore_alive, "a missing sign message must never make canValid false"

  def test_absence_never_costs_can_valid(self):
    """The camera parser's can_valid is ANDed into carState.canValid. A real alive-checked camera message with zero
    0x3CD past the timeout horizon: still valid. Control: stop the real message and it must go invalid."""
    from opendbc.can.packer import CANPacker
    car = Car()
    cam = car.CI.CS.get_can_parsers(car.CI.CP)[Bus.cam]
    assert ADDR in cam.message_states
    cam.vl["ACCDATA_3"]                                   # lazily registered, alive-checked
    addr, dat, _ = CANPacker(DBC).make_can_msg("ACCDATA_3", 2, {})
    t = 0
    for i in range(1500):                                 # 15 s, 100 Hz, no 0x3CD at all (a 1 Hz message's horizon is 10 s)
      t += DT
      cam.update([(t, [(addr, dat, 2)])])
      assert cam.can_valid, f"0x3CD silence invalidated the camera parser at tick {i}"
    for _ in range(600):
      t += DT
      cam.update([(t, [])])
      last = cam.can_valid
    assert not last, "control failed: the camera parser must still enforce its real messages"


class TestLogging:
  def test_one_line_when_it_becomes_valid_carrying_the_unit_flag(self, logs):
    car = Car()
    car.run(3.0, TSR_35_KPH)
    assert len(logs) == 1, logs
    assert "valid 35" in logs[0][1] and "TsrVlUnitMsgTxt_D_Rq=1" in logs[0][1] and "NOT used" in logs[0][1]

  def test_never_received_is_logged_only_after_the_grace(self, logs):
    car = Car()
    car.run(ford_carstate.UNIT_LOG_S - 1)
    assert logs == []
    car.run(2.0)
    assert len(logs) == 1 and "stale" in logs[0][1] and "never received" in logs[0][1], logs

  def test_going_stale_is_logged(self, logs):
    car = Car()
    car.run(ford_carstate.UNIT_LOG_S + 1, TSR_55)
    car.run(ford_carstate.UNIT_LOG_S + 1)
    assert [m for _, m in logs if "stale" in m and "frame age" in m], logs

  def test_a_flapping_value_is_rate_limited(self, logs):
    car = Car()
    for _ in range(20):
      car.run(0.5, TSR_55, every=1)
      car.run(0.5, TSR_60_CHG, every=1)
    assert 1 <= len(logs) <= 3, logs            # 20 s of flapping: at most one line per UNIT_LOG_S

  def test_a_decode_exception_reports_stale_and_logs(self, logs, monkeypatch):
    car = Car()
    car.run(1.0, TSR_55)
    cam = car.CI.can_parsers[Bus.cam]
    monkeypatch.setitem(cam.ts_nanos, "Traffic_RecognitnData", None)   # makes the ts lookup raise TypeError
    car.run(0.02, TSR_55)
    assert car.sign() == (0.0, SignStatus.stale)
    assert [lvl for lvl, m in logs if "decode failed" in m] == ["exception"], logs

"""truckdecode2pnw (B): carState.cruiseState.speedClusterUnit on Ford CAN FD -- the unit the cluster shows the set in.

Every frame goes through the real DBC and the real Ford CarInterface.update, the path card runs. The Mph, NoDataExists
and English payloads are REAL frames from local Lightning rlogs (drives/2026-09-12/central-oregon-weekend/TRUCK_DECODE.md):
  IPMA_MPH     0x3D9 src 2  0000012f--817cabeb76--1, Sat 2026-09-12 06:25 PT     IsaVLimUnit_D_Rq = 2 "Mph"
  IPMA_NODATA  0x3D9 src 2  00000132--592eb3d350--6, Sat 2026-09-12 14:18 PT     IsaVLimUnit_D_Rq = 3 "NoDataExists"
  CLUSTER_ENG  0x430 src 0  0000012f--817cabeb76--1                             MetricActv_B_Actl = 0 (English)
The owner's cluster is mph, so a Kph / Metric frame was never observed: those two are the real frames with ONLY the
unit bits changed (IsaVLimUnit_D_Rq -> 1, MetricActv_B_Actl -> 1), checked against the DBC below.
"""
import pytest

from opendbc.can.parser import CANParser
from opendbc.car import gen_empty_fingerprint, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.carlog import carlog
from opendbc.car.common.conversions import Conversions as CV
from opendbc.car.ford import carstate as ford_carstate
from opendbc.car.ford.values import CAR

SpeedUnit = structs.CarState.CruiseState.SpeedUnit
DBC = "ford_lincoln_base_pt"
DT = 10_000_000

IPMA_MPH = "fe8cfe28401800f5"
IPMA_NODATA = "feccfe10401800fb"
IPMA_KPH = "fe4cfe28401800f5"        # IPMA_MPH with bits 15..14 = 01
CLUSTER_ENG = "09008cd108046401"
CLUSTER_METRIC = "49008cd108046401"  # CLUSTER_ENG with bit 6 = 1


def _decode(msg, sig, hexdat, addr, bus):
  p = CANParser(DBC, [(msg, float("nan"))], bus)
  p.update([(DT, [(addr, bytes.fromhex(hexdat), bus)])])
  return p.vl[msg][sig]


def test_the_payloads_mean_what_the_names_say():
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_MPH, 0x3D9, 2) == 2        # "Mph"
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_NODATA, 0x3D9, 2) == 3     # "NoDataExists"
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_KPH, 0x3D9, 2) == 1        # "Kph"
  assert _decode("Cluster_Info1_FD1", "MetricActv_B_Actl", CLUSTER_ENG, 0x430, 0) == 0
  assert _decode("Cluster_Info1_FD1", "MetricActv_B_Actl", CLUSTER_METRIC, 0x430, 0) == 1
  # only the unit bits differ: every other IPMA_Data2 signal decodes the same
  for sig in ("IsaVLim_D_Rq", "IaccVLim_D_Rq", "TsrRegionTxt_D_Stat", "LongCtrlEnbl_D_Rq"):
    assert _decode("IPMA_Data2", sig, IPMA_KPH, 0x3D9, 2) == _decode("IPMA_Data2", sig, IPMA_MPH, 0x3D9, 2)


@pytest.fixture
def logs(monkeypatch):
  seen = []

  def capture(level):
    def log(msg, *a, **kw):
      if "truckdecode2pnw" in str(msg):
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
    self.cs = self.CI.update([(self.t, [])])

  def run(self, seconds, frames=()):
    """100 Hz; `frames` (addr, hex, src) are sent every tick."""
    batch = [(a, bytes.fromhex(h), src) for a, h, src in frames]
    for _ in range(int(round(seconds * 100))):
      self.t += DT
      self.cs = self.CI.update([(self.t, batch)])
    return self.cs


def unit_of(ipma=None, cluster=None, ipma_src=2, seconds=1.0):
  car = Car()
  frames = ([(0x3D9, ipma, ipma_src)] if ipma else []) + ([(0x430, cluster, 0)] if cluster else [])
  return car.run(seconds, frames).cruiseState.speedClusterUnit


class TestDecode:
  def test_real_mph_frames_are_mph(self, logs):
    assert unit_of(IPMA_MPH, CLUSTER_ENG) == SpeedUnit.mph

  def test_both_metric_is_kph(self, logs):
    assert unit_of(IPMA_KPH, CLUSTER_METRIC) == SpeedUnit.kph

  def test_real_nodataexists_is_unknown(self, logs):
    assert unit_of(IPMA_NODATA, CLUSTER_ENG) == SpeedUnit.unknown

  @pytest.mark.parametrize("ipma, cluster", [(IPMA_KPH, CLUSTER_ENG), (IPMA_MPH, CLUSTER_METRIC)])
  def test_disagreement_is_unknown(self, logs, ipma, cluster):
    """A Kph speed-limit unit on an English cluster is exactly the sign-region case: never believed alone."""
    assert unit_of(ipma, cluster) == SpeedUnit.unknown

  def test_camera_message_missing_is_unknown(self, logs):
    assert unit_of(None, CLUSTER_ENG) == SpeedUnit.unknown

  def test_cluster_message_missing_is_unknown(self, logs):
    """MetricActv_B_Actl's English value is the parser's default 0: a never-received cluster frame must not read English."""
    assert unit_of(IPMA_MPH, None) == SpeedUnit.unknown

  def test_the_relayed_copy_on_bus0_does_not_decode(self, logs):
    """0x3D9 originates on the camera bus; the panda's relay onto bus 0 carries src 128 and is not the camera."""
    assert unit_of(IPMA_MPH, CLUSTER_ENG, ipma_src=128) == SpeedUnit.unknown

  def test_speed_is_untouched_by_the_unit(self, logs):
    """cruiseState.speed keeps upstream's CAN FD mph assumption whatever the unit says: ICBM and the resume
    executor are built on it. Only speedClusterUnit changes."""
    from opendbc.can.packer import CANPacker
    eb = CANPacker(DBC).make_can_msg("EngBrakeData", 0, {"Veh_V_DsplyCcSet": 55, "CcStat_D_Actl": 5})
    speeds = {}
    for name, ipma, cluster in (("mph", IPMA_MPH, CLUSTER_ENG), ("kph", IPMA_KPH, CLUSTER_METRIC)):
      car = Car()
      cs = car.run(1.0, [(0x3D9, ipma, 2), (0x430, cluster, 0), (eb[0], eb[1].hex(), 0)])
      speeds[name] = (cs.cruiseState.speed, cs.cruiseState.speedClusterUnit)
    assert speeds["mph"] == (pytest.approx(55 * CV.MPH_TO_MS), SpeedUnit.mph)
    assert speeds["kph"] == (pytest.approx(55 * CV.MPH_TO_MS), SpeedUnit.kph)


class TestRegistration:
  def test_ipma_data2_is_on_the_camera_parser_and_ignore_alive(self):
    from opendbc.car import Bus
    car = Car()
    cam = car.CI.can_parsers[Bus.cam]
    st = cam.message_states[0x3D9]
    assert st.ignore_alive, "a missing unit message must never make canValid false"

  def test_absence_never_costs_can_valid(self):
    """The failure that would matter: the camera parser's can_valid is ANDed into carState.canValid. Drive a real
    alive-checked camera message (registered lazily, exactly as carstate's cp_cam.vl indexing does) with zero
    IPMA_Data2 past the timeout horizon: still valid. Control: stop the real message and it must go invalid."""
    from opendbc.can.packer import CANPacker
    from opendbc.car import Bus
    cam = Car().CI.CS.get_can_parsers(Car().CI.CP)[Bus.cam]
    assert 0x3D9 in cam.message_states
    cam.vl["ACCDATA_3"]                                   # lazily registered, alive-checked
    addr, dat, _ = CANPacker(DBC).make_can_msg("ACCDATA_3", 2, {})
    t = 0
    for i in range(600):                                  # 6 s, 100 Hz, no IPMA_Data2 at all
      t += DT
      cam.update([(t, [(addr, dat, 2)])])
      assert cam.can_valid, f"IPMA_Data2 silence invalidated the camera parser at tick {i}"
    for _ in range(600):
      t += DT
      cam.update([(t, [])])
      last = cam.can_valid
    assert not last, "control failed: the camera parser must still enforce its real messages"

  def test_non_canfd_ford_is_untouched(self, logs):
    car = Car(CAR.FORD_EXPLORER_MK6)
    from opendbc.car import Bus
    from opendbc.car.ford.values import FordFlags
    assert not car.CI.CP.flags & FordFlags.CANFD
    assert 0x3D9 not in car.CI.can_parsers[Bus.cam].message_states
    # past the never-received threshold: a car that does not decode the unit must not report it missing either
    assert car.run(ford_carstate.UNIT_LOG_S + 5, [(0x3D9, IPMA_MPH, 2)]).cruiseState.speedClusterUnit == SpeedUnit.unknown
    assert logs == []


class TestLogging:
  def test_normal_start_is_one_line(self, logs):
    car = Car()
    car.run(60, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_ENG, 0)])
    assert len(logs) == 1 and "unit mph" in logs[0][1] and "IsaVLimUnit_D_Rq=2" in logs[0][1], logs

  def test_never_received_is_said_after_the_threshold(self, logs):
    car = Car()
    car.run(ford_carstate.UNIT_LOG_S - 0.5, [(0x430, CLUSTER_ENG, 0)])
    assert logs == []
    car.run(60, [(0x430, CLUSTER_ENG, 0)])
    assert len(logs) == 1 and "unit unknown" in logs[0][1] and "IsaVLimUnit_D_Rq=None" in logs[0][1]
    assert "mph assumption" in logs[0][1]

  def test_a_change_to_unknown_is_logged_with_the_raw_values(self, logs):
    car = Car()
    car.run(15, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_ENG, 0)])
    car.run(15, [(0x3D9, IPMA_NODATA, 2), (0x430, CLUSTER_ENG, 0)])
    assert [m.split(" (")[0].rsplit(" ", 1)[-1] for _, m in logs] == ["mph", "unknown"], logs
    assert "IsaVLimUnit_D_Rq=3" in logs[1][1] and "MetricActv_B_Actl=0" in logs[1][1]

  def test_a_flapping_value_is_rate_limited(self, logs):
    car = Car()
    car.run(1, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_ENG, 0)])
    for _ in range(300):              # 30 s of the value changing every 50 ms
      car.run(0.05, [(0x3D9, IPMA_NODATA, 2), (0x430, CLUSTER_ENG, 0)])
      car.run(0.05, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_ENG, 0)])
    assert 2 <= len(logs) <= 1 + 30 / ford_carstate.UNIT_LOG_S + 1, logs


def test_tesla_carstate_reports_unknown():
  """The Raven's real carstate, 1 s of updates: the field is the schema default and nothing on the Tesla sets it."""
  from opendbc.car.tesla.values import CAR as TESLA
  p = TESLA.TESLA_MODEL_S_HW3
  CI = interfaces[p](interfaces[p].get_params(p, gen_empty_fingerprint(), [], False, False, False))
  cs = None
  for i in range(100):
    cs = CI.update([(i * DT, [])])
  assert cs.cruiseState.speedClusterUnit == SpeedUnit.unknown

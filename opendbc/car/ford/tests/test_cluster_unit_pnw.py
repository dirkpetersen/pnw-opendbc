"""units2pnw: carState.cruiseState.speedClusterUnit on Ford CAN FD -- the unit the cluster shows the set in.

The source is Cluster_Info1_FD1 (0x430) MetricActv_B_Actl alone: 1 kph, 0 mph, unknown until the message has been
received. IPMA_Data2 (0x3D9) IsaVLimUnit_D_Rq is telemetry only -- on the owner's truck it read "Mph" while the cluster
and Veh_V_DsplyCcSet were in km/h (drives/2026-09-14/units-kmh/DRIVE_REPORT.md).

Every frame goes through the real DBC and the real Ford CarInterface.update, the path card runs. ALL payloads below are
REAL frames from local Lightning rlogs, except IPMA_KPH (never observed: the camera kept "Mph" on the km/h truck):
  CLUSTER_ENG     0x430 src 0  0000012f--817cabeb76--1, Sat 2026-09-12 06:25 PT   MetricActv_B_Actl = 0 (English)
  CLUSTER_METRIC  0x430 src 0  0000015e--10e6920ab7--3, Mon 2026-09-14 12:40 PT   MetricActv_B_Actl = 1 (Metric)
  IPMA_MPH        0x3D9 src 2  0000012f--817cabeb76--1                            IsaVLimUnit_D_Rq = 2 "Mph"
  IPMA_MPH_KMH    0x3D9 src 2  0000015e--10e6920ab7--3 (the km/h truck)           IsaVLimUnit_D_Rq = 2 "Mph"
  IPMA_NODATA     0x3D9 src 2  00000132--592eb3d350--6, Sat 2026-09-12 14:18 PT   IsaVLimUnit_D_Rq = 3 "NoDataExists"
  IPMA_KPH        IPMA_MPH with only bits 15..14 changed to 01 (checked against the DBC below)
Set speed (0x165 EngBrakeData, src 0), all real except EB_KMH_55 (the 21:16 frame itself was only parsed on the device):
  EB_MPH_35       00000131--4c7547abb4--11, Sat 2026-09-12 12:26 PT (mph truck)      Veh_V_DsplyCcSet 35, CcStat 5 Active
  EB_KMH_42       0000013f--14c6a10731--0, Sun 2026-09-13 21:14 PT (km/h truck)     Veh_V_DsplyCcSet 42, CcStat 5 Active
  EB_KMH_42_STBY  0000013f--14c6a10731--2, Sun 2026-09-13 21:16:16 PT (Corvallis)   Veh_V_DsplyCcSet 42, CcStat 3 Standby
  EB_KMH_55       EB_KMH_42 with only the set byte changed to 55 (checked against the DBC below)
  CLUSTER_METRIC_2116  0x430, 0000013f--14c6a10731--2, 21:16:16 PT                  MetricActv_B_Actl = 1
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
IPMA_MPH_KMH = "328cfe28401800f5"
IPMA_NODATA = "feccfe10401800fb"
IPMA_KPH = "fe4cfe28401800f5"
CLUSTER_ENG = "09008cd108046401"
CLUSTER_METRIC = "49008e7908046401"
CLUSTER_METRIC_2116 = "49008e6d08046401"
EB_MPH_35 = "10cd230000000000"
EB_KMH_42 = "10cd2a0000000000"
EB_KMH_42_STBY = "50c32a0000000000"
EB_KMH_55 = "10cd370000000000"


def _decode(msg, sig, hexdat, addr, bus):
  p = CANParser(DBC, [(msg, float("nan"))], bus)
  p.update([(DT, [(addr, bytes.fromhex(hexdat), bus)])])
  return p.vl[msg][sig]


def test_the_payloads_mean_what_the_names_say():
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_MPH, 0x3D9, 2) == 2        # "Mph"
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_MPH_KMH, 0x3D9, 2) == 2    # "Mph", on the km/h truck
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_NODATA, 0x3D9, 2) == 3     # "NoDataExists"
  assert _decode("IPMA_Data2", "IsaVLimUnit_D_Rq", IPMA_KPH, 0x3D9, 2) == 1        # "Kph"
  assert _decode("Cluster_Info1_FD1", "MetricActv_B_Actl", CLUSTER_ENG, 0x430, 0) == 0
  assert _decode("Cluster_Info1_FD1", "MetricActv_B_Actl", CLUSTER_METRIC, 0x430, 0) == 1
  for sig in ("IsaVLim_D_Rq", "IaccVLim_D_Rq", "TsrRegionTxt_D_Stat", "LongCtrlEnbl_D_Rq"):
    assert _decode("IPMA_Data2", sig, IPMA_KPH, 0x3D9, 2) == _decode("IPMA_Data2", sig, IPMA_MPH, 0x3D9, 2)
  assert _decode("Cluster_Info1_FD1", "MetricActv_B_Actl", CLUSTER_METRIC_2116, 0x430, 0) == 1
  for h, want_set, want_stat in ((EB_MPH_35, 35, 5), (EB_KMH_42, 42, 5), (EB_KMH_42_STBY, 42, 3), (EB_KMH_55, 55, 5)):
    assert (_decode("EngBrakeData", "Veh_V_DsplyCcSet", h, 0x165, 0), _decode("EngBrakeData", "CcStat_D_Actl", h, 0x165, 0)) \
      == (want_set, want_stat), h
  from opendbc.can.dbc import DBC as _D
  for sig in _D(DBC).name_to_msg["EngBrakeData"].sigs:
    if sig != "Veh_V_DsplyCcSet":
      assert _decode("EngBrakeData", sig, EB_KMH_55, 0x165, 0) == _decode("EngBrakeData", sig, EB_KMH_42, 0x165, 0), sig


@pytest.fixture
def logs(monkeypatch):
  seen = []

  def capture(level):
    def log(msg, *a, **kw):
      if "units2pnw" in str(msg):
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


def unit_of(ipma=None, cluster=None, cluster_src=0, seconds=1.0):
  car = Car()
  frames = ([(0x3D9, ipma, 2)] if ipma else []) + ([(0x430, cluster, cluster_src)] if cluster else [])
  return car.run(seconds, frames).cruiseState.speedClusterUnit


class TestDecode:
  def test_real_mph_truck_is_mph(self, logs):
    assert unit_of(IPMA_MPH, CLUSTER_ENG) == SpeedUnit.mph

  def test_real_kmh_truck_is_kph_although_the_camera_says_mph(self, logs):
    """The 2026-09-14 truck, both frames real: the cluster is metric, IsaVLimUnit_D_Rq still "Mph". The old
    two-signals-must-agree decode reported this as not-kph."""
    assert unit_of(IPMA_MPH_KMH, CLUSTER_METRIC) == SpeedUnit.kph

  @pytest.mark.parametrize("ipma", [None, IPMA_MPH, IPMA_NODATA, IPMA_KPH])
  def test_the_camera_unit_never_selects(self, logs, ipma):
    assert unit_of(ipma, CLUSTER_METRIC) == SpeedUnit.kph
    assert unit_of(ipma, CLUSTER_ENG) == SpeedUnit.mph

  @pytest.mark.parametrize("ipma", [None, IPMA_MPH, IPMA_KPH])
  def test_cluster_message_never_received_is_unknown_not_mph(self, logs, ipma):
    """MetricActv_B_Actl's English value is the parser's default 0: a never-received frame must not read mph."""
    assert unit_of(ipma, None) == SpeedUnit.unknown

  def test_a_cluster_frame_on_the_camera_bus_does_not_count(self, logs):
    """The pt parser reads bus 0; a 0x430 seen only on another bus is not the GWM's frame."""
    assert unit_of(None, CLUSTER_METRIC, cluster_src=2) == SpeedUnit.unknown

  def test_the_unit_follows_a_live_change(self, logs):
    car = Car()
    assert car.run(1, [(0x430, CLUSTER_ENG, 0)]).cruiseState.speedClusterUnit == SpeedUnit.mph
    assert car.run(0.2, [(0x430, CLUSTER_METRIC, 0)]).cruiseState.speedClusterUnit == SpeedUnit.kph
    assert car.run(0.2, [(0x430, CLUSTER_ENG, 0)]).cruiseState.speedClusterUnit == SpeedUnit.mph


class TestSpeed:
  """units2pnw (2/3): cruiseState.speed is TRUE m/s -- Veh_V_DsplyCcSet in the cluster's unit."""

  def test_real_mph_truck(self, logs):
    cs = Car().run(1.0, [(0x430, CLUSTER_ENG, 0), (0x165, EB_MPH_35, 0)])
    assert (cs.cruiseState.speed, cs.cruiseState.speedClusterUnit) == (pytest.approx(35 * CV.MPH_TO_MS), SpeedUnit.mph)

  def test_real_kmh_truck(self, logs):
    cs = Car().run(1.0, [(0x3D9, IPMA_MPH_KMH, 2), (0x430, CLUSTER_METRIC, 0), (0x165, EB_KMH_42, 0)])
    assert (cs.cruiseState.speed, cs.cruiseState.speedClusterUnit) == (pytest.approx(42 * CV.KPH_TO_MS), SpeedUnit.kph)
    assert cs.cruiseState.speedCluster == pytest.approx(42 * CV.KPH_TO_MS), "the UI's vCruiseCluster source follows"

  def test_corvallis_2116_standby_42_then_engaged_55_are_kmh(self, logs):
    """Sun 2026-09-13 21:16 PT: standby 42, our SET- engaged at "55" while the truck did 34.4 mph = 55.3 km/h. With
    the cluster's real metric frame from that segment, 55 is 15.28 m/s -- the tap speed -- not 24.59."""
    car = Car()
    stby = car.run(1.0, [(0x430, CLUSTER_METRIC_2116, 0), (0x165, EB_KMH_42_STBY, 0)])
    assert (stby.cruiseState.enabled, stby.cruiseState.speed) == (False, pytest.approx(42 / 3.6))
    eng = car.run(0.2, [(0x430, CLUSTER_METRIC_2116, 0), (0x165, EB_KMH_55, 0)])
    assert (eng.cruiseState.enabled, eng.cruiseState.speed) == (True, pytest.approx(15.278, abs=1e-3))

  def test_never_received_cluster_keeps_the_mph_assumption(self, logs):
    car = Car()
    cs = car.run(ford_carstate.UNIT_LOG_S + 1, [(0x165, EB_KMH_55, 0)])
    assert (cs.cruiseState.speed, cs.cruiseState.speedClusterUnit) == (pytest.approx(55 * CV.MPH_TO_MS), SpeedUnit.unknown)
    assert len(logs) == 1 and "ASSUMES mph" in logs[0][1], logs

  def test_speed_follows_a_unit_change(self, logs):
    car = Car()
    assert car.run(1, [(0x430, CLUSTER_ENG, 0), (0x165, EB_KMH_42, 0)]).cruiseState.speed == pytest.approx(42 * CV.MPH_TO_MS)
    assert car.run(0.2, [(0x430, CLUSTER_METRIC, 0), (0x165, EB_KMH_42, 0)]).cruiseState.speed == pytest.approx(42 * CV.KPH_TO_MS)

  def test_non_canfd_ford_speed_is_upstreams(self, logs):
    """A metric 0x430 must not reach a non-CAN FD Ford: its speed keeps upstream's INSTRUMENT_PANEL.METRIC_UNITS path."""
    cs = Car(CAR.FORD_EXPLORER_MK6).run(1.0, [(0x430, CLUSTER_METRIC, 0), (0x165, EB_KMH_42, 0)])
    assert cs.cruiseState.speed == pytest.approx(42 * CV.MPH_TO_MS)


class TestRegistration:
  def test_ipma_data2_is_on_the_camera_parser_and_ignore_alive(self):
    from opendbc.car import Bus
    car = Car()
    cam = car.CI.can_parsers[Bus.cam]
    st = cam.message_states[0x3D9]
    assert st.ignore_alive, "a missing telemetry message must never make canValid false"

  def test_absence_never_costs_can_valid(self):
    """The camera parser's can_valid is ANDed into carState.canValid. Drive a real alive-checked camera message with
    zero IPMA_Data2 past the timeout horizon: still valid. Control: stop the real message and it must go invalid."""
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
    # past the never-received threshold, with the metric frame present: the decode is CAN FD only
    cs = car.run(ford_carstate.UNIT_LOG_S + 5, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_METRIC, 0)])
    assert cs.cruiseState.speedClusterUnit == SpeedUnit.unknown
    assert logs == []


class TestLogging:
  def test_normal_start_is_one_line_with_both_raw_values(self, logs):
    car = Car()
    car.run(60, [(0x3D9, IPMA_MPH_KMH, 2), (0x430, CLUSTER_METRIC, 0)])
    assert len(logs) == 1 and "unit kph" in logs[0][1], logs
    assert "MetricActv_B_Actl=1" in logs[0][1] and "IsaVLimUnit_D_Rq=2" in logs[0][1], logs

  def test_never_received_is_said_after_the_threshold(self, logs):
    car = Car()
    car.run(ford_carstate.UNIT_LOG_S - 0.5, [(0x3D9, IPMA_MPH, 2)])
    assert logs == []
    car.run(60, [(0x3D9, IPMA_MPH, 2)])
    assert len(logs) == 1 and "unit unknown" in logs[0][1] and "MetricActv_B_Actl=None" in logs[0][1], logs
    assert "mph" in logs[0][1].split(" -- ")[1]

  def test_a_change_is_logged_with_the_raw_values(self, logs):
    car = Car()
    car.run(15, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_ENG, 0)])
    car.run(15, [(0x3D9, IPMA_MPH, 2), (0x430, CLUSTER_METRIC, 0)])
    assert [m.split(" (")[0].rsplit(" ", 1)[-1] for _, m in logs] == ["mph", "kph"], logs
    assert "MetricActv_B_Actl=0" in logs[0][1] and "MetricActv_B_Actl=1" in logs[1][1]

  def test_a_flapping_value_is_rate_limited(self, logs):
    car = Car()
    car.run(1, [(0x430, CLUSTER_ENG, 0)])
    for _ in range(300):              # 30 s of the bit changing every 50 ms
      car.run(0.05, [(0x430, CLUSTER_METRIC, 0)])
      car.run(0.05, [(0x430, CLUSTER_ENG, 0)])
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

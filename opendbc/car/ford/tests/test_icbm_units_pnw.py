"""units2pnw (3/3): the ICBM executor's tap is one CLUSTER unit -- 1 km/h on a km/h cluster.

carState.cruiseState.speed is true m/s (2/3), but one SET tap moves the set by one unit of the cluster. Measured on the
owner's truck after its cluster switched to km/h (qlog 0000013f seg 0, Sun 2026-09-13 21:14:22-37 PT): 42 -> 27 -> 42
in steps of exactly 1 at the executor's 0.4 s cadence. The deadband and both RestoreGuard thresholds are in taps.
"""
import time
from types import SimpleNamespace

import pytest

from opendbc.car import structs
from opendbc.car.ford.icbm_pnw import (DEADBAND_MS, STEP_KPH_MS, STEP_MS, TAP_PERIOD_S, IcbmCommand, PressGovernor,
                                       RestoreGuard, decide_press)

SpeedUnit = structs.CarState.CruiseState.SpeedUnit
NOW = 1000.0
K = 1.0 / 3.6


def cap(target_ms, ceiling_ms=30.0, ts=NOW, d="dec"):
  return IcbmCommand(target_ms=target_ms, ceiling_ms=ceiling_ms, ts=ts, dir=d)


def test_the_kmh_step():
  assert STEP_KPH_MS == pytest.approx(0.27778, abs=1e-5)
  assert STEP_MS == pytest.approx(0.44704)


class TestDeadband:
  def test_a_cap_less_than_one_mph_tap_but_more_than_0p6_kmh_taps_above_the_target_presses_on_kmh(self):
    """43 km/h set, target 42.1 km/h: 0.25 m/s above. That is 0.9 of a km/h tap -- press -- but under the 1 mph
    deadband (0.268), which would leave the set a whole km/h above the target."""
    stock, target = 43 * K, 42.1 * K
    assert DEADBAND_MS > stock - target > 0.6 * STEP_KPH_MS
    assert decide_press(stock, cap(target), NOW, True, False, STEP_KPH_MS) == "dec"
    assert decide_press(stock, cap(target), NOW, True, False) is None, "control: the mph default keeps its deadband"

  def test_no_chatter_within_0p6_of_a_kmh_tap(self):
    assert decide_press(42.5 * K, cap(42 * K), NOW, True, False, STEP_KPH_MS) is None
    assert decide_press(41.5 * K, cap(42 * K, 42 * K, d="inc"), NOW, True, False, STEP_KPH_MS) is None

  def test_restore_presses_up_by_one_kmh_tap(self):
    assert decide_press(41 * K, cap(42 * K, 42 * K, d="inc"), NOW, True, False, STEP_KPH_MS) == "inc"
    # 0.25 m/s short of the ceiling: 0.9 of a km/h tap (press), inside the 1 mph deadband (would stop a km/h short)
    assert decide_press(41.1 * K, cap(42 * K, 42 * K, d="inc"), NOW, True, False, STEP_KPH_MS) == "inc"
    assert decide_press(41.1 * K, cap(42 * K, 42 * K, d="inc"), NOW, True, False) is None
    assert decide_press(42 * K, cap(45 * K, 42 * K, d="inc"), NOW, True, False, STEP_KPH_MS) is None   # ceiling

  def test_the_default_is_the_mph_tap(self):
    """Every existing caller that passes no step keeps 1 mph: an mph cluster is unchanged."""
    s = 60 * 0.44704
    for d in (DEADBAND_MS - 0.01, DEADBAND_MS + 0.01):
      assert decide_press(s + d, cap(s, 40.0), NOW, True, False) == decide_press(s + d, cap(s, 40.0), NOW, True, False, STEP_MS)


class TestRestoreGuard:
  def test_own_kmh_cadence_passes(self):
    g, t, s = RestoreGuard(), NOW, 40 * K
    assert g.filter("inc", s, t, True, 45 * K, False, STEP_KPH_MS) == "inc"
    for _ in range(5):
      t += TAP_PERIOD_S
      s += STEP_KPH_MS
      assert g.filter("inc", s, t, True, 45 * K, False, STEP_KPH_MS) == "inc"
    assert not g.blocked

  def test_two_driver_kmh_taps_in_one_frame_block_on_kmh(self):
    """+2 km/h (0.556 m/s) inside one 10 ms frame is faster than our taps can move it: a human. On the 1 mph tap
    thresholds (0.726 m/s) the same jump passes -- the km/h cluster would hide two driver taps."""
    g = RestoreGuard()
    g.filter("inc", 40 * K, NOW, True, 45 * K, False, STEP_KPH_MS)
    assert g.filter("inc", 42 * K, NOW + 0.01, True, 45 * K, False, STEP_KPH_MS) is None and g.blocked
    g2 = RestoreGuard()
    g2.filter("inc", 40 * K, NOW, True, 45 * K)
    assert g2.filter("inc", 42 * K, NOW + 0.01, True, 45 * K) == "inc" and not g2.blocked

  def test_a_fractional_kmh_drop_blocks_on_kmh_only(self):
    """The decrease threshold is 0.6 of a tap: 0.2 m/s is past it on km/h (0.167), inside it on mph (0.268)."""
    g = RestoreGuard()
    g.filter("inc", 40 * K, NOW, True, 45 * K, False, STEP_KPH_MS)
    assert g.filter("inc", 40 * K - 0.2, NOW + 0.5, True, 45 * K, False, STEP_KPH_MS) is None and g.blocked
    g2 = RestoreGuard()
    g2.filter("inc", 40 * K, NOW, True, 45 * K)
    assert g2.filter("inc", 40 * K - 0.2, NOW + 0.5, True, 45 * K) == "inc"


def _executor(unit, stock_ms, target_ms, ceiling_ms=30.0, d="dec"):
  """CarController._icbm_buttons on a stub controller, with a real carState struct: the wiring from
  speedClusterUnit to the step. frame 1 skips the 4 Hz mem-param read; the command is injected."""
  from opendbc.car.ford.carcontroller import CarController
  stub = SimpleNamespace(frame=1, _icbm_params=None, _last_dec_ts=None, _sa_cmd=None, _icbm_guard=RestoreGuard(),
                         _icbm_governor=PressGovernor(),
                         _icbm_cmd=IcbmCommand(target_ms=target_ms, ceiling_ms=ceiling_ms, ts=time.time(), dir=d))
  out = structs.CarState()
  out.cruiseState.enabled = True
  out.cruiseState.speed = stock_ms
  out.cruiseState.speedClusterUnit = unit
  return CarController._icbm_buttons(stub, SimpleNamespace(out=out))


@pytest.mark.parametrize("unit, want", [(SpeedUnit.kph, "dec"), (SpeedUnit.mph, None), (SpeedUnit.unknown, None)])
def test_the_controller_takes_the_step_from_the_cluster_unit(unit, want):
  assert _executor(unit, 43 * K, 42.1 * K) == want


def test_the_controller_restore_guard_gets_the_step_too():
  """Same stub, restore command: a +2 km/h jump between two controller frames latches the guard only on kph."""
  from opendbc.car.ford.carcontroller import CarController
  for unit, blocked in ((SpeedUnit.kph, True), (SpeedUnit.mph, False)):
    stub = SimpleNamespace(frame=1, _icbm_params=None, _last_dec_ts=None, _sa_cmd=None, _icbm_guard=RestoreGuard(),
                           _icbm_governor=PressGovernor(),
                           _icbm_cmd=IcbmCommand(target_ms=45 * K, ceiling_ms=45 * K, ts=time.time(), dir="inc"))
    for stock in (40 * K, 42 * K):
      out = structs.CarState()
      out.cruiseState.enabled = True
      out.cruiseState.speed = stock
      out.cruiseState.speedClusterUnit = unit
      CarController._icbm_buttons(stub, SimpleNamespace(out=out))
    assert stub._icbm_guard.blocked is blocked, unit

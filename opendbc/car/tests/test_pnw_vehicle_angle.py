"""
angleenable — unit tests for PnwVehicle.angle_lat (opendbc/car/pnw_vehicle.py): the driver-facing
FordAngleLateral settings toggle must only ever enable angle-primary lateral on a car with the
four_signal_lat capability (today: the Ford F-150 Lightning, the only car with the matching
flashed 4-signal/angle-mode ford.h panda safety), and must stay False on every other car
regardless of the param value. Pure unit tests against PnwVehicle directly — no CarInterface, no
CAN, no panda.

Run: pytest opendbc/car/tests/test_pnw_vehicle_angle.py -q
(needs openpilot.common.params importable — run from the pnw-pilot venv, like the other angle2pnw
gates; skipped entirely on a bare opendbc checkout, matching PnwVehicle's own guarded import.)

Note: openpilot.common.params.Params is a compiled Cython extension type (common/params_pyx.so)
and its methods can't be monkeypatched in place (`TypeError: cannot set 'get_bool' attribute of
immutable type`), so these tests replace the whole Params CLASS in the module PnwVehicle imports
it from, matching the way PnwVehicle actually calls it (`Params().get_bool(...)`, a fresh
construction each time).
"""
from unittest import mock

import pytest

from opendbc.car import structs
from opendbc.car.pnw_vehicle import PnwVehicle

pytest.importorskip("openpilot.common.params", reason="needs openpilot.common.params importable (pnw-pilot venv)")


def _cp(fingerprint: str):
  return structs.CarParams(carFingerprint=fingerprint)


def _mock_params(get_bool_return=None, get_bool_side_effect=None):
  """Patch openpilot.common.params.Params (the class PnwVehicle imports and constructs) so
  Params().get_bool(...) returns/raises what the test wants, without touching real device params."""
  mock_instance = mock.MagicMock()
  if get_bool_side_effect is not None:
    mock_instance.get_bool.side_effect = get_bool_side_effect
  else:
    mock_instance.get_bool.return_value = get_bool_return
  return mock.patch("openpilot.common.params.Params", return_value=mock_instance)


def test_angle_lat_true_when_capable_and_param_true():
  with _mock_params(get_bool_return=True):
    veh = PnwVehicle(_cp("FORD_F_150_LIGHTNING_MK1"))
  assert veh.four_signal_lat is True
  assert veh.angle_lat is True


def test_angle_lat_false_when_capable_and_param_false():
  with _mock_params(get_bool_return=False):
    veh = PnwVehicle(_cp("FORD_F_150_LIGHTNING_MK1"))
  assert veh.four_signal_lat is True
  assert veh.angle_lat is False


def test_angle_lat_false_on_tesla_regardless_of_param():
  """Tesla has no four_signal_lat capability -- angle_lat must stay False even if the param is
  somehow True (e.g. stale from a previous Ford session on the shared device)."""
  with _mock_params(get_bool_return=True):
    veh = PnwVehicle(_cp("TESLA_MODEL_S_HW3"))
  assert veh.four_signal_lat is False
  assert veh.angle_lat is False


def test_angle_lat_false_on_other_ford_regardless_of_param():
  """A Ford platform without the flashed 4-signal/angle-mode safety build (four_signal_lat False)
  must never get angle_lat True even with the param on -- the capability gate, not the param
  alone, is what proves the matching panda safety is present."""
  with _mock_params(get_bool_return=True):
    veh = PnwVehicle(_cp("FORD_F_150_MK14"))
  assert veh.four_signal_lat is False
  assert veh.angle_lat is False


def test_angle_lat_false_when_params_read_raises():
  """RUNTIME-GUARDED: any failure reading/importing Params (e.g. a bare opendbc checkout, or a
  transient params error) must leave angle_lat False, never raise out of PnwVehicle.__init__."""
  with _mock_params(get_bool_side_effect=RuntimeError("boom")):
    veh = PnwVehicle(_cp("FORD_F_150_LIGHTNING_MK1"))
  assert veh.angle_lat is False


def test_angle_lat_false_with_no_carparams():
  veh = PnwVehicle(None)
  assert veh.angle_lat is False

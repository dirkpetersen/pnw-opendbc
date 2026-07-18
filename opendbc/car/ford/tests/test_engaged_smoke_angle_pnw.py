"""
angle2pnw — engaged-path smoke test for the bp-7.0 angle-primary lateral strategy
(LateralAngleExt), mirroring test_engaged_smoke_pnw.py's discipline for the 4-signal path (born
from the 2026-07-11 numpy/capnp field failure that test closes).

FIRST PASS: PnwVehicle.angle_lat is hardcoded False (see pnw_vehicle.py), so CarController never
constructs LateralAngleExt in production. This file proves two separate things that must BOTH
hold before this port is safe to flip live:

  1. The master gate is genuinely off — a normal CarInterface never constructs the angle-mode
     extension (test_angle_mode_off_by_default). Silently skipping this and only testing #2 would
     miss a bug where the capability accidentally activates.
  2. The MECHANISM itself is correct when exercised directly (bypassing the gate by assigning
     ci.CC._latext_angle post-construction, exactly as fordsafety2pnw's own
     test_ext_failures_fall_back_not_crash bypasses _longext/_latext) — full engaged loop at real
     speeds/curvatures, asserting the angle path is actually ACTIVE (no silent-fallback false
     pass — the exact 2026-07-11 gap) and outputs are capnp-safe floats, plus the mandatory
     fault-injection fallback proof (gate 5: inject a raising stub, assert the stock path resumes).

Run: pytest opendbc/car/ford/tests/test_engaged_smoke_angle_pnw.py -q
(needs cereal on PYTHONPATH — run from the pnw-pilot venv, like the deploy gates do)
"""

import importlib.util
import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR as FORD

HAVE_CEREAL = importlib.util.find_spec("cereal") is not None

V_EGO_SWEEP = [0.0, 5.0, 15.0, 31.0]
CURVATURE_SWEEP = [0.0, 0.002, -0.002, 0.02, -0.02]
FRAMES_PER_CASE = 60


def _make_interface(platform, alpha_long=False):
  CarInterface = interfaces[platform]
  fingerprints = {b: {} for b in range(7)}
  car_params = CarInterface.get_params(platform, fingerprints, [], alpha_long=alpha_long,
                                       is_release=False, docs=False)
  return CarInterface(car_params.as_reader())


def _set_speed(ci, v_ego):
  ci.CS.out.vEgo = float(v_ego)
  ci.CS.out.vEgoRaw = float(v_ego)
  ci.CS.out.standstill = v_ego < 0.1
  ci.CS.out.canValid = True


def _engaged_cc(curvature):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = True
  cc.longActive = False
  cc.actuators.curvature = float(curvature)
  cc.cruiseControl.cancel = False
  return cc.as_reader()


def test_angle_mode_off_by_default():
  """The master gate (PnwVehicle.angle_lat) is hardcoded False this pass — a normal
  CarInterface must NEVER construct LateralAngleExt. Guards against the capability
  accidentally activating (e.g. a future edit to pnw_vehicle.py flipping the fingerprint gate
  without also wiring the runtime toggle this pass deliberately omits)."""
  ci = _make_interface(FORD.FORD_F_150_LIGHTNING_MK1)
  ci.update([])
  assert ci.CC._latext_angle is None, \
    "angle2pnw's master gate is not truly off — LateralAngleExt got constructed"
  # the 4-signal path must still be the live default (untouched by this port)
  if HAVE_CEREAL:
    assert ci.CC._latext is not None


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH — run from the pnw-pilot venv")
def test_angle_mode_engaged_smoke():
  """Bypass the (hardcoded-off) capability gate by assigning _latext_angle directly — proves the
  MECHANISM (not the gate) is correct: full engaged loop at real speeds/curvatures, asserts the
  angle path is actually ACTIVE (nonzero path_angle for nonzero curvature at speed — the exact
  silent-fallback trap the 4-signal smoke test was born from) and every actuator output is a
  plain, capnp-safe float."""
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt, FORD_DBC_PATH_ANGLE_MIN, FORD_DBC_PATH_ANGLE_MAX

  ci = _make_interface(FORD.FORD_F_150_LIGHTNING_MK1)
  ci.update([])
  assert ci.CC._latext is not None, "4-signal path must be live before we swap it out below"
  ci.CC._latext = None  # mutually exclusive with the angle path, matching carcontroller's dispatch
  ci.CC._latext_angle = LateralAngleExt(ci.CP)

  now_nanos = 0
  saw_nonzero_path_angle = False
  for v_ego in V_EGO_SWEEP:
    _set_speed(ci, v_ego)
    for curv in CURVATURE_SWEEP:
      cc = _engaged_cc(curv)
      for _ in range(FRAMES_PER_CASE):
        new_actuators, _can_sends = ci.apply(cc, now_nanos)
        now_nanos += int(DT_CTRL * 1e9)

      assert ci.CC._latext_angle is not None, \
        f"LateralAngleExt silently fell back to stock at vEgo={v_ego} curv={curv} — the 2026-07-11 gap"
      pa = ci.CC._latext_angle.path_angle_last
      assert FORD_DBC_PATH_ANGLE_MIN <= pa <= FORD_DBC_PATH_ANGLE_MAX, \
        f"path_angle {pa} outside DBC range at vEgo={v_ego} curv={curv}"
      if v_ego > 1.0 and curv != 0.0 and abs(pa) > 1e-6:
        saw_nonzero_path_angle = True

      for field in ("curvature", "accel", "gas"):
        val = getattr(new_actuators, field)
        assert type(val) is float, f"new_actuators.{field} is {type(val)} at vEgo={v_ego}"
      # curvature (c2) must stay pinned at the inactive sentinel on the wire in angle mode
      assert new_actuators.curvature == 0.0

  assert saw_nonzero_path_angle, \
    "angle path never produced a nonzero path_angle across the whole sweep — looks degenerate"


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH — run from the pnw-pilot venv")
def test_angle_mode_fault_injection_falls_back():
  """Gate 5: a failure in LateralAngleExt must fall back to the stock curvature-only path for the
  drive, never raise out of apply() / kill card — same never-kill-card discipline as
  test_ext_failures_fall_back_not_crash proves for the 4-signal path."""
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt

  ci = _make_interface(FORD.FORD_F_150_LIGHTNING_MK1)
  ci.update([])
  ci.CC._latext = None
  ci.CC._latext_angle = LateralAngleExt(ci.CP)
  _set_speed(ci, 15.0)
  cc = _engaged_cc(0.005)

  # one healthy frame first, to prove the extension is really live before we break it
  na, _ = ci.apply(cc, 0)
  assert ci.CC._latext_angle is not None

  class BoomAngle:
    def update_sm(self): pass
    def update(self, *a, **k): raise RuntimeError("injected")

  ci.CC._latext_angle = BoomAngle()
  now = int(DT_CTRL * 1e9)
  for _ in range(60):
    na, _can_sends = ci.apply(cc, now)  # must not raise
    now += int(DT_CTRL * 1e9)
    assert type(na.curvature) is float

  assert ci.CC._latext_angle is None, "fault injection did not disarm the failing extension"

  # stock path resumes cleanly on subsequent frames (no lingering half-broken state)
  for _ in range(10):
    na, _can_sends = ci.apply(cc, now)
    now += int(DT_CTRL * 1e9)
  assert type(na.curvature) is float

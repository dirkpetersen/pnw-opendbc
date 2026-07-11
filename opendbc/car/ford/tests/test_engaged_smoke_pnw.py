"""
fordsafety2pnw — engaged-path smoke test.

Born from the 2026-07-11 field failure: the 4-signal LateralCurvExt math returns
numpy.float64, and the uncast `new_actuators.curvature = self.apply_curvature_last`
raised capnp KjException ("unsupported type") the moment lateral engaged — card died
on every cruise-button press, on the road. Two prior test layers missed it:

  * test_car_interfaces fuzzes with vEgo=0, where the lateral ext short-circuits to a
    plain-float 0.0 and the numpy path never runs;
  * on a bare opendbc checkout the LateralCurvExt import fails and the carcontroller
    SILENTLY falls back to the stock path (by design, see carcontroller.__init__), so
    a test environment without cereal greenlights code it never executed.

This test closes both holes: it requires the 4-signal path to be ACTIVE (loud skip if
cereal is genuinely absent, hard fail if construction breaks for any other reason),
then drives the full engaged control loop at realistic speeds and curvatures and
asserts every actuator output is a plain Python float.

Run: pytest opendbc/car/ford/tests/test_engaged_smoke_pnw.py -q
(needs cereal on PYTHONPATH — run from the pnw-pilot venv, like the deploy gates do)
"""

import importlib.util
import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR as FORD

HAVE_CEREAL = importlib.util.find_spec("cereal") is not None

# (platform, expects_4signal) — the Lightning must run the 4-signal path; the other
# Fords exercise the same carcontroller via their stock/pc_blend paths.
PLATFORMS = [
  (FORD.FORD_F_150_LIGHTNING_MK1, True),
  (FORD.FORD_F_150_MK14, False),
]

V_EGO_SWEEP = [0.0, 5.0, 15.0, 31.0]          # m/s: standstill, city, highway entry, freeway
CURVATURE_SWEEP = [0.0, 0.002, -0.002, 0.02, -0.02]  # 1/m: straight, gentle, near-limit
FRAMES_PER_CASE = 60                          # covers STEER_STEP(5) and ACC_UI_STEP(20) multiple times


def _make_interface(platform, alpha_long=False):
  CarInterface = interfaces[platform]
  fingerprints = {b: {} for b in range(7)}
  car_params = CarInterface.get_params(platform, fingerprints, [], alpha_long=alpha_long,
                                       is_release=False, docs=False)
  return CarInterface(car_params.as_reader())


def _set_speed(ci, v_ego):
  # CS.out is opendbc structs.CarState — a plain mutable Python object, not capnp
  ci.CS.out.vEgo = float(v_ego)
  ci.CS.out.vEgoRaw = float(v_ego)
  ci.CS.out.standstill = v_ego < 0.1
  ci.CS.out.canValid = True


def _engaged_cc(curvature):
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = True
  cc.longActive = False        # ICBM/op-long orthogonal; lateral is what crashed
  cc.actuators.curvature = float(curvature)
  cc.cruiseControl.cancel = False
  return cc.as_reader()


@pytest.mark.parametrize("platform,expects_4signal", PLATFORMS)
def test_engaged_lateral_smoke(platform, expects_4signal):
  ci = _make_interface(platform)
  ci.update([])  # seed CS.out

  if expects_4signal:
    if not HAVE_CEREAL:
      pytest.skip("cereal not on PYTHONPATH — 4-signal path CANNOT be exercised; "
                  "run from the pnw-pilot venv (this skip must not appear in deploy gates)")
    # The silent-fallback trap: if LateralCurvExt construction failed, the controller
    # quietly runs the stock path and this test would pass while testing nothing.
    assert ci.CC._latext is not None, \
      "4-signal LateralCurvExt is NOT active despite four_signal_lat capability — " \
      "silent fallback would ship untested lateral code (the 2026-07-11 gap)"

  now_nanos = 0
  for v_ego in V_EGO_SWEEP:
    _set_speed(ci, v_ego)
    for curv in CURVATURE_SWEEP:
      cc = _engaged_cc(curv)
      for _ in range(FRAMES_PER_CASE):
        # KjException on numpy leakage raises HERE (this is the exact line-370 crash)
        new_actuators, _can_sends = ci.apply(cc, now_nanos)
        now_nanos += int(DT_CTRL * 1e9)

      # belt-and-suspenders: outputs must be capnp-safe plain floats
      for field in ("curvature", "accel", "gas"):
        val = getattr(new_actuators, field)
        assert type(val) is float, f"{platform} new_actuators.{field} is {type(val)} at vEgo={v_ego}"


def test_disengaged_then_engage_transition():
  """The field failure fired on the disengaged->engaged edge (cruise-button press).
  Replay that exact transition at speed."""
  ci = _make_interface(FORD.FORD_F_150_LIGHTNING_MK1)
  ci.update([])
  if not HAVE_CEREAL:
    pytest.skip("cereal not on PYTHONPATH")
  _set_speed(ci, 15.0)

  now_nanos = 0
  disengaged = structs.CarControl().as_reader()
  for _ in range(40):
    ci.apply(disengaged, now_nanos)
    now_nanos += int(DT_CTRL * 1e9)

  engaged = _engaged_cc(0.004)
  for _ in range(40):
    new_actuators, _ = ci.apply(engaged, now_nanos)
    now_nanos += int(DT_CTRL * 1e9)
  assert type(new_actuators.curvature) is float


def test_oplong_bp_follow_smoke():
  """fordlong2pnw: op-long engaged sweep through the BP follow path (LongitudinalExt),
  incl. above the 50 mph deadband; outputs must stay plain floats."""
  if not HAVE_CEREAL:
    pytest.skip("cereal not on PYTHONPATH")
  ci = _make_interface(FORD.FORD_F_150_LIGHTNING_MK1, alpha_long=True)
  ci.update([])
  assert ci.CC._latext is not None
  assert ci.CC._longext is not None, "bp_long_follow capability did not construct LongitudinalExt"

  now = 0
  for v_ego in [0.0, 15.0, 25.0, 31.0]:
    _set_speed(ci, v_ego)
    for accel in [0.5, 0.0, -1.5, -3.0]:
      cc = structs.CarControl()
      cc.enabled = True
      cc.latActive = True
      cc.longActive = True
      cc.actuators.curvature = 0.002
      cc.actuators.accel = float(accel)
      cc = cc.as_reader()
      for _ in range(FRAMES_PER_CASE):
        na, _ = ci.apply(cc, now)
        now += int(DT_CTRL * 1e9)
      for field in ("curvature", "accel", "gas"):
        assert type(getattr(na, field)) is float


def test_ext_failures_fall_back_not_crash():
  """Injected failures in EITHER ext must fall back to stock paths, never raise out of apply()."""
  if not HAVE_CEREAL:
    pytest.skip("cereal not on PYTHONPATH")
  ci = _make_interface(FORD.FORD_F_150_LIGHTNING_MK1, alpha_long=True)
  ci.update([])
  _set_speed(ci, 15.0)
  cc = structs.CarControl()
  cc.enabled = True
  cc.latActive = True
  cc.longActive = True
  cc.actuators.curvature = 0.002
  cc.actuators.accel = -1.0
  cc = cc.as_reader()

  class BoomLong:
    disable_downhill_comp_UI = True
    def update(self, *a, **k): raise RuntimeError("injected")
  class BoomLat:
    def update_sm(self): pass
    def update(self, *a, **k): raise RuntimeError("injected")

  ci.CC._longext = BoomLong()
  now = 0
  for _ in range(60):
    ci.apply(cc, now); now += int(DT_CTRL * 1e9)
  assert ci.CC._longext is None, "long fallback did not disarm the failing ext"

  ci.CC._latext = BoomLat()
  for _ in range(60):
    ci.apply(cc, now); now += int(DT_CTRL * 1e9)
  assert ci.CC._latext is None, "lat fallback did not disarm the failing ext"

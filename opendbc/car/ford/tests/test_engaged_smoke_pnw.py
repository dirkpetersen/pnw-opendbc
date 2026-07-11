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


def _make_interface(platform):
  CarInterface = interfaces[platform]
  fingerprints = {b: {} for b in range(7)}
  car_params = CarInterface.get_params(platform, fingerprints, [], alpha_long=False,
                                       is_release=False, docs=False)
  return CarInterface(car_params.as_reader())


def _set_speed(ci, v_ego):
  cs = ci.CS.out.as_builder()
  cs.vEgo = float(v_ego)
  cs.vEgoRaw = float(v_ego)
  cs.standstill = v_ego < 0.1
  cs.canValid = True
  ci.CS.out = cs.as_reader()


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
      for field in ("curvature", "accel", "gas", "steer"):
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

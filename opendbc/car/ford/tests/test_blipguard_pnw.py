"""
blipguard2pnw -- the PSCM-unstick blip must never fire mid-curve.

Both blips in lateral_angle_pnw.py (the proactive hand-off blip on the falling edge of a sustained
driver press, and the reactive post-override stall blip) hold lateral inactive for
_STALL_BLIP_FRAMES -- a real 300 ms steering release. Harmless on a straight; mid-curve it is
300 ms of no lateral command while the road is turning. Ported from BluePilot bp-dev 9012f76666,
which gates both fire sites on ``abs(path_angle_last) < _BLIP_MAX_PATH_ANGLE``.

These drive the REAL update() loop (stub CC/CS/actuators, real SubMaster). Where a test needs a
specific frame to be the fire frame it says so and arms the accumulator explicitly; the two
behavioural tests below arm NOTHING -- the stall accumulates on its own from the commanded
geometry, so a blip that fires during the setup phase is a test failure rather than something the
fixture papers over.

The stubs carry no modelV2, so ``predicted_curvature`` is 0 and the exit-biased blend would halve
every commanded curvature before the deviation clip sees it, forcing physically absurd inputs to
reach a realistic regime. Each fixture therefore sets ``path_angle_blend_ratio = 0`` (and pins
_apply_tuning) so the numbers below are the real ones: 0-3.75 m/s^2 of measured lateral, which is
what these blips actually fire in.

Run: pytest opendbc/car/ford/tests/test_blipguard_pnw.py -q   (needs cereal -- pnw-pilot venv)
"""

import importlib.util
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR as FORD, CarControllerParams

HAVE_CEREAL = importlib.util.find_spec("cereal") is not None

pytestmark = pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH -- run from the pnw-pilot venv")


class _Out:
  def __init__(self):
    self.vEgo = 25.0
    self.vEgoRaw = 25.0
    self.yawRate = 0.0
    self.steeringPressed = False
    self.steeringAngleDeg = 0.0


class _CS:
  def __init__(self):
    self.out = _Out()
    self.lat_ctl_lim_stat = 0


class _CC:
  latActive = True


class _Actuators:
  def __init__(self, curvature=0.0):
    self.curvature = float(curvature)


def _ext(v_ego, measured_curvature):
  """A LateralAngleExt plus a CS whose measured curvature (= -yawRate / vEgoRaw) really is
  `measured_curvature`, so the deviation clip and path_angle see a car that is genuinely in the
  curve rather than a stub that is not turning at all."""
  CarInterface = interfaces[FORD.FORD_F_150_LIGHTNING_MK1]
  CP = CarInterface.get_params(FORD.FORD_F_150_LIGHTNING_MK1, {b: {} for b in range(7)}, [],
                               alpha_long=False, is_release=False, docs=False).as_reader()
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  ext = LateralAngleExt(CP)
  ext.path_angle_blend_ratio = 0.0        # no modelV2 in the stub -- see module docstring
  ext._apply_tuning = lambda: None        # keep the throttled tuning re-poll from undoing it
  cs = _CS()
  cs.out.vEgo = cs.out.vEgoRaw = float(v_ego)
  cs.out.yawRate = -float(measured_curvature) * float(v_ego)
  return ext, CP, cs


def _drive(ext, CP, cs, curvature, frames, pressed=False):
  cs.out.steeringPressed = pressed
  act = _Actuators(curvature)
  for _ in range(frames):
    ext.update(_CC(), cs, act, CP)


# --- proactive hand-off blip (falling edge of a sustained press) --------------------------------

@pytest.mark.parametrize("measured,expect_blip", [
  (0.000, True),    # straight: path_angle 0.000 rad
  (0.006, False),   # steady curve at 25 m/s = 3.75 m/s^2 lateral; path_angle 0.156 rad
])
def test_press_release_blip_only_on_a_straight(measured, expect_blip):
  from opendbc.car.ford.lateral_angle_pnw import _BLIP_MAX_PATH_ANGLE

  ext, CP, cs = _ext(25.0, measured)
  # Sustained press with a small wheel angle, so the human-turn override never latches and eats
  # the press timer. Commanded curvature tracks the car -- no stall, nothing else in play.
  _drive(ext, CP, cs, measured, 40, pressed=True)
  assert ext.angle_human_turn_active is False, "human-turn override latched -- fixture is wrong"
  assert ext.stall_blip_frames_left == 0, "a pulse fired during the press phase"
  assert ext.stall_blip_count == 0, "a stall pulse fired during the setup drive"

  in_curve = abs(ext.path_angle_last) >= _BLIP_MAX_PATH_ANGLE
  assert in_curve is (not expect_blip), \
    f"fixture did not reach the intended regime: path_angle_last={ext.path_angle_last}"

  # Falling edge -- this is the only frame under test.
  cs.out.steeringPressed = False
  ext.update(_CC(), cs, _Actuators(measured), CP)
  assert (ext.stall_blip_frames_left > 0) is expect_blip


# --- reactive post-override stall blip ----------------------------------------------------------

@pytest.mark.parametrize("v_ego,measured,expect_blip", [
  (20.0, 0.000, True),    # straight: path_angle stays 0.000 rad
  (25.0, 0.006, False),   # steady 3.75 m/s^2 curve: path_angle 0.156 -> 0.208 rad
])
def test_stall_blip_only_on_a_straight(v_ego, measured, expect_blip):
  """Nothing is poked here. Both cases warm up TRACKING the car (commanded == measured, so no
  stall, no deviation-clip binding, no blip), then switch to a command that leads measured by an
  IDENTICAL 0.005 1/m -- comfortably past _STALL_GAP_MIN -- and let the detector accumulate its
  own _STALL_HOLD_S. The only thing that differs between the two rows is path_angle."""
  from opendbc.car.ford.lateral_angle_pnw import _BLIP_MAX_PATH_ANGLE, _STALL_GAP_MIN

  stall_gap = 0.005
  desired = measured + stall_gap
  assert stall_gap > _STALL_GAP_MIN == 2.0 * CarControllerParams.CURVATURE_ERROR

  ext, CP, cs = _ext(v_ego, measured)
  _drive(ext, CP, cs, measured, 40)                    # tracking warm-up
  assert ext.stall_blip_count == 0 and ext.stall_blip_frames_left == 0, \
    "a blip fired during the warm-up -- the fixture would be hiding the behaviour under test"
  assert ext.stall_blip_hold_s == 0.0, "the stall accumulator was already running after warm-up"

  in_curve = abs(ext.path_angle_last) >= _BLIP_MAX_PATH_ANGLE
  assert in_curve is (not expect_blip), \
    f"fixture did not reach the intended regime: path_angle_last={ext.path_angle_last}"

  # Now stall: command leads measured, deviation clip binds, hold accumulates naturally.
  fired = False
  for _ in range(15):                                  # _STALL_HOLD_S is 10 frames at 20 Hz
    ext.update(_CC(), cs, _Actuators(desired), CP)
    fired = fired or ext.stall_blip_frames_left > 0
  assert fired is expect_blip
  assert (ext.stall_blip_count > 0) is expect_blip


def test_stall_guard_reads_this_frames_path_angle_not_last_frames():
  """Pins WHERE the guard reads. The stall site sits after `self.path_angle_last = path_angle`, so
  it must judge the command it is about to release, not the previous frame's. This drives a
  fresh ramp at 20 m/s (soft ROC 0.0258 rad/frame) so that on the frame under test the two
  straddle the threshold: last = 0.077, this = 0.103.

  This test DOES arm the accumulator (unlike the two above): the natural _STALL_HOLD_S is 10
  frames and the straddle window is 1 frame wide, so they cannot be made to coincide. Nothing
  else is poked, and the arming happens on a frame where the stall condition is already true."""
  from opendbc.car.ford.lateral_angle_pnw import _BLIP_MAX_PATH_ANGLE, _STALL_HOLD_S

  ext, CP, cs = _ext(20.0, 0.006)
  act = _Actuators(0.016)                              # leads measured by 0.010 -> stall condition
  for _ in range(3):                                   # ramp: 0.026, 0.052, 0.077
    ext.update(_CC(), cs, act, CP)
  last = ext.path_angle_last
  assert ext.stall_blip_frames_left == 0 and ext.stall_blip_count == 0

  ext.stall_blip_hold_s = _STALL_HOLD_S                # make the NEXT frame the fire frame
  ext.update(_CC(), cs, act, CP)
  this = ext.path_angle_last

  assert last < _BLIP_MAX_PATH_ANGLE <= this, \
    f"the straddle window moved: last={last}, this={this}"
  assert ext.stall_blip_frames_left == 0, \
    "the guard fired on a frame whose own path_angle is in a curve -- it is reading last frame's"


def test_guard_threshold_is_the_upstream_value():
  """Pinned: our path_angle is computed the same way as BluePilot's (kappa * v_ego * gain, same
  sign convention, same DBC range), so the threshold carries over unchanged from 9012f76666.
  A drift here is a deliberate retune, not an accident."""
  from opendbc.car.ford.lateral_angle_pnw import _BLIP_MAX_PATH_ANGLE, FORD_DBC_PATH_ANGLE_MAX
  assert _BLIP_MAX_PATH_ANGLE == 0.10
  assert 0.0 < _BLIP_MAX_PATH_ANGLE < 0.25 * FORD_DBC_PATH_ANGLE_MAX

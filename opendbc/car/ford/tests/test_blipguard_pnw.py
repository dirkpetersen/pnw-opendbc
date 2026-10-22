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
    self.aEgo = 0.0
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


# --- curveexit2pnw fix 1: no stall blip in a curve-exit correction --------------------------------

# The real 2026-10-21 22:39 PT exit (drives/2026-10-21/seattle-left-curve-overshoot, rlog seg 31), one row per 20 Hz
# steer frame: (vEgoRaw m/s, yawRate rad/s, actuators.curvature 1/m). The plan asks RIGHT (+) from 20.34 while the
# truck still yaws LEFT (yaw +, measured curvature -). Without fix 1 the stall blip fired at 22:39:22.09; with only a
# "truck crossed to the plan's side" end rule it re-armed at 22.84 in an open-loop replay.
_EXIT_2239 = [
  (8.186, +0.1206, +0.000682), (8.264, +0.1188, +0.002286), (8.186, +0.1132, +0.002382), (8.233, +0.1070, +0.002613),
  (8.294, +0.0990, +0.001571), (8.375, +0.0924, +0.002391), (8.328, +0.0910, +0.003024), (8.717, +0.0892, +0.003580),
  (8.531, +0.0842, +0.004079), (8.389, +0.0770, +0.004358), (8.500, +0.0736, +0.003537), (8.561, +0.0756, +0.004600),
  (8.639, +0.0792, +0.002436), (8.733, +0.0778, +0.004050), (8.858, +0.0744, +0.003204), (8.764, +0.0670, +0.003758),
  (8.781, +0.0602, +0.002971), (8.828, +0.0534, +0.004085), (8.875, +0.0478, +0.003801), (8.919, +0.0418, +0.004060),
  (8.967, +0.0382, +0.004578), (9.014, +0.0390, +0.004498), (9.078, +0.0402, +0.003430), (9.078, +0.0390, +0.003538),
  (9.156, +0.0358, +0.003617), (9.217, +0.0328, +0.003992), (9.264, +0.0282, +0.005156), (9.281, +0.0266, +0.005829),
  (9.342, +0.0196, +0.006000), (9.358, +0.0150, +0.006937), (9.406, +0.0090, +0.007413), (9.419, +0.0030, +0.007783),
  (9.419, +0.0016, +0.008007), (9.467, +0.0016, +0.007711), (9.531, +0.0024, +0.007488), (9.625, +0.0048, +0.008201),
  (9.639, +0.0034, +0.007970), (9.686, +0.0016, +0.007685), (9.686, +0.0008, +0.007930), (9.686, +0.0002, +0.008274),
  (9.764, -0.0048, +0.008501), (9.703, -0.0126, +0.008750), (9.733, -0.0262, +0.008745),
  # 22:39:22.44-23.04: the truck now curves right, but weakly; the plan asks for much more (no driver press yet)
  (9.781, -0.0288, +0.007938), (9.828, -0.0260, +0.008574), (9.842, -0.0200, +0.007886), (9.889, -0.0108, +0.008027),
  (9.794, -0.0090, +0.008583), (9.781, -0.0116, +0.009360), (9.811, -0.0128, +0.009310), (9.953, -0.0148, +0.009889),
  (9.906, -0.0150, +0.009469), (9.889, -0.0174, +0.009608), (9.875, -0.0184, +0.009944), (9.889, -0.0156, +0.010228),
  (9.875, -0.0152, +0.010629),
]


def _feed(ext, CP, cs, rows, v_override=None):
  """Feed (v, yaw, desired) rows; return the indices of frames on which a stall blip was armed or active."""
  blips = []
  for i, (v, yaw, des) in enumerate(rows):
    v = v_override if v_override is not None else v
    cs.out.vEgo = cs.out.vEgoRaw = v
    cs.out.yawRate = yaw if v_override is None else yaw * v_override / rows[i][0]   # keep the measured curvature
    ext.update(_CC(), cs, _Actuators(des), CP)
    if ext.stall_blip_frames_left > 0 or ext.angle_stall_blip_active:
      blips.append(i)
  return blips


def test_no_stall_blip_in_the_2239_exit():
  ext, CP, cs = _ext(8.2, -0.0147)
  _drive(ext, CP, cs, -0.0147, 20)       # in the curve, tracking
  assert _feed(ext, CP, cs, _EXIT_2239) == []
  assert ext.exit_episode is True, "the truck never tracked the plan in this window; the episode must still be open"


def test_no_stall_blip_in_the_2239_exit_above_the_panda_gate():
  """The same measured curvature and plan at 10.5 m/s, so the deviation clip binds on every frame whatever the
  clip's speed gate is (fix 2 moves it). Pins fix 1 on its own."""
  ext, CP, cs = _ext(10.5, -0.0147)
  _drive(ext, CP, cs, -0.0147, 20)
  assert _feed(ext, CP, cs, _EXIT_2239, v_override=10.5) == []


@pytest.mark.parametrize("measured", [0.0, 0.0003, -0.0003, 0.001])
def test_real_stall_still_blips(measured):
  """Straight (measured inside the noise floor, either sign) or a gentle curve the plan's way: the blip still fires."""
  ext, CP, cs = _ext(10.0, measured)
  _drive(ext, CP, cs, measured, 40)
  assert ext.stall_blip_count == 0
  _drive(ext, CP, cs, 0.006, 15)
  assert ext.stall_blip_count == 1


def test_real_stall_after_an_exit_still_blips():
  """The exit episode ends once the truck tracks the plan; a stall after that is a real stall."""
  ext, CP, cs = _ext(10.5, -0.0147)
  _drive(ext, CP, cs, -0.0147, 20)
  assert _feed(ext, CP, cs, _EXIT_2239, v_override=10.5) == []
  cs.out.yawRate = -0.007 * 10.5          # truck reaches the plan: +0.007 measured, +0.007 desired
  _drive(ext, CP, cs, 0.007, 1)
  assert ext.exit_episode is False
  cs.out.yawRate = -0.001 * 10.5          # then stalls well short of a plan that asks for more
  _drive(ext, CP, cs, 0.007, 15)
  assert ext.stall_blip_count == 1


def test_episode_time_limit_lets_a_late_real_stall_blip():
  """A truck that never tracks the plan after an exit: no blip inside _EXIT_EPISODE_MAX_S, the blip after it."""
  from opendbc.car.ford.lateral_angle_pnw import _EXIT_EPISODE_MAX_S, _STALL_HOLD_S, _STEER_DT
  ext, CP, cs = _ext(10.5, -0.0147)
  _drive(ext, CP, cs, -0.0147, 20)
  cs.out.yawRate = 0.004 * 10.5           # truck still curves left (-0.004) ...
  _drive(ext, CP, cs, 0.003, 1)           # ... while the plan asks right: the episode starts
  assert ext.exit_episode is True
  cs.out.yawRate = -0.001 * 10.5          # truck curves right weakly, plan wants much more, for good
  inside = int(round((_EXIT_EPISODE_MAX_S - 2 * _STEER_DT) / _STEER_DT))
  _drive(ext, CP, cs, 0.007, inside)
  assert ext.stall_blip_count == 0, "blip fired inside the exit episode"
  _drive(ext, CP, cs, 0.007, int(round(_STALL_HOLD_S / _STEER_DT)) + 4)
  assert ext.stall_blip_count == 1, "a stall that outlasts the episode must still get its blip"

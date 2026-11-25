"""
fordlat_pnw — pnw Ford lateral helpers (stock-panda-safety only).

HumanTurnHold: BluePilot's (alan-polk) human-turn reset, simplified for stock safety. When the
driver holds the wheel past HT_ANGLE_DEG for HT_HOLD_S (a real manual turn, not a nudge), the
commanded curvature is flushed to zero THROUGH the normal rate limiter — so on release the command
ramps back up from ~0 instead of slamming in from the value it accumulated while fighting the
driver (the "released the wheel and it steered into the other lane" lurch, driver report
2026-07-11; pre-existing, observed both pre- and post-blend). BluePilot's full version relies on a
custom panda reset latch to jump straight to 0; feeding 0 as the DESIRED value and letting the
stock rate limiter do the ramp needs no safety changes at all.
"""

import itertools
import math
import time

from opendbc.car.carlog import carlog

HT_ANGLE_DEG = 45.0   # |steering angle| above this counts as a manual turn (bp value)
HT_HOLD_S = 1.5       # sustained this long (bp value — avoids resets on in-curve nudges)
_STEER_DT = 0.05      # called at the 20 Hz steer step


class HumanTurnHold:
  """Pure & unit-tested: tick(steering_pressed, steering_angle_deg) -> True while the flush is
  active (sustained manual turn in progress)."""

  def __init__(self):
    self._hold_s = 0.0

  def tick(self, steering_pressed: bool, steering_angle_deg: float) -> bool:
    if steering_pressed and abs(steering_angle_deg) > HT_ANGLE_DEG:
      self._hold_s += _STEER_DT
    else:
      self._hold_s = 0.0
    return self._hold_s >= HT_HOLD_S


# bp71items2pnw (BluePilot bluepilot-lateral 5a31711): a NaN/inf in modelV2 does not raise in the
# lateral math; it reaches the CAN packer, which raises (int(floor(nan))), and card dies while steering.
NONFINITE_HOLD_FRAMES = 5     # model frames (20 Hz): hold the last good frame for 0.25 s, then no model terms
NONFINITE_LOG_PERIOD_S = 5.0
MODEL_IDX_N = 33  # ModelConstants.IDX_N: the consumers np.interp orientationRate.z / position.y against T_IDXS


def model_lateral_finite(model) -> bool:
  """True when every modelV2 field the Ford lateral reads is finite and long enough for how it is indexed
  (np.interp against T_IDXS; laneLines[1..2].y[0]; laneLineProbs[1..2]). Real modeld frames: 33 / 33 / 4x33 / 4."""
  if not (len(model.orientationRate.z) == MODEL_IDX_N and len(model.position.y) == MODEL_IDX_N and
          len(model.laneLines) >= 3 and len(model.laneLines[1].y) > 0 and len(model.laneLines[2].y) > 0 and
          len(model.laneLineProbs) >= 3):
    return False
  vals = itertools.chain(model.orientationRate.z, model.position.y, model.laneLineProbs,
                         *(ll.y for ll in model.laneLines))
  return all(math.isfinite(v) for v in vals)


class FiniteModelGate:
  """step(sm, current) once per control frame; filter(model) on each NEW modelV2 frame. A non-finite or short frame is dropped: the last good frame is held
  for NONFINITE_HOLD_FRAMES, then None (the lateral runs on the planner curvature only, no model terms).
  Every drop is logged (rate-limited), and the switch to None is always logged."""

  def __init__(self):
    self._good = None
    self.bad_run = 0
    self.dropped = 0
    self._last_log = -math.inf

  def filter(self, model):
    if model_lateral_finite(model):
      self._good = model
      self.bad_run = 0
      return model
    self.bad_run += 1
    self.dropped += 1
    held = self._good if self.bad_run <= NONFINITE_HOLD_FRAMES else None
    now = time.monotonic()
    if now - self._last_log >= NONFINITE_LOG_PERIOD_S or self.bad_run == NONFINITE_HOLD_FRAMES + 1:
      self._last_log = now
      carlog.error(f"bp71items2pnw: dropped modelV2 frame with NaN/inf or short arrays ({self.bad_run} in a row, " +
                   f"{self.dropped} total); " + ("holding last good frame" if held is not None else "Ford lateral runs WITHOUT model terms"))
    return held

  def step(self, sm, current):
    """The model the lateral may use this frame: the gated new frame, `current` while modelV2 is alive, else None."""
    if sm.updated['modelV2']:
      return self.filter(sm['modelV2'])
    if current is not None and not sm.alive['modelV2']:
      self._good = None  # never resume on a frame from before the outage
      now = time.monotonic()
      if now - self._last_log >= NONFINITE_LOG_PERIOD_S:
        self._last_log = now
        carlog.error("bp71items2pnw: modelV2 not alive -- Ford lateral drops the held model frame, runs WITHOUT model terms")
      return None
    return current

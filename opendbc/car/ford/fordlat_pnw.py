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

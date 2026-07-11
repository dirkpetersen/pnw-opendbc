"""icbm2pnw executor tests — pure logic, no capnp/params needed."""

from opendbc.car.ford.icbm_pnw import (IcbmCommand, decide_press, PressGovernor,
                                       MPH_TO_MS, STEP_MS, PRESS_FRAMES, GAP_FRAMES, STALE_LIMIT_S)

NOW = 1000.0


def cmd(target_mph, ceiling_mph, ts=NOW):
  return IcbmCommand(target_ms=target_mph * MPH_TO_MS, ceiling_ms=ceiling_mph * MPH_TO_MS, ts=ts)


def test_dec_toward_lower_target():
  # set 60, curve target 45 -> press down
  assert decide_press(60 * MPH_TO_MS, cmd(45, 60), NOW, True, False) == "dec"


def test_strictly_dec_only_never_inc():
  # stock set BELOW the target: a restore would need inc — dec-only design returns None, always
  assert decide_press(45 * MPH_TO_MS, cmd(60, 60), NOW, True, False) is None
  # at the target: silent
  assert decide_press(60 * MPH_TO_MS, cmd(60, 60), NOW, True, False) is None
  # brain asks ABOVE the driver ceiling: clamped, silent
  assert decide_press(60 * MPH_TO_MS, cmd(75, 60), NOW, True, False) is None


def test_deadband_no_chatter():
  # within 0.6 mph of target: no press
  assert decide_press(45.4 * MPH_TO_MS, cmd(45, 60), NOW, True, False) is None


def test_gates():
  # cruise off / driver override / stale heartbeat / no set speed -> nothing
  assert decide_press(60 * MPH_TO_MS, cmd(45, 60), NOW, False, False) is None
  assert decide_press(60 * MPH_TO_MS, cmd(45, 60), NOW, True, True) is None
  assert decide_press(60 * MPH_TO_MS, cmd(45, 60, ts=NOW - STALE_LIMIT_S - 0.1), NOW, True, False) is None
  assert decide_press(0.0, cmd(45, 60), NOW, True, False) is None
  assert decide_press(60 * MPH_TO_MS, None, NOW, True, False) is None


def test_zero_or_negative_target_never_presses():
  assert decide_press(60 * MPH_TO_MS, cmd(0, 60), NOW, True, False) is None
  assert decide_press(60 * MPH_TO_MS, IcbmCommand(-1.0, 60 * MPH_TO_MS, NOW), NOW, True, False) is None


def test_governor_tap_pattern():
  g = PressGovernor()
  presses = []
  for f in range(100):
    out = g.update(f, "dec")
    presses.append(out)
  # press asserted for PRESS_FRAMES, then released for >= GAP_FRAMES
  assert presses[:PRESS_FRAMES] == ["dec"] * PRESS_FRAMES
  assert presses[PRESS_FRAMES:PRESS_FRAMES + GAP_FRAMES] == [None] * GAP_FRAMES
  assert presses[PRESS_FRAMES + GAP_FRAMES] == "dec"      # next tap starts after the gap
  # ~3 taps in the first second (100 frames), never a continuous hold
  starts = sum(1 for i in range(1, 100) if presses[i] == "dec" and presses[i - 1] != "dec")
  assert 2 <= starts + (1 if presses[0] == "dec" else 0) <= 4


def test_governor_aborts_mid_press_when_intent_vanishes():
  # preconditions vanish DURING a press (cruise off / driver override): release immediately
  g = PressGovernor()
  assert g.update(0, "dec") == "dec"
  assert g.update(1, "dec") == "dec"
  assert g.update(2, None) is None       # aborted mid-press, not held to PRESS_FRAMES
  assert g.update(3, "dec") is None      # and the gap applies before any new press
  assert g.update(2 + GAP_FRAMES + 1, "dec") == "dec"


def test_governor_releases_when_intent_stops():
  g = PressGovernor()
  for f in range(PRESS_FRAMES):        # one full press
    g.update(f, "dec")
  for f in range(PRESS_FRAMES, PRESS_FRAMES + GAP_FRAMES + 50):
    assert g.update(f, None) is None   # no intent -> silent forever


def test_step_constant_matches_ford_tap():
  assert abs(STEP_MS - 1.0 * MPH_TO_MS) < 1e-9

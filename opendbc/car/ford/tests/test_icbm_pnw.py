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


# ---- icbmrestore2pnw: guarded restore (inc) path ---------------------------------------------------
from opendbc.car.ford.icbm_pnw import RestoreGuard


def rcmd(target_mph, ceiling_mph, ts=NOW):
  return IcbmCommand(target_ms=target_mph * MPH_TO_MS, ceiling_ms=ceiling_mph * MPH_TO_MS,
                     ts=ts, dir="inc")


def test_restore_inc_toward_ceiling():
  # restoring: set 45, restore target 60 (== ceiling) -> press up
  assert decide_press(45 * MPH_TO_MS, rcmd(60, 60), NOW, True, False) == "inc"


def test_restore_never_above_ceiling():
  # brain asks above the ceiling: clamped to the ceiling; at/above it -> silent
  assert decide_press(60 * MPH_TO_MS, rcmd(75, 60), NOW, True, False) is None
  assert decide_press(59.7 * MPH_TO_MS, rcmd(75, 60), NOW, True, False) is None  # within deadband
  assert decide_press(55 * MPH_TO_MS, rcmd(75, 60), NOW, True, False) == "inc"   # below: only to 60


def test_restore_command_never_decs():
  # stock somehow ABOVE the restore target: an inc command must NOT dec (no oscillation)
  assert decide_press(65 * MPH_TO_MS, rcmd(60, 60), NOW, True, False) is None


def test_cap_command_never_incs():
  # unchanged rule: a cap (dir absent/dec) never presses up even with stock below target
  assert decide_press(45 * MPH_TO_MS, cmd(60, 60), NOW, True, False) is None


def test_restore_gates_cruise_override_stale():
  assert decide_press(45 * MPH_TO_MS, rcmd(60, 60), NOW, False, False) is None       # ACC off
  assert decide_press(45 * MPH_TO_MS, rcmd(60, 60), NOW, True, True) is None         # pedals
  assert decide_press(45 * MPH_TO_MS, rcmd(60, 60, ts=NOW - STALE_LIMIT_S - 0.1), NOW, True, False) is None
  assert decide_press(0.0, rcmd(60, 60), NOW, True, False) is None                   # no set reported


def test_guard_passes_own_cadence():
  g = RestoreGuard()
  t, s = NOW, 45 * MPH_TO_MS
  assert g.filter("inc", s, t, True) == "inc"
  for _ in range(5):                      # +1 mph per 0.5 s = our own tap cadence: fine
    t += 0.5
    s += STEP_MS
    assert g.filter("inc", s, t, True) == "inc"
  assert not g.blocked


def test_guard_blocks_on_set_decrease_and_latches():
  g = RestoreGuard()
  g.filter("inc", 45 * MPH_TO_MS, NOW, True)
  # driver pressed SET-: set went down while we only press up -> block, and STAY blocked
  assert g.filter("inc", 44 * MPH_TO_MS, NOW + 0.5, True) is None
  assert g.blocked
  assert g.filter("inc", 45 * MPH_TO_MS, NOW + 1.0, True) is None   # still blocked this episode
  # episode ends (silent/empty command) -> latch clears; a NEW restore episode works again
  assert g.filter(None, 45 * MPH_TO_MS, NOW + 2.0, False) is None
  assert not g.blocked
  assert g.filter("inc", 45 * MPH_TO_MS, NOW + 3.0, True) == "inc"


def test_guard_blocks_on_driver_hold_jump():
  g = RestoreGuard()
  g.filter("inc", 45 * MPH_TO_MS, NOW, True)
  # +5 mph in 0.3 s: Ford SET+ hold (5 mph steps) -> a human is on the stalk -> block
  assert g.filter("inc", 50 * MPH_TO_MS, NOW + 0.3, True) is None
  assert g.blocked


def test_guard_never_touches_dec():
  g = RestoreGuard()
  # cap phase (restoring=False): dec passes untouched regardless of set movement
  assert g.filter("dec", 60 * MPH_TO_MS, NOW, False) == "dec"
  assert g.filter("dec", 45 * MPH_TO_MS, NOW + 0.1, False) == "dec"
  assert not g.blocked


def test_guard_dec_command_clears_restore_block():
  g = RestoreGuard()
  g.filter("inc", 45 * MPH_TO_MS, NOW, True)
  assert g.filter("inc", 43 * MPH_TO_MS, NOW + 0.5, True) is None    # blocked (manual dec)
  # a NEW cap engages (dec command, restoring=False): dec passes AND the latch clears
  assert g.filter("dec", 43 * MPH_TO_MS, NOW + 1.0, False) == "dec"
  assert not g.blocked


def test_governor_inc_tap_pattern_matches_dec_cadence():
  g = PressGovernor()
  frames = [g.update(f, "inc") for f in range(100)]
  # discrete taps: PRESS_FRAMES asserted, then a gap of at least GAP_FRAMES
  assert frames[:PRESS_FRAMES] == ["inc"] * PRESS_FRAMES
  assert all(b is None for b in frames[PRESS_FRAMES:PRESS_FRAMES + GAP_FRAMES])
  assert "inc" in frames[PRESS_FRAMES + GAP_FRAMES:]


# ---- speedadjust-exec2pnw: arbitrate() reduces ANY number of brains (icbm2pnw curve,
# speedadjust2pnw police/limit, and any future one) to the ONE unified button-management target ----
from opendbc.car.ford.icbm_pnw import arbitrate


def test_arbitrate_only_icbm_dec():
  assert arbitrate([cmd(45, 60), None], NOW) == cmd(45, 60)


def test_arbitrate_only_speedadjust_dec():
  assert arbitrate([None, cmd(50, 60)], NOW) == cmd(50, 60)


def test_arbitrate_both_dec_lower_target_wins():
  # icbm wants 45 (curve), speedadjust wants 50 (limit drop) -> the MORE restrictive (lower) wins
  icbm, sa = cmd(45, 60), cmd(50, 60)
  assert arbitrate([icbm, sa], NOW) == icbm
  # reversed: speedadjust more restrictive
  icbm2, sa2 = cmd(55, 60), cmd(40, 60)
  assert arbitrate([icbm2, sa2], NOW) == sa2


def test_arbitrate_dec_always_wins_over_inc():
  # icbm finished its curve and wants to restore (inc); speedadjust independently needs a NEW dec
  # (police report just appeared) -> the dec must win, never the inc, even though icbm "had the bus"
  restoring = rcmd(60, 60)
  new_cap = cmd(50, 60)
  assert arbitrate([restoring, new_cap], NOW) == new_cap
  assert arbitrate([new_cap, restoring], NOW) == new_cap


def test_arbitrate_inc_only_when_neither_wants_dec():
  restoring = rcmd(60, 60)
  assert arbitrate([restoring, None], NOW) == restoring
  assert arbitrate([None, restoring], NOW) == restoring


def test_arbitrate_both_inc_lower_wins():
  a, b = rcmd(60, 60), rcmd(55, 55)
  assert arbitrate([a, b], NOW) == b
  assert arbitrate([b, a], NOW) == b


def test_arbitrate_both_none():
  assert arbitrate([None, None], NOW) is None


def test_arbitrate_empty_list():
  assert arbitrate([], NOW) is None


def test_arbitrate_stale_dec_ignored():
  # a stale icbm heartbeat must not be able to veto or out-compete a fresh speedadjust dec
  stale_icbm = cmd(30, 60, ts=NOW - STALE_LIMIT_S - 0.1)   # would "win" on target alone (30 < 50)
  fresh_sa = cmd(50, 60)
  assert arbitrate([stale_icbm, fresh_sa], NOW) == fresh_sa


def test_arbitrate_stale_inc_ignored():
  stale_restore = rcmd(60, 60, ts=NOW - STALE_LIMIT_S - 0.1)
  assert arbitrate([stale_restore, None], NOW) is None


def test_arbitrate_malformed_input_treated_as_absent():
  # dir isn't validated by arbitrate() itself (icbm_buttons only ever hands it a real IcbmCommand or
  # None), but a defensive check costs nothing: an object without .dir/.ts must not raise.
  class _Bad:
    pass
  assert arbitrate([_Bad(), None], NOW) is None
  assert arbitrate([None, cmd(50, 60)], NOW) == cmd(50, 60)


def test_arbitrate_generalizes_to_a_third_source():
  # a hypothetical THIRD brain (any future car-agnostic reason) joins the list -- still just "lowest
  # fresh dec wins", no code change needed at the call site beyond appending to the list.
  a, b, c = cmd(55, 60), cmd(50, 60), cmd(45, 60)
  assert arbitrate([a, b, c], NOW) == c
  assert arbitrate([c, b, a], NOW) == c

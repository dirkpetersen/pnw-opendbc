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


# ---- restore2pnw-hardening (2026-08, Gemini + Fable review of speedadjust-exec2pnw) ----------------

def test_arbitrate_dec_selection_uses_effective_bound_not_raw_target():
  # Gemini's finding: selecting purely by raw target_ms could rank a malformed/adversarial command
  # (whose target transiently exceeds its OWN ceiling) as "more conservative" than a command whose
  # true effective bound (min(target, ceiling)) is actually stricter. Construct exactly that: A's raw
  # target (60) is HIGHER than B's (50), but A's ceiling (40) makes its true effective bound (40)
  # stricter than B's (50) -- the correct winner is A, not B.
  a = IcbmCommand(target_ms=60 * MPH_TO_MS, ceiling_ms=40 * MPH_TO_MS, ts=NOW)   # effective bound 40
  b = IcbmCommand(target_ms=50 * MPH_TO_MS, ceiling_ms=90 * MPH_TO_MS, ts=NOW)   # effective bound 50
  assert arbitrate([a, b], NOW) == a
  assert arbitrate([b, a], NOW) == a


def test_arbitrate_inc_selection_uses_effective_bound_not_raw_target():
  # same divergence on the restore (inc) side.
  a = IcbmCommand(target_ms=60 * MPH_TO_MS, ceiling_ms=40 * MPH_TO_MS, ts=NOW, dir="inc")
  b = IcbmCommand(target_ms=50 * MPH_TO_MS, ceiling_ms=90 * MPH_TO_MS, ts=NOW, dir="inc")
  assert arbitrate([a, b], NOW) == a
  assert arbitrate([b, a], NOW) == a


def test_arbitrate_selection_unchanged_when_target_equals_ceiling():
  # every command either brain actually publishes today has target_ms == ceiling_ms at its own
  # restore point / dec target, so the effective-bound key is a no-op in current practice -- this is
  # the regression guard that the existing behavior (tested extensively above) never moved.
  icbm, sa = cmd(45, 60), cmd(50, 60)
  assert arbitrate([icbm, sa], NOW) == icbm
  restoring = rcmd(60, 60)
  assert arbitrate([restoring, None], NOW) == restoring


def test_arbitrate_inc_debounced_right_after_a_dec():
  # oscillation guard: a dec that just won the bus must not be immediately followed by an inc on the
  # very next poll -- require the bus to be dec-free for INC_AFTER_DEC_DEBOUNCE_S first.
  from opendbc.car.ford.icbm_pnw import INC_AFTER_DEC_DEBOUNCE_S
  restoring = rcmd(60, 60)
  last_dec_ts = NOW - 0.05                    # a dec won the bus 50 ms ago
  assert arbitrate([restoring, None], NOW, last_dec_ts) is None            # debounced
  later = NOW + INC_AFTER_DEC_DEBOUNCE_S + 0.01
  assert arbitrate([restoring, None], later, last_dec_ts) == restoring     # debounce elapsed


def test_arbitrate_inc_debounce_ignored_when_no_prior_dec():
  # the default (no last_dec_ts, or None) -- unchanged behavior, no debounce applied.
  restoring = rcmd(60, 60)
  assert arbitrate([restoring, None], NOW) == restoring
  assert arbitrate([restoring, None], NOW, None) == restoring


def test_arbitrate_dec_never_debounced():
  # the debounce only ever gates inc; a fresh dec always asserts immediately regardless of last_dec_ts.
  new_cap = cmd(50, 60)
  assert arbitrate([new_cap, None], NOW, NOW - 0.01) == new_cap


def test_guard_survives_dec_interlude_same_episode():
  # restore2pnw-hardening: a DIFFERENT brain's dec briefly winning the shared bus mid-restore must NOT
  # wipe the veto latch for the SAME restore episode (same ceiling) still pending underneath it.
  g = RestoreGuard()
  ceil = 60 * MPH_TO_MS
  assert g.filter("inc", 45 * MPH_TO_MS, NOW, True, ceil) == "inc"
  # the driver taps SET- during the restore: blocked
  assert g.filter("inc", 44 * MPH_TO_MS, NOW + 0.5, True, ceil) is None
  assert g.blocked
  # a different brain's dec wins the bus this tick -- the CALLER still reports restoring=True/ceiling
  # unchanged (the SAME episode is still pending underneath, per the carcontroller contract) -- dec
  # itself must pass through untouched, and the veto must SURVIVE
  assert g.filter("dec", 42 * MPH_TO_MS, NOW + 1.0, True, ceil) == "dec"
  assert g.blocked
  # the interlude clears and the SAME restore resumes -- still vetoed, no SET+ oscillation
  assert g.filter("inc", 42 * MPH_TO_MS, NOW + 1.5, True, ceil) is None
  assert g.blocked


def test_guard_new_episode_different_ceiling_resets_latch():
  # a genuinely NEW restore episode (different ceiling) is a fresh start -- the old episode's veto must
  # not leak into it.
  g = RestoreGuard()
  ceil1 = 60 * MPH_TO_MS
  g.filter("inc", 45 * MPH_TO_MS, NOW, True, ceil1)
  g.filter("inc", 44 * MPH_TO_MS, NOW + 0.5, True, ceil1)
  assert g.blocked
  ceil2 = 55 * MPH_TO_MS
  assert g.filter("inc", 50 * MPH_TO_MS, NOW + 1.0, True, ceil2) == "inc"
  assert not g.blocked


def test_guard_fully_idle_clears_latch_even_with_stale_ceiling():
  # restoring=False (no brain has ANY live inc offer, not even an interlude) always fully stands down,
  # regardless of what ceiling happens to be passed (default None here, mirrors the old call sites).
  g = RestoreGuard()
  ceil = 60 * MPH_TO_MS
  g.filter("inc", 45 * MPH_TO_MS, NOW, True, ceil)
  g.filter("inc", 44 * MPH_TO_MS, NOW + 0.5, True, ceil)
  assert g.blocked
  assert g.filter(None, 44 * MPH_TO_MS, NOW + 1.0, False) is None
  assert not g.blocked
  assert g.filter("inc", 44 * MPH_TO_MS, NOW + 1.5, True, ceil) == "inc"   # fresh episode, same ceiling ok


# ---- Fable fail-safe fix: dec_owns_bus freezes movement-judgment during a dec interlude ---------------

def test_guard_dec_interlude_no_false_latch():
  # Reproduces Fable's sim proof: an OPEN restore (ceiling 75, not yet blocked) crossed by a benign
  # curve-ICBM dec interlude (target 55, NO driver input) must NOT falsely latch the veto -- only a
  # genuine driver movement may. Before the fix, the interlude's own ~13 SET- taps (moving stock_set_ms
  # down each tick, indistinguishable from a driver SET- to the old unconditional judgment) tripped
  # guard.blocked=True permanently, and the restore never resumed.
  g = RestoreGuard()
  ceil = 75 * MPH_TO_MS
  assert g.filter("inc", 55 * MPH_TO_MS, NOW, True, ceil) == "inc"
  assert not g.blocked
  # the dec interlude wins the bus repeatedly: caller reports restoring=True/ceiling unchanged (the
  # SAME episode is still pending underneath, per the carcontroller contract) and dec_owns_bus=True
  t, s = NOW + 0.1, 55 * MPH_TO_MS
  for _ in range(13):
    t += 0.3
    s -= STEP_MS
    assert g.filter("dec", s, t, True, ceil, dec_owns_bus=True) == "dec"   # dec never filtered
    assert not g.blocked                                                  # and never falsely latched
  # interlude ends -- the SAME restore resumes toward the ceiling, no manual SET+ needed
  assert g.filter("inc", s, t + 0.5, True, ceil) == "inc"
  assert not g.blocked


def test_guard_genuine_set_minus_during_restore_still_vetoes():
  # unchanged path: a GENUINE driver SET- during an inc/restore tick (dec_owns_bus=False, the default —
  # no dec owns the bus) must still veto.
  g = RestoreGuard()
  ceil = 60 * MPH_TO_MS
  assert g.filter("inc", 45 * MPH_TO_MS, NOW, True, ceil) == "inc"
  assert g.filter("inc", 44 * MPH_TO_MS, NOW + 0.5, True, ceil) is None
  assert g.blocked


# ---- restore2pnw-hardening: math.isfinite guard on the shared mem-param parser ----------------------
# _parse_button_cmd lives on FordCarController (carcontroller.py), not icbm_pnw.py -- exercised via a
# minimal stand-in since constructing a full CarController needs CarParams/CAN plumbing this test file
# doesn't otherwise touch. The parser logic itself (json.loads + shape/finite checks) has no CarController
# dependency, so this mirrors it exactly against the real implementation's behavior contract.

def test_non_finite_target_rejected():
  import math
  from opendbc.car.ford.carcontroller import CarController
  parse = CarController._parse_button_cmd
  bad = {"target": float("nan"), "ceiling": 60.0, "ts": NOW}
  assert parse(None, bad) is None
  bad2 = {"target": float("inf"), "ceiling": 60.0, "ts": NOW}
  assert parse(None, bad2) is None
  bad3 = {"target": 50.0, "ceiling": float("-inf"), "ts": NOW}
  assert parse(None, bad3) is None
  bad4 = {"target": 50.0, "ceiling": 60.0, "ts": float("nan")}
  assert parse(None, bad4) is None
  good = {"target": 50.0, "ceiling": 60.0, "ts": NOW}
  result = parse(None, good)
  assert result is not None and math.isfinite(result.target_ms)


def test_non_finite_via_json_string_rejected():
  # json.loads (Python's, non-standard-but-permissive) happily parses bare NaN/Infinity literals.
  from opendbc.car.ford.carcontroller import CarController
  parse = CarController._parse_button_cmd
  raw = f'{{"target": NaN, "ceiling": 60.0, "ts": {NOW}}}'
  assert parse(None, raw) is None
  raw2 = f'{{"target": Infinity, "ceiling": 60.0, "ts": {NOW}}}'
  assert parse(None, raw2) is None


# --- gasset2pnw: the SET mode --------------------------------------------------------------------

def test_parses_a_set_mode_offer():
  """The brain may now ask for SET (establish the driver's gas-chosen speed) as well as RESUME.
  Both literals must parse; anything else must still be rejected."""
  from opendbc.car.ford.icbm_pnw import parse_resume_cmd, RESUME_DIR, SET_DIR
  import json
  for d in (RESUME_DIR, SET_DIR):
    cmd = parse_resume_cmd(json.dumps({"dir": d, "ts": 1.0, "eid": 2.0, "set": 18.0}))
    assert cmd is not None and cmd.mode == d, d
    assert cmd.set_ms == 18.0
  assert parse_resume_cmd(json.dumps({"dir": "dec", "ts": 1.0, "eid": 2.0, "set": 18.0})) is None
  assert parse_resume_cmd(json.dumps({"dir": "", "ts": 1.0, "eid": 2.0, "set": 18.0})) is None


def test_set_mode_is_not_refused_by_the_resume_ceiling():
  """The stock-set ceiling exists so a RESUME can never target above what the driver set. A SET
  establishes the current speed outright and has no remembered value to exceed -- and the truck
  reports roughly the CURRENT speed while in standby, so applying that ceiling to a SET would
  refuse it for a condition that does not exist."""
  from opendbc.car.ford.icbm_pnw import parse_resume_cmd, decide_resume, RESUME_DIR, SET_DIR
  import json
  # live standby reading sits ABOVE the target, which is normal for a set-to-current
  live = 20.0

  def mk(d):
    return parse_resume_cmd(json.dumps({"dir": d, "ts": 100.0, "eid": 1.0, "set": 18.0}))
  assert decide_resume(mk(SET_DIR), 100.0, False, True, False, live) is True
  assert decide_resume(mk(RESUME_DIR), 100.0, False, True, False, live) is False


def test_set_mode_still_refuses_while_cruise_is_engaged():
  """A SET tap while ACC is engaged would MOVE the driver's set speed rather than establish it."""
  from opendbc.car.ford.icbm_pnw import parse_resume_cmd, decide_resume, SET_DIR
  import json
  cmd = parse_resume_cmd(json.dumps({"dir": SET_DIR, "ts": 100.0, "eid": 1.0, "set": 18.0}))
  assert decide_resume(cmd, 100.0, True, True, False, 0.0) is False       # cruise_enabled
  assert decide_resume(cmd, 100.0, False, False, False, 0.0) is False     # master off
  assert decide_resume(cmd, 100.0, False, True, True, 0.0) is False       # driver on a pedal

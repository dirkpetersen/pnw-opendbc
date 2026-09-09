"""lightning-extra2pnw — the Pro Power Onboard re-arm envelope.

Every bound in ProPowerArmer gets a test that FAILS if the bound is removed. The module is pure, so
these drive it directly with a PpoInputs sequence.
"""
from opendbc.car.ford.lightning_extra_pnw import (PPO_MAX_ATTEMPTS, PPO_PRESS_ON, PPO_PRESS_S,
                                                  PPO_SETTLE_S, PPO_VERIFY_S, PpoInputs,
                                                  ProPowerArmer)

DT = 0.01


def run(armer, secs, **kw):
  """Tick the armer for `secs`, returning every payload it asked to transmit."""
  base = dict(standstill=True, parked=True, state_valid=True, ppo_on=False, enabled=True,
              can_valid=True)
  base.update(kw)
  out = []
  t = getattr(run, "_t", 0.0)
  for _ in range(int(secs / DT)):
    p = armer.update(PpoInputs(now=t, **base))
    if p is not None:
      out.append(p)
    t += DT
  run._t = t
  return out


def fresh():
  run._t = 0.0
  return ProPowerArmer()


def test_it_arms_pro_power_when_the_truck_comes_up_with_it_off():
  a = fresh()
  assert run(a, PPO_SETTLE_S - 1.0) == [], "must not act before the settle window"
  sent = run(a, PPO_PRESS_S + 2.0)
  assert sent, "with Pro Power OFF it must press"
  assert all(p == PPO_PRESS_ON for p in sent), "the only payload it may send is the ON press"
  # the press is bounded to roughly PPO_PRESS_S worth of ticks, not held forever
  assert len(sent) <= int((PPO_PRESS_S + 0.05) / DT)


def test_it_does_nothing_when_pro_power_is_already_on():
  a = fresh()
  assert run(a, PPO_SETTLE_S + 3.0, ppo_on=True) == []
  assert a.phase == ProPowerArmer.DONE and "already armed" in a.note


def test_it_never_touches_anything_while_the_truck_is_moving():
  """The outer bound. Pro Power is a parked-truck function; a moving truck must never see a frame."""
  a = fresh()
  assert run(a, PPO_SETTLE_S + 5.0, standstill=False) == []
  assert a.attempts == 0


def test_a_truck_that_pulls_away_mid_press_stops_being_pressed():
  a = fresh()
  run(a, PPO_SETTLE_S + 0.1)
  assert a.phase == ProPowerArmer.PRESSING, "precondition: mid-press"
  assert run(a, 1.0, standstill=False) == [], "must stop transmitting the instant it moves"
  assert "moved" in a.note


def test_it_stops_after_MAX_ATTEMPTS_and_says_why():
  """Verified, not assumed: if the state bit never flips we give up LOUDLY rather than mashing a
  body button forever. This is the case where the panda has not been flashed."""
  a = fresh()
  run(a, PPO_SETTLE_S + (PPO_PRESS_S + PPO_VERIFY_S + 0.2) * (PPO_MAX_ATTEMPTS + 2))
  assert a.phase == ProPowerArmer.FAILED
  assert a.attempts == PPO_MAX_ATTEMPTS, f"bounded to {PPO_MAX_ATTEMPTS}, got {a.attempts}"
  assert "gave up" in a.note and "allowlist" in a.note


def test_a_press_that_works_stops_immediately():
  a = fresh()
  run(a, PPO_SETTLE_S + 0.1)
  assert a.attempts == 1
  run(a, PPO_PRESS_S + 0.5, ppo_on=True)          # the state bit flips
  assert a.phase == ProPowerArmer.DONE
  assert run(a, 5.0) == [], "done means done -- no further presses this ignition cycle"


def test_it_waits_rather_than_guessing_when_the_state_has_not_been_read():
  a = fresh()
  assert run(a, PPO_SETTLE_S + 5.0, state_valid=False) == []
  assert a.attempts == 0, "an unread state is not 'off'"


def test_a_never_reported_state_says_so_once_instead_of_sitting_silent():
  """Rule 2. A Ford that never sends these messages, or a Lightning whose modules never reported,
  is otherwise indistinguishable from 'working, nothing to do' -- silence forever, no line
  anywhere (Fable review 2026-09-07, must-fix 3b)."""
  a = fresh()
  run(a, PPO_SETTLE_S * 2 + 1.0, state_valid=False)
  assert a.phase == ProPowerArmer.FAILED, "must resolve rather than idle in silence"
  assert "never reported" in a.note, a.note
  assert a.attempts == 0, "and it must still never have pressed"


def test_it_DOES_press_at_a_red_light_in_Drive():
  """ppostandstill2pnw (2026-09-09) — this test asserted the OPPOSITE until the driver overruled it.

  The old bound was standstill AND Park, reasoning that a `card` restart mid-drive would otherwise
  press at the next red light. Real consequence, 2026-09-09: the truck was started and driven away
  within 16 s, the armer never saw Park-at-standstill, and Pro Power stayed OFF for the whole drive
  with a cooler of food aboard. Driver: "why does it have to be in park ... this is not a driving
  critical function I just wanted it to be on."

  The panda's `!vehicle_moving` half is NOT relaxed and still bounds this in C."""
  a = fresh()
  sent = run(a, PPO_SETTLE_S + 5.0, parked=False)
  assert sent, "stopped in Drive must now press"
  assert all(f == PPO_PRESS_ON for f in sent), "and only ever the ON-press payload"
  # the stub never flips ppo_on, so it retries -- what matters is that it pressed AT ALL, and that
  # relaxing the gear gate did not relax the attempt cap
  assert 1 <= a.attempts <= PPO_MAX_ATTEMPTS


def test_a_press_in_Drive_says_so_in_the_log():
  """A red-light arm and a parked arm must be distinguishable in a drive log without inference."""
  a = fresh()
  run(a, PPO_SETTLE_S + 1.0, parked=False)
  note = a.take_note()
  assert "NOT in Park" in note, note

  b = fresh()
  run(b, PPO_SETTLE_S + 1.0, parked=True)
  note_b = b.take_note()
  # "in Park" alone is satisfied by "NOT in Park" -- the assertion has to exclude it (Fable review)
  assert "in Park" in note_b and "NOT" not in note_b, note_b


def test_moving_is_still_refused_regardless_of_gear():
  """The bound that did NOT move. Relaxing Park must not have relaxed standstill."""
  for parked in (True, False):
    a = fresh()
    assert run(a, PPO_SETTLE_S + 5.0, standstill=False, parked=parked) == [], \
      f"a moving truck must never see a frame (parked={parked})"
    assert a.attempts == 0


def test_it_can_never_turn_pro_power_OFF():
  """The only payload the module is able to emit is the ON press -- there is no path to an OFF."""
  import inspect
  from opendbc.car.ford import lightning_extra_pnw as m
  src = inspect.getsource(m.ProPowerArmer)
  assert "PPO_PRESS_ON" in src
  assert "PPO_IDLE" not in src, "the armer must never transmit the idle/OFF payload"
  a = fresh()
  sent = run(a, PPO_SETTLE_S + (PPO_PRESS_S + PPO_VERIFY_S + 0.2) * (PPO_MAX_ATTEMPTS + 2))
  assert set(sent) <= {PPO_PRESS_ON}


# --- carstate: "never received" must never read as "off" ----------------------------------------

def test_a_ford_that_never_sends_these_messages_stays_inert():
  """The hazard this guards, found in review 2026-09-07 and verified empirically:

  `cp.vl["X"].get("sig")` returns **0.0**, not None, for a message that has NEVER been received --
  a registered message is lazily populated with defaults. So an `is None` guard never fires, and
  `"X" not in cp.vl` is False too (the message IS registered). Both obvious guards fail open.

  Unguarded, ANY Ford on this DBC that does not transmit these messages would have read
  `ppo_valid=True, ppo_on=False`, and the armer would have spoofed 0x455 on a truck where that
  address may mean something else entirely. Presence is therefore tested with `ts_nanos`.
  """
  from opendbc.can.parser import CANParser
  from opendbc.car import CanData
  from opendbc.car.ford.carstate import CarState

  class Bare:
    ppo_on = False
    ppo_valid = False

  cp = CANParser("ford_lincoln_base_pt",
                 [("EffDrvModeData", float("nan")), ("HEV_Powertrain_Data6", float("nan"))], 0)
  cs = Bare()

  # the obvious guards, pinned so nobody "simplifies" the presence test back into a fail-open one
  assert cp.vl["EffDrvModeData"].get("PnwProPwrOnbd_B_Stat") == 0.0, "an unreceived signal reads 0.0"
  assert "EffDrvModeData" in cp.vl, "a registered-but-unreceived message IS in cp.vl"

  CarState._read_pro_power(cs, cp)
  assert not cs.ppo_valid, "never received must NOT read as a valid OFF"

  cp.update([(1_000_000, [CanData(0x44A, bytes.fromhex("0000000000bf1000"), 0)])])
  CarState._read_pro_power(cs, cp)
  assert cs.ppo_valid and cs.ppo_on, "a real ON frame must read armed"

  cp.update([(2_000_000, [CanData(0x44A, bytes.fromhex("0000000000bf0000"), 0)])])
  CarState._read_pro_power(cs, cp)
  assert cs.ppo_valid and not cs.ppo_on, "a real OFF frame must read disarmed"


# ---------------------------------------------------------------------------------------------
# Regression guards for the 2026-09-07 silent-inertness bug. These are deliberately NOT tests of
# the pure module -- the module was always correct. The bug lived in how ford/carcontroller.py
# BUILT its PpoInputs, which is exactly the seam the pure-module tests cannot see, because they
# hand `parked` in ready-made.
# ---------------------------------------------------------------------------------------------

def test_park_must_be_compared_as_an_enum_not_stringified():
  """`CS.out` is a capnp struct and capnp renders an enum as the BARE member name.

  The shipped carcontroller compared `str(CS.out.gearShifter) == "GearShifter.park"`, which is
  always False, so `parked` never went true and the armer never transmitted anything at all. This
  fails if anyone reintroduces a str() comparison, and it fails on the real struct type rather
  than a stand-in.
  """
  from opendbc.car import structs
  cs = structs.CarState.new_message()
  cs.gearShifter = structs.CarState.GearShifter.park

  # what carcontroller.py must do
  assert (cs.gearShifter == structs.CarState.GearShifter.park) is True
  assert (cs.as_reader().gearShifter == structs.CarState.GearShifter.park) is True
  # what it must never go back to -- capnp gives 'park', not 'GearShifter.park'
  assert str(cs.gearShifter) == "park"
  assert (str(cs.gearShifter) == "GearShifter.park") is False

  # and the value that must NOT read as parked
  cs.gearShifter = structs.CarState.GearShifter.drive
  assert (cs.gearShifter == structs.CarState.GearShifter.park) is False


def test_carcontroller_does_not_stringify_the_gear():
  """Guard the CALL SITE, not just the semantics.

  The assertions above are about capnp and would still pass with the bug fully reintroduced -- they
  document why it is a bug, they do not prevent it. This one reads the source that actually builds
  PpoInputs and fails if the str() form comes back. Crude, and deliberately so: the seam between
  the carcontroller and the pure module is the one place a unit test of the module cannot reach,
  and it is where the bug lived.
  """
  import ast
  import inspect
  from opendbc.car.ford import carcontroller

  # Parse, don't grep. A substring test over the source would pass on a mention inside a comment
  # (this very file's fix carries a comment ABOUT the str() form) and would break on reformatting.
  tree = ast.parse(inspect.getsource(carcontroller))
  parked = [kw.value for node in ast.walk(tree)
            if isinstance(node, ast.Call) and getattr(node.func, "id", None) == "_PpoInputs"
            for kw in node.keywords if kw.arg == "parked"]
  assert len(parked) == 1, f"expected exactly one _PpoInputs(parked=...) call site, found {len(parked)}"
  expr = parked[0]

  # It must be a comparison, and nothing in it may be a str() call.
  assert isinstance(expr, ast.Compare), ast.dump(expr)
  assert not [n for n in ast.walk(expr)
              if isinstance(n, ast.Call) and getattr(n.func, "id", None) == "str"], \
      "capnp renders the enum as 'park', so str(gearShifter) never equals 'GearShifter.park'"
  # ...and it must compare against the GearShifter.park enum member, not a string literal.
  assert not [n for n in ast.walk(expr) if isinstance(n, ast.Constant) and isinstance(n.value, str)], \
      "compare the enum member, not a string literal"
  assert ast.unparse(expr.comparators[0]).endswith("GearShifter.park"), ast.unparse(expr)


def test_being_held_at_the_gate_eventually_says_so():
  """Rule 2: "held at the gate" must not be indistinguishable from "nothing to do".

  Logging only on a PHASE change is what let the dead Park test look like a healthy feature for a
  whole ignition cycle. A note must surface even though the phase never leaves IDLE.
  """
  armer = ProPowerArmer()
  run._t = 0.0
  assert armer.take_note() == ""

  # never at a standstill -> the armer can never act, and must say which gate is holding it
  # (ppostandstill2pnw: this used to be parked=False; the gear no longer gates, so the only gate
  # left to be held by is standstill)
  run(armer, PPO_SETTLE_S * 2 + 1.0, standstill=False)
  assert armer.phase == ProPowerArmer.IDLE
  note = armer.take_note()
  assert "not at a standstill" in note, note
  assert armer.take_note() == "", "a note must be drained exactly once, not repeated every tick"

  # and it says it ONCE, not on every subsequent tick
  run(armer, 30.0, standstill=False)
  assert armer.take_note() == ""


def test_gate_note_names_standstill_when_that_is_what_is_holding():
  armer = ProPowerArmer()
  run._t = 0.0
  run(armer, PPO_SETTLE_S * 2 + 1.0, standstill=False)
  assert "not at a standstill" in armer.take_note()


def test_a_dead_can_bus_never_produces_a_press():
  """Stale CAN keeps its last decoded values, and `ppo_valid` is a forever-latch.

  Without the can_valid bound, an armer running on a quiet bus reads stale standstill/parked/
  state_valid, spends all three presses into silence, and then reports a confident failure.
  """
  armer = ProPowerArmer()
  run._t = 0.0
  assert run(armer, PPO_SETTLE_S * 3, can_valid=False) == []
  assert armer.phase == ProPowerArmer.IDLE
  assert armer.attempts == 0
  # ...and it starts working the moment the bus comes back
  assert run(armer, PPO_SETTLE_S + PPO_PRESS_S, can_valid=True) != []


def test_stop_and_go_creep_cannot_machine_gun_the_body_button():
  """ppostandstill2pnw (Fable review 2026-09-09). THE regression this change could have introduced.

  The attempt cap used to be enforced only in the VERIFYING branch, so a press ABORTED by movement
  never counted. With the old Park gate that was self-limiting. With standstill-only, every brief
  stop in stop-and-go traffic starts a fresh press: simulated at 60 presses / 180 frames in 60 s of
  creep before the fix, phase never reaching FAILED.

  Creep pattern: 0.3 s stopped (shorter than PPO_PRESS_S, so every press aborts), 0.7 s rolling."""
  a = fresh()
  frames = []
  t = 0.0
  # get past the settle window first, stopped
  for _ in range(int(PPO_SETTLE_S / DT) + 10):
    a.update(PpoInputs(now=t, standstill=False, parked=False, state_valid=True,
                       ppo_on=False, enabled=True, can_valid=True))
    t += DT
  for _cycle in range(60):                       # 60 s of creep
    for stopped, secs in ((True, 0.3), (False, 0.7)):
      for _ in range(int(secs / DT)):
        p = a.update(PpoInputs(now=t, standstill=stopped, parked=False, state_valid=True,
                               ppo_on=False, enabled=True, can_valid=True))
        if p is not None:
          frames.append(p)
        t += DT
  assert a.attempts <= PPO_MAX_ATTEMPTS, f"pressed {a.attempts} times in creep -- the cap must count aborts"
  assert a.phase == ProPowerArmer.FAILED, "and it must STOP, not sit in IDLE re-pressing forever"
  assert len(frames) <= PPO_MAX_ATTEMPTS * int(PPO_PRESS_S / DT) + 5, \
    f"{len(frames)} frames handed to pandad in 60 s of creep"

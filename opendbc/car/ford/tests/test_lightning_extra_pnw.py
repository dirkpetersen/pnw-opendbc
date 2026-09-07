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
  base = dict(standstill=True, parked=True, state_valid=True, ppo_on=False, enabled=True)
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


def test_it_will_not_press_at_a_red_light_in_Drive():
  """standstill alone is not 'parked'. `card` restarts on crash, so a fresh armer can appear
  mid-drive; without the gear gate it would press at the next red light, in Drive (CLAUDE.md rule 3
  -- the GEAR is the truth source)."""
  a = fresh()
  assert run(a, PPO_SETTLE_S + 5.0, parked=False) == [], "stopped but in Drive must send nothing"
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

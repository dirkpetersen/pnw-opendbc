"""
madsresume2pnw -- the EXECUTOR half (opendbc/car/ford/icbm_pnw.py).

The brain (selfdrive/controls/lib/madsresume_pnw.py, openpilot side) decides WHEN a resume may be
offered. This half decides whether the button may actually be asserted, from the real car state at
the carcontroller layer, independently of the brain. These tests pin that independence:

  * the resume path is DISJOINT from the SET+/- path -- neither parser will accept the other's
    payload, and a "res" command reaching decide_press() can never be reinterpreted as a SET- tap;
  * RES is never asserted while stock cruise is ENGAGED (where RES is a +1 mph SET+ on Ford, i.e.
    the one thing this feature must never do);
  * exactly ONE press per offer, ever.
"""

import pytest

from opendbc.car.ford import icbm_pnw as I
from opendbc.car.ford.icbm_pnw import (IcbmCommand, ResumeCommand, ResumePress, arbitrate,
                                       decide_press, decide_resume, parse_resume_cmd)

SET = 29.0
NOW = 1000.0


def cmd(ts=NOW, eid=1.0, set_ms=SET):
  return ResumeCommand(ts=ts, eid=eid, set_ms=set_ms)


def ok(**kw):
  """decide_resume with every gate satisfied unless overridden."""
  a = dict(cmd=cmd(), now=NOW, cruise_enabled=False, cruise_available=True,
           driver_override=False, stock_set_ms=0.0)
  a.update(kw)
  return decide_resume(a["cmd"], a["now"], a["cruise_enabled"], a["cruise_available"],
                       a["driver_override"], a["stock_set_ms"])


# ---------------------------------------------------------------------------------------------
# decide_resume -- the gate list
# ---------------------------------------------------------------------------------------------

def test_baseline_permits():
  assert ok() is True


def test_no_command_refuses():
  assert ok(cmd=None) is False


def test_cruise_enabled_refuses():
  """THE gate. RES while ACC is engaged is a SET+ on Ford: it would RAISE the driver's set speed."""
  assert ok(cruise_enabled=True) is False


def test_acc_main_off_refuses():
  assert ok(cruise_available=False) is False


def test_driver_override_refuses():
  assert ok(driver_override=True) is False


def test_stale_offer_refuses():
  assert ok(now=NOW + I.RESUME_STALE_LIMIT_S + 0.01) is False
  assert ok(now=NOW + I.RESUME_STALE_LIMIT_S - 0.01) is True


def test_resume_freshness_is_tighter_than_the_set_speed_path():
  """A resume acts on a world the brain verified moments ago; it must not use the 2 s set-tap bound."""
  assert I.RESUME_STALE_LIMIT_S < I.STALE_LIMIT_S


def test_live_set_speed_above_the_captured_one_refuses():
  assert ok(stock_set_ms=SET + 2.0) is False


def test_live_set_speed_at_or_below_the_captured_one_permits():
  assert ok(stock_set_ms=SET) is True
  assert ok(stock_set_ms=SET - 5.0) is True


def test_absent_live_set_speed_permits():
  """The Lightning may report 0 for the set speed in ACC standby; that is not evidence of a raise."""
  assert ok(stock_set_ms=0.0) is True


def test_nonfinite_live_set_speed_refuses():
  """A gate whose input is unreadable must FAIL CLOSED. NaN/inf comparisons are False, so an
  `isfinite(x) and x > ...` short-circuit silently PERMITS -- that was a real bug this caught."""
  assert ok(stock_set_ms=float("nan")) is False
  assert ok(stock_set_ms=float("inf")) is False
  assert ok(stock_set_ms=float("-inf")) is False


def test_bad_types_never_raise():
  assert ok(stock_set_ms="x") is False
  assert decide_resume(ResumeCommand(ts="x", eid=1.0, set_ms=SET), NOW, False, True, False, 0.0) is False


# ---------------------------------------------------------------------------------------------
# parse_resume_cmd -- fail-closed, and disjoint from the SET+/- payloads
# ---------------------------------------------------------------------------------------------

def test_parses_a_well_formed_offer():
  c = parse_resume_cmd('{"dir":"res","ts":1.0,"eid":2.0,"set":29.0}')
  assert c is not None and c.ts == 1.0 and c.eid == 2.0 and c.set_ms == 29.0


def test_parses_a_dict_as_well_as_a_string():
  assert parse_resume_cmd({"dir": "res", "ts": 1.0, "eid": 2.0, "set": 29.0}) is not None


@pytest.mark.parametrize("raw", [
  None, "", "{", "[]", 5,
  '{"ts":1.0,"eid":2.0,"set":29.0}',                        # no dir
  '{"dir":"dec","ts":1.0,"eid":2.0,"set":29.0}',            # a SET- payload
  '{"dir":"inc","ts":1.0,"eid":2.0,"set":29.0}',            # a SET+ payload
  '{"dir":"res","eid":2.0,"set":29.0}',                     # no ts
  '{"dir":"res","ts":1.0,"set":29.0}',                      # no eid
  '{"dir":"res","ts":1.0,"eid":2.0}',                       # no set
  '{"dir":"res","ts":NaN,"eid":2.0,"set":29.0}',
  '{"dir":"res","ts":1.0,"eid":Infinity,"set":29.0}',
  '{"dir":"res","ts":1.0,"eid":2.0,"set":NaN}',
  '{"dir":"res","ts":1.0,"eid":2.0,"set":0.0}',             # no captured set speed
  '{"dir":"res","ts":1.0,"eid":2.0,"set":-5.0}',
])
def test_malformed_payloads_fail_closed(raw):
  assert parse_resume_cmd(raw) is None


def test_a_set_speed_target_payload_is_never_read_as_a_resume():
  """IcbmTarget / SpeedAdjustTarget shape ({target, ceiling, ts}) must not parse here. Two
  independent reasons reject it -- the required-key check and the `dir` whitelist -- so the last
  case below carries EVERY resume key and is rejected on the direction alone."""
  assert parse_resume_cmd('{"target":20.0,"ceiling":29.0,"ts":1.0}') is None
  assert parse_resume_cmd('{"target":20.0,"ceiling":29.0,"ts":1.0,"dir":"inc"}') is None
  assert parse_resume_cmd('{"dir":"dec","ts":1.0,"eid":2.0,"set":29.0,"target":20.0,"ceiling":29.0}') is None


# ---------------------------------------------------------------------------------------------
# The SET+/- path must be unable to act on a resume command (and vice versa)
# ---------------------------------------------------------------------------------------------

def test_decide_press_stands_down_on_an_unknown_direction():
  """Before madsresume2pnw this fell through to the DEC branch and would have pressed SET-."""
  c = IcbmCommand(target_ms=20.0, ceiling_ms=29.0, ts=NOW, dir="res")
  assert decide_press(29.0, c, NOW, True, False) is None
  assert decide_press(29.0, IcbmCommand(20.0, 29.0, NOW, dir="bogus"), NOW, True, False) is None


def test_decide_press_still_works_for_dec_and_inc():
  """The unknown-direction guard must not have broken the paths that were already shipping."""
  assert decide_press(29.0, IcbmCommand(20.0, 29.0, NOW, dir="dec"), NOW, True, False) == "dec"
  assert decide_press(20.0, IcbmCommand(29.0, 29.0, NOW, dir="inc"), NOW, True, False) == "inc"


def test_arbitrate_ignores_a_resume_command():
  c = IcbmCommand(target_ms=20.0, ceiling_ms=29.0, ts=NOW, dir="res")
  assert arbitrate([c], NOW) is None
  # ...and does not let it displace a real cap
  dec = IcbmCommand(target_ms=22.0, ceiling_ms=29.0, ts=NOW, dir="dec")
  assert arbitrate([c, dec], NOW) is dec


# ---------------------------------------------------------------------------------------------
# Review-driven hardening (Gemini, 2026-09-06)
# ---------------------------------------------------------------------------------------------

def test_publish_rounding_does_not_burn_the_press():
  """Fable C1: the brain publishes round(now, 3), which can land up to 0.5 ms in the future. A hard
  dt<0 refusal would give one ok=False frame, which ResumePress treats as a mid-press abort -- and
  the eid is then spent, losing the resume for the whole brake event."""
  assert ok(now=NOW - 0.0005) is True
  assert ok(now=NOW - I.RESUME_FUTURE_TOL_S + 0.001) is True


def test_a_heartbeat_from_the_future_refuses():
  """A negative age means the two clocks disagree (a step, a restart, a replayed payload). That is
  an unreadable input, not 'extra fresh'."""
  assert ok(now=NOW - I.RESUME_FUTURE_TOL_S - 0.001) is False
  assert ok(now=NOW - 60.0) is False


def test_the_executor_polls_every_frame_while_it_holds_a_command():
  """The brain's fast-abort is only real if the withdrawal is READ at the control rate. Polling at
  a flat 4 Hz kept pressing off a cached command for up to 250 ms after the brain cleared the
  mem-param. Pinned against the source so the cadence cannot silently regress."""
  import inspect
  from opendbc.car.ford import carcontroller
  src = inspect.getsource(carcontroller.CarController._resume_button)
  assert "if self._resume_cmd is not None or (self.frame % 25) == 0:" in src, \
    "the mem-param must be re-read EVERY frame while a resume command is held"


def test_the_executor_uses_a_monotonic_clock_for_the_resume_heartbeat():
  """This device has a dead RTC and steps its wall clock when it first syncs; a backward step would
  make a minutes-old offer look fresh inside a 0.5 s bound."""
  import inspect
  from opendbc.car.ford import carcontroller
  src = inspect.getsource(carcontroller.CarController._resume_button)
  assert "time.monotonic()" in src and "time.time()" not in src


# ---------------------------------------------------------------------------------------------
# ResumePress -- exactly one press per offer
# ---------------------------------------------------------------------------------------------

def test_one_press_per_offer():
  p = ResumePress()
  c = cmd(eid=7.0)
  asserted = [p.update(f, c, True) for f in range(500)]
  assert asserted[:I.RESUME_PRESS_FRAMES] == [True] * I.RESUME_PRESS_FRAMES
  assert not any(asserted[I.RESUME_PRESS_FRAMES:]), "the offer must never press twice"
  assert sum(asserted) == I.RESUME_PRESS_FRAMES


def test_a_republished_offer_with_the_same_eid_never_presses_again():
  p = ResumePress()
  c = cmd(eid=7.0)
  for f in range(200):
    p.update(f, c, True)
  # brain re-publishes the identical offer (fresh ts, same eid) -- e.g. a duplicated mem-param read
  again = [p.update(f, ResumeCommand(ts=NOW + 5, eid=7.0, set_ms=SET), True) for f in range(200, 400)]
  assert not any(again)


def test_an_older_eid_can_never_press_again():
  """Fable C2: eid is a monotonic stamp, so an eid at or BEFORE the spent one is the same offer
  again or a replayed older one. `<=`, not `==`."""
  p = ResumePress()
  for f in range(200):
    p.update(f, cmd(eid=7.0), True)
  assert not any(p.update(f, cmd(eid=6.5), True) for f in range(200, 400))
  assert not any(p.update(f, cmd(eid=7.0), True) for f in range(400, 600))


def test_the_executor_logs_rather_than_silently_disabling_itself():
  """Fable S1: a dead executor while the brain keeps publishing looks exactly like 'the gates
  refused' in the drive log."""
  import inspect
  from opendbc.car.ford import carcontroller
  init_src = inspect.getsource(carcontroller.CarController.__init__)
  assert "auto-resume executor is INERT" in init_src, "construction failures must be logged"
  poll_src = inspect.getsource(carcontroller.CarController._resume_button)
  assert "auto-resume executor is blind" in poll_src, "repeated mem-param read failures must be logged"


def test_a_genuinely_new_offer_presses_again():
  p = ResumePress()
  for f in range(200):
    p.update(f, cmd(eid=7.0), True)
  again = [p.update(f, cmd(eid=8.0), True) for f in range(200, 400)]
  assert sum(again) == I.RESUME_PRESS_FRAMES


def test_press_aborts_mid_press_when_a_gate_drops():
  p = ResumePress()
  c = cmd(eid=7.0)
  assert p.update(0, c, True) is True
  assert p.update(1, c, True) is True
  assert p.update(2, c, False) is False, "must abort mid-press, not complete the tap"
  # ...and the eid is spent: it cannot be retried.
  assert not any(p.update(f, c, True) for f in range(3, 200))


def test_press_aborts_if_the_offer_is_withdrawn_mid_press():
  p = ResumePress()
  c = cmd(eid=7.0)
  assert p.update(0, c, True) is True
  assert p.update(1, None, True) is False


def test_nothing_is_pressed_without_an_offer():
  p = ResumePress()
  assert not any(p.update(f, None, True) for f in range(100))
  assert not any(p.update(f, cmd(), False) for f in range(100, 200))
  assert p.used_eid is None

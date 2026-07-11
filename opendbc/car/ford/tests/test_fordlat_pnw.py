"""Human-turn reset tests (pure, capnp-free)."""
from opendbc.car.ford.fordlat_pnw import HumanTurnHold, HT_HOLD_S, _STEER_DT


def ticks(seconds):
  return int(round(seconds / _STEER_DT))


def test_sustained_hold_triggers():
  h = HumanTurnHold()
  fired = [h.tick(True, 60.0) for _ in range(ticks(HT_HOLD_S) + 2)]
  assert fired[0] is False                      # not instantly
  assert fired[-1] is True                      # after the sustained hold


def test_nudge_never_triggers():
  h = HumanTurnHold()
  # nudges: pressed but small angle, or big angle but brief
  assert not any(h.tick(True, 20.0) for _ in range(ticks(5.0)))     # small angle forever
  h2 = HumanTurnHold()
  for _ in range(ticks(HT_HOLD_S) - 2):                              # big angle, too brief
    h2.tick(True, 90.0)
  assert h2.tick(False, 0.0) is False                                # released before threshold
  assert h2.tick(True, 90.0) is False                                # counter restarted


def test_release_resets_immediately():
  h = HumanTurnHold()
  for _ in range(ticks(HT_HOLD_S) + 5):
    h.tick(True, 60.0)
  assert h.tick(True, 60.0) is True
  assert h.tick(False, 60.0) is False           # hands off -> flush ends, ramp-back begins
  assert h.tick(True, 60.0) is False            # fresh hold starts counting from zero

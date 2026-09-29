"""teslastalk2b: HoldOffGate -- the first value and the first onset are never held, flips inside the hold-off are
counted, and the final state is always emitted."""
from opendbc.car.holdoff_pnw import HoldOffGate


def run(gate, seq):
  """seq: [(t, value)] -> list of (t, value, prev, n) emissions."""
  out = []
  for t, v in seq:
    r = gate.step(v, t)
    if r is not None:
      out.append((t, v, *r))
  return out


def test_first_value_is_emitted_and_does_not_start_a_holdoff():
  g = HoldOffGate(2.0, initial=None)
  assert run(g, [(0.0, False), (0.1, True)]) == [(0.0, False, None, 0), (0.1, True, False, 0)]


def test_flips_inside_the_holdoff_are_counted_and_the_final_value_is_emitted_when_it_ends():
  g = HoldOffGate(2.0, initial=False)
  seq = [(0.0, True)] + [(0.5 * i, i % 2 == 0) for i in range(1, 4)] + [(0.5 * i, False) for i in range(4, 9)]
  out = run(g, seq)
  assert out[0] == (0.0, True, False, 0)                  # first onset: immediate
  assert len(out) == 2 and out[1][1] is False and out[1][3] == 3 and out[1][0] >= 2.0    # trailing emit, 3 flips counted


def test_a_flap_that_ends_where_it_began_is_still_reported():
  g = HoldOffGate(2.0, initial=False)
  out = run(g, [(0.0, True), (0.5, False), (1.0, True)] + [(1.0 + 0.5 * i, True) for i in range(1, 6)])
  assert out[-1][1] is True and out[-1][2] is True and out[-1][3] == 2


def test_a_quiet_gate_emits_nothing():
  g = HoldOffGate(2.0, initial=False)
  assert run(g, [(0.1 * i, False) for i in range(1000)]) == []


def test_a_change_after_the_holdoff_is_immediate():
  g = HoldOffGate(2.0, initial=False)
  assert [e[:2] for e in run(g, [(0.0, True), (5.0, False), (9.0, True)])] == [(0.0, True), (5.0, False), (9.0, True)]

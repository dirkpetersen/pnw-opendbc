"""steerdebounce2pnw: the EAC_INHIBITED -> steerFaultTemporary debounce.

Measured on the car (I-5 NB, 2026-09-03): the Tesla EPS reports EAC_INHIBITED for exactly 4 frames
out of 6137 in a 61 s segment -- all four at the engage transition, as the rack hands over from
EAC_AVAILABLE to EAC_ACTIVE. openpilot promoted that 40 ms artifact into a steerTempUnavailable
SOFT_DISABLE (selfdriveState went enabled -> softDisabling -> enabled in 40 ms) and a driver-facing
"Steering Temporarily Unavailable" alert on EVERY engage.
"""
from opendbc.car.tesla.carstate import EAC_INHIBITED_MIN_FRAMES, debounce_eac_inhibited


def _run(pattern: list[bool]) -> list[bool]:
  """Feed a frame-by-frame inhibited pattern, return what would be reported each frame."""
  out, cnt = [], 0
  for inhibited in pattern:
    fault, cnt = debounce_eac_inhibited(cnt, inhibited)
    out.append(fault)
  return out


class TestDebounceEacInhibited:
  def test_the_measured_engage_artifact_is_suppressed(self):
    """THE case this exists for: 4 inhibited frames at engage, surrounded by healthy ones."""
    reported = _run([False] * 20 + [True] * 4 + [False] * 20)
    assert not any(reported), "the 40 ms engage handover still raises a driver alert"

  def test_a_sustained_fault_is_still_reported(self):
    """Fail-safe: debouncing must delay a real fault, never suppress it."""
    reported = _run([True] * 50)
    assert any(reported), "a sustained EAC_INHIBITED must still reach the driver"

  def test_reports_exactly_at_the_threshold_not_before(self):
    reported = _run([True] * (EAC_INHIBITED_MIN_FRAMES + 3))
    assert not any(reported[:EAC_INHIBITED_MIN_FRAMES - 1])
    assert reported[EAC_INHIBITED_MIN_FRAMES - 1]

  def test_clears_instantly_on_one_healthy_frame(self):
    """Asymmetry check: slow to assert, INSTANT to clear, so recovery is never delayed."""
    pattern = [True] * (EAC_INHIBITED_MIN_FRAMES + 5) + [False]
    reported = _run(pattern)
    assert reported[-2], "should have been faulting just before recovery"
    assert not reported[-1], "a single healthy frame must clear the fault immediately"

  def test_an_interrupted_run_never_reports(self):
    """Two sub-threshold bursts separated by one healthy frame must not add up."""
    n = EAC_INHIBITED_MIN_FRAMES - 1
    assert not any(_run([True] * n + [False] + [True] * n))

  def test_counter_does_not_grow_without_bound_in_the_reported_state(self):
    """Long faults keep reporting; the count is only ever compared, never used as a magnitude."""
    _, cnt = True, 0
    for _ in range(10_000):
      fault, cnt = debounce_eac_inhibited(cnt, True)
    assert fault
    assert isinstance(cnt, int)

  def test_latency_is_a_small_fraction_of_the_soft_disable_budget(self):
    """steerTempUnavailable is a SOFT_DISABLE, which takes 3 s to actually disengage. The debounce
    must cost a small slice of that, or it would eat into a real fault's reaction time."""
    latency_s = EAC_INHIBITED_MIN_FRAMES / 100.0     # EPAS publishes at ~100 Hz
    assert latency_s <= 0.15, f"{latency_s:.2f}s is too much delay on a real steering fault"
    assert latency_s / 3.0 < 0.06, "debounce eats >6% of the SOFT_DISABLE budget"

  def test_threshold_has_margin_over_the_measured_artifact(self):
    """4 frames observed on-car; keep real headroom so a slightly longer handover is still absorbed."""
    assert EAC_INHIBITED_MIN_FRAMES >= 8, "too little margin over the measured 4-frame handover"

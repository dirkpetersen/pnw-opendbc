"""steerdebounce2pnw: separating the EPS engage-handover artifact from a real steering inhibit.

Evidence (decoder + rlogs archived under drives/2026-09-03/hotspot-drive-tesla/):
  * ~18,600 EPAS frames over 4 segments. The ONLY inhibited run near an engage was
    4 frames carrying EAC_ERROR_IDLE -- the EPS saying "inhibited, no reason".
  * Every real inhibit observed (2026-08-31 seg58, driver override) ran 19-20 frames
    carrying EAC_ERROR_HANDS_ON.
The discriminator is therefore the ERROR CODE, not the duration -- which is why a coded inhibit is
reported with ZERO added latency and only the uncoded class is filtered.
"""
from opendbc.car.tesla.carstate import (
  EAC_IDLE_CAP,
  EAC_IDLE_REPORT,
  EAC_IDLE_RISE,
  next_steer_fault_temporary,
)

CODED, UNCODED, HEALTHY = (True, False), (True, True), (False, True)


def _run(seq):
  out, cnt = [], 0
  for inhibited, err_idle in seq:
    fault, cnt = next_steer_fault_temporary(cnt, inhibited, err_idle)
    out.append(fault)
  return out


class TestCodedInhibitsAreInstant:
  def test_a_single_coded_frame_reports_immediately(self):
    """A real fault must not be delayed at all -- this is the whole point of keying on the code."""
    assert _run([HEALTHY] * 20 + [CODED])[-1]

  def test_the_measured_real_inhibit_reports_on_frame_one(self):
    """The 19-20 frame HANDS_ON runs seen on 2026-08-31 must report instantly, not after a debounce."""
    reported = _run([HEALTHY] * 10 + [CODED] * 20)
    assert reported[10], "a coded inhibit was delayed"
    assert all(reported[10:])

  def test_a_flapping_coded_inhibit_is_never_hidden(self):
    """Fable's objection: a consecutive-count rule lets a fast-flapping fault vanish. A coded
    inhibit bypasses the counter entirely, so no duty cycle can hide it."""
    reported = _run([CODED, HEALTHY] * 50)
    assert sum(reported) == 50


class TestUncodedIsFiltered:
  def test_the_measured_engage_artifact_is_suppressed(self):
    """THE case this exists for: 4 uncoded frames at engage."""
    assert not any(_run([HEALTHY] * 20 + [UNCODED] * 4 + [HEALTHY] * 20))

  def test_a_sustained_uncoded_inhibit_is_still_reported(self):
    """Fail-safe: filtering must delay an uncoded inhibit, never suppress it outright."""
    reported = _run([UNCODED] * 40)
    assert any(reported), "a sustained uncoded inhibit must still reach the driver"
    n = EAC_IDLE_REPORT // EAC_IDLE_RISE
    assert reported[n - 1] and not any(reported[:n - 1]), f"expected first report at frame {n}"

  def test_a_flapping_uncoded_inhibit_ratchets_up_instead_of_hiding(self):
    """The reason this is a leaky counter and not a consecutive count: above ~25% duty the rise
    outpaces the decay, so a persistent flap is reported rather than reset away forever."""
    assert any(_run([UNCODED, HEALTHY] * 60)), "50% duty uncoded flap stayed invisible"

  def test_a_sparse_uncoded_flap_stays_quiet(self):
    """Below the rise/decay ratio it should stay quiet -- that is the artifact class behaving."""
    assert not any(_run([UNCODED] + [HEALTHY] * 9) * 20)

  def test_the_fault_is_held_not_cleared_on_the_first_healthy_frame(self):
    """Fail-safe for a fault flag is to HOLD. Clearing instantly was the first cut's mistake."""
    seq = [UNCODED] * 20 + [HEALTHY] * 3
    reported = _run(seq)
    assert reported[-1], "fault cleared on the first healthy frames instead of decaying"

  def test_it_does_eventually_clear(self):
    assert not _run([UNCODED] * 20 + [HEALTHY] * (EAC_IDLE_CAP + 5))[-1]

  def test_counter_is_bounded(self):
    _, cnt = next_steer_fault_temporary(0, True, True)
    for _ in range(10_000):
      _, cnt = next_steer_fault_temporary(cnt, True, True)
    assert cnt <= EAC_IDLE_CAP

  def test_a_coded_inhibit_resets_the_uncoded_counter(self):
    """A coded inhibit reports on its own; it must not also leave the uncoded counter armed."""
    _, cnt = next_steer_fault_temporary(EAC_IDLE_REPORT - 1, True, False)
    assert cnt == 0

  def test_healthy_is_never_a_fault(self):
    assert not any(_run([HEALTHY] * 500))

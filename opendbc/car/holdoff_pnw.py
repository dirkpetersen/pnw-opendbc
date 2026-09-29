"""teslastalk2b: a hold-off for a flapping boolean verdict, so a log line / record is not written on every flip.

Used for the Raven's EPS-refusal verdict, which can flip every ~0.51 s (the wheel-touched path re-fires
steerTempUnavailableSilent every 51 frames) -- ~4 records/s if every change is written. The unit of `now` is the
caller's (frames in carstate, seconds in the ces_events logger); `holdoff` is in the same unit.

Rules (Rule 2: nothing silent):
  * the FIRST value is emitted (initial=None) and does not start a hold-off, so the first real onset is never held;
  * a change while no hold-off is running is emitted at once and starts one;
  * a change while a hold-off is running is COUNTED, not emitted; when the hold-off ends the CURRENT value is emitted
    together with the count -- so the final state (e.g. the final clear) is always recorded, and a flap that ends
    where it began is still reported (same value, count > 0).
"""


class HoldOffGate:
  def __init__(self, holdoff: float, initial: bool | None = None):
    self.holdoff = holdoff
    self._emitted = initial
    self._cur = initial
    self._t: float | None = None
    self.suppressed = 0

  def step(self, value: bool, now: float) -> tuple[bool | None, int] | None:
    """Call every tick. Returns (previous emitted value, changes suppressed since it) when to emit, else None."""
    value = bool(value)
    changed = value != self._cur
    self._cur = value
    held = self._t is not None and (now - self._t) < self.holdoff
    if changed and held:
      self.suppressed += 1
    if held or (value == self._emitted and self.suppressed == 0):
      return None
    prev, n = self._emitted, self.suppressed
    self._emitted, self.suppressed = value, 0
    self._t = None if prev is None else now
    return prev, n

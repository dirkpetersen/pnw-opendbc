"""
icbm2pnw — Intelligent Cruise Button Management for the F-150 Lightning (PNW fork).

Steers the STOCK ACC set speed by spoofing the driver's SET +/- taps on Steering_Data_FD1 (0x083,
already in the Ford safety TX allowlist for cancel/resume — no panda change). Ford registers a tap
as a 1 mph step. The "brain" (what speed to target) lives in the pnw layer (CES/VTSC-derived); this
module is only the deterministic executor: closed-loop on the truck's own reported set speed.

Safety envelope (enforced HERE, independent of the brain) — Gemini-hardened 2026-07-11:
  - DEC-ONLY remains the rule for CAPS: a cap command ("dir" absent / "dec") can ONLY LOWER the
    stock set speed, never raise it.
  - icbmrestore2pnw (driver-requested): a GUARDED restore path exists for commands explicitly
    marked "dir": "inc" — used ONLY to return the set speed to the driver's OWN latched ceiling
    after a curve episode. Hard bounds enforced HERE independent of the brain: never above the
    ceiling, never while cruise is off / driver pedals / heartbeat stale, never a dec from an inc
    command (no oscillation), and the RestoreGuard latch kills the inc path for the remainder of
    the episode the instant the stock set moves in a way this executor did not command (any
    decrease, or a rise faster than our own tap cadence — a driver SET+ hold steps 5 mph and trips
    it immediately). The brain additionally bounds the episode (45 s window, pedal/ACC/new-cap
    aborts) — see ces_pnw.IcbmEpisode.
  - acts only while the stock ACC is actively engaged (cruise enabled), never engages/resumes it
  - the brain's target is still clamped to the latched driver ceiling for sanity
  - stale/absent target (no fresh brain heartbeat) => no presses at all
  - driver gas/brake pauses pressing (driver always wins); a press aborts MID-PRESS the instant its
    preconditions vanish (cruise off / override / stale) rather than completing the tap

speedadjust-exec2pnw: this module's decide_press/PressGovernor/RestoreGuard are shared by a SECOND
brain, speedadjust2pnw (police-ahead / lower-speed-limit reduce-only cap), which publishes a
SpeedAdjustTarget mem-param in the exact same {target, ceiling, ts, dir?} shape as IcbmTarget. Only
one command can be asserted on the shared SET+/SET- buttons at a time, so carcontroller.py calls
arbitrate() (below) to pick between the two brains' commands before calling decide_press() — see
arbitrate()'s docstring for the rule (DEC always wins; more-restrictive dec wins between two decs).
"""

from dataclasses import dataclass

# Ford registers one SET tap = 1 mph. Send a press for PRESS_FRAMES consecutive control frames
# (100 Hz) so the 10 Hz SCCM stream reliably carries it, then release for at least GAP_FRAMES.
MPH_TO_MS = 0.44704
STEP_MS = 1.0 * MPH_TO_MS         # one tap's worth of set-speed change
DEADBAND_MS = 0.6 * STEP_MS       # don't chase differences smaller than this
PRESS_FRAMES = 10                 # 100 ms press
GAP_FRAMES = 30                   # 300 ms release between taps — clearly discrete taps, never a
                                  # merged "hold" (Ford holds step 5 mph; taps step 1 mph)
STALE_LIMIT_S = 2.0               # brain heartbeat older than this => do nothing
TAP_PERIOD_S = (PRESS_FRAMES + GAP_FRAMES) / 100.0   # min seconds between our own completed taps


@dataclass
class IcbmCommand:
  target_ms: float                # desired stock-ACC set speed (m/s)
  ceiling_ms: float               # driver's own set speed at cap entry (m/s) — never exceed
  ts: float                       # brain wall-clock heartbeat (seconds)
  dir: str = "dec"                # icbmrestore2pnw: "dec" = cap (default), "inc" = guarded restore


def decide_press(stock_set_ms: float, cmd: IcbmCommand | None, now: float,
                 cruise_enabled: bool, driver_override: bool) -> str | None:
  """Pure decision: which button (if any) SHOULD be pressed this instant, ignoring cadence.
  Returns 'dec', 'inc' or None. Cadence/timing is PressGovernor's job."""
  if cmd is None or not cruise_enabled or driver_override:
    return None
  if now - cmd.ts > STALE_LIMIT_S:
    return None
  if stock_set_ms <= 0:           # no valid set speed reported
    return None
  # clamp the brain's target into the safety envelope: never above the driver's ceiling
  target = min(cmd.target_ms, cmd.ceiling_ms)
  if target <= 0:
    return None
  if getattr(cmd, "dir", "dec") == "inc":
    # icbmrestore2pnw RESTORE path: press UP toward the (ceiling-clamped) restore target only.
    # NEVER a dec from an inc command — the two directions can't oscillate within one command, and
    # a cap (dec) command always replaces an inc one at the brain (dec wins).
    if stock_set_ms < target - DEADBAND_MS:
      return "inc"
    return None                   # reached (or passed) the restore point: silent
  if stock_set_ms > target + DEADBAND_MS:
    return "dec"
  return None                     # caps stay DEC-ONLY: never press up on a cap command


def arbitrate(icbm_cmd: "IcbmCommand | None", sa_cmd: "IcbmCommand | None", now: float) -> "IcbmCommand | None":
  """speedadjust-exec2pnw: the truck has exactly ONE set of stock-ACC buttons, shared by TWO brains —
  icbm2pnw (curve slow-downs, ces_pnw.py) and speedadjust2pnw (police/limit slow-downs,
  speedadjust_controller.py). Only one command may be asserted per frame; this picks it.

  Rule (mirrors the icbm2pnw episode machine's own "a NEW cap (DEC ALWAYS WINS...)" principle,
  extended across sources instead of within one source's episodes):
    1. If EITHER source has a fresh ("dir" == "dec", heartbeat within STALE_LIMIT_S) cap command,
       a reduction is required. Assert whichever of the two wants the LOWER target speed (the more
       restrictive requirement always governs) — passed through UNCHANGED (its own ceiling/ts/dir),
       no cross-source merging of ceiling values. Fresh vs stale is checked HERE (not left to
       decide_press) precisely so a dead/stale source can never contribute a target to the min().
    2. Only when NEITHER source currently wants a dec does an "inc" (restore) get to run. If both
       sources simultaneously offer a fresh restore (two independent restores landing the same
       tick — expected to be rare/never in practice today since only icbm2pnw's episode machine
       emits "inc"), assert the one with the LOWER target/ceiling — restoring toward the more
       conservative of the two ceilings can never overshoot either source's own bound.
    3. Otherwise (both sources silent/idle/stale): None — the executor stays quiet.

  Pure; never raises (bad input just fails the freshness/shape checks and is treated as absent).
  decide_press() independently re-checks staleness/ceiling/etc. on whatever this returns — this
  function only decides WHICH of the (at most) two live commands wins the shared bus."""
  def _fresh(c, want_dir):
    if c is None:
      return None
    if getattr(c, "dir", "dec") != want_dir:
      return None
    try:
      if now - c.ts > STALE_LIMIT_S:
        return None
    except (TypeError, AttributeError):
      return None
    return c

  a_dec, b_dec = _fresh(icbm_cmd, "dec"), _fresh(sa_cmd, "dec")
  if a_dec is not None and b_dec is not None:
    return a_dec if a_dec.target_ms <= b_dec.target_ms else b_dec
  if a_dec is not None:
    return a_dec
  if b_dec is not None:
    return b_dec

  a_inc, b_inc = _fresh(icbm_cmd, "inc"), _fresh(sa_cmd, "inc")
  if a_inc is not None and b_inc is not None:
    return a_inc if a_inc.target_ms <= b_inc.target_ms else b_inc
  if a_inc is not None:
    return a_inc
  if b_inc is not None:
    return b_inc
  return None


class RestoreGuard:
  """icbmrestore2pnw: executor-side human-detection latch for the restore (inc) path.

  While a restore command is active, the ONLY thing that should move the stock set speed is this
  executor's own +1 mph taps (at most one per TAP_PERIOD_S). So between observations:
    - ANY decrease of the set speed        -> a human pressed SET- (or the ACC did something we
                                              don't understand) -> BLOCK
    - a rise faster than our tap cadence   -> a human is pressing/holding SET+ (Ford hold = 5 mph
      (> elapsed/TAP_PERIOD_S + 1 taps)       steps, trips this immediately) -> BLOCK
  BLOCK latches for the remainder of the restore episode: inc intents are swallowed until the
  episode ends (an empty command or a dec command clears the latch — dec is never filtered).
  Residual (documented): a single driver SET+ tap during restore is indistinguishable from our own
  tap at this granularity; it is same-direction, still ceiling-bounded, and harmless."""

  def __init__(self):
    self._blocked = False
    self._last_set = None
    self._last_t = None

  @property
  def blocked(self) -> bool:
    return self._blocked

  def filter(self, intent: str | None, stock_set_ms: float, now: float, restoring: bool) -> str | None:
    """Pass every frame. `restoring` = the current brain command is an inc/restore command."""
    if not restoring:
      # episode over (silent) or a cap owns the bus (dec): clear the latch, never filter dec
      self._blocked = False
      self._last_set = None
      self._last_t = None
      return intent
    if self._last_set is not None and stock_set_ms > 0:
      dt = max(now - (self._last_t or now), 0.0)
      if stock_set_ms < self._last_set - 0.6 * STEP_MS:
        self._blocked = True                            # set went DOWN while we only press up
      elif stock_set_ms > self._last_set + STEP_MS * (dt / TAP_PERIOD_S + 1.6):
        self._blocked = True                            # rose faster than our own taps can
    if stock_set_ms > 0:
      self._last_set = stock_set_ms
      self._last_t = now
    return None if self._blocked else intent


class PressGovernor:
  """Turns decide_press() intents into a press/release tap pattern at the control rate (100 Hz)."""

  def __init__(self):
    self._press_until = -1        # frame index the current press ends at
    self._next_allowed = 0        # earliest frame a new press may start
    self._active: str | None = None

  def update(self, frame: int, intent: str | None) -> str | None:
    """Returns the button to assert THIS frame ('dec') or None."""
    if self._active is not None:
      if intent != self._active:
        # preconditions vanished mid-press (cruise off / override / stale): abort IMMEDIATELY
        self._active = None
        self._next_allowed = frame + GAP_FRAMES
        return None
      if frame < self._press_until:
        return self._active       # keep asserting through the press window
      self._active = None
      self._next_allowed = frame + GAP_FRAMES
    if intent is None or frame < self._next_allowed:
      return None
    self._active = intent
    self._press_until = frame + PRESS_FRAMES
    return intent

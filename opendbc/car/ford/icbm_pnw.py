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

speedadjust-exec2pnw: this module's decide_press/PressGovernor/RestoreGuard are ONE generic executor,
shared by however many pnw brains want to steer the stock set speed — today icbm2pnw (curve slow-
downs) and speedadjust2pnw (police-ahead / lower-speed-limit reduce-only cap), each publishing its
own mem-param in the identical {target, ceiling, ts, dir?} shape. Only one command can be asserted on
the shared SET+/SET- buttons at a time, so carcontroller.py calls arbitrate() (below) to reduce every
brain's command down to the ONE unified button-management target before calling decide_press() — see
arbitrate()'s docstring for the rule (DEC always wins; most-restrictive/lowest dec wins across
sources). decide_press()/PressGovernor/RestoreGuard themselves have no notion of "which brain" — they
are fully generic over the winning command's source.
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
# restore2pnw-hardening (2026-08, Gemini + Fable review of speedadjust-exec2pnw): after ANY fresh dec
# wins the shared bus, require it to stay clear this long before an inc may be asserted again — damps
# a marginal candidate flapping at its own binding threshold against an open restore window
# (SET-/SET+ oscillation).
INC_AFTER_DEC_DEBOUNCE_S = 1.0


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
  direction = getattr(cmd, "dir", "dec")
  if direction not in ("dec", "inc"):
    # madsresume2pnw: this function handles the SET-/SET+ set-speed directions ONLY. Any other
    # direction (today "res", the auto-resume tap) has its own path -- decide_resume() below -- and
    # must NEVER be silently reinterpreted here. Without this the old code fell through to the DEC
    # branch and would have pressed SET- for a resume command.
    return None
  if direction == "inc":
    # icbmrestore2pnw RESTORE path: press UP toward the (ceiling-clamped) restore target only.
    # NEVER a dec from an inc command — the two directions can't oscillate within one command, and
    # a cap (dec) command always replaces an inc one at the brain (dec wins).
    if stock_set_ms < target - DEADBAND_MS:
      return "inc"
    return None                   # reached (or passed) the restore point: silent
  if stock_set_ms > target + DEADBAND_MS:
    return "dec"
  return None                     # caps stay DEC-ONLY: never press up on a cap command


def arbitrate(cmds: "list[IcbmCommand | None]", now: float,
              last_dec_ts: float | None = None) -> "IcbmCommand | None":
  """The truck has exactly ONE set of stock-ACC buttons, shared by however many pnw brains want to
  steer the set speed — today icbm2pnw (curve slow-downs, ces_pnw.py) and speedadjust2pnw
  (police-ahead / lower-posted-limit slow-downs, speedadjust_controller.py), each publishing its own
  {target, ceiling, ts, dir?} mem-param. This is the ONE unified "button-management target": every
  poll, exactly one command is asserted on the shared bus, chosen from `cmds` (any number of brains —
  the list can grow) by this single rule. `decide_press()` never sees more than one command; it has
  no notion of "which brain" — the executor is fully generic over the source.

  Rule (mirrors the icbm2pnw episode machine's own "a NEW cap (DEC ALWAYS WINS...)" principle,
  extended across sources instead of within one source's episodes):
    1. If ANY source has a fresh ("dir" == "dec", heartbeat within STALE_LIMIT_S) cap command, a
       reduction is required. Assert whichever wants the most conservative EFFECTIVE bound (the
       single most-restrictive requirement always governs) — passed through UNCHANGED (its own
       ceiling/ts/dir), no cross-source merging of ceiling values. Fresh vs stale is checked HERE
       (not left to decide_press) precisely so a dead/stale source can never contribute to the min().
    2. Only when NO source currently wants a dec does an "inc" (restore) get to run, AND only after
       the bus has been dec-free for INC_AFTER_DEC_DEBOUNCE_S (`last_dec_ts`, tracked by the caller
       across polls — this function stays pure/stateless) — a marginal candidate flapping right at its
       own binding threshold must not chatter the buttons SET-/SET+/SET-/... If more than one source
       simultaneously offers a fresh restore (independent restores landing the same tick — expected to
       be rare/never in practice today since only icbm2pnw's episode machine emits "inc"), assert the
       one with the most conservative EFFECTIVE bound — restoring toward the most conservative of the
       offered ceilings can never overshoot any source's own bound.
    3. Otherwise (every source silent/idle/stale, or debounced): None — the executor stays quiet.

  Selection key (both steps 1 and 2): `min(c.target_ms, c.ceiling_ms)`, NOT bare `c.target_ms` — every
  command published today always has target_ms == ceiling_ms at its OWN restore point (a full restore
  to the latched ceiling), so this is a no-op in current practice; it is still the correct key rather
  than a coincidentally-equivalent one, because it is robust to a future brain whose target and ceiling
  legitimately differ (e.g. a partial-restore step), or to a malformed command whose target
  transiently exceeds its own ceiling — a raw-target comparison could rank such a command as "more
  conservative" than it actually is. decide_press() independently re-clamps `min(target_ms, ceiling_ms)`
  on whatever single command wins here, so this is defense-in-depth, not the only clamp.

  Pure; never raises (bad input just fails the freshness/shape checks and is treated as absent).
  decide_press() independently re-checks staleness/ceiling/etc. on whatever this returns — this
  function only decides WHICH of the live commands wins the shared bus."""
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

  def _bound(c):
    return min(c.target_ms, c.ceiling_ms)

  decs = [f for f in (_fresh(c, "dec") for c in cmds) if f is not None]
  if decs:
    return min(decs, key=_bound)

  incs = [f for f in (_fresh(c, "inc") for c in cmds) if f is not None]
  if incs:
    if last_dec_ts is not None:
      try:
        if now - last_dec_ts < INC_AFTER_DEC_DEBOUNCE_S:
          return None       # a dec was live too recently — hold off on inc to avoid flapping
      except TypeError:
        pass
    return min(incs, key=_bound)

  return None


_UNSET = object()   # RestoreGuard episode-identity sentinel — distinct from a legitimate ceiling=None


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
  tap at this granularity; it is same-direction, still ceiling-bounded, and harmless.

  restore2pnw-hardening (cross-brain dec-interlude fix): the veto latch is keyed to the restore
  EPISODE's identity (its `ceiling`), not merely to the `restoring` flag toggling. Without this, a
  brief dec from a DIFFERENT brain winning the shared bus mid-restore (e.g. a curve-ICBM dec while
  speedadjust's restore is still logically in flight underneath) would report `restoring=False` for
  those ticks under the old scheme, wiping `_blocked`/`_last_set` — resuming the restore afterward as
  if the driver's earlier SET- veto had never happened. Callers must pass `restoring`/`ceiling`
  derived from whether ANY brain still has a live inc offer pending (see
  carcontroller.py:_icbm_buttons's `pending_inc`), not just whichever command arbitrate() picked to
  press THIS tick — that is what lets the SAME episode survive a preempting dec interlude here.

  Fable fail-safe fix (2026-08): movement-judgment is FROZEN on ticks where a dec command owns the
  shared bus (`dec_owns_bus=True`, whether that dec is a different brain's cap during this episode's
  restore window, or this episode's own cap). Without this, the dec interlude's OWN SET- taps move
  stock_set_ms down exactly like a driver SET- would — the judgment below can't tell the difference —
  and would falsely latch `_blocked` on an otherwise-clean restore, permanently vetoing it (the truck
  never resumes toward the ceiling until the driver manually taps SET+). A genuine driver decrease
  DURING a dec interlude is already covered independently by the brain-side 1.7-step `_pub_ceiling`
  ratchets on both brains (speedadjust_controller.py) backing off their own target, so this guard does
  not need to (and must not) judge movement while a dec owns the bus — it just drops its movement
  baseline (`_last_set = None`) so the next inc/restore tick re-baselines cleanly against the
  post-interlude set speed instead of comparing across the interlude."""

  def __init__(self):
    self._blocked = False
    self._last_set = None
    self._last_t = None
    self._episode_ceiling = _UNSET   # identity of the restore episode currently being tracked

  @property
  def blocked(self) -> bool:
    return self._blocked

  def filter(self, intent: str | None, stock_set_ms: float, now: float, restoring: bool,
             ceiling: float | None = None, dec_owns_bus: bool = False) -> str | None:
    """Pass every frame. `restoring` = SOME brain still has a live inc/restore offer pending (not
    merely "the command arbitrate() picked to press this exact tick is an inc" — see class docstring).
    `ceiling` identifies WHICH restore episode is being offered; a change in ceiling while restoring
    is treated as a genuinely NEW episode (fresh latch); the SAME ceiling persisting across a dec
    interlude (restoring stays True, ceiling unchanged) preserves the latch instead of clearing it.
    `dec_owns_bus` = the command arbitrate() picked to press THIS tick is a dec (see class docstring's
    Fable fail-safe fix): movement-judgment is skipped entirely on these ticks — the latch is neither
    set NOR cleared, only the movement baseline is dropped. `dec` (or no) intent is NEVER filtered by
    this latch, regardless of `_blocked`."""
    if not restoring:
      # no brain has a live inc offer at all -> fully stand down, clear the latch, never filter dec
      self._blocked = False
      self._last_set = None
      self._last_t = None
      self._episode_ceiling = _UNSET
      return intent
    if self._episode_ceiling is _UNSET or self._episode_ceiling != ceiling:
      # first restore ever, or a genuinely different episode (different ceiling) -> fresh latch. A
      # same-ceiling dec interlude from a different brain never reaches this branch (restoring/ceiling
      # reflect the still-pending inc offer underneath it, per the caller contract above).
      self._blocked = False
      self._last_set = None
      self._last_t = None
      self._episode_ceiling = ceiling
    if dec_owns_bus:
      # Fable fail-safe fix: a dec owns the bus this tick (a curve-ICBM cap interlude, or this
      # episode's own cap) — its own SET- taps would look identical to a driver SET- to the judgment
      # below, so freeze judgment entirely (no latch, no clear) and just drop the movement baseline;
      # see class docstring. Genuine driver decreases during the interlude are already covered by the
      # brain-side ratchets, not this guard.
      self._last_set = None
      self._last_t = None
    else:
      if self._last_set is not None and stock_set_ms > 0:
        dt = max(now - (self._last_t or now), 0.0)
        if stock_set_ms < self._last_set - 0.6 * STEP_MS:
          self._blocked = True                          # set went DOWN while we only press up
        elif stock_set_ms > self._last_set + STEP_MS * (dt / TAP_PERIOD_S + 1.6):
          self._blocked = True                          # rose faster than our own taps can
      if stock_set_ms > 0:
        self._last_set = stock_set_ms
        self._last_t = now
    if intent != "inc":
      return intent            # dec (or no intent) is NEVER filtered by the restore veto latch
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


# ------------------------------------------------------------------------------------------------
# madsresume2pnw — the RESUME tap. A separate command, a separate parser, a separate decision, a
# separate one-shot press latch. Deliberately shares NOTHING with the dec/inc set-speed path above
# except the CAN frame (0x083) it ultimately rides on.
#
# Why it cannot go through decide_press(): a resume happens precisely when stock cruise is OFF, and
# decide_press() returns None for `not cruise_enabled` (correctly -- a set-speed tap while cruise is
# off is meaningless). The two have OPPOSITE cruise preconditions, so they are opposite functions.
#
# THE CRITICAL GATE: `cruise_enabled` must be FALSE. On Ford, RES while ACC is engaged is a
# SET+ (+1 mph) -- pressing it there would raise the driver's set speed, which is the one thing
# this whole feature is forbidden to do. The brain gates on this too; this is the independent
# executor-side enforcement, read straight off the real CAN state.
# ------------------------------------------------------------------------------------------------

RESUME_DIR = "res"
# gasset2pnw: the SECOND thing this executor may tap. "res" restores the PCM's own remembered set
# speed and therefore ACCELERATES; "set" establishes the speed the driver has just chosen with the
# accelerator and commands no speed change at all. Same frame, same one-shot latch, different bit.
SET_DIR = "set"
# A resume offer is only actionable for a moment. The brain re-publishes it (same `eid`, fresh
# `ts`) every tick its gates still hold and withdraws it the instant one stops -- so a tight
# freshness bound is what makes the press land within ~half a second of a tick where the lead/TTC/
# set-speed gates were all verified, instead of up to STALE_LIMIT_S later against a stale world.
#
# The freshness bound is a BACKSTOP, not the withdrawal path: the caller re-reads the mem-param
# every frame while it holds a command (carcontroller._resume_button), so a withdrawal lands within
# one 10 ms frame. Gemini review 2026-09-06 caught the earlier version, which polled at a flat 4 Hz
# and therefore kept pressing off a CACHED command for up to 250 ms after the brain had already
# withdrawn the offer because a lead cut in -- defeating the brain's whole fast-abort design.
RESUME_STALE_LIMIT_S = 0.5

# `ts` on a resume offer is MONOTONIC (time.monotonic(), which is CLOCK_MONOTONIC and therefore
# shared across processes on this host), NOT wall clock. The dec/inc set-speed path uses wall clock
# for historical reasons and is left alone, but a resume must not: this device has a dead RTC and
# takes a large clock STEP when it first syncs, and a backward step would make an offer published
# minutes ago look fresh (or would instantly kill a live one). Gemini review 2026-09-06.
# See also the negative-dt guard in decide_resume().
RESUME_PRESS_FRAMES = PRESS_FRAMES        # one discrete tap, same shape as a SET tap
# How far in the FUTURE a heartbeat may sit before it is judged a clock disagreement rather than
# publish-time rounding. See the dt check in decide_resume().
RESUME_FUTURE_TOL_S = 0.02


@dataclass
class ResumeCommand:
  """madsresume2pnw brain -> executor. A DIFFERENT type from IcbmCommand on purpose: nothing that
  handles set-speed targets can be handed one of these by accident, and vice versa."""
  ts: float                       # MONOTONIC heartbeat, re-published while the offer stands
  eid: float                      # episode id -- CONSTANT for one offer; the one-shot press key
  # the target speed (m/s): the driver's captured pre-brake set speed for "res", or the speed they
  # reached on the accelerator for "set". Used for the executor's own independent ceiling check.
  set_ms: float
  # which button. "res" (RESUME) or "set" (SET at current speed).
  mode: str = RESUME_DIR


def parse_resume_cmd(raw) -> "ResumeCommand | None":
  """Fail-closed parser for the MadsResumeTarget mem-param. NEVER raises. Anything malformed,
  non-finite, wrongly-directed or missing a field yields None -> no press.

  Shape: {"dir": "res", "ts": <float>, "eid": <float>, "set": <float m/s>}. The "dir" field is
  REQUIRED to be exactly "res" -- an IcbmTarget/SpeedAdjustTarget payload accidentally routed here
  is rejected, exactly as a "res" payload is rejected by _parse_button_cmd's dec/inc whitelist."""
  import json
  import math as _math
  try:
    if isinstance(raw, (bytes, str)) and raw:
      raw = json.loads(raw)
    if not isinstance(raw, dict):
      return None
    mode = str(raw.get("dir", ""))
    if mode not in (RESUME_DIR, SET_DIR):
      return None
    if not all(k in raw for k in ("ts", "eid", "set")):
      return None
    ts, eid, set_ms = float(raw["ts"]), float(raw["eid"]), float(raw["set"])
    if not (_math.isfinite(ts) and _math.isfinite(eid) and _math.isfinite(set_ms)):
      return None
    if set_ms <= 0.0:
      return None                 # no captured set speed -> the brain must not have offered at all
    return ResumeCommand(ts=ts, eid=eid, set_ms=set_ms, mode=mode)
  except Exception:
    return None


def decide_resume(cmd: "ResumeCommand | None", now: float, cruise_enabled: bool,
                  cruise_available: bool, driver_override: bool, stock_set_ms: float) -> bool:
  """PURE. May the RESUME button be asserted this instant? Every gate here is independent of the
  brain and is read off the real car state at the carcontroller layer. `now` must be MONOTONIC,
  matching the clock the brain stamped `cmd.ts` with.

    cmd absent / wrong shape        -> no (fail-closed)
    heartbeat older than 0.5 s      -> no (the brain has withdrawn the offer, or died)
    heartbeat in the FUTURE         -> no (clocks disagree: unreadable input, not "extra fresh")
    cruise_enabled                  -> NO. RES while engaged is a SET+ on Ford: it would RAISE the
                                       driver's set speed. This is the hard one.
    not cruise_available            -> no (ACC main off: nothing to resume; the press is meaningless)
    driver_override (gas or brake)  -> no (the driver owns the pedals; they always win)
    truck reports a set speed ABOVE
      the driver's captured one     -> no (would resume above what the driver set)
  """
  if cmd is None or cruise_enabled or not cruise_available or driver_override:
    return False
  try:
    dt = now - cmd.ts
    if dt > RESUME_STALE_LIMIT_S:
      return False
    # A NEGATIVE age means the two clocks disagree (a step, a restart, a replayed payload). That is
    # not "extra fresh" -- it is an unreadable input, and an unreadable input is a refusal.
    #
    # The tolerance is not zero (Fable C1): the brain publishes `round(now, 3)`, which can land up
    # to 0.5 ms in the FUTURE, and a same-instant read would then see dt < 0 for one frame. That one
    # frame is enough for ResumePress to treat it as a mid-press abort and BURN the eid -- the
    # resume is lost for the whole brake event, logged only as `noCruise`. 20 ms is far below any
    # real clock disagreement and far above publish rounding.
    if dt < -RESUME_FUTURE_TOL_S:
      return False
  except TypeError:
    return False
  try:
    live = float(stock_set_ms)
  except (TypeError, ValueError):
    return False
  # A live reading ABOVE the brain's captured set speed means something moved the dial after the
  # capture -- refuse. (A lower or absent/zero reading is fine: resume can only go to the PCM's own
  # remembered set, which is at or below what the driver set.)
  #
  # A NON-FINITE reading is a REFUSAL, not a pass. It means gate 6's live half cannot be evaluated
  # at all, and a gate whose input is missing must fail closed -- an earlier draft of this function
  # short-circuited on `isfinite(live) and ...`, which quietly PERMITTED on a NaN/inf. Caught by
  # test_nonfinite_live_set_speed_refuses.
  import math as _math
  if not _math.isfinite(live):
    return False
  # The ceiling check is RESUME-specific: it exists so a resume can never target above what the
  # driver set. A SET establishes the current speed outright -- there is no remembered value for it
  # to exceed, and the truck's standby reading tracks current speed rather than a set point, so
  # applying this check to a SET would refuse it for a reason that does not exist.
  if cmd.mode == RESUME_DIR and live > 0.0 and live > cmd.set_ms + DEADBAND_MS:
    return False
  return True


class ResumePress:
  """One press per unique `eid`, ever. The brain latches once-per-brake-event on its side; this is
  the independent executor-side latch, so a repeated/duplicated/replayed offer cannot produce a
  second tap even if the brain misbehaves.

  Mid-press abort discipline matches PressGovernor: if `ok` stops holding (cruise came back, driver
  touched a pedal, the offer went stale) the press is dropped IMMEDIATELY rather than completing."""

  def __init__(self):
    self._used_eid = None         # eid already consumed -- never pressed again
    self._active_eid = None
    self._press_until = -1

  @property
  def used_eid(self):
    return self._used_eid

  def update(self, frame: int, cmd: "ResumeCommand | None", ok: bool) -> bool:
    """Returns True if the RESUME button should be asserted on this frame."""
    if self._active_eid is not None:
      if not ok or cmd is None or cmd.eid != self._active_eid or frame >= self._press_until:
        self._active_eid = None
        return False
      return True
    if not ok or cmd is None:
      return False
    # `<=`, not `==` (Fable C2): eid is a monotonic stamp, so an eid at or BEFORE the one already
    # spent is either the same offer again or an older/replayed one. Both must be refused, and this
    # is free.
    if self._used_eid is not None and cmd.eid <= self._used_eid:
      return False
    self._used_eid = cmd.eid
    self._active_eid = cmd.eid
    self._press_until = frame + RESUME_PRESS_FRAMES
    return True

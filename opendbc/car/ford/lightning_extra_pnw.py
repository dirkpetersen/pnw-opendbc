"""lightning-extra2pnw — Ford F-150 Lightning body conveniences that openpilot can fix and the
truck cannot.

FIRST AND ONLY FEATURE: re-arm PRO POWER ONBOARD at ignition.

The driver's complaint, 2026-09-07: the truck has a setting for "keep Pro Power on when the truck is
off" — the whole point of which is running a fridge overnight — and it RESETS TO OFF on every
ignition cycle. Drive fifty feet, shut off, and the freezer is dead. "No family member can actually
remember this."

WHAT WE FOUND ON THE BUS (measured, drives/2026-09-07/propower-toggle-scan/)
The driver toggled the setting ten times at ~15 s intervals while a full-rate CAN capture ran. Three
messages responded, every time, at exactly the press moments:

    0x455 byte 1   rests 0x80, pulses 0.5 s to 0x40 or 0x00, ALTERNATING   the button event
    0x44A byte 6 bit 4   0 <-> 1, latched                                  the state (1 = ON)
    0x480 byte 3 bit 0   0x82 <-> 0x83, latched                            the state (1 = ON)

The pulse leads the state change by 0.5 s — button, then module, then state broadcast. The pulse
VALUE carries the requested state (bit 6 set = ON), while bit 7 drops from its resting 1. First
press ON, tenth press OFF, matching the driver's own account exactly; the off->off symmetry is what
makes the reading conclusive rather than suggestive.

Frame shape, measured on the bus 2026-09-07 over a 20 s live sample while parked with the screen on:
0x455 arrives at **10.0 Hz** with a constant `0080000000000000` payload (200/200 frames), 0x44A at
2 Hz and 0x480 at 1 Hz. Eight bytes, no rolling counter, no checksum -- which is why this is
spoofable at all, and it also settles the SecOC question: bytes 2-7 are zero in every frame observed,
so there is no MAC or freshness value to forge.

WHAT IS INFERENCE RATHER THAN MEASUREMENT, stated plainly because the capture cannot separate them:
the pulse value alternated 0x40 / 0x00 and the state alternated with it, so "0x40 requests ON" and
"0x40 is simply a toggle marker" fit the data equally well. We assume the former. If it is the
latter, a press still cannot leave Pro Power OFF for long: the armer verifies the state bits after
every press and retries, and the panda refuses any payload but 0x40 -- so the failure mode is
"nothing changed, loudly", not a setting turned off behind the driver's back.

WHAT THIS MODULE IS
Pure decision logic — no CAN, no params, no I/O — so the whole envelope is unit-testable. The caller
(ford/carcontroller.py) supplies car state and sends the frame this returns.

THE ENVELOPE, and why each bound exists
  * STANDSTILL. Any standstill -- the gear is NOT required to be Park.

    ppostandstill2pnw (2026-09-09), an EXPLICIT DRIVER DECISION that relaxes a bound this file used
    to hold. It previously also required Park, reasoning: "`card` restarts on crash, so a fresh armer
    can appear mid-drive and would otherwise press at the next red light, in Drive." That reasoning
    was sound and the driver has now overruled its conclusion, in these words: "why does it have to
    be in park ... this is not a driving critical function I just wanted it to be on."

    What forced the question: on 2026-09-09 the truck was started and driven away within 16 s, so the
    armer never saw Park-at-standstill and Pro Power stayed OFF for the whole drive -- with a cooler
    of food in the back. Park-only means the ONLY chance to arm is the driver happening to sit still
    in Park with the ignition on. Any standstill turns that into "the first red light".

    So a press can now occur at a stop light, in Drive, possibly with openpilot ENGAGED. Weighed:
      - the panda still refuses the frame while the truck is moving (ford.h `!vehicle_moving`, from
        the ABS signal, independent of anything here) -- that half is NOT relaxed and is not ours to
        relax without a reflash;
      - the payload is still pinned in C to the single "press, request ON" byte, so no state here can
        turn the driver's setting OFF or emit anything else;
      - it is a BODY module button, not a control message. Nothing about propulsion, steering or
        braking is addressed by 0x455;
      - the attempt cap below is unchanged and now matters MORE, not less.
    Accepted cost, stated plainly: the duplicate-ID exposure documented in PRO-POWER-CAN-CONTROL.md
    (we transmit 0x455 while its real sender also transmits it) no longer happens only in Park with
    openpilot idle. Measured on the truck 2026-09-08 it produced ZERO new CAN errors across a 6-frame
    press -- bus-0 totalErrorCnt stayed at 1 over 2,412 pandaStates samples -- but that was one
    observation of a rare event, in Park. It is now also possible while stopped and engaged.
  * ABORT THE MOMENT THE TRUCK MOVES. Unchanged, and it is what makes the above bounded: a press in
    progress stops transmitting immediately if standstill drops.
  * AT MOST MAX_ATTEMPTS PRESSES PER `card` PROCESS, and only while the state reads OFF. The truck forgets the setting once per
    cycle, so re-arming it is a once-per-cycle job. An unbounded retry loop mashing a body button is
    exactly the failure this bound exists to make impossible.
    Known and accepted limit (Gemini review 2026-09-07): the bound is per ARMER INSTANCE, and `card`
    constructs a fresh one when it restarts, so a `card` crash inside one ignition cycle does allow a
    second round of attempts. Deliberately not defended against with a param or a file: the first
    thing a fresh armer checks is `ppo_on`, so a successful earlier press means the new instance does
    nothing at all; a `card` crash-loop is a far larger problem than a repeated body-button press;
    and persisting state across restarts to protect a comfort feature is more machinery than the risk
    justifies.
  * VERIFIED, NOT ASSUMED. After each press we watch the STATE bits. If they do not flip, we say so
    loudly and stop. We are spoofing an undocumented frame whose real sender keeps transmitting at
    10 Hz alongside us; whether the receiving module honours our copy is a question about the truck
    that could not be answered from a desk, and this is how it gets answered on the road.
  * NEVER TURNS IT OFF. The only payload this module can emit is PPO_ON, pinned in the panda in C.
    There is no path in this module, or reachable from it, that can request OFF.
  * ...BUT IT DOES NOW TURN IT BACK ON, every PPO_REARM_S. ⚠️ THIS OVERTURNS THE SECOND HALF OF THE
    RULE ABOVE, which used to read "if they turn it OFF deliberately after we have acted, we do not
    fight them — we are done for the cycle." That is no longer true and the driver asked for exactly
    that: "please rearm every 15 min".

    Consequence, stated plainly because it is a real loss: THE DRIVER CAN NO LONGER TURN THIS OFF
    AND HAVE IT STAY OFF while driving. Switch it off and it comes back within 15 minutes. There is
    currently no opt-out short of the whole feature. If that becomes annoying the answer is a
    settings toggle, not a shorter timer.

    Why it is nonetheless right here: the thing being fought is not the driver, it is the TRUCK.
    Measured 2026-09-09 (see PPO_REARM_S), a request-OFF press arrived from the truck side 19.6 s
    after we armed it. Against that, one-shot-per-ignition loses every time.

    Bounded: at most PPO_MAX_ATTEMPTS presses per re-arm window, so at most 3 per 15 min = 12/hour
    worst case, each a 0.5 s pulse on a body-module button.
"""
from dataclasses import dataclass

# The undocumented HMI/button frame. Not in the DBC (the two STATE bits are, as
# PnwProPwrOnbd_B_Stat / _B_Stat2), so the payload is built by hand from the captured bytes.
PPO_ADDR = 0x455
PPO_IDLE = b"\x00\x80\x00\x00\x00\x00\x00\x00"   # what the real sender transmits at rest, 10 Hz
PPO_PRESS_ON = b"\x00\x40\x00\x00\x00\x00\x00\x00"   # bit7 clear = "pressing", bit6 set = "-> ON"

# Cadence and duration copied from the DRIVER's own presses, not invented: the real pulse measured
# 0.5 s, and the frame's native rate is 10 Hz.
PPO_PRESS_S = 0.5
PPO_RATE_HZ = 10.0

# Let the bus settle before trusting a state read. At ignition the modules come up staggered and an
# early frame can carry a default rather than the retained setting.
PPO_SETTLE_S = 8.0
# How long to watch the state bits after a press before calling it failed.
PPO_VERIFY_S = 3.0
# Total presses per ignition cycle. Three is enough to ride out one dropped frame and small enough
# that a misunderstanding of the protocol cannot become a body-button machine gun.
PPO_MAX_ATTEMPTS = 3
# pporearm2pnw (2026-09-09, driver: "please rearm every 15 min"). How long after finishing -- armed,
# or out of attempts -- before the armer wakes up and looks again.
#
# THIS REVERSES THE "we do not fight the driver" CONTRACT, deliberately and at the driver's
# instruction. Read the envelope note below before changing it.
#
# What produced the request, measured from raw CAN 2026-09-09:
#     08:13:08.110  STATE  0 -> 1        our press armed it
#     08:13:27.699  BUTTON byte1=0x00    a REQUEST-OFF press, 19.6 s later, six frames
#     08:13:28.239  STATE  1 -> 0        off again
# byte1 0x00 is "request OFF", which the panda physically forbids us from sending (the payload is
# pinned in C to 0x40), so that press came from the truck side. Something clears the setting shortly
# after it is set, and a one-shot-per-ignition armer can never win against that.
PPO_REARM_S = 15 * 60.0


@dataclass
class PpoInputs:
  """One tick of everything the decision is allowed to see. Plain python only."""
  now: float           # monotonic seconds
  standstill: bool     # the outer bound: nothing happens unless the truck is stopped
  # ppostandstill2pnw: DIAGNOSTIC ONLY since 2026-09-09 -- `parked` is reported in the log line but
  # no longer gates the press (see THE ENVELOPE). Kept on the input so a drive log can tell a Park
  # arm from a red-light arm.
  parked: bool         # gearShifter == park
  state_valid: bool    # both state signals have been seen this drive
  ppo_on: bool         # the latched state: True = Pro Power is armed for engine-off
  enabled: bool        # feature master (currently always True; a place for a future opt-out)
  # CS.out.canValid -- see the outer gate. Deliberately has NO default: a caller that has not
  # thought about bus liveness should not compile.
  can_valid: bool


class ProPowerArmer:
  """Re-arm Pro Power Onboard once per ignition, verified, bounded, and never silently."""

  # phases, for telemetry and for reading the code
  IDLE, PRESSING, VERIFYING, DONE, FAILED = "idle", "pressing", "verifying", "done", "failed"

  def __init__(self):
    self.phase = self.IDLE
    self.attempts = 0
    self._t0 = None          # first tick we saw, for the settle window
    self._press_until = 0.0
    self._verify_until = 0.0
    # pporearm2pnw: when to wake up and look again. None = never (the feature is genuinely inert, so
    # retrying is pure noise); a float = re-arm at that monotonic time.
    self._rearm_at = None
    self.note = ""           # last line said, for tests and introspection
    self._pending_note = ""  # drained by the caller via take_note(); see WHY below
    self._said_unread = False
    self._said_blocked = False

  def _say(self, note: str) -> None:
    """Queue a line for the caller to log.

    WHY THIS EXISTS RATHER THAN JUST SETTING self.note: the caller used to log only when the PHASE
    changed, so any condition that keeps the armer parked in IDLE was, by construction, silent --
    and that is exactly the failure that hid the dead Park gate for a whole day (Fable review
    2026-09-07). A note now reaches the log whether or not the phase moved.
    """
    self.note = note
    self._pending_note = note

  def take_note(self) -> str:
    """Pop the queued line, or "" if there is nothing to say."""
    note, self._pending_note = self._pending_note, ""
    return note

  def update(self, i: PpoInputs) -> bytes | None:
    """Returns the payload to transmit on PPO_ADDR this tick, or None. NEVER raises."""
    if self._t0 is None:
      self._t0 = i.now

    if not i.enabled:
      return None

    if self.phase in (self.DONE, self.FAILED):
      # pporearm2pnw: wake up and look again, rather than being finished for the ignition cycle.
      if self._rearm_at is None or i.now < self._rearm_at:
        return None
      self.phase = self.IDLE
      self.attempts = 0                 # a fresh budget per window -- see the envelope's bound
      self._rearm_at = None
      # let the one-shot diagnostics speak again in the new window: a feature that goes quiet after
      # its first cycle is the failure mode this file already paid for once.
      self._said_blocked = False
      self._said_unread = False
      self._say(f"re-arm window: {PPO_REARM_S / 60:.0f} min elapsed, checking Pro Power again")

    # Outer bound. A moving truck never sees a frame from this module, in any phase -- including
    # mid-press: if the driver pulls away while we are pressing, we stop transmitting immediately.
    # Fable review 2026-09-07 (L2): `card` calls apply() whenever carControl is alive, regardless of
    # bus health, and `cp.vl` KEEPS ITS LAST VALUES when messages go stale -- while `ppo_valid` is a
    # forever-latch. So on a quiet powertrain bus (modules asleep mid-charge, say) a fresh armer
    # would read stale standstill/parked/state_valid, spend all three presses into silence, and then
    # report a confident failure. Require a live bus before believing any of it.
    if not i.can_valid:
      return None

    # ppostandstill2pnw: STANDSTILL ONLY. `parked` is still carried on PpoInputs and still reported
    # in the log line below, because "where was it standing when it pressed" is exactly what a drive
    # log needs to answer -- but it no longer gates.
    if not i.standstill:
      if self.phase == self.PRESSING:
        self.phase = self.IDLE
        self._say("aborted: truck moved mid-press")
      elif not self._said_blocked and (i.now - self._t0) > PPO_SETTLE_S * 2:
        # Rule 2. "Held at the gate" and "nothing to do" used to look identical from the log, and a
        # dead `parked` test therefore read as a working feature for a whole ignition cycle. Name
        # the gate that is holding, once.
        self._said_blocked = True
        self._say(f"idle after {PPO_SETTLE_S * 2:.0f}s -- held because the truck is not at a standstill")
      return None

    if self.phase == self.PRESSING:
      if i.now < self._press_until:
        return PPO_PRESS_ON
      self.phase = self.VERIFYING
      self._verify_until = i.now + PPO_VERIFY_S
      return None

    if self.phase == self.VERIFYING:
      if i.ppo_on:
        self.phase = self.DONE
        self._rearm_at = i.now + PPO_REARM_S      # pporearm2pnw: look again in 15 min
        self._say(f"Pro Power armed after {self.attempts} press(es)")
      elif i.now >= self._verify_until:
        if self.attempts >= PPO_MAX_ATTEMPTS:
          self.phase = self.FAILED
          self._rearm_at = i.now + PPO_REARM_S    # pporearm2pnw: the module may honour a later press
          self._say(" ".join([
            f"gave up after {self.attempts} presses -- the state bit never flipped.",
            "Either the panda is dropping the TX (0x455 must be in the Ford allowlist and the",
            "panda flashed) or the module ignores a spoofed press.",
          ]))
        else:
          self.phase = self.IDLE
          self._say(f"press {self.attempts} not confirmed in {PPO_VERIFY_S:.0f}s -- retrying")
      return None

    # IDLE: decide whether there is anything to do at all.
    if not i.state_valid:
      # Rule 2: say so ONCE rather than sitting idle forever in silence. A Ford that never sends
      # these messages, or a Lightning whose modules never reported, is otherwise indistinguishable
      # from "the feature is working and had nothing to do".
      if not self._said_unread and (i.now - self._t0) > PPO_SETTLE_S * 2:
        self._said_unread = True
        self._say("Pro Power state never reported on the bus -- feature inert this drive")
        self.phase = self.FAILED
        # pporearm2pnw: deliberately NO re-arm here. The other terminal states are "it might work
        # next time"; this one is "the truck never told us the state at all", and waking every
        # 15 minutes to re-discover that is noise in the log with nothing to act on.
        self._rearm_at = None
      return None
    if i.now - self._t0 < PPO_SETTLE_S:
      return None                                  # too early to trust the read
    if i.ppo_on:
      self.phase = self.DONE
      self._rearm_at = i.now + PPO_REARM_S        # pporearm2pnw: it can be cleared later; look again
      self._say("already armed; nothing to do")
      return None

    # ppostandstill2pnw (Fable review 2026-09-09): the attempt cap MUST be enforced here, not only in
    # VERIFYING. A press aborted by movement never reaches VERIFYING, so before this the counter
    # climbed and the cap never fired. Under the old Park gate that was rare and self-limiting -- you
    # had to leave Park mid-press, and could not press again without returning to Park. Under the
    # standstill-only gate every sub-0.5 s stop in stop-and-go traffic is a fresh press: simulated,
    # 60 presses / 180 frames in 60 s of creep, phase never reaching FAILED. That is exactly the
    # "body-button machine gun" the bound below exists to make impossible, and the envelope note
    # claiming the cap "matters MORE, not less" was not true until this existed.
    # Accepted trade-off: three creep-aborts before a real stop now burn the cycle. Correct priority
    # for a comfort feature -- a spent budget is a missed convenience, an unbounded one is a body
    # module being hammered.
    if self.attempts >= PPO_MAX_ATTEMPTS:
      self.phase = self.FAILED
      self._rearm_at = i.now + PPO_REARM_S        # pporearm2pnw: creep-aborts should not end the drive
      self._say(f"gave up: {self.attempts} presses started, none confirmed -- retrying in {PPO_REARM_S / 60:.0f} min")
      return None

    self.attempts += 1
    self.phase = self.PRESSING
    self._press_until = i.now + PPO_PRESS_S
    # ppostandstill2pnw: name the gear. A press in Drive is now legitimate, so a drive log must be
    # able to distinguish "armed while parked up" from "armed at a red light" without inference.
    where = "in Park" if i.parked else "at a standstill, NOT in Park"
    self._say(f"pressing to arm Pro Power {where} (attempt {self.attempts})")
    return PPO_PRESS_ON

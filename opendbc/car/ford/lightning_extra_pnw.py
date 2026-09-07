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
  * STANDSTILL **AND IN PARK**. Pro Power is a parked-truck function (tools, a fridge, a campsite).
    Standstill alone is not enough: `card` restarts on crash, so a fresh armer can appear mid-drive
    and would otherwise press at the next red light, in Drive. Park is the honest expression of
    "parked" (CLAUDE.md rule 3: IsOnroad is not "being driven", the gear is). The panda enforces the
    standstill half independently, in C, from the same ABS signal.
  * ONCE PER IGNITION, and at most MAX_ATTEMPTS presses. The truck forgets the setting once per
    cycle, so re-arming it is a once-per-cycle job. An unbounded retry loop mashing a body button is
    exactly the failure this bound exists to make impossible.
  * VERIFIED, NOT ASSUMED. After each press we watch the STATE bits. If they do not flip, we say so
    loudly and stop. We are spoofing an undocumented frame whose real sender keeps transmitting at
    10 Hz alongside us; whether the receiving module honours our copy is a question about the truck
    that could not be answered from a desk, and this is how it gets answered on the road.
  * NEVER TURNS IT OFF. The only payload this module can emit is PPO_ON. If the driver has it ON
    already we do nothing; if they turn it OFF deliberately after we have acted, we do not fight
    them — we are done for the cycle.
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


@dataclass
class PpoInputs:
  """One tick of everything the decision is allowed to see. Plain python only."""
  now: float           # monotonic seconds
  standstill: bool     # the outer bound: nothing happens unless the truck is stopped
  parked: bool         # gearShifter == park; see the envelope note about card restarts mid-drive
  state_valid: bool    # both state signals have been seen this drive
  ppo_on: bool         # the latched state: True = Pro Power is armed for engine-off
  enabled: bool        # feature master (currently always True; a place for a future opt-out)


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
    self.note = ""           # one-line reason, for the caller to log
    self._said_unread = False

  def update(self, i: PpoInputs) -> bytes | None:
    """Returns the payload to transmit on PPO_ADDR this tick, or None. NEVER raises."""
    if self._t0 is None:
      self._t0 = i.now

    if not i.enabled or self.phase in (self.DONE, self.FAILED):
      return None

    # Outer bound. A moving truck never sees a frame from this module, in any phase -- including
    # mid-press: if the driver pulls away while we are pressing, we stop transmitting immediately.
    if not (i.standstill and i.parked):
      if self.phase == self.PRESSING:
        self.phase = self.IDLE
        self.note = "aborted: truck moved or left Park mid-press"
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
        self.note = f"Pro Power armed after {self.attempts} press(es)"
      elif i.now >= self._verify_until:
        if self.attempts >= PPO_MAX_ATTEMPTS:
          self.phase = self.FAILED
          self.note = " ".join([
            f"gave up after {self.attempts} presses -- the state bit never flipped.",
            "Either the panda is dropping the TX (0x455 must be in the Ford allowlist and the",
            "panda flashed) or the module ignores a spoofed press.",
          ])
        else:
          self.phase = self.IDLE
      return None

    # IDLE: decide whether there is anything to do at all.
    if not i.state_valid:
      # Rule 2: say so ONCE rather than sitting idle forever in silence. A Ford that never sends
      # these messages, or a Lightning whose modules never reported, is otherwise indistinguishable
      # from "the feature is working and had nothing to do".
      if not self._said_unread and (i.now - self._t0) > PPO_SETTLE_S * 2:
        self._said_unread = True
        self.note = "Pro Power state never reported on the bus -- feature inert this drive"
        self.phase = self.FAILED
      return None
    if i.now - self._t0 < PPO_SETTLE_S:
      return None                                  # too early to trust the read
    if i.ppo_on:
      self.phase = self.DONE
      self.note = "already armed; nothing to do"
      return None

    self.attempts += 1
    self.phase = self.PRESSING
    self._press_until = i.now + PPO_PRESS_S
    self.note = f"pressing to arm Pro Power (attempt {self.attempts})"
    return PPO_PRESS_ON

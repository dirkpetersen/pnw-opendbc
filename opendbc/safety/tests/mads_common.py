"""mads2pnw -- shared assertions for the parallel lateral-authority flag.

The load-bearing claim these tests exist to hold down:

    controls_allowed_lateral is a PARALLEL authority added BESIDE the brake
    disengage, not a weakening of it. `controls_allowed` is still cleared on
    the rising edge of the brake pedal in EVERY configuration -- MADS off,
    MADS on/REMAIN_ACTIVE, MADS on/PAUSE, MADS on/DISENGAGE.

Mix into a car's safety test. The car test must provide:
  _mads_lateral_tx()   -- send one benign lateral command, return bool tx-allowed
  _mads_engage()       -- put the car in the normal engaged state
  _mads_brake(pressed) -- rx a brake-pedal message
  _mads_disengage_no_brake() -- rx a non-brake disengage (cruise cancel)
"""

import abc

from opendbc.safety import ALTERNATIVE_EXPERIENCE


class MadsSteeringModeOnBrake:
  """Mirrors sunnypilot's MadsSteeringModeOnBrake (openpilot side: MadsLateralOnBrake toggle)."""
  REMAIN_ACTIVE = 0
  PAUSE = 1
  DISENGAGE = 2


class MadsLateralOnBrakeTestBase(abc.ABC):
  @abc.abstractmethod
  def _mads_lateral_tx(self) -> bool:
    ...

  @abc.abstractmethod
  def _mads_engage(self) -> None:
    ...

  @abc.abstractmethod
  def _mads_brake(self, pressed: bool) -> None:
    ...

  @abc.abstractmethod
  def _mads_disengage_no_brake(self) -> None:
    """rx whatever makes this car drop controls_allowed WITHOUT the brake (cruise cancel)."""

  # --- helpers -------------------------------------------------------------

  def _mads_apply(self, enabled: bool, steering_mode: int = MadsSteeringModeOnBrake.REMAIN_ACTIVE) -> None:
    """Push the alternative_experience bits, then re-init the safety mode -- exactly the order
    real firmware sees (USB 0xdf, then USB 0xdc set-safety-mode)."""
    self.safety.set_mads_params(enabled,
                                steering_mode == MadsSteeringModeOnBrake.DISENGAGE,
                                steering_mode == MadsSteeringModeOnBrake.PAUSE)
    self._mads_reinit_safety_mode()

  def _mads_reinit_safety_mode(self) -> None:
    """Re-run set_safety_hooks with whatever mode/param this test class set up, so the mixin
    works for every subclass (including ones that override the safety param)."""
    self.safety.set_safety_hooks(self.safety.get_current_safety_mode(),
                                 self.safety.get_current_safety_param())

  # --- the core claim ------------------------------------------------------

  def test_mads_brake_always_clears_controls_allowed(self):
    """THE load-bearing claim. In every MADS configuration, including fully off, a brake press
    still clears controls_allowed. The brake check itself is never touched."""
    for enabled in (False, True):
      for mode in (MadsSteeringModeOnBrake.REMAIN_ACTIVE, MadsSteeringModeOnBrake.PAUSE,
                   MadsSteeringModeOnBrake.DISENGAGE):
        with self.subTest(mads_enabled=enabled, steering_mode=mode):
          self._mads_apply(enabled, mode)
          self._mads_engage()
          self.assertTrue(self.safety.get_controls_allowed())
          self._mads_brake(True)
          self.assertFalse(self.safety.get_controls_allowed(),
                           "brake press must always clear controls_allowed")

  def test_mads_off_is_todays_behaviour(self):
    """MADS off (the shipping default): a brake press blocks lateral, exactly as before."""
    self._mads_apply(False)
    self._mads_engage()
    self.assertTrue(self._mads_lateral_tx())
    self._mads_brake(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._mads_lateral_tx(), "MADS off: lateral must still die on brake")

  def test_mads_remain_active_lateral_survives_brake(self):
    """THE FEATURE: MADS on, REMAIN_ACTIVE -- brake takes longitudinal, leaves lateral."""
    self._mads_apply(True, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
    self._mads_engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._mads_brake(True)
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._mads_lateral_tx(), "MADS REMAIN_ACTIVE: lateral must survive the brake")

  def test_mads_disengage_mode_kills_lateral_on_brake(self):
    self._mads_apply(True, MadsSteeringModeOnBrake.DISENGAGE)
    self._mads_engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._mads_brake(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._mads_lateral_tx())

  def test_mads_pause_mode_pauses_then_resumes(self):
    self._mads_apply(True, MadsSteeringModeOnBrake.PAUSE)
    self._mads_engage()
    self._mads_brake(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._mads_lateral_tx())
    # releasing the brake restores lateral (brake was the only disengage reason)
    self._mads_brake(False)
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_mads_cancel_drops_lateral(self):
    """pnw addition beyond upstream: openpilot losing controls for a NON-brake reason (a CANCEL
    press, a fault, anything) also ends lateral authority. Without this the panda would keep
    permitting lateral indefinitely while the driver believes openpilot is off -- upstream can
    leave the latch standing because it has an openpilot-side MADS state machine and the
    heartbeat_engaged_mads watchdog; this tree has neither."""
    self._mads_apply(True, MadsSteeringModeOnBrake.REMAIN_ACTIVE)
    self._mads_engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._mads_disengage_no_brake()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral(),
                     "a non-brake disengage must take lateral authority with it")
    self.assertFalse(self._mads_lateral_tx())

  def test_mads_acc_main_never_engages_lateral(self):
    """The only ENGAGE source is openpilot's own controls_allowed rising edge. ACC main coming on
    (the upstream engage source we dropped) must not arm lateral by itself."""
    self._mads_apply(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    for _ in range(5):
      self.safety.set_acc_main_on(False)
      self.safety.set_acc_main_on(True)
      self._mads_brake(False)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  # --- the opt-in chain ----------------------------------------------------

  def test_mads_requires_the_alt_exp_bit(self):
    """Without ALT_EXP_ENABLE_MADS the state machine can never flip the global on, no matter how
    many engage edges it sees."""
    self.safety.set_alternative_experience(0)
    self._mads_reinit_safety_mode()
    self.assertFalse(self.safety.get_mads_system_enabled())
    for _ in range(5):
      self._mads_engage()
      self._mads_brake(True)
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_mads_bit_only_applied_at_safety_mode_init(self):
    """alternative_experience is latched into the MADS state machine by set_safety_hooks, not
    read live. This is why the openpilot toggle needs a reboot (documented in MADS2PNW.md)."""
    self._mads_apply(False)
    self.assertFalse(self.safety.get_mads_system_enabled())
    self.safety.set_alternative_experience(ALTERNATIVE_EXPERIENCE.ENABLE_MADS)
    self.assertFalse(self.safety.get_mads_system_enabled(), "must not take effect until re-init")
    self._mads_reinit_safety_mode()
    self.assertTrue(self.safety.get_mads_system_enabled())

  def test_mads_lateral_cleared_on_safety_mode_change(self):
    """Any safety-mode change -- including the panda's drop to SILENT when the heartbeat is
    lost -- clears the latched lateral authority."""
    self._mads_apply(True)
    self._mads_engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._mads_reinit_safety_mode()
    self.assertFalse(self.safety.get_controls_allowed_lateral())

  # --- fault paths ---------------------------------------------------------

  def test_mads_lateral_cleared_on_lagging_rx(self):
    """safety_tick's lag detector drops lateral authority as well as controls_allowed."""
    self._mads_apply(True)
    self._mads_engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.safety.set_timer(2_000_000)  # 2s, past every rx_check's lag threshold
    self.safety.safety_tick_current_safety_config()
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())

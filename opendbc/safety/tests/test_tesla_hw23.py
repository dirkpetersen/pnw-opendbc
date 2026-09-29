#!/usr/bin/env python3
import unittest
import numpy as np

from opendbc.car.lateral import get_max_angle_delta_vm, get_max_angle_vm
from opendbc.car.tesla.values import CarControllerParams, TeslaSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.car.vehicle_model import VehicleModel
from opendbc.can import CANDefine
from opendbc.safety.tests.libsafety import libsafety_py
import opendbc.safety.tests.common as common
from opendbc.safety.tests.common import CANPackerSafety, away_round, round_speed
from opendbc.safety.tests.mads_common import MadsLateralOnBrakeTestBase, MadsBitsRefusedTestBase, MadsSteeringModeOnBrake

MSG_DAS_steeringControl = 0x488
MSG_APS_eacMonitor = 0x27d
MSG_DAS_Control_HW23 = 0x2bf


def round_angle(apply_angle, can_offset=0):
  apply_angle_can = (apply_angle + 1638.35) / 0.1 + can_offset
  # 0.49999_ == 0.5
  rnd_offset = 1e-5 if apply_angle >= 0 else -1e-5
  return away_round(apply_angle_can + rnd_offset) * 0.1 - 1638.35


class TeslaLegacyLateralBase(common.CarSafetyTest, common.AngleSteeringSafetyTest):
  """Base class for Tesla Legacy lateral (steering) control tests"""

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Angle control limits
  STEER_ANGLE_MAX = 360  # deg
  DEG_TO_CAN = 10

  # Tesla uses get_max_angle_delta_vm and get_max_angle_vm for real lateral accel and jerk limits
  ANGLE_RATE_BP = None
  ANGLE_RATE_UP = None
  ANGLE_RATE_DOWN = None

  # Real time limits
  LATERAL_FREQUENCY = 50  # Hz

  cnt_epas = 0
  cnt_angle_cmd = 0
  packer: CANPackerSafety
  packer_chassis: CANPackerSafety
  packer_powertrain: CANPackerSafety

  def _get_steer_cmd_angle_max(self, speed):
    return get_max_angle_vm(max(speed, 1), self.VM, CarControllerParams)

  def setUp(self):
    from opendbc.car.tesla.interface import CarInterface
    self.VM = VehicleModel(CarInterface.get_non_essential_params("TESLA_MODEL_S_HW3"))

    # Tesla Legacy uses tesla_can DBC
    self.packer = CANPackerSafety("tesla_can")
    self.packer_chassis = CANPackerSafety("tesla_can")
    self.packer_powertrain = CANPackerSafety("tesla_powertrain")

    self.steer_control_types = {'NONE': 0, 'ANGLE_CONTROL': 1, 'LANE_KEEP_ASSIST': 2, 'EMERGENCY_LANE_KEEP': 3}

  def _angle_cmd_msg(self, angle: float, state: bool | int, increment_timer: bool = True, bus: int = 0):
    values = {"DAS_steeringAngleRequest": angle, "DAS_steeringControlType": state}
    if increment_timer:
      self.safety.set_timer(self.cnt_angle_cmd * int(1e6 / self.LATERAL_FREQUENCY))
      self.__class__.cnt_angle_cmd += 1
    return self.packer.make_can_msg_safety("DAS_steeringControl", bus, values)

  def _angle_meas_msg(self, angle: float, hands_on_level: int = 0, eac_status: int = 1, eac_error_code: int = 0):
    # Legacy uses EPAS_sysStatus message with EPAS_* signals (not EPAS3S_*)
    values = {
      "EPAS_internalSAS": angle,
      "EPAS_handsOnLevel": hands_on_level,
      "EPAS_eacStatus": eac_status,
      "EPAS_eacErrorCode": eac_error_code
    }
    return self.packer.make_can_msg_safety("EPAS_sysStatus", 0, values)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer_chassis.make_can_msg_safety("BrakeMessage", self.chassis_bus, values)

  def _speed_msg(self, speed):
    values = {"ESP_vehicleSpeed": speed * 3.6}  # Convert m/s to km/h
    return self.packer_chassis.make_can_msg_safety("ESP_B", self.chassis_bus, values)

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2, "DI_speedUnits": 1}  # 1 = KPH
    return self.packer_chassis.make_can_msg_safety("DI_state", self.chassis_bus, values)

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer_chassis.make_can_msg_safety("DI_torque1", self.chassis_bus, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0, "DI_speedUnits": 1}  # 1 = KPH
    return self.packer_chassis.make_can_msg_safety("DI_state", self.chassis_bus, values)

  def test_rx_hook(self):
    # Test angle command reception
    for i in range(5):
      msg = self._angle_cmd_msg(0, True, bus=2)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._rx(msg))
      self.assertTrue(self.safety.get_controls_allowed())

    # Test speed message reception
    for i in range(5):
      msg = self._speed_msg(0)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._rx(msg))
      self.assertTrue(self.safety.get_controls_allowed())

  def test_vehicle_speed_measurements(self):
    self._common_measurement_test(self._speed_msg, 0, 285 / 3.6, 1,
                                  self.safety.get_vehicle_speed_min, self.safety.get_vehicle_speed_max)

  def test_steering_wheel_disengage(self):
    for hands_on_level in range(4):
      for eac_status in range(8):
        for eac_error_code in range(16):
          self.safety.set_controls_allowed(True)

          should_disengage = hands_on_level >= 3 or (eac_status == 0 and eac_error_code == 9)
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=hands_on_level,
                                                        eac_status=eac_status, eac_error_code=eac_error_code)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertEqual(should_disengage, self.safety.get_steering_disengage_prev())

          # Should not recover
          self.assertTrue(self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0)))
          self.assertNotEqual(should_disengage, self.safety.get_controls_allowed())
          self.assertFalse(self.safety.get_steering_disengage_prev())

  def test_steering_control_type(self):
    self.safety.set_controls_allowed(True)
    for steer_control_type in range(4):
      should_tx = steer_control_type in (self.steer_control_types["NONE"],
                                         self.steer_control_types["ANGLE_CONTROL"])
      self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(0, state=steer_control_type)))

  def test_stock_lkas_passthrough(self):
    no_lkas_msg = self._angle_cmd_msg(0, state=False)
    no_lkas_msg_cam = self._angle_cmd_msg(0, state=True, bus=2)
    lkas_msg_cam = self._angle_cmd_msg(0, state=self.steer_control_types['LANE_KEEP_ASSIST'], bus=2)

    # stock system sends no LKAS -> block forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_lkas_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_lkas_msg_cam.addr))
    self.assertTrue(self._tx(no_lkas_msg))

    # stock system sends LKAS -> allow forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(lkas_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, lkas_msg_cam.addr))
    self.assertFalse(self._tx(no_lkas_msg))

  def test_angle_cmd_when_enabled(self):
    # We properly test lateral acceleration and jerk below
    pass

  #########################################################################
  # These tests are only relevant for longitudinal control (external panda).
  #########################################################################

  def test_prev_gas(self):
    pass

  def test_no_disengage_on_gas(self):
    pass

  #########################################################################
  #########################################################################

  def test_lateral_accel_limit(self):
    for speed in np.linspace(0, 40, 50):  # Reduced iterations for legacy tests
      speed = max(speed, 1)
      speed = round_speed(away_round(speed / 0.01 * 3.6) * 0.01 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)

        angle_unit_offset = -1 if sign == -1 else 0
        max_angle = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 1) * sign
        max_angle = np.clip(max_angle, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self.safety.set_desired_angle_last(round(max_angle * self.DEG_TO_CAN))

        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle, True)))

        # 1 unit above limit
        max_angle_raw = round_angle(get_max_angle_vm(speed, self.VM, CarControllerParams), angle_unit_offset + 2) * sign
        max_angle = np.clip(max_angle_raw, -self.STEER_ANGLE_MAX, self.STEER_ANGLE_MAX)
        self._tx(self._angle_cmd_msg(max_angle, True))

        # at low speeds max angle is above 360, so adding 1 has no effect
        should_tx = abs(max_angle_raw) >= self.STEER_ANGLE_MAX
        self.assertEqual(should_tx, self._tx(self._angle_cmd_msg(max_angle, True)))

  def test_lateral_jerk_limit(self):
    for speed in np.linspace(0, 40, 50):  # Reduced iterations for legacy tests
      speed = max(speed, 1)
      speed = round_speed(away_round(speed / 0.01 * 3.6) * 0.01 / 3.6)
      for sign in (-1, 1):
        self.safety.set_controls_allowed(True)
        self._reset_speed_measurement(speed + 1)
        self._tx(self._angle_cmd_msg(0, True))

        angle_unit_offset = 1 if sign == -1 else 0

        # Stay within limits - Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams),
                                      angle_unit_offset) * sign
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))

        # Inject too high rates - Up
        max_angle_delta = round_angle(get_max_angle_delta_vm(speed, self.VM, CarControllerParams),
                                      angle_unit_offset + 1) * sign
        self.assertFalse(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Don't change
        self.safety.set_desired_angle_last(round(max_angle_delta * self.DEG_TO_CAN))
        self.assertTrue(self._tx(self._angle_cmd_msg(max_angle_delta, True)))

        # Down
        self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

        # Recover
        self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))


class TeslaLegacyLongitudinalBase(common.CarSafetyTest, common.LongitudinalAccelSafetyTest):
  """Base class for Tesla Legacy longitudinal (acceleration) control tests"""

  STANDSTILL_THRESHOLD = 0.1
  GAS_PRESSED_THRESHOLD = 3

  # Long control limits
  MAX_ACCEL = 2.0
  MIN_ACCEL = -3.48
  INACTIVE_ACCEL = 0.0

  def setUp(self):
    # Tesla Legacy uses tesla_can DBC
    self.packer = CANPackerSafety("tesla_powertrain")

  def _vehicle_moving_msg(self, speed: float):
    values = {"DI_cruiseState": 3 if speed <= self.STANDSTILL_THRESHOLD else 2, "DI_speedUnits": 1}  # 1 = KPH
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _user_brake_msg(self, brake):
    values = {"driverBrakeStatus": 2 if brake else 1}
    return self.packer.make_can_msg_safety("BrakeMessage", 0, values)

  def _user_gas_msg(self, gas):
    values = {"DI_pedalPos": gas}
    return self.packer.make_can_msg_safety("DI_torque1", 0, values)

  def _pcm_status_msg(self, enable):
    values = {"DI_cruiseState": 2 if enable else 0, "DI_speedUnits": 1}  # 1 = KPH
    return self.packer.make_can_msg_safety("DI_state", 0, values)

  def _long_control_msg(self, set_speed, acc_state=0, jerk_limits=(0, 0), accel_limits=(0, 0), aeb_event=0, bus=0):
    values = {
      "DAS_setSpeed": set_speed,
      "DAS_accState": acc_state,
      "DAS_aebEvent": aeb_event,
      "DAS_jerkMin": jerk_limits[0],
      "DAS_jerkMax": jerk_limits[1],
      "DAS_accelMin": accel_limits[0],
      "DAS_accelMax": accel_limits[1],
    }
    return self.packer.make_can_msg_safety("DAS_control", bus, values)

  def _accel_msg(self, accel: float):
    return self._long_control_msg(10, accel_limits=(accel, max(accel, 0)))

  def test_rx_hook(self):
    # Test longitudinal command reception
    for i in range(5):
      msg = self._long_control_msg(0, bus=2)
      self.safety.set_controls_allowed(True)
      self.assertTrue(self._rx(msg))
      self.assertTrue(self.safety.get_controls_allowed())

  def test_no_aeb(self):
    for aeb_event in range(4):
      self.assertEqual(self._tx(self._long_control_msg(10, aeb_event=aeb_event)), aeb_event == 0)

  def test_stock_aeb_passthrough(self):
    no_aeb_msg = self._long_control_msg(10, aeb_event=0)
    no_aeb_msg_cam = self._long_control_msg(10, aeb_event=0, bus=2)
    aeb_msg_cam = self._long_control_msg(10, aeb_event=1, bus=2)

    # stock system sends no AEB -> block forwarding, and OP is allowed to TX
    self.assertEqual(1, self._rx(no_aeb_msg_cam))
    self.assertEqual(-1, self.safety.safety_fwd_hook(2, no_aeb_msg_cam.addr))
    self.assertTrue(self._tx(no_aeb_msg))

    # stock system sends AEB -> allow forwarding, and OP is not allowed to TX
    self.assertEqual(1, self._rx(aeb_msg_cam))
    self.assertEqual(0, self.safety.safety_fwd_hook(2, aeb_msg_cam.addr))
    self.assertFalse(self._tx(no_aeb_msg))

  def test_prevent_reverse(self):
    self.safety.set_controls_allowed(True)

    # accel_min and accel_max are positive
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(1.1, 0.8))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(1.1, 0.8))))

    # accel_min and accel_max are both zero
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(0, 0))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, 0))))

    # accel_min and accel_max have opposing signs
    self.assertTrue(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-0.8, 1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0.8, -1.3))))
    self.assertTrue(self._tx(self._long_control_msg(set_speed=0, accel_limits=(0, -1.3))))

    # accel_min and accel_max are negative
    self.assertFalse(self._tx(self._long_control_msg(set_speed=10, accel_limits=(-1.1, -0.6))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.6, -1.1))))
    self.assertFalse(self._tx(self._long_control_msg(set_speed=0, accel_limits=(-0.1, -0.1))))


# Lateral control tests (HW2 and HW3 non-external panda)
class TestTeslaHW2Safety(TeslaLegacyLateralBase, MadsBitsRefusedTestBase):
  chassis_bus = 0

  def test_mads_hw2_lateral_still_dies_on_brake(self):
    """The pre-teslamads2pnw behaviour, pinned for the config that still refuses the bits."""
    self._mads_push_bits_and_reinit()
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))
    self.assertTrue(self._tx(self._angle_cmd_msg(0, True)))
    self._rx(self._user_brake_msg(True))
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._tx(self._angle_cmd_msg(0, True)))

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
    self.TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW2))
    self.safety.init_tests()


class TestTeslaHW3Safety(TeslaLegacyLateralBase, MadsLateralOnBrakeTestBase):
  """The Raven's INTERNAL panda: the one teslaLegacy config that accepts the MADS bits
  (teslamads2pnw). The generic lateral-survives-brake claims come from MadsLateralOnBrakeTestBase; the
  Tesla-specific wiring (DI_cruiseState -> acc_main_on, steering_disengage, real frame order) is below."""
  chassis_bus = 1  # HW3 uses bus 1 for chassis

  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_steeringControl, MSG_APS_eacMonitor)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_steeringControl, MSG_APS_eacMonitor]}
    self.TX_MSGS = [[MSG_DAS_steeringControl, 0], [MSG_APS_eacMonitor, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy, int(TeslaSafetyFlags.FLAG_HW3))
    self.safety.init_tests()

    self.packer = CANPackerSafety("tesla_raven_party")

  # --- MadsLateralOnBrakeTestBase hooks ------------------------------------

  def _mads_lateral_tx(self) -> bool:
    # a neutral angle command: inside every limit, so only the lateral authority gate can block it
    return self._tx(self._angle_cmd_msg(0, True))

  def _mads_engage(self) -> None:
    self._rx(self._pcm_status_msg(False))
    self._rx(self._pcm_status_msg(True))

  def _mads_brake(self, pressed: bool) -> None:
    # BrakeMessage alone. On the Raven a brake press drops DI_cruiseState ENABLED -> STANDBY (never OFF),
    # which is a separate 0x368 frame; the tests that care send it explicitly (_di_state).
    self._rx(self._user_brake_msg(pressed))

  def _mads_disengage_no_brake(self) -> None:
    # stalk cancel / any non-brake cruise drop: ENABLED -> STANDBY, main still on
    self._rx(self._di_state(1))

  def _di_state(self, cruise_state: int):
    values = {"DI_cruiseState": cruise_state, "DI_speedUnits": 1}
    return self.packer_chassis.make_can_msg_safety("DI_state", self.chassis_bus, values)

  # --- Tesla-specific --------------------------------------------------------

  def test_mads_bits_accepted_on_internal_hw3(self):
    self._mads_apply(True)
    self.assertTrue(self.safety.get_mads_system_enabled())

  def test_mads_tesla_measured_frame_order_survives_brake(self):
    """The order measured on the Raven (drives/2026-09-28/tesla-brake-census): the brake frame is logged
    19-51 ms BEFORE the DI_state frame that drops cruise to STANDBY. The brake is already `braking` when
    controls_allowed falls, so the plain survive-a-braked-falling-edge path holds it -- no re-latch window."""
    self._mads_apply(True)
    self._mads_engage()
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx(self._user_brake_msg(True))
    self._rx(self._di_state(1))                 # ENABLED -> STANDBY, ~40 ms later
    for _ in range(20):                          # ... and the stream continues, brake held then released
      self._rx(self._di_state(1))
    self._rx(self._user_brake_msg(False))
    for _ in range(20):
      self._rx(self._di_state(1))
      self.assertFalse(self.safety.get_controls_allowed(), "brake takes longitudinal authority for good")
      self.assertTrue(self.safety.get_controls_allowed_lateral())
    self.assertTrue(self._mads_lateral_tx())

  def test_mads_tesla_standby_is_main_on_off_and_fault_revoke(self):
    """acc_main_on from DI_cruiseState: STANDBY(1) is main ON (a brake or a stalk cancel lands there and must
    not revoke); OFF(0) and FAULT(5) are main OFF and revoke latched lateral."""
    for state, revoked in ((1, False), (0, True), (5, True)):
      with self.subTest(di_cruise_state=state):
        self._mads_apply(True)
        self._mads_engage()
        self._rx(self._user_brake_msg(True))
        self.assertTrue(self.safety.get_controls_allowed_lateral())
        self._rx(self._di_state(state))
        self.assertEqual(state != 0 and state != 5, self.safety.get_acc_main_on())
        self.assertEqual(not revoked, self.safety.get_controls_allowed_lateral())
        self.assertEqual(not revoked, self._mads_lateral_tx())

  def test_mads_tesla_stalk_cancel_without_brake_drops_lateral(self):
    """A cancel with the brake up (stalk push -> STANDBY) is an OP_DISENGAGE, not a brake: lateral ends."""
    self._mads_apply(True)
    self._mads_engage()
    self._rx(self._di_state(1))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertFalse(self._mads_lateral_tx())

  def test_mads_tesla_stalk_cancel_while_lateral_only_changes_nothing(self):
    """KNOWN LIMIT, pinned on purpose: after a brake the cruise state is already STANDBY, so a stalk cancel
    there is STANDBY -> STANDBY (no edge) and does NOT end lateral. The driver's exits are a firm steer, the
    heartbeat, main OFF/FAULT, and openpilot's own events. If this test starts failing, the behaviour changed."""
    self._mads_apply(True)
    self._mads_engage()
    self._rx(self._user_brake_msg(True))
    self._rx(self._di_state(1))
    self._rx(self._user_brake_msg(False))
    for _ in range(5):
      self._rx(self._di_state(1))
    self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_mads_tesla_firm_steer_kills_lateral(self):
    """steering_disengage (handsOnLevel >= 3, or EAC not-ok with error 9) revokes lateral, both while fully
    engaged and while lateral-only, and the tx dies with it."""
    for hands_on, eac_status, eac_err in ((3, 1, 0), (0, 0, 9)):
      for lateral_only in (False, True):
        with self.subTest(hands_on=hands_on, eac_err=eac_err, lateral_only=lateral_only):
          self._mads_apply(True)
          self._mads_engage()
          if lateral_only:
            self._rx(self._user_brake_msg(True))
            self._rx(self._di_state(1))
          self.assertTrue(self.safety.get_controls_allowed_lateral())
          self._rx(self._angle_meas_msg(0, hands_on_level=hands_on, eac_status=eac_status, eac_error_code=eac_err))
          self.assertFalse(self.safety.get_controls_allowed_lateral())
          self.assertFalse(self.safety.get_controls_allowed())
          self.assertFalse(self._mads_lateral_tx())
          # releasing the wheel does not bring it back
          self._rx(self._angle_meas_msg(0, hands_on_level=0, eac_status=1, eac_error_code=0))
          self.assertFalse(self.safety.get_controls_allowed_lateral())

  def test_mads_tesla_light_hands_on_does_not_revoke(self):
    """handsOnLevel 1-2 is the coop-steer band; it is NOT steering_disengage and must not end lateral."""
    self._mads_apply(True)
    self._mads_engage()
    self._rx(self._user_brake_msg(True))
    self._rx(self._di_state(1))
    for level in (1, 2):
      self._rx(self._angle_meas_msg(0, hands_on_level=level))
      self.assertTrue(self.safety.get_controls_allowed_lateral())

  def test_mads_tesla_limits_still_enforced_after_brake(self):
    """Authority, not amnesty: with lateral latched through a brake the angle rate / lateral-accel limits
    still apply. A 100 deg step from 0 must be refused."""
    self._mads_apply(True)
    self._mads_engage()
    self._rx(self._user_brake_msg(True))
    self._rx(self._di_state(1))
    self._reset_speed_measurement(30)    # m/s: the lateral-accel angle bound is a few degrees here
    self.safety.set_desired_angle_last(0)
    self.assertTrue(self._mads_lateral_tx())
    self.assertFalse(self._tx(self._angle_cmd_msg(100, True)))

  def test_mads_tesla_no_longitudinal_authority_on_brake(self):
    """Lateral survives; controls_allowed (longitudinal, gas, everything else) does not, and cannot come
    back without cruise engaging again."""
    self._mads_apply(True)
    self._mads_engage()
    self._rx(self._user_brake_msg(True))
    self._rx(self._di_state(1))
    self._rx(self._user_brake_msg(False))
    self.assertFalse(self.safety.get_controls_allowed())
    self.assertTrue(self.safety.get_controls_allowed_lateral())
    self._rx(self._di_state(2))          # cruise re-engaged by the stalk pull: full authority returns
    self.assertTrue(self.safety.get_controls_allowed())

  def test_mads_tesla_heartbeat_watchdog_revokes_lateral_only(self):
    """The watchdog runs on the Raven's internal panda now: with openpilot not asking it revokes latched
    lateral after 3 ticks, and never touches controls_allowed."""
    self._mads_apply(True)
    self._mads_engage()
    self._rx(self._user_brake_msg(True))
    self._rx(self._di_state(1))
    self.safety.set_heartbeat_engaged_mads(False)
    for _ in range(3):
      self.assertTrue(self.safety.get_controls_allowed_lateral())
      self.safety.mads_heartbeat_engaged_check()
    self.assertFalse(self.safety.get_controls_allowed_lateral())
    self.assertEqual(MadsSteeringModeOnBrake.REMAIN_ACTIVE, 0)  # (import used by the generic mixin too)

# Longitudinal control tests (external panda configurations)
class TestTeslaHW2ExternalPandaSafety(TeslaLegacyLongitudinalBase, MadsBitsRefusedTestBase):
  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_Control_HW23,)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_Control_HW23]}
    self.TX_MSGS = [[MSG_DAS_Control_HW23, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy,
                                 int(TeslaSafetyFlags.FLAG_HW2 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA))
    self.safety.init_tests()


class TestTeslaHW3ExternalPandaSafety(TeslaLegacyLongitudinalBase, MadsBitsRefusedTestBase):
  def setUp(self):
    super().setUp()
    self.RELAY_MALFUNCTION_ADDRS = {0: (MSG_DAS_Control_HW23,)}
    self.FWD_BLACKLISTED_ADDRS = {2: [MSG_DAS_Control_HW23]}
    self.TX_MSGS = [[MSG_DAS_Control_HW23, 0]]

    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.teslaLegacy,
                                 int(TeslaSafetyFlags.FLAG_HW3 | TeslaSafetyFlags.FLAG_EXTERNAL_PANDA))
    self.safety.init_tests()


if __name__ == "__main__":
  unittest.main()

#!/usr/bin/env python3
import numpy as np
import random
import unittest

import opendbc.safety.tests.common as common
from opendbc.car.ford.carcontroller import MAX_LATERAL_ACCEL
from opendbc.car.ford.values import FordSafetyFlags
from opendbc.car.structs import CarParams
from opendbc.safety.tests.libsafety import libsafety_py
from opendbc.safety.tests.common import CANPackerSafety

MSG_EngBrakeData = 0x165           # RX from PCM, for driver brake pedal and cruise state
MSG_EngVehicleSpThrottle = 0x204   # RX from PCM, for driver throttle input
MSG_BrakeSysFeatures = 0x415       # RX from ABS, for vehicle speed
MSG_EngVehicleSpThrottle2 = 0x202  # RX from PCM, for second vehicle speed
MSG_Yaw_Data_FD1 = 0x91            # RX from RCM, for yaw rate
MSG_Steering_Data_FD1 = 0x083      # TX by OP, various driver switches and LKAS/CC buttons
MSG_ACCDATA = 0x186                # TX by OP, ACC controls
MSG_ACCDATA_3 = 0x18A              # TX by OP, ACC/TJA user interface
MSG_Lane_Assist_Data1 = 0x3CA      # TX by OP, Lane Keep Assist
MSG_LateralMotionControl = 0x3D3   # TX by OP, Lateral Control message
MSG_LateralMotionControl2 = 0x3D6  # TX by OP, alternate Lateral Control message
MSG_IPMA_Data = 0x3D8              # TX by OP, IPMA and LKAS user interface


def checksum(msg):
  addr, dat, bus = msg
  ret = bytearray(dat)

  if addr == MSG_Yaw_Data_FD1:
    chksum = dat[0] + dat[1]  # VehRol_W_Actl
    chksum += dat[2] + dat[3]  # VehYaw_W_Actl
    chksum += dat[5]  # VehRollYaw_No_Cnt
    chksum += dat[6] >> 6  # VehRolWActl_D_Qf
    chksum += (dat[6] >> 4) & 0x3  # VehYawWActl_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[4] = chksum

  elif addr == MSG_BrakeSysFeatures:
    chksum = dat[0] + dat[1]  # Veh_V_ActlBrk
    chksum += (dat[2] >> 2) & 0xf  # VehVActlBrk_No_Cnt
    chksum += dat[2] >> 6  # VehVActlBrk_D_Qf
    chksum = 0xff - (chksum & 0xff)
    ret[3] = chksum

  elif addr == MSG_EngVehicleSpThrottle2:
    chksum = (dat[2] >> 3) & 0xf  # VehVActlEng_No_Cnt
    chksum += (dat[4] >> 5) & 0x3  # VehVActlEng_D_Qf
    chksum += dat[6] + dat[7]  # Veh_V_ActlEng
    chksum = 0xff - (chksum & 0xff)
    ret[1] = chksum

  return addr, ret, bus


class Buttons:
  CANCEL = 0
  RESUME = 1
  TJA_TOGGLE = 2


# Ford safety has four different configurations tested here:
#  * CAN with openpilot longitudinal
#  * CAN FD with stock longitudinal
#  * CAN FD with openpilot longitudinal

class TestFordSafetyBase(common.CarSafetyTest):
  STANDSTILL_THRESHOLD = 1
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_LateralMotionControl2, MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_LateralMotionControl2, MSG_IPMA_Data]}

  STEER_MESSAGE = 0

  # Curvature control limits
  DEG_TO_CAN = 50000  # 1 / (2e-5) rad to can
  MAX_CURVATURE = 0.02
  MAX_CURVATURE_ERROR = 0.002
  CURVATURE_ERROR_MIN_SPEED = 10.0  # m/s

  # fordsafety2pnw: match the BluePilot (alan-polk) FORD_LIMITS tables in ford.h — looser
  # symmetric ROCs (BP's former "down" table for both up and down)
  ANGLE_RATE_BP = [5., 16., 25.]
  ANGLE_RATE_UP = [0.0025, 0.0014, 0.00018]  # windup limit
  ANGLE_RATE_DOWN = [0.0025, 0.0014, 0.00018]  # unwind limit

  # fordsafety2pnw: BluePilot 4-signal limits (must match ford.h)
  MAX_PATH_ANGLE = 0.25          # rad, FORD_PATH_ANGLE_MIN/MAX
  PATH_ANGLE_DEG_TO_CAN = 2000   # 1 / 0.0005
  PATH_ANGLE_RATE_BP = [5., 15., 25.]
  PATH_ANGLE_RATE = [0.003, 0.0015, 0.002]

  MAX_PATH_OFFSET = 1.0          # m, FORD_PATH_OFFSET_MIN/MAX
  PATH_OFFSET_DEG_TO_CAN = 100   # 1 / 0.01
  PATH_OFFSET_RATE_BP = [5., 15., 25.]
  PATH_OFFSET_RATE = [0.05, 0.025, 0.01]

  MAX_CURVATURE_RATE = 0.00102375           # 1/m^2, FORD_CURVATURE_RATE_MIN/MAX (asymmetric min -0.001024)
  CURVATURE_RATE_DEG_TO_CAN = 4000000       # CAN: 1 / 2.5e-7 (CAN FD overrides to 1e6)

  RESET_BYPASS_LATCH_DURATION = 60          # frames (~3 s at 20 Hz), ford.h reset_bypass_latch_counter

  cnt_speed = 0
  cnt_speed_2 = 0
  cnt_yaw_rate = 0

  packer: CANPackerSafety
  safety: libsafety_py.LibSafety

  def get_canfd_curvature_limits(self, speed):
    # Round it in accordance with the safety
    curvature_accel_limit = MAX_LATERAL_ACCEL / (max(speed, 1) ** 2)
    curvature_accel_limit_lower = int(curvature_accel_limit * self.DEG_TO_CAN - 1) / self.DEG_TO_CAN
    curvature_accel_limit_upper = int(curvature_accel_limit * self.DEG_TO_CAN + 1) / self.DEG_TO_CAN
    return curvature_accel_limit_lower, curvature_accel_limit_upper

  def _set_prev_desired_angle(self, t):
    t = round(t * self.DEG_TO_CAN)
    self.safety.set_desired_angle_last(t)

  def _reset_curvature_measurement(self, curvature, speed):
    for _ in range(6):
      self._rx(self._speed_msg(speed))
      self._rx(self._yaw_rate_msg(curvature, speed))

  # fordsafety2pnw: CAN-unit converters mirroring ford.h signal extraction
  def _curv_can(self, curvature: float) -> int:
    return round(curvature * self.DEG_TO_CAN)

  def _path_angle_can(self, path_angle: float) -> int:
    return round((path_angle + 0.5) * self.PATH_ANGLE_DEG_TO_CAN) - 1000

  def _path_offset_can(self, path_offset: float) -> int:
    return round((path_offset + 5.12) * self.PATH_OFFSET_DEG_TO_CAN) - 512

  def _curvature_rate_can(self, curvature_rate: float) -> int:
    scale = self.CURVATURE_RATE_DEG_TO_CAN
    inactive = round(0.001024 * scale)
    return round((curvature_rate + 0.001024) * scale) - inactive

  def _safety_interp_can(self, x, bp, v):
    """fordsafety2pnw: mirror helpers.h safety_interpolate (float32 arithmetic) multiplied by
    DEG_TO_CAN (float32, as in steer_angle_cmd_checks). The BluePilot rate tables hit float32
    rounding edges that float64 np.interp does not (e.g. 0.0024f * 50000f < 120)."""
    x = np.float32(x)
    bp32 = [np.float32(p) for p in bp]
    v32 = [np.float32(p) for p in v]
    if x <= bp32[0]:
      ret = v32[0]
    else:
      ret = v32[-1]
      for i in range(len(bp32) - 1):
        if x < bp32[i + 1]:
          x0, y0 = bp32[i], v32[i]
          dx = np.float32(max(np.float32(bp32[i + 1] - x0), np.float32(0.0001)))
          dy = np.float32(v32[i + 1] - y0)
          ret = np.float32(np.float32(np.float32(dy * np.float32(x - x0)) / dx) + y0)
          break
    return float(np.float32(ret * np.float32(self.DEG_TO_CAN)))

  def _drain_reset_latch(self):
    """fordsafety2pnw: ford.h arms reset_bypass_latch_counter (60 frames) on ANY lateral frame
    with curvature == 0 and path_angle == 0, and the counter is C static state — it survives
    init_tests() and leaks between test methods/classes (libsafety is a singleton). Count it
    down with non-reset frames so violation expectations are deterministic. The drain frames
    (steer disabled, curvature = 1 CAN unit) are themselves blocked; they leave
    desired_angle_last / path_angle_last / path_offset_last / curvature_rate_last at 0."""
    for _ in range(self.RESET_BYPASS_LATCH_DURATION + 1):
      self._tx(self._lat_ctl_msg(False, 0, 0, 2e-5, 0))

  # Driver brake pedal
  def _user_brake_msg(self, brake: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    enable = self.safety.get_controls_allowed()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # ABS vehicle speed
  def _speed_msg(self, speed: float, quality_flag=True):
    values = {"Veh_V_ActlBrk": speed * 3.6, "VehVActlBrk_D_Qf": 3 if quality_flag else 0, "VehVActlBrk_No_Cnt": self.cnt_speed % 16}
    self.__class__.cnt_speed += 1
    return self.packer.make_can_msg_safety("BrakeSysFeatures", 0, values, fix_checksum=checksum)

  # PCM vehicle speed
  def _speed_msg_2(self, speed: float, quality_flag=True):
    # Ford relies on speed for driver curvature limiting, so it checks two sources
    values = {"Veh_V_ActlEng": speed * 3.6, "VehVActlEng_D_Qf": 3 if quality_flag else 0, "VehVActlEng_No_Cnt": self.cnt_speed_2 % 16}
    self.__class__.cnt_speed_2 += 1
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle2", 0, values, fix_checksum=checksum)

  # Standstill state
  def _vehicle_moving_msg(self, speed: float):
    values = {"VehStop_D_Stat": 1 if speed <= self.STANDSTILL_THRESHOLD else random.choice((0, 2, 3))}
    return self.packer.make_can_msg_safety("DesiredTorqBrk", 0, values)

  # Current curvature
  def _yaw_rate_msg(self, curvature: float, speed: float, quality_flag=True):
    values = {"VehYaw_W_Actl": curvature * speed, "VehYawWActl_D_Qf": 3 if quality_flag else 0,
              "VehRollYaw_No_Cnt": self.cnt_yaw_rate % 256}
    self.__class__.cnt_yaw_rate += 1
    return self.packer.make_can_msg_safety("Yaw_Data_FD1", 0, values, fix_checksum=checksum)

  # Drive throttle input
  def _user_gas_msg(self, gas: float):
    values = {"ApedPos_Pc_ActlArb": gas}
    return self.packer.make_can_msg_safety("EngVehicleSpThrottle", 0, values)

  # Cruise status
  def _pcm_status_msg(self, enable: bool):
    # brake pedal and cruise state share same message, so we have to send
    # the other signal too
    brake = self.safety.get_brake_pressed_prev()
    values = {
      "BpedDrvAppl_D_Actl": 2 if brake else 1,
      "CcStat_D_Actl": 5 if enable else 0,
    }
    return self.packer.make_can_msg_safety("EngBrakeData", 0, values)

  # LKAS command
  def _lkas_command_msg(self, action: int):
    values = {
      "LkaActvStats_D2_Req": action,
    }
    return self.packer.make_can_msg_safety("Lane_Assist_Data1", 0, values)

  # angle2pnw: angle_mode_engaged + shadow_curvature, packed into Lane_Assist_Data1 bits with no
  # DBC signal mapped to them (see fordcan_pnw.create_lka_msg / ford.h's FORD_Lane_Assist_Data1
  # tx_hook check). No named-signal packer path exists for these bits, so build the raw bytes the
  # same way fordcan_pnw.create_lka_msg does, then wrap with libsafety_py.make_CANPacket directly
  # (bypassing make_can_msg_safety, which only knows named DBC signals).
  def _lka_angle_msg(self, angle_mode_engaged: bool, shadow_curvature: float = 0.0):
    addr, dat, bus = self.packer.make_can_msg("Lane_Assist_Data1", 0, {"LkaActvStats_D2_Req": 0})
    dat = bytearray(dat)
    raw = int(round(shadow_curvature / 1e-6))
    raw = max(-32768, min(32767, raw)) & 0xFFFF
    dat[4] |= 1 if angle_mode_engaged else 0
    dat[5] = (raw >> 8) & 0xFF
    dat[6] = raw & 0xFF
    return libsafety_py.make_CANPacket(addr, bus, bytes(dat))

  # LCA command
  def _lat_ctl_msg(self, enabled: bool, path_offset: float, path_angle: float, curvature: float, curvature_rate: float):
    if self.STEER_MESSAGE == MSG_LateralMotionControl:
      values = {
        "LatCtl_D_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCurv_NoRate_Actl": curvature_rate,  # Curvature rate [-0.001024|0.00102375] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl", 0, values)
    elif self.STEER_MESSAGE == MSG_LateralMotionControl2:
      values = {
        "LatCtl_D2_Rq": 1 if enabled else 0,
        "LatCtlPathOffst_L_Actl": path_offset,     # Path offset [-5.12|5.11] meter
        "LatCtlPath_An_Actl": path_angle,          # Path angle [-0.5|0.5235] radians
        "LatCtlCrv_NoRate2_Actl": curvature_rate,  # Curvature rate [-0.001024|0.001023] 1/meter^2
        "LatCtlCurv_No_Actl": curvature,           # Curvature [-0.02|0.02094] 1/meter
      }
      return self.packer.make_can_msg_safety("LateralMotionControl2", 0, values)

  # Cruise control buttons
  def _acc_button_msg(self, button: int, bus: int):
    values = {
      "CcAslButtnCnclPress": 1 if button == Buttons.CANCEL else 0,
      "CcAsllButtnResPress": 1 if button == Buttons.RESUME else 0,
      "TjaButtnOnOffPress": 1 if button == Buttons.TJA_TOGGLE else 0,
    }
    return self.packer.make_can_msg_safety("Steering_Data_FD1", bus, values)

  def test_rx_hook(self):
    # checksum, counter, and quality flag checks
    for quality_flag in [True, False]:
      for msg_type in ["speed", "speed_2", "yaw"]:
        self.safety.set_controls_allowed(True)
        # send multiple times to verify counter checks
        for _ in range(10):
          if msg_type == "speed":
            msg = self._speed_msg(0, quality_flag=quality_flag)
          elif msg_type == "speed_2":
            msg = self._speed_msg_2(0, quality_flag=quality_flag)
          elif msg_type == "yaw":
            msg = self._yaw_rate_msg(0, 0, quality_flag=quality_flag)

          self.assertEqual(quality_flag, self._rx(msg))
          self.assertEqual(quality_flag, self.safety.get_controls_allowed())

        # Mess with checksum to make it fail, checksum is not checked for 2nd speed
        msg[0].data[3] = 0  # Speed checksum & half of yaw signal
        should_rx = msg_type == "speed_2" and quality_flag
        self.assertEqual(should_rx, self._rx(msg))
        self.assertEqual(should_rx, self.safety.get_controls_allowed())

  def test_angle_measurements(self):
    """Tests rx hook correctly parses the curvature measurement from the vehicle speed and yaw rate"""
    for speed in np.arange(0.5, 40, 0.5):
      for curvature in np.arange(0, self.MAX_CURVATURE * 2, 2e-3):
        self._rx(self._speed_msg(speed))
        for c in (curvature, -curvature, 0, 0, 0, 0):
          self._rx(self._yaw_rate_msg(c, speed))

        self.assertEqual(self.safety.get_angle_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_angle_meas_max(), round(curvature * self.DEG_TO_CAN))

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_angle_meas_min(), round(-curvature * self.DEG_TO_CAN))
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

        self._rx(self._yaw_rate_msg(0, speed))
        self.assertEqual(self.safety.get_angle_meas_min(), 0)
        self.assertEqual(self.safety.get_angle_meas_max(), 0)

  def test_max_lateral_acceleration(self):
    # Ford CAN FD can achieve a higher max lateral acceleration than CAN so we limit curvature based on speed
    self._drain_reset_latch()  # fordsafety2pnw: C-static latch state can leak in from other tests
    step = 1 / self.DEG_TO_CAN
    for speed in np.arange(0, 40, 0.5):
      # Clip so we test curvature limiting at low speed due to low max curvature
      _, curvature_accel_limit_upper = self.get_canfd_curvature_limits(speed)
      curvature_accel_limit_upper = np.clip(curvature_accel_limit_upper, -self.MAX_CURVATURE, self.MAX_CURVATURE)

      # Test boundary curvature values around the limit, rounded to CAN precision
      lower = curvature_accel_limit_upper * 0.8
      upper = min(curvature_accel_limit_upper * 1.2, self.MAX_CURVATURE)
      test_curvatures = {round(c * self.DEG_TO_CAN) / self.DEG_TO_CAN
                         for c in self._boundary_values([curvature_accel_limit_upper], lower, upper, step)
                         if 0 <= c <= self.MAX_CURVATURE}

      for sign in (-1, 1):
        for curvature in sorted(test_curvatures):
          curvature = sign * curvature
          self.safety.set_controls_allowed(True)
          self._set_prev_desired_angle(curvature)
          self._reset_curvature_measurement(curvature, speed)

          should_tx = abs(curvature) <= curvature_accel_limit_upper
          if self._curv_can(curvature) == 0:
            should_tx = True  # fordsafety2pnw: curvature==0 && path_angle==0 is a reset frame
          self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, curvature, 0)))
          if self._curv_can(curvature) == 0:
            self._drain_reset_latch()  # the reset frame armed the bypass latch

  def test_steer_allowed(self):
    # fordsafety2pnw: expectations rewritten for the BluePilot (alan-polk) 4-signal ford.h.
    # Semantics under the ported safety:
    #   - a frame with curvature == 0 AND path_angle == 0 is a "reset frame": it is ALWAYS
    #     allowed (even with controls_allowed=False — the reset latch bypass, asserted here
    #     deliberately to pin the ported behavior) and arms a 60-frame bypass latch, which we
    #     drain before the next assertion.
    #   - otherwise, with steer request off everything must be zero (and curvature==0&&pa==0 is
    #     the reset frame case), so all remaining request-off combos are blocked.
    #   - with steer request on: controls must be allowed; path_angle and path_offset must be
    #     within their rate-of-change limits from the previous frame (0 after the drain — the
    #     grid values are far above the per-frame ROC so only 0 passes) and value limits
    #     (|path_angle| <= 0.25 rad, |path_offset| <= 1.0 m); curvature_rate's value/ROC limits
    #     exceed the DBC-representable range, so it never blocks (see
    #     test_curvature_rate_signal_checks); curvature tracks meas/prev as before, plus the
    #     CAN FD max lateral acceleration limit.
    path_offsets = np.arange(-5.12, 5.11, 2.5).round()
    path_angles = np.arange(-0.5, 0.5235, 0.25).round(1)
    curvature_rates = np.arange(-0.001024, 0.00102375, 0.001).round(3)
    curvatures = np.arange(-0.02, 0.02094, 0.01).round(2)

    self._drain_reset_latch()
    latch_armed = False

    for speed in (self.CURVATURE_ERROR_MIN_SPEED - 1,
                  self.CURVATURE_ERROR_MIN_SPEED + 1):
      _, curvature_accel_limit_upper = self.get_canfd_curvature_limits(speed)
      for controls_allowed in (True, False):
        for steer_control_enabled in (True, False):
          for path_offset in path_offsets:
            for path_angle in path_angles:
              for curvature_rate in curvature_rates:
                for curvature in curvatures:
                  if latch_armed:
                    self._drain_reset_latch()
                    latch_armed = False
                  else:
                    # path_angle/path_offset/curvature_rate "last" state updates on EVERY tx
                    # attempt (even blocked ones) — zero it so each case is judged from 0
                    self._tx(self._lat_ctl_msg(False, 0, 0, 2e-5, 0))

                  self.safety.set_controls_allowed(controls_allowed)
                  self._set_prev_desired_angle(curvature)
                  self._reset_curvature_measurement(curvature, speed)

                  curv0 = self._curv_can(curvature) == 0
                  pa0 = self._path_angle_can(path_angle) == 0
                  po0 = self._path_offset_can(path_offset) == 0
                  cr0 = self._curvature_rate_can(curvature_rate) == 0

                  # pnw-hardened latch: the reset-frame bypass is ENGAGED-ONLY. When disengaged the
                  # latch is inert and each frame is judged by the normal per-signal checks (stock
                  # semantics): only an all-zero steer command is allowed while controls_allowed=False.
                  if curv0 and pa0 and controls_allowed:
                    # reset frame while engaged: bypass all checks, arms the 60-frame latch
                    should_tx = True
                    latch_armed = True
                  elif not steer_control_enabled:
                    # steer request off: the frame must be FULLY neutral (all four signals zero);
                    # any nonzero signal is a malformed off-frame and is blocked
                    should_tx = curv0 and pa0 and po0 and cr0
                  elif controls_allowed:
                    # steer on, engaged: path_angle & path_offset within ROC-of-0 (only ~0 passes),
                    # curvature within the CAN FD lat-accel limit, curvature_rate unbounded
                    should_tx = pa0 and po0
                    if self.STEER_MESSAGE == MSG_LateralMotionControl2:
                      should_tx = should_tx and abs(curvature) <= curvature_accel_limit_upper
                  else:
                    # steer on, DISENGAGED: requesting steer control while not engaged is illegal
                    # regardless of the commanded values -> always blocked (the closed hole)
                    should_tx = False

                  with self.subTest(controls_allowed=controls_allowed, steer_control_enabled=steer_control_enabled,
                                    path_offset=float(path_offset), path_angle=float(path_angle), curvature_rate=float(curvature_rate),
                                    curvature=float(curvature)):
                    self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(steer_control_enabled, path_offset, path_angle, curvature, curvature_rate)))

  def test_curvature_rate_limits(self):
    """
    When the curvature error is exceeded, commanded curvature must start moving towards meas respecting rate limits.
    Since safety allows higher rate limits to avoid false positives, we need to allow a lower rate to move towards meas.
    """
    self.safety.set_controls_allowed(True)
    self._drain_reset_latch()  # fordsafety2pnw: C-static latch state can leak in from other tests
    self.safety.set_controls_allowed(True)
    # safety fudges the speed (1 m/s) and rate limits (1 CAN unit) to avoid false positives
    small_curvature = 1 / self.DEG_TO_CAN  # significant small amount of curvature to cross boundary

    for speed in np.arange(0, 40, 0.5):
      curvature_accel_limit_lower, curvature_accel_limit_upper = self.get_canfd_curvature_limits(speed)
      limit_command = speed > self.CURVATURE_ERROR_MIN_SPEED
      # ensure our limits match the safety's rounded limits (float32-mirrored, see _safety_interp_can)
      max_delta_up = int(self._safety_interp_can(speed - 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP) + 1) / self.DEG_TO_CAN
      max_delta_up_lower = int(self._safety_interp_can(speed + 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_UP) - 1) / self.DEG_TO_CAN

      # (the upstream +1e-3 fudge is dropped — _safety_interp_can mirrors the C float32 math exactly)
      max_delta_down = int(self._safety_interp_can(speed - 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN) + 1) / self.DEG_TO_CAN
      max_delta_down_lower = int(self._safety_interp_can(speed + 1, self.ANGLE_RATE_BP, self.ANGLE_RATE_DOWN) - 1) / self.DEG_TO_CAN

      up_cases = (self.MAX_CURVATURE_ERROR * 2, [
        (not limit_command, 0, 0),
        (not limit_command, 0, max_delta_up_lower - small_curvature),
        (True, 1e-9, max_delta_down),  # TODO: safety should not allow down limits at 0
        # fordsafety2pnw: exactly at the (inclusive) relaxed bound → allowed; the upstream
        # `not limit_command` expectation relied on the test's float64 math computing a bound
        # one CAN unit above the safety's float32 one, which _safety_interp_can now mirrors
        (True, 1e-9, max_delta_up_lower),  # TODO: safety should not allow down limits at 0
        (True, 0, max_delta_up_lower),
        (True, 0, max_delta_up),
        (False, 0, max_delta_up + small_curvature),
        # stay at boundary limit
        (True, self.MAX_CURVATURE_ERROR - small_curvature, self.MAX_CURVATURE_ERROR - small_curvature),
        # 1 unit below boundary limit
        (not limit_command, self.MAX_CURVATURE_ERROR - small_curvature * 2, self.MAX_CURVATURE_ERROR - small_curvature * 2),
        # shouldn't allow command to move outside the boundary limit if last was inside
        (not limit_command, self.MAX_CURVATURE_ERROR - small_curvature, self.MAX_CURVATURE_ERROR - small_curvature * 2),
      ])

      down_cases = (self.MAX_CURVATURE - self.MAX_CURVATURE_ERROR * 2, [
        (not limit_command, self.MAX_CURVATURE, self.MAX_CURVATURE),
        (not limit_command, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_down_lower + small_curvature),
        (True, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_down_lower),
        (True, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_down),
        (False, self.MAX_CURVATURE, self.MAX_CURVATURE - max_delta_down - small_curvature),
      ])

      for sign in (-1, 1):
        for angle_meas, cases in (up_cases, down_cases):
          self._reset_curvature_measurement(sign * angle_meas, speed)
          for should_tx, initial_curvature, desired_curvature in cases:

            # Only CAN FD has the max lateral acceleration limit
            if self.STEER_MESSAGE == MSG_LateralMotionControl2:
              if should_tx:
                # can not send if the curvature is above the max lateral acceleration
                should_tx = should_tx and abs(desired_curvature) <= curvature_accel_limit_upper
              else:
                # if desired curvature violates driver curvature error, it can only send if
                # the curvature is being limited by max lateral acceleration
                should_tx = should_tx or curvature_accel_limit_lower <= abs(desired_curvature) <= curvature_accel_limit_upper

            # small curvature ensures we're using up limits. at 0, safety allows down limits to allow to account for rounding errors
            curvature_offset = small_curvature if initial_curvature == 0 else 0
            self._set_prev_desired_angle(sign * (curvature_offset + initial_curvature))
            self.assertEqual(should_tx, self._tx(self._lat_ctl_msg(True, 0, 0, sign * (curvature_offset + desired_curvature), 0)))

  # fordsafety2pnw: dedicated tests for the BluePilot (alan-polk) 4-signal safety checks.
  # BluePilot shipped NO test coverage for these paths (its test_ford.py is the stock suite);
  # these tests were written for the pnw port to pin the ported behavior exactly as compiled.
  def test_reset_latch_behavior(self):
    """Reset-bypass latch, pnw-hardened: gated on controls_allowed. While ENGAGED, a neutral frame
    (curvature==0, path_angle==0) arms a 60-frame window in which otherwise-rate-violating frames
    still TX (the human-turn-reset ramp). The latch preserves BluePilot's engaged behavior exactly."""
    self._drain_reset_latch()
    speed = 5.
    self._reset_curvature_measurement(0, speed)
    self.safety.set_controls_allowed(True)

    # otherwise-violating frame blocked with the latch drained
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))
    # reset frame arms the latch
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))
    # violating frames pass for exactly RESET_BYPASS_LATCH_DURATION frames, then blocked
    for _ in range(self.RESET_BYPASS_LATCH_DURATION):
      self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))

  def test_reset_latch_blocked_when_disengaged(self):
    """ANTI-HOLE (pnw-hardening 2026-07-11): the latch must NEVER bypass the disengaged-steering
    block. With controls_allowed=False, neutral frames do NOT arm the latch, and a nonzero steer
    command is blocked — even immediately after neutral frames (the exact sequence openpilot emits
    while disengaged). This is the panda's core guarantee; regressing it re-opens the controls_allowed
    bypass BluePilot shipped."""
    self._drain_reset_latch()
    self._reset_curvature_measurement(0, 5.)

    # arm the latch while ENGAGED, then disengage — the stale window must not carry over
    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))       # arms latch
    self.safety.set_controls_allowed(False)

    # neutral frames while disengaged (what openpilot sends) must NOT re-arm a bypass
    for _ in range(5):
      self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))
    # a nonzero steer command while disengaged is BLOCKED, latch or no latch
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))
    # and again right after another neutral frame (no arming path exists while disengaged)
    self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))

  # angle2pnw: dedicated tests for the angle-primary lateral mode safety additions (bp-7.0 port,
  # first pass). Mirrors the fordsafety2pnw doctrine above -- BluePilot shipped no test coverage
  # for its own angle-mode ford.h either; these pin the ported-and-hardened behavior exactly as
  # compiled, and specifically the two ANTI-HOLE properties this merge had to preserve: the
  # corroborated wide-range path_angle value gate cannot be forged by a curvature-only frame, and
  # neither it nor the reset-bypass latch can ever substitute for controls_allowed.

  def test_angle_mode_value_range_requires_engaged_flag(self):
    """ANTI-HOLE: the wide DBC path_angle range (+-0.5/0.5235 rad) only applies once
    ford_bp_angle_mode_engaged is corroborated via Lane_Assist_Data1 -- a frame cannot unlock it by
    merely sending curvature=0. 0.35 rad is inside the wide DBC range but outside the tight
    curvature-mode +-0.25 rad cap that applies whenever the corroboration is absent."""
    self._drain_reset_latch()
    self.safety.set_controls_allowed(True)
    speed = 5.
    self._reset_curvature_measurement(0, speed)

    self._tx(self._lka_angle_msg(False))  # ensure the static starts disarmed
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.35, 0, 0)))

    # Corroborate angle mode -- desired_path_angle_last is already primed to 0.35 from the blocked
    # frame above (state updates even on blocked frames, see test_path_angle_limits), so this is a
    # zero-delta repeat and isolates the value check specifically (no ROC interaction).
    self._tx(self._lka_angle_msg(True))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.35, 0, 0)))

    # Revoking the flag re-tightens the value range on the very next frame.
    self._tx(self._lka_angle_msg(False))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.35, 0, 0)))

  def test_angle_mode_curvature_zero_requires_controls_allowed(self):
    """ANTI-HOLE: when desired_curvature == 0 (angle mode's normal operating point -- it holds
    curvature at the inactive sentinel while path_angle does the real steering),
    steer_angle_cmd_checks's deviation-vs-measured logic goes moot, so this port substitutes an
    explicit controls_allowed gate in its place (bp-7.0 used `controls_allowed ||
    controls_allowed_lateral` here for MADS; this tree has none, so plain controls_allowed --
    strictly narrower -- is substituted; see the identical comment at both call sites in ford.h).
    Prove the substitution actually gates: a small path_angle command, comfortably inside both the
    ROC and (engaged, widened) value budgets so nothing else could explain a block, is allowed
    while engaged and blocked while disengaged."""
    self._drain_reset_latch()
    self._reset_curvature_measurement(0, 5.)
    self._tx(self._lka_angle_msg(True))  # corroborate angle mode -- the realistic operating point

    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.03, 0, 0)))

    self.safety.set_controls_allowed(False)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.03, 0, 0)))

  def test_angle_mode_reset_latch_blocked_when_disengaged(self):
    """ANTI-HOLE: the reset-bypass latch's pnw-hardening (see
    test_reset_latch_blocked_when_disengaged above) must hold identically when angle mode is
    corroborated engaged -- the widened path_angle value range must never interact with or weaken
    the latch's controls_allowed gate. Mirrors that test exactly (same 0.2 rad / 0.01 curvature
    probe, chosen there so only the latch bypass -- not the ROC budget -- could explain a pass),
    with ford_bp_angle_mode_engaged=True threaded through the whole sequence."""
    self._drain_reset_latch()
    self._reset_curvature_measurement(0, 5.)
    self._tx(self._lka_angle_msg(True))

    self.safety.set_controls_allowed(True)
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0)))       # arms latch
    self.safety.set_controls_allowed(False)

    for _ in range(5):
      self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))
    self._tx(self._lat_ctl_msg(True, 0, 0, 0, 0))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.2, 0.01, 0)))

  def test_angle_mode_shadow_curvature_deviation_check(self):
    """angle2pnw: ford_shadow_curvature_error_check fires when angle mode is confirmed engaged and
    the commanded shadow_curvature diverges from measured curvature by more than the tolerance,
    above angle_error_min_speed -- the only cross-check angle mode has against reality, since
    desired_curvature stays pinned at the inactive sentinel (0) on the wire in that mode."""
    self._drain_reset_latch()
    speed = 15.  # > FORD_STEERING_LIMITS.angle_error_min_speed (10.0)
    self._reset_curvature_measurement(0, speed)  # measured curvature ~= 0
    self.safety.set_controls_allowed(True)

    self._tx(self._lka_angle_msg(True, shadow_curvature=0.0))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.03, 0, 0)))

    # shadow_curvature far from measured (0.015 1/m, well beyond MAX_CURVATURE_ERROR=0.002): the
    # corroborated deviation check blocks it even though desired_curvature is 0 on the wire.
    self._tx(self._lka_angle_msg(True, shadow_curvature=0.015))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, 0.03, 0, 0)))

    # Dropping the engaged flag (curvature mode) skips the check entirely -- same frame passes.
    self._tx(self._lka_angle_msg(False))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0.03, 0, 0)))

  def test_path_angle_limits(self):
    """Path angle: per-frame rate-of-change limit + |0.25| rad value limit (BluePilot values)."""
    self._drain_reset_latch()
    self.safety.set_controls_allowed(True)
    speed = 5.
    self._reset_curvature_measurement(2e-5, speed)
    curv = 2e-5  # 1 CAN unit — keeps every frame a non-reset frame
    unit = 1 / self.PATH_ANGLE_DEG_TO_CAN

    # per-frame ROC allowance (safety fudges speed by -1 m/s): interp * deg_to_can + 1
    delta = int(np.interp(speed - 1, self.PATH_ANGLE_RATE_BP, self.PATH_ANGLE_RATE) * self.PATH_ANGLE_DEG_TO_CAN + 1)

    # stepping within the ROC from 0 is allowed
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, delta * unit, curv, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 2 * delta * unit, curv, 0)))
    # exceeding the ROC is blocked ("last" state updates even on blocked frames)
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, (3 * delta + 1) * unit, curv, 0)))

    # value limit: jump the last (blocked by ROC, but state updates), then probe the edge
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, self.MAX_PATH_ANGLE, curv, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, 0, self.MAX_PATH_ANGLE + unit, curv, 0)))  # 1 unit above: value violation
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, self.MAX_PATH_ANGLE, curv, 0)))          # at the limit: allowed

    # steer request off: nonzero path_angle blocked (curvature nonzero, so not a reset frame)
    self.assertFalse(self._tx(self._lat_ctl_msg(False, 0, delta * unit, curv, 0)))

  def test_path_offset_limits(self):
    """Path offset: per-frame rate-of-change limit + |1.0| m value limit (BluePilot values)."""
    self._drain_reset_latch()
    self.safety.set_controls_allowed(True)
    speed = 5.
    self._reset_curvature_measurement(2e-5, speed)
    curv = 2e-5
    unit = 1 / self.PATH_OFFSET_DEG_TO_CAN

    delta = int(np.interp(speed - 1, self.PATH_OFFSET_RATE_BP, self.PATH_OFFSET_RATE) * self.PATH_OFFSET_DEG_TO_CAN + 1)

    self.assertTrue(self._tx(self._lat_ctl_msg(True, delta * unit, 0, curv, 0)))
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 2 * delta * unit, 0, curv, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, (3 * delta + 1) * unit, 0, curv, 0)))

    self.assertFalse(self._tx(self._lat_ctl_msg(True, self.MAX_PATH_OFFSET, 0, curv, 0)))
    self.assertFalse(self._tx(self._lat_ctl_msg(True, self.MAX_PATH_OFFSET + unit, 0, curv, 0)))  # 1 unit above: value violation
    self.assertTrue(self._tx(self._lat_ctl_msg(True, self.MAX_PATH_OFFSET, 0, curv, 0)))          # at the limit: allowed

    self.assertFalse(self._tx(self._lat_ctl_msg(False, delta * unit, 0, curv, 0)))

  def test_curvature_rate_signal_checks(self):
    """Documents ported-as-is BluePilot behavior: the curvature_rate value and ROC limits are
    WIDER than (or equal to) the DBC-representable range, so curvature_rate never blocks a
    frame while the steer request is on. With the request off a nonzero curvature_rate is a
    violation — but only observable alongside a nonzero curvature/path_angle, because with
    both zero the frame is a reset frame and bypasses everything."""
    self._drain_reset_latch()
    self.safety.set_controls_allowed(True)
    speed = 5.
    self._reset_curvature_measurement(2e-5, speed)
    curv = 2e-5
    max_cr = 0.001  # near the representable maximum

    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, curv, max_cr)))   # full jump from 0: allowed
    self.assertTrue(self._tx(self._lat_ctl_msg(True, 0, 0, curv, -max_cr)))  # full reversal: allowed

    # steer request off: nonzero curvature_rate blocked when the frame is not a reset frame
    self.assertFalse(self._tx(self._lat_ctl_msg(False, 0, 0, curv, max_cr)))
    # ...but with curvature == 0 and path_angle == 0 it is a reset frame and TXes
    self.assertTrue(self._tx(self._lat_ctl_msg(False, 0, 0, 0, max_cr)))
    self._drain_reset_latch()  # that reset frame armed the latch

  def test_prevent_lkas_action(self):
    self.safety.set_controls_allowed(1)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

    self.safety.set_controls_allowed(0)
    self.assertFalse(self._tx(self._lkas_command_msg(1)))

  def test_acc_buttons(self):
    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for enabled in (True, False):
        self._rx(self._pcm_status_msg(enabled))
        self.assertTrue(self._tx(self._acc_button_msg(Buttons.TJA_TOGGLE, 2)))

    for allowed in (0, 1):
      self.safety.set_controls_allowed(allowed)
      for bus in (0, 2):
        self.assertEqual(allowed, self._tx(self._acc_button_msg(Buttons.RESUME, bus)))

    for enabled in (True, False):
      self._rx(self._pcm_status_msg(enabled))
      for bus in (0, 2):
        self.assertEqual(enabled, self._tx(self._acc_button_msg(Buttons.CANCEL, bus)))


class TestFordCANFDStockSafety(TestFordSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2
  CURVATURE_RATE_DEG_TO_CAN = 1000000  # fordsafety2pnw: CAN FD scaling (1 / 1e-6)

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.CANFD)
    self.safety.init_tests()


class TestFordLongitudinalSafetyBase(TestFordSafetyBase):
  MAX_ACCEL = 2.0  # accel is used for brakes, but openpilot can set positive values
  MIN_ACCEL = -3.5
  INACTIVE_ACCEL = 0.0

  MAX_GAS = 2.0
  MIN_GAS = -0.5
  INACTIVE_GAS = -5.0

  # ACC command
  def _acc_command_msg(self, gas: float, brake: float, brake_actuation: bool, cmbb_deny: bool = False):
    values = {
      "AccPrpl_A_Rq": gas,                              # [-5|5.23] m/s^2
      "AccPrpl_A_Pred": gas,                            # [-5|5.23] m/s^2
      "AccBrkTot_A_Rq": brake,                          # [-20|11.9449] m/s^2
      "AccBrkPrchg_B_Rq": 1 if brake_actuation else 0,  # Pre-charge brake request: 0=No, 1=Yes
      "AccBrkDecel_B_Rq": 1 if brake_actuation else 0,  # Deceleration request: 0=Inactive, 1=Active
      "CmbbDeny_B_Actl": 1 if cmbb_deny else 0,         # [0|1] deny AEB actuation
    }
    return self.packer.make_can_msg_safety("ACCDATA", 0, values)

  def test_stock_aeb(self):
    # Test that CmbbDeny_B_Actl is never 1, it prevents the ABS module from actuating AEB requests from ACCDATA_2
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for cmbb_deny in (True, False):
        should_tx = not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, self.INACTIVE_ACCEL, controls_allowed, cmbb_deny)))
        should_tx = controls_allowed and not cmbb_deny
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.MAX_GAS, self.MAX_ACCEL, controls_allowed, cmbb_deny)))

  def test_gas_safety_check(self):
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for gas in np.concatenate((np.arange(self.MIN_GAS - 2, self.MAX_GAS + 2, 0.05), [self.INACTIVE_GAS])):
        gas = round(gas, 2)  # floats might not hit exact boundary conditions without rounding
        should_tx = (controls_allowed and self.MIN_GAS <= gas <= self.MAX_GAS) or gas == self.INACTIVE_GAS
        self.assertEqual(should_tx, self._tx(self._acc_command_msg(gas, self.INACTIVE_ACCEL, controls_allowed)))

  def test_brake_safety_check(self):
    brake_values = self._boundary_values([self.MIN_ACCEL, self.MAX_ACCEL, self.INACTIVE_ACCEL],
                                         self.MIN_ACCEL - 2, self.MAX_ACCEL + 2, 0.05)
    for controls_allowed in (True, False):
      self.safety.set_controls_allowed(controls_allowed)
      for brake_actuation in (True, False):
        for brake in brake_values:
          should_tx = (controls_allowed and self.MIN_ACCEL <= brake <= self.MAX_ACCEL) or brake == self.INACTIVE_ACCEL
          should_tx = should_tx and (controls_allowed or not brake_actuation)
          self.assertEqual(should_tx, self._tx(self._acc_command_msg(self.INACTIVE_GAS, brake, brake_actuation)))


class TestFordLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    # Make sure we enforce long safety even without long flag for CAN
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, 0)
    self.safety.init_tests()

  def test_max_lateral_acceleration(self):
    # CAN does not limit curvature from lateral acceleration
    pass


class TestFordCANFDLongitudinalSafety(TestFordLongitudinalSafetyBase):
  STEER_MESSAGE = MSG_LateralMotionControl2
  CURVATURE_RATE_DEG_TO_CAN = 1000000  # fordsafety2pnw: CAN FD scaling (1 / 1e-6)

  TX_MSGS = [
    [MSG_Steering_Data_FD1, 0], [MSG_Steering_Data_FD1, 2], [MSG_ACCDATA, 0], [MSG_ACCDATA_3, 0], [MSG_Lane_Assist_Data1, 0],
    [MSG_LateralMotionControl2, 0], [MSG_IPMA_Data, 0],
  ]
  RELAY_MALFUNCTION_ADDRS = {0: (MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                                 MSG_IPMA_Data)}

  FWD_BLACKLISTED_ADDRS = {2: [MSG_ACCDATA, MSG_ACCDATA_3, MSG_Lane_Assist_Data1, MSG_LateralMotionControl2,
                               MSG_IPMA_Data]}

  def setUp(self):
    self.packer = CANPackerSafety("ford_lincoln_base_pt")
    self.safety = libsafety_py.libsafety
    self.safety.set_safety_hooks(CarParams.SafetyModel.ford, FordSafetyFlags.LONG_CONTROL | FordSafetyFlags.CANFD)
    self.safety.init_tests()


if __name__ == "__main__":
  unittest.main()

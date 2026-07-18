#pragma once

#include "opendbc/safety/declarations.h"

// fordsafety2pnw: 4-signal Ford lateral safety, ported faithfully from BluePilot (alan-polk),
// branch bluepilotdev/bp-dev opendbc_repo/opendbc/safety/modes/ford.h. The lateral section
// (value limits, per-signal rate-of-change checks, reset bypass latch, FORD_LIMITS rate tables)
// is numerically identical to the validated BluePilot source. Deliberate deviations from BP,
// each because the surrounding infrastructure differs in this tree (documented in FORDSAFETY2PNW
// notes / commit message):
//   - no MADS: BP's mads_button_press + acc_main_on rx additions and the Steering_Data_FD1
//     RxCheck are omitted (this tree has no sunnypilot MADS state machine).
//   - ford_init keeps the UPSTREAM longitudinal default (long is default for non-CANFD CAN);
//     BP disabled that default for its own fleet. Orthogonal to the 4-signal lateral port.
//
// angle2pnw (FIRST PASS, 2026-07-18): angle-primary lateral mode safety, ported from BluePilot
// (alan-polk) bp-7.0 opendbc_repo/opendbc/safety/modes/ford.h (captured at bp-7.0 tip
// 19858f2888). Adds: ford_bp_angle_mode_engaged / ford_bp_shadow_curvature_raw (read out of
// FORD_Lane_Assist_Data1's unused bits, corroborating the wide-range path_angle value check),
// ford_shadow_curvature_error_check (deviation-only cross-check against measured curvature when
// angle mode is confirmed engaged), and a SEPARATE angle-mode path_angle rate-limit table
// (FORD_PATH_ANGLE_LIMITS_ANGLE, only consulted when angle mode is confirmed engaged). Deliberate
// deviations from bp-7.0, each documented at the point of change below:
//   - the FORD_PATH_ANGLE_LIMITS_ANGLE split: bp-7.0 widens its SHARED FORD_PATH_ANGLE_LIMITS
//     ROC table ~10x for angle mode's full-swing actuator use, which would also silently loosen
//     the ALREADY-DEPLOYED curvature-mode path_angle trim signal (FORD_PATH_ANGLE_LIMITS is a
//     compile-time C struct — a runtime engaged-flag cannot make a shared table conditional). A
//     separate struct, selected at the call site by ford_bp_angle_mode_engaged, is strictly safer:
//     the currently-driven curvature path's ROC is untouched no matter what angle mode does.
//   - controls_allowed_lateral (MADS) substituted with controls_allowed everywhere it appeared in
//     bp-7.0's angle-mode diff (MADS symbols do not exist in this tree; this is strictly narrower).
//   - the reset-bypass latch KEEPS this tree's pnw-hardening (gated on controls_allowed, see
//     "pnw-hardening (2026-07-11)" comment below) in BOTH LMC and LMC2 — bp-7.0's own ford.h still
//     carries the unconditional bypass our 19ad2728 fix closed; NOT reintroduced here.
//   - bp-7.0's rx_hook MADS-only additions (mads_button_press, the Steering_Data_FD1 RxCheck, and
//     the acc_main_on write that rode along with them in the same diff hunk) are NOT ported —
//     mads_button_press doesn't exist in this tree, and acc_main_on is not read by any angle-mode
//     check below (verified: no reference to it in this file), so it is out of scope for this port.

// Safety-relevant CAN messages for Ford vehicles.
#define FORD_EngBrakeData          0x165U   // RX from PCM, for driver brake pedal and cruise state
#define FORD_EngVehicleSpThrottle  0x204U   // RX from PCM, for driver throttle input
#define FORD_DesiredTorqBrk        0x213U   // RX from ABS, for standstill state
#define FORD_BrakeSysFeatures      0x415U   // RX from ABS, for vehicle speed
#define FORD_EngVehicleSpThrottle2 0x202U   // RX from PCM, for second vehicle speed
#define FORD_Yaw_Data_FD1          0x91U    // RX from RCM, for yaw rate
#define FORD_Steering_Data_FD1     0x083U   // TX by OP, various driver switches and LKAS/CC buttons
#define FORD_ACCDATA               0x186U   // TX by OP, ACC controls
#define FORD_ACCDATA_3             0x18AU   // TX by OP, ACC/TJA user interface
#define FORD_Lane_Assist_Data1     0x3CAU   // TX by OP, Lane Keep Assist
#define FORD_LateralMotionControl  0x3D3U   // TX by OP, Lateral Control message
#define FORD_LateralMotionControl2 0x3D6U   // TX by OP, alternate Lateral Control message
#define FORD_IPMA_Data             0x3D8U   // TX by OP, IPMA and LKAS user interface

// CAN bus numbers.
#define FORD_MAIN_BUS 0U
#define FORD_CAM_BUS  2U

static uint8_t ford_get_counter(const CANPacket_t *msg) {
  uint8_t cnt = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cnt
    cnt = (msg->data[2] >> 2) & 0xFU;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYaw_No_Cnt
    cnt = msg->data[5];
  } else {
  }
  return cnt;
}

static uint32_t ford_get_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    // Signal: VehVActlBrk_No_Cs
    chksum = msg->data[3];
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    // Signal: VehRollYawW_No_Cs
    chksum = msg->data[4];
  } else {
  }
  return chksum;
}

static uint32_t ford_compute_checksum(const CANPacket_t *msg) {
  uint8_t chksum = 0;
  if (msg->addr == FORD_BrakeSysFeatures) {
    chksum += msg->data[0] + msg->data[1];  // Veh_V_ActlBrk
    chksum += msg->data[2] >> 6;                    // VehVActlBrk_D_Qf
    chksum += (msg->data[2] >> 2) & 0xFU;           // VehVActlBrk_No_Cnt
    chksum = 0xFFU - chksum;
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    chksum += msg->data[0] + msg->data[1];  // VehRol_W_Actl
    chksum += msg->data[2] + msg->data[3];  // VehYaw_W_Actl
    chksum += msg->data[5];                         // VehRollYaw_No_Cnt
    chksum += msg->data[6] >> 6;                    // VehRolWActl_D_Qf
    chksum += (msg->data[6] >> 4) & 0x3U;           // VehYawWActl_D_Qf
    chksum = 0xFFU - chksum;
  } else {
  }
  return chksum;
}

static bool ford_get_quality_flag_valid(const CANPacket_t *msg) {
  bool valid = false;
  if (msg->addr == FORD_BrakeSysFeatures) {
    valid = (msg->data[2] >> 6) == 0x3U;           // VehVActlBrk_D_Qf
  } else if (msg->addr == FORD_EngVehicleSpThrottle2) {
    valid = ((msg->data[4] >> 5) & 0x3U) == 0x3U;  // VehVActlEng_D_Qf
  } else if (msg->addr == FORD_Yaw_Data_FD1) {
    valid = ((msg->data[6] >> 4) & 0x3U) == 0x3U;  // VehYawWActl_D_Qf
  } else {
  }
  return valid;
}

#define FORD_INACTIVE_CURVATURE 1000U
#define FORD_INACTIVE_CURVATURE_RATE 4096U
#define FORD_INACTIVE_PATH_OFFSET 512U
#define FORD_INACTIVE_PATH_ANGLE 1000U

#define FORD_CANFD_INACTIVE_CURVATURE_RATE 1024U

// BluePilot: Control signal limits — curvature magnitude must match MAX_CURVATURE; rate tables must
// match opendbc/car/ford/values_pnw.py BP_ANGLE_LIMITS (FORD_LIMITS macro below).
#define FORD_CURVATURE_MIN -0.02f
#define FORD_CURVATURE_MAX 0.02f
#define FORD_CURVATURE_RATE_MIN -0.001024f
#define FORD_CURVATURE_RATE_MAX 0.00102375f
#define FORD_PATH_OFFSET_MIN -1.0f
#define FORD_PATH_OFFSET_MAX 1.0f
#define FORD_PATH_ANGLE_MIN -0.25f
#define FORD_PATH_ANGLE_MAX 0.25f

// angle2pnw: full DBC signal range for path_angle (LatCtlPath_An_Actl, 0.0005 scale). Used as the
// VALUE limit only when angle mode is confirmed engaged via ford_bp_angle_mode_engaged (see
// FORD_Lane_Assist_Data1 tx_hook check below) — a curvature-mode frame cannot unlock this wider
// range by merely sending curvature = 0. Matches lateral_angle_pnw.py's
// FORD_DBC_PATH_ANGLE_MIN/MAX. In curvature mode the tight +-0.25 cap above always applies instead.
#define FORD_DBC_PATH_ANGLE_MIN -0.5f
#define FORD_DBC_PATH_ANGLE_MAX 0.5235f



// Curvature rate limits
#define FORD_LIMITS(limit_lateral_acceleration) {                                               \
  .max_angle = 1000,          /* 0.02 curvature */                                              \
  .angle_deg_to_can = 50000,  /* 1 / (2e-5) rad to can */                                       \
  .max_angle_error = 100,     /* 0.002 * FORD_STEERING_LIMITS.angle_deg_to_can */               \
  /* BluePilot: looser symmetric ROCs (former down table); Python control uses stricter up row in values_pnw */ \
  .angle_rate_up_lookup = {                                                                     \
    {5., 16., 25.},                                                                             \
    {0.0025f, 0.0014f, 0.00018f}                                                                \
  },                                                                                            \
  .angle_rate_down_lookup = {                                                                   \
    {5., 16., 25.},                                                                             \
    {0.0025f, 0.0014f, 0.00018f}                                                                \
  },                                                                                            \
                                                                                                \
  /* no blending at low speed due to lack of torque wind-up and inaccurate current curvature */ \
  .angle_error_min_speed = 10.0,    /* m/s */                                                   \
  .frequency = 20U,                 /* LateralMotionControl / LateralMotionControl2 @ 20 Hz */   \
                                                                                                \
  .angle_is_curvature = (limit_lateral_acceleration),                                           \
  .enforce_angle_error = true,                                                                  \
  .inactive_angle_is_zero = true,                                                               \
}

// BluePilot: PathAngle rate limits
static const AngleSteeringLimits FORD_PATH_ANGLE_LIMITS = {
  .max_angle = 1000,
  // 0.0005
  .angle_deg_to_can = 2000,        // 1 / (2e-5) rad to can
  .max_angle_error = 4,           // 0.002 * FORD_STEERING_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.003, 0.0015, 0.002}
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.003, 0.0015, 0.002}
  },
  .angle_error_min_speed = 9.9,   // m/s
  .frequency = 100U,              // Hz

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// angle2pnw: dedicated angle-mode path_angle ROC — ONLY consulted when
// ford_bp_angle_mode_engaged is confirmed true (see the path_angle_cmd_checks call sites in the
// LMC/LMC2 blocks below). Deliberately a SEPARATE struct from FORD_PATH_ANGLE_LIMITS above (see
// the file-header comment for why splitting is the safer option vs bp-7.0's shared-table widen).
// Values below are bp-7.0's, unmodified: interp(v_ego, [9,10,15,25], [0.055,0.055,0.0425,0.009])
// rad/call from lateral_angle_pnw.py's soft ROC, scaled x1.02 so panda is 2% LOOSER than the
// Python control and never blocks a legitimate LMC/LMC2 frame. lookup_t is fixed at 3 points;
// Python's 9 & 10 m/s nodes are both 0.055 (flat top), so {10,15,25} reproduces the curve exactly
// and speeds <10 clamp to the first point.
// bp-7.0 note (carried forward): this strategy runs once per CarControllerParams.STEER_STEP (5)
// control ticks = 20Hz, not 100Hz — the y-values here are already the real-world-rate-correct,
// 20Hz-native numbers (bp-7.0 rescaled x5 from an earlier 100Hz-authored draft; our STEER_STEP=5
// cadence is confirmed to match, see opendbc/car/ford/values.py).
static const AngleSteeringLimits FORD_PATH_ANGLE_LIMITS_ANGLE = {
  .max_angle = 1000,
  .angle_deg_to_can = 2000,        // 1 / (0.0005) rad to can — same DBC scale as curvature-mode's
  .max_angle_error = 4,            // unused by path_angle_cmd_checks (see below), kept for parity
  .angle_rate_up_lookup = {
    .x = {10., 15., 25.},
    .y = {0.0561, 0.04335, 0.00918}
  },
  .angle_rate_down_lookup = {
    .x = {10., 15., 25.},
    .y = {0.0561, 0.04335, 0.00918}
  },
  .angle_error_min_speed = 9.9,    // unused by path_angle_cmd_checks, kept for parity with bp-7.0
  .frequency = 20U,                // Hz — LateralMotionControl/LateralMotionControl2 @ 20Hz

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// BluePilot: PathOffset rate limits
static const AngleSteeringLimits FORD_PATH_OFFSET_LIMITS = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 100,        // 1 / (0.01) meter to can
  .max_angle_error = 2,           // 0.02 * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_error_min_speed = 5.0,   // m/s - lower speed threshold for path offset
  .frequency = 20U,               // Hz - 20Hz message rate

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// BluePilot: CurvatureRate limits (CAN scaling)
static const AngleSteeringLimits FORD_CURVATURE_RATE_LIMITS_CAN = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 4000000,    // 1 / (2.5E-7) to can
  .max_angle_error = 2,           // 0.02 * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_error_min_speed = 5.0,   // m/s - lower speed threshold for path offset
  .frequency = 20U,               // Hz - 20Hz message rate

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

// BluePilot: CurvatureRate limits (CAN FD scaling)
static const AngleSteeringLimits FORD_CURVATURE_RATE_LIMITS_CANFD = {
  .max_angle = 100,               // 1.0 meter in CAN units (100 * 0.01)
  .angle_deg_to_can = 1000000,    // 1 / (1E-6) to can
  .max_angle_error = 2,           // 0.02 * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can
  .angle_rate_up_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_rate_down_lookup = {
    .x = {5., 15., 25.},
    .y = {0.05, 0.025, 0.01}     // Slower rate limits for path offset
  },
  .angle_error_min_speed = 5.0,   // m/s - lower speed threshold for path offset
  .frequency = 20U,               // Hz - 20Hz message rate

  .enforce_angle_error = true,
  .inactive_angle_is_zero = true,
};

static const AngleSteeringLimits FORD_STEERING_LIMITS = FORD_LIMITS(false);



static int desired_path_angle_last = 0;

// BluePilot: Reset latch: allows bypass for a short period after reset (both curvature and path_angle = 0)
// This enables smooth ramp-up after human turn detection without blocked messages
// Latch activates when reset detected, stays active for ~3 seconds (60 frames at 20Hz)
// Prevents exploitation by requiring reset state first and having a timeout
// BluePilot: openpilot must send curvature_rate ~= 0 during reset and keep apply_curvature_last
// aligned with the prior TX (see carcontroller BP path); else curvature_rate_cmd_checks can trip.
static uint8_t reset_bypass_latch_counter = 0;
static const uint8_t RESET_BYPASS_LATCH_DURATION = 60;  // ~3.0 seconds at 20Hz
static bool test = false;

// angle2pnw: angle_mode_engaged + shadow_curvature, read synchronously out of Lane_Assist_Data1's
// unused bits inside ford_tx_hook below (no separate CAN message, no RX -- panda does not
// self-receive its own TX). shadow_curvature is the curvature (kappa) that angle mode's
// path_angle was derived from (see lateral_angle_pnw.py's bp_kappa_cmd) -- angle mode holds the
// real curvature signal at the inactive sentinel (0) on the wire, so without this there is no
// commanded-vs-measured deviation check for angle mode at all (steer_angle_cmd_checks is only
// meaningfully enforced when desired_curvature != 0). Feeding shadow_curvature into an equivalent
// check when angle mode is confirmed engaged restores that protection (see
// ford_shadow_curvature_error_check below).
static bool ford_bp_angle_mode_engaged = false;
static int16_t ford_bp_shadow_curvature_raw = 0;  // wire units, scale 1e-6 1/m (see fordcan_pnw.py)

// shadow_curvature is packed at scale 1e-6 1/m; convert to the CAN units steer_angle_cmd_checks /
// ford_shadow_curvature_error_check expect, matching FORD_STEERING_LIMITS.angle_deg_to_can
// (50000, i.e. physical scale 2e-5): raw * 1e-6 * 50000 = raw * 0.05.
#define FORD_BP_SHADOW_CURVATURE_TO_CAN(raw) ((int)((float)(raw) * 0.05f))

static bool path_angle_cmd_checks(int desired_path_angle, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_angle_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_angle = desired_path_angle_last + delta_path_angle_roc;
    int lowest_desired_path_angle = desired_path_angle_last - delta_path_angle_roc;

    violation |= safety_max_limit_check(desired_path_angle, highest_desired_path_angle, lowest_desired_path_angle);
    if (test) {
      FORD_SAFETY_DBG("path_angle_cmd_checks 1: desired_path_angle: %d desired_path_angle_last: %d highest_desired_path_angle: %d lowest_desired_path_angle: %d violation: %d \n",
                      desired_path_angle, desired_path_angle_last, highest_desired_path_angle, lowest_desired_path_angle, (int)violation);
    }
  }
  desired_path_angle_last = desired_path_angle;

  if (!steer_control_enabled) {
    violation |= (desired_path_angle != 0);
  }
  if (test) {
    FORD_SAFETY_DBG("path_angle_cmd_checks 2: violation: %d \n", (int)violation);
  }

  return violation;
}

static int desired_path_offset_last = 0;

static bool path_offset_cmd_checks(int desired_path_offset, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int delta_path_offset_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_path_offset = desired_path_offset_last + delta_path_offset_roc;
    int lowest_desired_path_offset = desired_path_offset_last - delta_path_offset_roc;

    violation |= safety_max_limit_check(desired_path_offset, highest_desired_path_offset, lowest_desired_path_offset);
    if (test) {
      FORD_SAFETY_DBG("path_offset_cmd_checks 1: desired_path_offset: %d desired_path_offset_last: %d highest_desired_path_offset: %d lowest_desired_path_offset: %d violation: %d \n",
                      desired_path_offset, desired_path_offset_last, highest_desired_path_offset, lowest_desired_path_offset, (int)violation);
    }

  }
  desired_path_offset_last = desired_path_offset;

  if (!steer_control_enabled) {
    violation |= (desired_path_offset != 0);
  }
  if (test) {
    FORD_SAFETY_DBG("path_offset_cmd_checks 2: violation: %d \n", (int)violation);
  }

  return violation;
}

static int desired_curvature_rate_last = 0;

static bool curvature_rate_cmd_checks(int desired_curvature_rate, bool steer_control_enabled, const AngleSteeringLimits limits) {
  bool violation = false;

  if(steer_control_enabled){
    float speed = ((float)vehicle_speed.min / VEHICLE_SPEED_FACTOR) - 1.;

    int desired_curvature_rate_roc = (safety_interpolate(limits.angle_rate_up_lookup, speed) * limits.angle_deg_to_can) + 1.;

    int highest_desired_curvature_rate = desired_curvature_rate_last + desired_curvature_rate_roc;
    int lowest_desired_curvature_rate = desired_curvature_rate_last - desired_curvature_rate_roc;

    violation |= safety_max_limit_check(desired_curvature_rate, highest_desired_curvature_rate, lowest_desired_curvature_rate);
    if (test) {
      FORD_SAFETY_DBG("curvature_rate_cmd_checks 1: desired_curvature_rate: %d desired_curvature_rate_last: %d highest_desired_curvature_rate: %d lowest_desired_curvature_rate: %d violation: %d \n",
                      desired_curvature_rate, desired_curvature_rate_last, highest_desired_curvature_rate, lowest_desired_curvature_rate, (int)violation);
    }
  }
  desired_curvature_rate_last = desired_curvature_rate;


  if (!steer_control_enabled) {
    violation |= (desired_curvature_rate != 0);
  }
  if (test) {
    FORD_SAFETY_DBG("curvature_rate_cmd_checks 2: violation: %d \n", (int)violation);
  }

  return violation;
}

// angle2pnw: angle mode has no "current path_angle" measurement to check the command against,
// unlike curvature mode, which compares desired_curvature against angle_meas (measured curvature,
// from yaw rate). Without this, a large deviation between commanded path_angle and the car's
// ACTUAL curvature -- e.g. a pothole or driver override kicking the wheel -- would go unchecked:
// path_angle's own ROC (path_angle_cmd_checks / FORD_PATH_ANGLE_LIMITS_ANGLE) only bounds how
// fast the COMMAND changes, not how far it may sit from reality.
//
// Deliberately narrower than steer_angle_cmd_checks: no rate-of-change enforcement here (that's
// path_angle_cmd_checks's job), and no shared state with curvature mode. This is a pure per-frame
// proximity check: does this frame's steering intent make physical sense given where the car is.
static bool ford_shadow_curvature_error_check(int desired_curvature, bool steer_control_enabled,
                                              const AngleSteeringLimits limits) {
  bool violation = false;
  if (steer_control_enabled && limits.enforce_angle_error &&
      ((vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR) > limits.angle_error_min_speed)) {
    int lowest_allowed = angle_meas.min - limits.max_angle_error - 1;
    int highest_allowed = angle_meas.max + limits.max_angle_error + 1;
    violation = safety_max_limit_check(desired_curvature, highest_allowed, lowest_allowed);
  }
  return violation;
}


static void ford_rx_hook(const CANPacket_t *msg) {
  if (msg->bus == FORD_MAIN_BUS) {
    // Update in motion state from standstill signal
    if (msg->addr == FORD_DesiredTorqBrk) {
      // Signal: VehStop_D_Stat
      vehicle_moving = ((msg->data[3] >> 3) & 0x3U) != 1U;
    }

    // Update vehicle speed
    if (msg->addr == FORD_BrakeSysFeatures) {
      // Signal: Veh_V_ActlBrk
      UPDATE_VEHICLE_SPEED(((msg->data[0] << 8) | msg->data[1]) * 0.01 * KPH_TO_MS);
    }

    // Check vehicle speed against a second source
    if (msg->addr == FORD_EngVehicleSpThrottle2) {
      // Disable controls if speeds from ABS and PCM ECUs are too far apart.
      // Signal: Veh_V_ActlEng
      float filtered_pcm_speed = ((msg->data[6] << 8) | msg->data[7]) * 0.01 * KPH_TO_MS;
      speed_mismatch_check(filtered_pcm_speed);
    }

    // Update vehicle yaw rate
    if (msg->addr == FORD_Yaw_Data_FD1) {
      // Signal: VehYaw_W_Actl
      // TODO: we should use the speed which results in the closest angle measurement to the desired angle
      float ford_yaw_rate = (((msg->data[2] << 8U) | msg->data[3]) * 0.0002) - 6.5;
      float current_curvature = ford_yaw_rate / SAFETY_MAX(vehicle_speed.values[0] / VEHICLE_SPEED_FACTOR, 0.1);
      // convert current curvature into units on CAN for comparison with desired curvature
      update_sample(&angle_meas, ROUND(current_curvature * FORD_STEERING_LIMITS.angle_deg_to_can));
    }

    // Update gas pedal
    if (msg->addr == FORD_EngVehicleSpThrottle) {
      // Pedal position: (0.1 * val) in percent
      // Signal: ApedPos_Pc_ActlArb
      gas_pressed = (((msg->data[0] & 0x03U) << 8) | msg->data[1]) > 0U;
    }

    // Update brake pedal and cruise state
    if (msg->addr == FORD_EngBrakeData) {
      // Signal: BpedDrvAppl_D_Actl
      brake_pressed = ((msg->data[0] >> 4) & 0x3U) == 2U;

      // Signal: CcStat_D_Actl
      unsigned int cruise_state = msg->data[1] & 0x07U;
      bool cruise_engaged = (cruise_state == 4U) || (cruise_state == 5U);
      pcm_cruise_check(cruise_engaged);
    }
  }
}

static bool ford_tx_hook(const CANPacket_t *msg) {
  const LongitudinalLimits FORD_LONG_LIMITS = {
    // acceleration cmd limits (used for brakes)
    // Signal: AccBrkTot_A_Rq
    .max_accel = 5641,       //  1.9999 m/s^s
    .min_accel = 4231,       // -3.4991 m/s^2
    .inactive_accel = 5128,  // -0.0008 m/s^2

    // gas cmd limits
    // Signal: AccPrpl_A_Rq & AccPrpl_A_Pred
    .max_gas = 700,          //  2.0 m/s^2
    .min_gas = 450,          // -0.5 m/s^2
    .inactive_gas = 0,       // -5.0 m/s^2
  };

  bool tx = true;

  // Safety check for ACCDATA accel and brake requests
  if (msg->addr == FORD_ACCDATA) {
    // Signal: AccPrpl_A_Rq
    int gas = ((msg->data[6] & 0x3U) << 8) | msg->data[7];
    // Signal: AccPrpl_A_Pred
    int gas_pred = ((msg->data[2] & 0x3U) << 8) | msg->data[3];
    // Signal: AccBrkTot_A_Rq
    int accel = ((msg->data[0] & 0x1FU) << 8) | msg->data[1];
    // Signal: CmbbDeny_B_Actl
    bool cmbb_deny = (msg->data[4] >> 5) & 1U;

    // Signal: AccBrkPrchg_B_Rq & AccBrkDecel_B_Rq
    bool brake_actuation = ((msg->data[6] >> 6) & 1U) || ((msg->data[6] >> 7) & 1U);

    bool violation = false;
    violation |= longitudinal_accel_checks(accel, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas, FORD_LONG_LIMITS);
    violation |= longitudinal_gas_checks(gas_pred, FORD_LONG_LIMITS);

    // Safety check for stock AEB
    violation |= cmbb_deny; // do not prevent stock AEB actuation

    violation |= !get_longitudinal_allowed() && brake_actuation;

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Steering_Data_FD1 button signals
  // Note: Many other signals in this message are not relevant to safety (e.g. blinkers, wiper switches, high beam)
  // which we passthru in OP.
  if (msg->addr == FORD_Steering_Data_FD1) {
    // Violation if resume button is pressed while controls not allowed, or
    // if cancel button is pressed when cruise isn't engaged.
    bool violation = false;
    violation |= ((msg->data[1] >> 0) & 1U) && !cruise_engaged_prev;   // Signal: CcAslButtnCnclPress (cancel)
    violation |= ((msg->data[3] >> 1) & 1U) && !controls_allowed;     // Signal: CcAsllButtnResPress (resume)

    if (violation) {
      tx = false;
    }
  }

  // Safety check for Lane_Assist_Data1 action
  if (msg->addr == FORD_Lane_Assist_Data1) {
    // Do not allow steering using Lane_Assist_Data1 (Lane-Departure Aid).
    // This message must be sent for Lane Centering to work, and can include
    // values such as the steering angle or lane curvature for debugging,
    // but the action (LkaActvStats_D2_Req) must be set to zero.
    unsigned int action = msg->data[0] >> 5;
    if (action != 0U) {
      tx = false;
    }

    // angle2pnw: angle_mode_engaged + shadow_curvature packed into bits with no DBC signal mapped
    // to them (byte4 bit0, bytes 5-6 -- confirmed unused on real F-150 dashcam routes; see
    // fordcan_pnw.py's create_lka_msg for the full layout and rationale). Read directly out of the
    // message being transmitted right now, same as curvature/path_angle elsewhere in this file --
    // no separate CAN ID, no RX round-trip. Read unconditionally (not gated on the action check
    // above) so a bad action byte can't be used to also suppress this read.
    ford_bp_angle_mode_engaged = (msg->data[4] & 0x1U) != 0U;
    ford_bp_shadow_curvature_raw = (int16_t)((msg->data[5] << 8) | msg->data[6]);
  }

  // Safety check for LateralMotionControl action
  if (msg->addr == FORD_LateralMotionControl) {
    // Signal: LatCtl_D_Rq
    bool steer_control_enabled = ((msg->data[4] >> 2) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[0] << 3) | (msg->data[1] >> 5);
    unsigned int raw_curvature_rate = ((msg->data[1] & 0x1FU) << 8) | msg->data[2];
    unsigned int raw_path_angle = (msg->data[3] << 3) | (msg->data[4] >> 5);
    unsigned int raw_path_offset = (msg->data[5] << 2) | (msg->data[6] >> 6);
    // unsigned int raw_ramp_type = (msg->data[6] >> 4) & 0x3U;

    bool violation = false;

    // Check curvature value limits (convert to signed values first)
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.00002) - 0.02
    // So: raw = (physical + 0.02) / 0.00002 = (physical + 0.02) * 50000
    int curvature_min_can = (int)(FORD_CURVATURE_MIN * FORD_STEERING_LIMITS.angle_deg_to_can);
    int curvature_max_can = (int)(FORD_CURVATURE_MAX * FORD_STEERING_LIMITS.angle_deg_to_can);
    violation |= (desired_curvature < curvature_min_can) || (desired_curvature > curvature_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_curvature:%d, curvature_min_can:%d, curvature_max_can:%d, violation: %d\n",
                      desired_curvature, curvature_min_can, curvature_max_can, (int)violation);
    }

    // Check curvature rate value limits (convert to signed values first)
    int desired_curvature_rate = raw_curvature_rate - FORD_INACTIVE_CURVATURE_RATE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 2.5E-007) - 0.001024
    // So: raw = (physical + 0.001024) / 2.5E-007 = (physical + 0.001024) * 4000000
    int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * FORD_CURVATURE_RATE_LIMITS_CAN.angle_deg_to_can);
    int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * FORD_CURVATURE_RATE_LIMITS_CAN.angle_deg_to_can);
    violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_curvature_rate:%d, curvature_rate_min_can:%d, curvature_rate_max_can:%d, violation: %d\n",
                      desired_curvature_rate, curvature_rate_min_can, curvature_rate_max_can, (int)violation);
    }

    // Check path offset value limits (convert to signed values first)
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.01) - 5.12
    // So: raw = (physical + 5.12) / 0.01 = (physical + 5.12) * 100
    int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_path_offset:%d, path_offset_min_can:%d, path_offset_max_can:%d, violation: %d\n",
                      desired_path_offset, path_offset_min_can, path_offset_max_can, (int)violation);
    }

    // Check path angle value limits (convert to signed values first)
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.0005) - 0.5
    // So: raw = (physical + 0.5) / 0.0005 = (physical + 0.5) * 2000
    // angle2pnw: angle mode uses path_angle as the actuator and may swing to the full DBC range,
    // corroborated by ford_bp_angle_mode_engaged (read from Lane_Assist_Data1, see above) so a
    // frame can't unlock this wider range by merely setting curvature to 0. Curvature mode (the
    // default, and the ONLY mode driven so far) always keeps the tight 0.25 cap, including at
    // curvature == 0 (straight driving, or the reset/human-turn frame below).
    float path_angle_min_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MIN : FORD_PATH_ANGLE_MIN;
    float path_angle_max_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MAX : FORD_PATH_ANGLE_MAX;
    int path_angle_min_can = (int)(path_angle_min_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    int path_angle_max_can = (int)(path_angle_max_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: `desired_path_angle:%d, path_angle_min_can:%d, path_angle_max_can:%d, violation: %d\n",
                      desired_path_angle, path_angle_min_can, path_angle_max_can, (int)violation);
    }

    // Check angle error and steer_control_enabled for curvature
    // angle2pnw: angle mode holds curvature pinned at 0 while path_angle does the real steering,
    // so the deviation-vs-measured portion of steer_angle_cmd_checks would eventually trip as the
    // car actually turns (measured curvature moves, commanded curvature doesn't) -- skip applying
    // it when desired_curvature == 0. Still call it to keep desired_angle_last in sync, and
    // path_angle keeps its own checks regardless. steer_angle_cmd_checks also carries the
    // controls_allowed gate every prior mode relied on for every frame; restore that piece
    // explicitly so a steer_control_enabled frame at curvature == 0 can't bypass it. (bp-7.0 used
    // `controls_allowed || controls_allowed_lateral` here for its MADS support; this tree has no
    // MADS, so plain controls_allowed -- strictly narrower -- is substituted.)
    bool curvature_violation = steer_angle_cmd_checks(desired_curvature, steer_control_enabled, FORD_STEERING_LIMITS);
    if (desired_curvature != 0) {
      violation |= curvature_violation;
    } else {
      violation |= steer_control_enabled && !controls_allowed;
    }
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 1. desired_curvature violation: %d\n", (int)violation);
    }

    // angle2pnw: angle mode's own deviation-only check (no ROC -- path_angle_cmd_checks below
    // already rate-limits the real actuator) against shadow_curvature, once angle mode is
    // confirmed engaged via Lane_Assist_Data1. If desired_curvature == 0 but angle mode is NOT
    // confirmed, this is skipped -- that's ordinary curvature mode at zero (straight driving or
    // the reset/human-turn frame below), which needs no shadow-curvature check; it's still
    // bounded by the tight path_angle range above and steer_control_enabled's own checks.
    if ((desired_curvature == 0) && ford_bp_angle_mode_engaged) {
      int shadow_curvature_can = FORD_BP_SHADOW_CURVATURE_TO_CAN(ford_bp_shadow_curvature_raw);
      violation |= ford_shadow_curvature_error_check(shadow_curvature_can, steer_control_enabled, FORD_STEERING_LIMITS);
    }

    // Check path angle rate of change limits. angle2pnw: use the dedicated angle-mode ROC table
    // only when angle mode is confirmed engaged -- see FORD_PATH_ANGLE_LIMITS_ANGLE's comment for
    // why this must be a separate table, not a runtime-widened shared one.
    const AngleSteeringLimits *path_angle_limits = ford_bp_angle_mode_engaged ? &FORD_PATH_ANGLE_LIMITS_ANGLE : &FORD_PATH_ANGLE_LIMITS;
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, *path_angle_limits);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 2. desired_path_angle violation: %d\n", (int)violation);
    }

    // Check path offset rate of change limits
    violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 3. desired_path_offset violation: %d\n", (int)violation);
    }

    // Check curvature rate rate of change limits
    violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, FORD_CURVATURE_RATE_LIMITS_CAN);
    if (test) {
      FORD_SAFETY_DBG("CAN Out: 4. desired_curvature_rate violation: %d\n", (int)violation);
    }

    // Reset latch: activate when both curvature and path_angle are zero (reset/neutral state)
    // This allows smooth ramp-up after human turn detection without blocked messages
    // pnw-hardening (2026-07-11): the reset latch is gated on controls_allowed. Its only legitimate
    // job is to relax rate-of-change checks during the human-turn-reset ramp, which ALWAYS happens
    // while engaged. BluePilot's original set violation=false unconditionally, and because openpilot
    // sends neutral (curvature==0 && path_angle==0) frames continuously WHILE DISENGAGED, the latch
    // was ~permanently armed when disengaged -> a full bypass of controls_allowed (a buggy process
    // could steer while "off" and the panda would allow it). Forcing the counter to 0 whenever
    // !controls_allowed makes the disengaged-steering block fully enforced again, with ZERO loss of
    // the engaged ramp behavior. (Proven by test_reset_latch_blocked_when_disengaged.)
    if (!controls_allowed) {
      reset_bypass_latch_counter = 0;                        // disengaged: latch inert, full checks
    } else if ((desired_curvature == 0) && (desired_path_angle == 0)) {
      // Reset detected, activate latch for ramp period (engaged only)
      reset_bypass_latch_counter = RESET_BYPASS_LATCH_DURATION;
      violation = false;  // Immediate bypass for reset state
    } else if (reset_bypass_latch_counter > 0) {
      // Latch active, allow bypass during ramp-up period
      reset_bypass_latch_counter--;
      violation = false;
    }

    if (violation) {
      tx = false;
    }
  }

  // Safety check for LateralMotionControl2 action
  if (msg->addr == FORD_LateralMotionControl2) {
    static const AngleSteeringLimits FORD_CANFD_STEERING_LIMITS = FORD_LIMITS(true);

    // Signal: LatCtl_D2_Rq
    bool steer_control_enabled = ((msg->data[0] >> 4) & 0x7U) != 0U;
    unsigned int raw_curvature = (msg->data[2] << 3) | (msg->data[3] >> 5);
    unsigned int raw_curvature_rate = (msg->data[6] << 3) | (msg->data[7] >> 5);
    unsigned int raw_path_angle = ((msg->data[3] & 0x1FU) << 6) | (msg->data[4] >> 2);
    unsigned int raw_path_offset = ((msg->data[4] & 0x3U) << 8) | msg->data[5];
    // unsigned int raw_ramp_type = (msg->data[0] >> 1) & 0x3U;  // Extract bits 1-2 from byte 0

    bool violation = false;

    // Check curvature value limits (convert to signed values first)
    int desired_curvature = raw_curvature - FORD_INACTIVE_CURVATURE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.00002) - 0.02
    // So: raw = (physical + 0.02) / 0.00002 = (physical + 0.02) * 50000
    int curvature_min_can = (int)(FORD_CURVATURE_MIN * FORD_STEERING_LIMITS.angle_deg_to_can);
    int curvature_max_can = (int)(FORD_CURVATURE_MAX * FORD_STEERING_LIMITS.angle_deg_to_can);
    violation |= (desired_curvature < curvature_min_can) || (desired_curvature > curvature_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_curvature: %d, curvature_min_can: %d, curvature_max_can: %d, violation: %d\n",
                      desired_curvature, curvature_min_can, curvature_max_can, (int)violation);
    }

    // Check curvature rate value limits (convert to signed values first)
    int desired_curvature_rate = raw_curvature_rate - FORD_CANFD_INACTIVE_CURVATURE_RATE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 1E-006) - 0.001024
    // So: raw = (physical + 0.001024) / 1E-006 = (physical + 0.001024) * 1000000
    int curvature_rate_min_can = (int)(FORD_CURVATURE_RATE_MIN * FORD_CURVATURE_RATE_LIMITS_CANFD.angle_deg_to_can);
    int curvature_rate_max_can = (int)(FORD_CURVATURE_RATE_MAX * FORD_CURVATURE_RATE_LIMITS_CANFD.angle_deg_to_can);
    violation |= (desired_curvature_rate < curvature_rate_min_can) || (desired_curvature_rate > curvature_rate_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_curvature_rate: %d, curvature_rate_min_can: %d, curvature_rate_max_can: %d, violation: %d\n",
                      desired_curvature_rate, curvature_rate_min_can, curvature_rate_max_can, (int)violation);
    }

    // Check path offset value limits (convert to signed values first)
    int desired_path_offset = raw_path_offset - FORD_INACTIVE_PATH_OFFSET;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.01) - 5.12
    // So: raw = (physical + 5.12) / 0.01 = (physical + 5.12) * 100
    int path_offset_min_can = (int)(FORD_PATH_OFFSET_MIN * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    int path_offset_max_can = (int)(FORD_PATH_OFFSET_MAX * FORD_PATH_OFFSET_LIMITS.angle_deg_to_can);
    violation |= (desired_path_offset < path_offset_min_can) || (desired_path_offset > path_offset_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_path_offset: %d, path_offset_min_can: %d, path_offset_max_can: %d, violation: %d\n",
                      desired_path_offset, path_offset_min_can, path_offset_max_can, (int)violation);
    }

    // Check path angle value limits (convert to signed values first)
    int desired_path_angle = raw_path_angle - FORD_INACTIVE_PATH_ANGLE;
    // Convert physical limits to CAN units using DBC scaling: physical = (raw * 0.0005) - 0.5
    // So: raw = (physical + 0.5) / 0.0005 = (physical + 0.5) * 2000
    // angle2pnw: see the identical comment in the LMC (non-CANFD) block above.
    float path_angle_min_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MIN : FORD_PATH_ANGLE_MIN;
    float path_angle_max_phys = ford_bp_angle_mode_engaged ? FORD_DBC_PATH_ANGLE_MAX : FORD_PATH_ANGLE_MAX;
    int path_angle_min_can = (int)(path_angle_min_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    int path_angle_max_can = (int)(path_angle_max_phys * FORD_PATH_ANGLE_LIMITS.angle_deg_to_can);
    violation |= (desired_path_angle < path_angle_min_can) || (desired_path_angle > path_angle_max_can);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: `desired_path_angle: %d, path_angle_min_can: %d, path_angle_max_can: %d, violation: %d\n",
                      desired_path_angle, path_angle_min_can, path_angle_max_can, (int)violation);
    }

    // Check angle error and steer_control_enabled for curvature
    // angle2pnw: see the identical comment in the LMC (non-CANFD) block above.
    bool curvature_violation = steer_angle_cmd_checks(desired_curvature, steer_control_enabled, FORD_CANFD_STEERING_LIMITS);
    if (desired_curvature != 0) {
      violation |= curvature_violation;
    } else {
      violation |= steer_control_enabled && !controls_allowed;
    }
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 1. desired_curvature violation: %d\n", (int)violation);
    }

    // angle2pnw: shadow-curvature deviation check -- see the identical comment in the LMC block above.
    if ((desired_curvature == 0) && ford_bp_angle_mode_engaged) {
      int shadow_curvature_can = FORD_BP_SHADOW_CURVATURE_TO_CAN(ford_bp_shadow_curvature_raw);
      violation |= ford_shadow_curvature_error_check(shadow_curvature_can, steer_control_enabled, FORD_CANFD_STEERING_LIMITS);
    }

    // Check path angle rate of change limits. angle2pnw: dedicated angle-mode ROC table, see LMC block above.
    const AngleSteeringLimits *path_angle_limits = ford_bp_angle_mode_engaged ? &FORD_PATH_ANGLE_LIMITS_ANGLE : &FORD_PATH_ANGLE_LIMITS;
    violation |= path_angle_cmd_checks(desired_path_angle, steer_control_enabled, *path_angle_limits);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 2. desired_path_angle violation: %d\n", (int)violation);
    }

    // Check path offset rate of change limits
    violation |= path_offset_cmd_checks(desired_path_offset, steer_control_enabled, FORD_PATH_OFFSET_LIMITS);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 3. desired_path_offset violation: %d\n", (int)violation);
    }

    // Check curvature rate rate of change limits
    violation |= curvature_rate_cmd_checks(desired_curvature_rate, steer_control_enabled, FORD_CURVATURE_RATE_LIMITS_CANFD);
    if (test) {
      FORD_SAFETY_DBG("CANFD Out: 4. desired_curvature_rate violation: %d\n", (int)violation);
    }

    // Reset latch: activate when both curvature and path_angle are zero (reset/neutral state)
    // This allows smooth ramp-up after human turn detection without blocked messages
    // pnw-hardening (2026-07-11): the reset latch is gated on controls_allowed. Its only legitimate
    // job is to relax rate-of-change checks during the human-turn-reset ramp, which ALWAYS happens
    // while engaged. BluePilot's original set violation=false unconditionally, and because openpilot
    // sends neutral (curvature==0 && path_angle==0) frames continuously WHILE DISENGAGED, the latch
    // was ~permanently armed when disengaged -> a full bypass of controls_allowed (a buggy process
    // could steer while "off" and the panda would allow it). Forcing the counter to 0 whenever
    // !controls_allowed makes the disengaged-steering block fully enforced again, with ZERO loss of
    // the engaged ramp behavior. (Proven by test_reset_latch_blocked_when_disengaged.)
    if (!controls_allowed) {
      reset_bypass_latch_counter = 0;                        // disengaged: latch inert, full checks
    } else if ((desired_curvature == 0) && (desired_path_angle == 0)) {
      // Reset detected, activate latch for ramp period (engaged only)
      reset_bypass_latch_counter = RESET_BYPASS_LATCH_DURATION;
      violation = false;  // Immediate bypass for reset state
    } else if (reset_bypass_latch_counter > 0) {
      // Latch active, allow bypass during ramp-up period
      reset_bypass_latch_counter--;
      violation = false;
    }

    if (violation) {
      tx = false;
    }
    if(test) {
      FORD_SAFETY_DBG("CANFD Out - final: violation: %d\n", (int)violation);
    }
  }

  return tx;
}

static safety_config ford_init(uint16_t param) {
  // warning: quality flags are not yet checked in openpilot's CAN parser,
  // this may be the cause of blocked messages
  static RxCheck ford_rx_checks[] = {
    {.msg = {{FORD_BrakeSysFeatures, 0, 8, 50U, .max_counter = 15U}, { 0 }, { 0 }}},
    // FORD_EngVehicleSpThrottle2 has a counter that either randomly skips or by 2, likely ECU bug
    // Some hybrid models also experience a bug where this checksum mismatches for one or two frames under heavy acceleration with ACC
    // It has been confirmed that the Bronco Sport's camera only disallows ACC for bad quality flags, not counters or checksums, so we match that
    {.msg = {{FORD_EngVehicleSpThrottle2, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_Yaw_Data_FD1, 0, 8, 100U, .max_counter = 255U}, { 0 }, { 0 }}},
    // These messages have no counter or checksum
    {.msg = {{FORD_EngBrakeData, 0, 8, 10U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_EngVehicleSpThrottle, 0, 8, 100U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
    {.msg = {{FORD_DesiredTorqBrk, 0, 8, 50U, .ignore_checksum = true, .ignore_counter = true, .ignore_quality_flag = true}, { 0 }, { 0 }}},
  };

  #define FORD_COMMON_TX_MSGS \
    {FORD_Steering_Data_FD1, 0, 8, .check_relay = false}, \
    {FORD_Steering_Data_FD1, 2, 8, .check_relay = false}, \
    {FORD_ACCDATA_3, 0, 8, .check_relay = true},          \
    {FORD_Lane_Assist_Data1, 0, 8, .check_relay = true},  \
    {FORD_IPMA_Data, 0, 8, .check_relay = true},          \

  static const CanMsg FORD_CANFD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_CANFD_STOCK_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_LateralMotionControl2, 0, 8, .check_relay = true},
  };

  static const CanMsg FORD_LONG_TX_MSGS[] = {
    FORD_COMMON_TX_MSGS
    {FORD_ACCDATA, 0, 8, .check_relay = true},
    {FORD_LateralMotionControl, 0, 8, .check_relay = true},
  };

  const uint16_t FORD_PARAM_CANFD = 2;
  const bool ford_canfd = GET_FLAG(param, FORD_PARAM_CANFD);

  bool ford_longitudinal = false;

#ifdef ALLOW_DEBUG
  const uint16_t FORD_PARAM_LONGITUDINAL = 1;
  ford_longitudinal = GET_FLAG(param, FORD_PARAM_LONGITUDINAL);
#endif

  // Longitudinal is the default for CAN, and optional for CAN FD w/ ALLOW_DEBUG
  ford_longitudinal = !ford_canfd || ford_longitudinal;

  safety_config ret;
  if (ford_canfd) {
    ret = ford_longitudinal ? BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_LONG_TX_MSGS) : \
                              BUILD_SAFETY_CFG(ford_rx_checks, FORD_CANFD_STOCK_TX_MSGS);
  } else {
    ret = BUILD_SAFETY_CFG(ford_rx_checks, FORD_LONG_TX_MSGS);
  }
  return ret;
}

const safety_hooks ford_hooks = {
  .init = ford_init,
  .rx = ford_rx_hook,
  .tx = ford_tx_hook,
  .get_counter = ford_get_counter,
  .get_checksum = ford_get_checksum,
  .compute_checksum = ford_compute_checksum,
  .get_quality_flag_valid = ford_get_quality_flag_valid,
};

/**
 * mads2pnw -- "lateral survives a brake press".
 *
 * Ported from sunnypilot's MADS (Modular Assistive Driving System), by way of
 * sunny/bluepilot `vin-lightning-2024-25`
 * (opendbc/safety/sunnypilot/{mads.h,mads_declarations.h}).
 *
 * Original: Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other
 * contributors. Licensed under the MIT License; see sunnypilot's LICENSE.md.
 *
 * ===========================================================================
 * WHAT THIS IS, AND WHY IT IS NOT A WEAKENING OF THE BRAKE CHECK
 * ===========================================================================
 * `generic_rx_checks()` in safety.h clears `controls_allowed` on the rising
 * edge of the brake pedal. That check is UNTOUCHED by this port, in every
 * path, for every car. What this adds is a SECOND, PARALLEL authority flag --
 * `controls_allowed_lateral` -- that only the LATERAL tx gates consult, as
 * `(controls_allowed || controls_allowed_lateral)`. Longitudinal, gas, and
 * every other gate keep looking at `controls_allowed` alone, so a brake press
 * still removes longitudinal authority exactly as before.
 *
 * The flag is opt-in three times over:
 *   1. it can only ever be set when `ALT_EXP_ENABLE_MADS` is in the
 *      alternative_experience bitfield the device pushes to the panda, which
 *      openpilot only sets for the F-150 Lightning with the (default-OFF)
 *      `MadsLateralOnBrake` toggle on;
 *   2. `m_update_control_state()` gates the write on `system_enabled`, so a
 *      build with MADS disabled can never flip the global on;
 *   3. `mads_set_system_state()` (called from `set_safety_hooks`) re-inits the
 *      whole struct and clears the flag on every safety-mode change.
 *
 * ===========================================================================
 * DELIBERATE DEVIATIONS FROM THE SUNNYPILOT/BLUEPILOT SOURCE
 * ===========================================================================
 *   - Both non-openpilot ENGAGE sources are dropped: the MADS BUTTON
 *     (`mads_button_press`) and the ACC-main RISING edge. No car in THIS tree
 *     produces a MADS button (the bluepilot source does parse one --
 *     sunny/bluepilot/.../modes/ford.h -- it simply was never ported here), and
 *     `acc_main_on` is written here only as a REVOKE source -- but with no
 *     openpilot-side MADS state machine to arbitrate them, an engage source
 *     openpilot did not ask for is exactly the wrong thing to keep. The ONLY
 *     way `controls_allowed_lateral` is ever set here is the rising edge of
 *     openpilot's own `controls_allowed`. The ACC-main FALLING edge and the
 *     steering-override path ARE kept -- those are DISENGAGE sources.
 *   - ADDED beyond upstream: `MADS_DISENGAGE_REASON_OP_DISENGAGE`. openpilot
 *     losing controls for any reason OTHER than the brake also ends lateral
 *     authority. Upstream can leave the latch standing (it has an
 *     openpilot-side state machine and the heartbeat watchdog); this tree has
 *     neither, so a CANCEL press would otherwise leave the panda permitting
 *     lateral forever.
 *   - `mads_state_update()` is called from `safety_rx_hook()`, immediately
 *     after `generic_rx_checks()` -- ONCE per received CAN message. bluepilot
 *     calls it from inside `stock_ecu_check()`, which runs once per
 *     relay-checked tx_msg per rx message (i.e. 0..N times per message, N
 *     depending on the safety config). That makes its edge detector fire a
 *     variable number of times per frame, which is not what an edge detector
 *     wants. The placement here is once-per-frame and deterministic.
 *   - `mads_set_alternative_experience()` IS actually called from
 *     `set_safety_hooks()`. In bluepilot it is called ONLY from the test
 *     harness, so MADS there is dead code in real firmware
 *     (`grep -rn mads panda/board/` finds nothing). Without this call
 *     `system_enabled` is never true and the flag can never be set.
 *   - `mads_exit_controls()` is additionally called when a safety RX message
 *     fails its checksum/counter/quality checks (`is_msg_valid`). Upstream
 *     clears only `controls_allowed` there. Clearing lateral too is a
 *     hardening in the safe direction; it is tested.
 *   - `mads_heartbeat_engaged_check()` / `heartbeat_engaged_mads` ARE now
 *     ported (madsheartbeat2pnw), matching upstream byte for byte. They were
 *     deliberately left out of the first mads2pnw commit because the panda had
 *     no way to receive the flag; `pnw-panda madsheartbeat2pnw` adds it
 *     (`board/main_comms.h` case 0xf3 -> `req->param2`, and the 1 Hz call in
 *     `board/main.c`), and `pandad` sends `madsState.enabled` in param2.
 *
 *     WHAT THE WATCHDOG IS FOR, precisely. It is NOT the "openpilot died"
 *     case -- that one is already covered: no 0xf3 at all -> heartbeat_counter
 *     climbs -> panda drops to SAFETY_SILENT -> set_safety_hooks ->
 *     mads_set_system_state(false,..) -> m_mads_state_init() ->
 *     controls_allowed_lateral = false. The watchdog covers the case where
 *     openpilot is STILL TALKING but has stopped intending lateral (selfdrived
 *     restarted, madsState went stale/invalid, MADS turned itself off): within
 *     3 s of param2 reading 0 while the latch is up, the panda revokes it.
 *
 *     DIRECTION. The check can only ever REVOKE. It contains no path that sets
 *     `controls_allowed_lateral` true, and `heartbeat_engaged_mads` defaults to
 *     false, so a panda that never hears from a MADS-aware openpilot -- an old
 *     device, a crashed pandad, param2 always 0 -- revokes after 3 ticks and
 *     stays revoked. Missing == revoke. There is no "grant" edge anywhere in
 *     this function.
 */

#pragma once

// ===============================
// Type Definitions and Enums
// ===============================

typedef enum __attribute__((packed)) {
  MADS_EDGE_NO_CHANGE = 0,  ///< No state change detected
  MADS_EDGE_RISING = 1,     ///< State changed from false to true
  MADS_EDGE_FALLING = 2     ///< State changed from true to false
} EdgeTransition;

typedef enum __attribute__((packed)) {
  MADS_DISENGAGE_REASON_NONE = 0,                         ///< No disengagement
  MADS_DISENGAGE_REASON_BRAKE = 1,                        ///< Brake pedal pressed
  MADS_DISENGAGE_REASON_LAG = 2,                          ///< Lagging or invalid safety message
  MADS_DISENGAGE_REASON_ACC_MAIN_OFF = 8,                 ///< ACC system turned off
  MADS_DISENGAGE_REASON_OP_DISENGAGE = 16,                ///< openpilot lost controls for a non-brake reason
  MADS_DISENGAGE_REASON_HEARTBEAT_ENGAGED_MISMATCH = 32,  ///< openpilot stopped asking for lateral (0xf3 param2)
  MADS_DISENGAGE_REASON_STEERING_DISENGAGE = 64,          ///< Steering override/disengage
} DisengageReason;

// ===============================
// Constants and Defines
// ===============================

#define ALT_EXP_ENABLE_MADS 1024
#define ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE 2048
#define ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE 4096

// ===============================
// Data Structures
// ===============================

typedef struct {
  DisengageReason active_reason;    // The reason that actually disengaged lateral controls
  DisengageReason pending_reasons;  // All conditions that would've prevented engagement while disengaged
} DisengageState;

typedef struct {
  EdgeTransition transition;
  bool current : 1;
  bool previous : 1;
} BinaryStateTracking;

// madsbrakerace2pnw: how long after an OP_DISENGAGE revoke a brake may still RE-LATCH lateral.
//
// MEASURED ON THE TRUCK 2026-09-06. The driver braked with "Disengage on brake" OFF and everything
// disengaged. On the Ford BOTH authorities read brake and cruise from the SAME 10 Hz message
// (EngBrakeData 0x165), and the PCM drops cruise on the pedal FASTER than BpedDrvAppl reports it --
// so `op_controls_allowed` falls with `braking.current` still false and the revoke below fires
// before the brake exists. The brake then lands on the NEXT 0x165 frame, up to ~100 ms later.
//
// 300 ms = three 10 Hz frames of margin. Deliberately a TIME bound, not a tick count:
// mads_state_update() runs from safety_rx_hook() once per RECEIVED CAN MESSAGE, so a tick counter
// would expire in a few milliseconds of ordinary bus traffic.
#define MADS_BRAKE_RELATCH_US 300000U

typedef struct {
  BinaryStateTracking acc_main;
  BinaryStateTracking op_controls_allowed;
  BinaryStateTracking braking;
  BinaryStateTracking mads_steering_disengage;

  DisengageState current_disengage;

  // madsbrakerace2pnw: microsecond timestamp of the last OP_DISENGAGE revoke, plus an EXPLICIT
  // pending flag. The flag is not optional: microsecond_timer_get() legitimately returns 0 (the
  // libsafety harness starts there, and the hardware timer wraps through it), so overloading 0 as
  // "nothing pending" would silently disarm the window exactly at t=0 and on every wrap.
  // Authority is ALWAYS revoked immediately (never held longer than before this change); this only
  // bounds how long a late brake may restore it.
  uint32_t op_disengage_ts;
  bool op_disengage_pending;

  bool system_enabled : 1;
  bool disengage_lateral_on_brake : 1;
  bool pause_lateral_on_brake : 1;
  bool controls_requested_lateral : 1;
} MADSState;

// ===============================
// Global Variables
// ===============================

extern MADSState m_mads_state;

extern bool controls_allowed_lateral;

// State for the LATERAL heartbeat watchdog (madsheartbeat2pnw). heartbeat_engaged_mads is written
// ONLY by the panda board code from heartbeat USB command 0xf3 param2 -- "openpilot still intends
// lateral authority". It is the exact mirror of `heartbeat_engaged` (0xf3 param1) in safety.h.
extern bool heartbeat_engaged_mads;
extern uint32_t heartbeat_engaged_mads_mismatches;

// ===============================
// External Function Declarations
// ===============================

extern void mads_set_system_state(bool enabled, bool disengage_lateral_on_brake, bool pause_lateral_on_brake);
extern void mads_set_alternative_experience(const int *mode);
extern void mads_state_update(bool op_acc_main, bool op_allowed, bool is_braking, bool steering_disengage);
extern void mads_exit_controls(DisengageReason reason);
extern void mads_heartbeat_engaged_check(void);

// ===============================
// Inline Function Implementations, must be included in the header file to comply with MISRA-C:2012 Rule 8.10
// These are really only used internally.
// ===============================
extern EdgeTransition m_get_edge_transition(bool current, bool last);
extern void m_mads_state_init(void);
extern void m_update_binary_state(BinaryStateTracking *state);
extern void m_update_control_state(void);

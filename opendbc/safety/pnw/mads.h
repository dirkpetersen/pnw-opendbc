/**
 * mads2pnw -- see mads_declarations.h for the full rationale, the opt-in
 * chain, and the deliberate deviations from the sunnypilot/bluepilot source.
 *
 * Original: Copyright (c) 2021-, Haibin Wen, sunnypilot, and a number of other
 * contributors. Licensed under the MIT License; see sunnypilot's LICENSE.md.
 */

#pragma once

#include "opendbc/safety/pnw/mads_declarations.h"

// ===============================
// Global Variables
// ===============================

MADSState m_mads_state;

bool controls_allowed_lateral = false;

// ===============================
// State Update Helpers
// ===============================

inline EdgeTransition m_get_edge_transition(const bool current, const bool last) {
  EdgeTransition state;

  if (current && !last) {
    state = MADS_EDGE_RISING;
  } else if (!current && last) {
    state = MADS_EDGE_FALLING;
  } else {
    state = MADS_EDGE_NO_CHANGE;
  }

  return state;
}

inline void m_mads_state_init(void) {
  m_mads_state.system_enabled = false;
  m_mads_state.disengage_lateral_on_brake = false;
  m_mads_state.pause_lateral_on_brake = false;

  m_mads_state.acc_main.current = false;
  m_mads_state.acc_main.previous = false;
  m_mads_state.acc_main.transition = MADS_EDGE_NO_CHANGE;

  m_mads_state.op_controls_allowed.current = false;
  m_mads_state.op_controls_allowed.previous = false;
  m_mads_state.op_controls_allowed.transition = MADS_EDGE_NO_CHANGE;

  m_mads_state.braking.current = false;
  m_mads_state.braking.previous = false;
  m_mads_state.braking.transition = MADS_EDGE_NO_CHANGE;

  m_mads_state.mads_steering_disengage.current = false;
  m_mads_state.mads_steering_disengage.previous = false;
  m_mads_state.mads_steering_disengage.transition = MADS_EDGE_NO_CHANGE;

  m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
  m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;

  m_mads_state.controls_requested_lateral = false;
  controls_allowed_lateral = false;
}

inline void m_update_binary_state(BinaryStateTracking *state) {
  state->transition = m_get_edge_transition(state->current, state->previous);
  state->previous = state->current;
}

/**
 * @brief Updates the MADS lateral-authority state from the current system conditions.
 */
inline void m_update_control_state(void) {
  bool allowed = true;

  // The ONE engage source: openpilot itself becoming engaged. The MADS button and the ACC-main
  // RISING edge are both dropped (see mads_declarations.h) -- an engage source that openpilot did
  // not ask for is exactly what we do not want when there is no openpilot-side MADS state machine
  // to arbitrate it. ACC-main FALLING is kept below as a DISENGAGE source.
  if (m_mads_state.op_controls_allowed.transition == MADS_EDGE_RISING) {
    m_mads_state.controls_requested_lateral = true;
  }

  // Primary control blockers - these prevent any further control processing
  if (m_mads_state.acc_main.transition == MADS_EDGE_FALLING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_ACC_MAIN_OFF);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (m_mads_state.mads_steering_disengage.transition == MADS_EDGE_RISING) {
    mads_exit_controls(MADS_DISENGAGE_REASON_STEERING_DISENGAGE);
    allowed = false;  // No matter what, no further control processing on this cycle
  }

  if (m_mads_state.disengage_lateral_on_brake && (m_mads_state.braking.transition == MADS_EDGE_RISING)) {
    mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
    allowed = false;
  }

  // pnw addition, NOT in upstream sunnypilot: openpilot losing controls for ANY reason other than
  // the brake ends lateral authority too. Upstream can leave the latch standing because it has an
  // openpilot-side MADS state machine plus the heartbeat_engaged_mads watchdog to take it down;
  // this tree has neither (yet), so without this a CANCEL press -- or any other non-brake
  // disengage -- would leave the panda permitting lateral indefinitely while the driver believes
  // openpilot is off. The brake case is precisely the one the driver asked to survive, so it (and
  // only it) is excluded. See test_mads_cancel_drops_lateral / the brake tests either side of it.
  if ((m_mads_state.op_controls_allowed.transition == MADS_EDGE_FALLING) && !m_mads_state.braking.current) {
    mads_exit_controls(MADS_DISENGAGE_REASON_OP_DISENGAGE);
    allowed = false;
  }

  // Secondary control conditions - only checked if primary conditions don't block further control processing
  if (allowed && m_mads_state.pause_lateral_on_brake) {
    // Brake rising edge immediately blocks controls
    // Brake release might request controls if brake was the ONLY reason for disengagement
    if (m_mads_state.braking.transition == MADS_EDGE_RISING) {
      mads_exit_controls(MADS_DISENGAGE_REASON_BRAKE);
      allowed = false;
    } else if ((m_mads_state.braking.transition == MADS_EDGE_FALLING) &&
               (m_mads_state.current_disengage.active_reason == MADS_DISENGAGE_REASON_BRAKE) &&
               (m_mads_state.current_disengage.pending_reasons == MADS_DISENGAGE_REASON_BRAKE)) {
      m_mads_state.controls_requested_lateral = true;
    } else if (m_mads_state.braking.current) {
      allowed = false;
    } else {
    }
  }

  // Process control request if conditions allow. The write is gated on system_enabled so that a
  // build with MADS disabled (every car but the opted-in Lightning) can never flip the global on.
  if (allowed && m_mads_state.system_enabled && m_mads_state.controls_requested_lateral && !controls_allowed_lateral) {
    m_mads_state.controls_requested_lateral = false;
    controls_allowed_lateral = true;
    m_mads_state.current_disengage.active_reason = MADS_DISENGAGE_REASON_NONE;
    m_mads_state.current_disengage.pending_reasons = MADS_DISENGAGE_REASON_NONE;
  }
}

// ===============================
// Function Implementations
// ===============================

inline void mads_set_alternative_experience(const int *mode) {
  const bool mads_enabled = (*mode & ALT_EXP_ENABLE_MADS) != 0;
  const bool disengage_lateral_on_brake = (*mode & ALT_EXP_MADS_DISENGAGE_LATERAL_ON_BRAKE) != 0;
  const bool pause_lateral_on_brake = (*mode & ALT_EXP_MADS_PAUSE_LATERAL_ON_BRAKE) != 0;

  mads_set_system_state(mads_enabled, disengage_lateral_on_brake, pause_lateral_on_brake);
}

extern inline void mads_set_system_state(const bool enabled, const bool disengage_lateral_on_brake, const bool pause_lateral_on_brake) {
  m_mads_state_init();
  m_mads_state.system_enabled = enabled;
  m_mads_state.disengage_lateral_on_brake = disengage_lateral_on_brake;
  m_mads_state.pause_lateral_on_brake = pause_lateral_on_brake;
}

inline void mads_exit_controls(const DisengageReason reason) {
  // Always track this as a pending reason
  m_mads_state.current_disengage.pending_reasons |= reason;

  if (controls_allowed_lateral) {
    m_mads_state.current_disengage.active_reason = reason;
    m_mads_state.controls_requested_lateral = false;
    controls_allowed_lateral = false;
  }
}

inline void mads_state_update(const bool op_acc_main, const bool op_allowed, const bool is_braking, const bool _steering_disengage) {
  m_mads_state.acc_main.current = op_acc_main;
  m_mads_state.op_controls_allowed.current = op_allowed;
  m_mads_state.braking.current = is_braking;
  m_mads_state.mads_steering_disengage.current = _steering_disengage;

  m_update_binary_state(&m_mads_state.acc_main);
  m_update_binary_state(&m_mads_state.op_controls_allowed);
  m_update_binary_state(&m_mads_state.braking);
  m_update_binary_state(&m_mads_state.mads_steering_disengage);

  m_update_control_state();
}

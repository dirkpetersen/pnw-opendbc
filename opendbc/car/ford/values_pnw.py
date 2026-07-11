"""
fordsafety2pnw — Ford lateral control constants, ported from BluePilot (alan-polk),
bluepilotdev/bp-dev opendbc_repo/opendbc/sunnypilot/car/ford/values_ext.py.

Only the lateral-control constants are ported (BUTTONS / device-mount helpers in the BP source
belong to BP features not present in this tree).
"""

from opendbc.car.lateral import AngleSteeringLimits

# BluePilot: Max curvature for steering command (m^-1), from DBC file limits
CURVATURE_MAX = 0.02

# BluePilot: Curvature rate limits — 3-point breakpoints for smoother lateral control.
# Upstream opendbc uses 2-point ([5, 25]) with more conservative values.
# These allow higher rates at low speed for responsiveness, lower rates at mid-speed
# for comfort, and very low rates at highway speed for stability.
#
# Control (Python) uses stricter windup than unwind so OP stays inside panda when apply_std
# picks the wrong table vs steer_angle_cmd_checks. Safety firmware uses looser symmetric ROCs
# (former "down" table for both up/down) — see opendbc/safety/modes/ford.h FORD_LIMITS.
# Tests: test_ford.py ANGLE_RATE_* match ford.h, not the stricter BP_ANGLE_LIMITS up row.
_BP_ANGLE_RATE_UP = ([5, 16, 25], [0.0025, 0.0012, 0.00008])
_BP_ANGLE_RATE_DOWN = ([5, 16, 25], [0.0025, 0.0014, 0.00018])
BP_ANGLE_LIMITS = AngleSteeringLimits(
  0.02,  # Max curvature for steering command, m^-1
  _BP_ANGLE_RATE_UP,
  _BP_ANGLE_RATE_DOWN,
)

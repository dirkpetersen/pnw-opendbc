"""
pnw_vehicle (opendbc side) — capability view over CarParams for pnw features living in opendbc.

Same rule as the openpilot-layer selfdrive/controls/lib/pnw_vehicle.py (driver directive
2026-07-11): feature code asks about CAPABILITIES, never about fingerprints. This module is the
opendbc-importable mirror (opendbc cannot import openpilot.*); keep the two in sync when adding
cars. Pure + defensive: works with structs.CarParams or None.
"""


class PnwVehicle:
  def __init__(self, CP):
    fp = str(getattr(CP, 'carFingerprint', '') or '') if CP is not None else ''

    # openpilot owns gas/brake (op-long / alpha-long active)
    self.op_long: bool = bool(getattr(CP, 'openpilotLongitudinalControl', False)) if CP is not None else False

    # stock-ACC set-speed steering via SET +/- taps on the SCCM stream (icbm_pnw executor)
    self.stock_acc_buttons: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # ICBM executor runs: buttons available AND openpilot does NOT own longitudinal
    self.icbm: bool = self.stock_acc_buttons and not self.op_long

    # predicted-curvature blend (fordlat2pnw): Ford curvature-only lateral cars where the
    # BluePilot-derived turn-exit blend is validated
    self.pc_blend: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # human-turn reset (fordlat_pnw.HumanTurnHold): flush commanded curvature during a sustained
    # manual turn so release ramps from ~0 (kills the post-override other-lane lurch)
    self.ht_reset: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # fordsafety2pnw: BluePilot (alan-polk) 4-signal lateral control (curvature + curvature_rate +
    # path_offset + path_angle via LateralCurvExt). REQUIRES the matching 4-signal panda safety
    # (opendbc/safety/modes/ford.h from the fordsafety2pnw port) — with stock ford safety the
    # nonzero curvature_rate would be blocked and lateral would go dead. When this path is active
    # it OWNS lateral: LateralCurvExt has its own predicted-curvature blend and human-turn reset,
    # so the standalone pc_blend / ht_reset helpers are bypassed by the carcontroller.
    self.four_signal_lat: bool = fp == "FORD_F_150_LIGHTNING_MK1"

    # fordlong2pnw: BluePilot (alan-polk) highway follow control (LongitudinalExt) — shapes
    # gas/accel by lead state above ~50 mph, split brake/precharge hysteresis. Only meaningful
    # when openpilot owns longitudinal, so gate on op_long: inert until Alpha Long is enabled.
    self.bp_long_follow: bool = self.op_long and fp == "FORD_F_150_LIGHTNING_MK1"

    # angle2pnw (2026-07-18): BluePilot (alan-polk) bp-7.0 angle-primary lateral strategy
    # (LateralAngleExt) — derives path_angle directly from kappa*v*gain instead of the 4-signal
    # curvature stack (see docs/pnw/ANGLE2PNW.md). Mutually exclusive with four_signal_lat;
    # requires the matching ford.h angle-mode safety additions (shadow_curvature cross-check +
    # corroborated wide-range path_angle gate) from the same port.
    #
    # angleenable: angle_lat is the master gate, now driver-flippable via the FordAngleLateral
    # settings toggle (default OFF — deployed behavior is unchanged until the driver opts in).
    # It is GATED on four_signal_lat rather than a bare fingerprint check: four_signal_lat is
    # already the capability that identifies "this car has the flashed 4-signal/angle-mode ford.h
    # panda safety" (see its comment above), so angle_lat can only ever be True on a car that both
    # (a) is the Lightning and (b) is running the matching safety build — the same guarantee the
    # first pass's hardcoded False gave, now conditional on an explicit opt-in instead of always
    # off. The Params() read/import is RUNTIME-GUARDED (opendbc cannot assume openpilot.* is
    # importable on a bare checkout): any failure — bare opendbc, missing param, anything — leaves
    # angle_lat False, matching every other params-gated capability in this tree.
    self.angle_lat: bool = False
    if self.four_signal_lat:
      try:
        from openpilot.common.params import Params
        self.angle_lat = bool(Params().get_bool("FordAngleLateral"))
      except Exception:
        self.angle_lat = False

    # Per-platform path_angle gain defaults (low-curvature, high-curvature), BluePilot bp-7.0
    # values — not user-tunable in BP either, fixed to body style. Populated for every Ford body
    # style (not just the Lightning) because pnw_vehicle is the correct home for ANY
    # carFingerprint-conditioned data, even for a platform we don't currently drive; consumed only
    # when angle_lat is eventually enabled for that platform.
    _canfd_bof_cars = ("FORD_F_150_MK14", "FORD_F_150_LIGHTNING_MK1", "FORD_EXPEDITION_MK4", "FORD_RANGER_MK2")
    _canfd_suv_cars = ("FORD_MUSTANG_MACH_E_MK1", "FORD_ESCAPE_MK4_5")
    if fp in _canfd_bof_cars:
      self.angle_gain: tuple[float, float] = (0.95, 0.95)
    elif fp in _canfd_suv_cars:
      self.angle_gain = (1.00, 1.05)
    else:
      self.angle_gain = (1.00, 1.15)

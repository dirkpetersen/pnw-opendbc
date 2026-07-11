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

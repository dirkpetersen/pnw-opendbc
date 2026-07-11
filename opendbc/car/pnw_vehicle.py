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

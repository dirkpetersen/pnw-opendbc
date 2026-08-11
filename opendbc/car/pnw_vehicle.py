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

    # speedadjust-exec2pnw: the ONE generic capability that gates the shared stock-ACC button-tap
    # executor (icbm_pnw.py) — true whenever this car has stock-ACC buttons AND openpilot does NOT
    # own longitudinal. Car-agnostic by construction (no per-feature fingerprint checks): ANY
    # car-agnostic pnw brain that publishes a {target, ceiling, ts, dir?} mem-param gets slowdowns
    # for free on any car declaring this capability, arbitrated against every other live brain's
    # command by icbm_pnw.arbitrate() — the executor has no notion of "which feature" asked. Today
    # only the Lightning declares it; the fingerprint check lives HERE ONLY (pnw_vehicle's whole job),
    # never in feature/brain code (ces_pnw.py, speedadjust_controller.py, icbm_pnw.py all read only
    # this boolean).
    self.button_management: bool = self.stock_acc_buttons and not self.op_long

    # icbm2pnw: back-compat alias — same condition, kept in case anything still names it `icbm`
    # specifically (the curve brain's own capability reads, e.g. icbm_map_scale/icbm_firm_decel
    # below, are already unconditional / neutral-by-default and don't depend on this alias).
    self.icbm: bool = self.button_management

    # speedadjust-exec2pnw: same capability under the feature's own name too, for any call site that
    # wants to name the feature rather than the umbrella mechanism.
    self.speedadjust_buttons: bool = self.button_management

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

    # angle2pnw-faithful2 (2026-07-19): BluePilot bp-7.0 angle-primary lateral strategy
    # (LateralAngleExt) — Alan Polk's "Return of Angle Control" design (bluepilot.dev, 2026-07-15;
    # spec archived in drives/2026-07-18/lightning-angle-steering/). Sets c2 (curvature) and c3
    # (curvature_rate) to ZERO on the LMC/LMC2 wire and derives path_angle (c1) directly as
    # path_angle = kappa_cmd * v_ego * curvature_factor(...) — see lateral_angle_pnw.py for the
    # full, faithfully-ported control law. Independent of four_signal_lat (does not require the
    # 4-signal curvature_rate panda safety); requires only the angle-mode value-range + ROC
    # additions carried into this port's opendbc/safety/modes/ford.h.
    #
    # angle_lat is the master gate, driver-flippable via the FordAngleLateral settings toggle
    # (default OFF — common/params_keys.h registers it PERSISTENT/BOOL/"0"). Gated directly on
    # carFingerprint here (not on four_signal_lat, a DIFFERENT and unrelated capability for the
    # old 4-signal curvature-rate path) because angle mode's panda safety is a self-contained
    # addition that does not depend on four_signal_lat's curvature_rate machinery being flashed.
    # Params() read/import is RUNTIME-GUARDED (opendbc cannot assume openpilot.* is importable on a
    # bare checkout): any failure leaves angle_lat False, matching every other params-gated
    # capability in this tree.
    self.angle_lat: bool = False
    if fp == "FORD_F_150_LIGHTNING_MK1":
      try:
        from openpilot.common.params import Params
        self.angle_lat = bool(Params().get_bool("FordAngleLateral"))
      except Exception:
        self.angle_lat = False

    # Per-platform path_angle gain defaults (low-curvature, high-curvature gain-table endpoints),
    # Alan Polk's bp-7.0 values verbatim (his _GAIN_CAN / _GAIN_CANFD_BOF / _GAIN_CANFD_SUV and
    # their _CANFD_BOF_CARS / _CANFD_SUV_CARS set membership, opendbc/sunnypilot/car/ford/
    # lateral_angle_ext.py lines 36-52) — not user-tunable in his code either, fixed to body style.
    # He drives an F-150 Lightning himself, so the CANFD_BOF pair applies to our truck directly
    # with no reinterpretation. Populated for every Ford body style (not just the Lightning)
    # because pnw_vehicle is the correct home for ANY carFingerprint-conditioned data, even for a
    # platform we don't currently drive; consumed only when angle_lat is enabled for that platform
    # (see lateral_angle_pnw.LateralAngleExt.__init__, which also allows on-road JSON-overlay
    # tuning of this pair without moving the fingerprint check out of this file).
    _canfd_bof_cars = ("FORD_F_150_MK14", "FORD_F_150_LIGHTNING_MK1", "FORD_EXPEDITION_MK4", "FORD_RANGER_MK2")
    _canfd_suv_cars = ("FORD_MUSTANG_MACH_E_MK1", "FORD_ESCAPE_MK4_5")
    if fp in _canfd_bof_cars:
      self.angle_gain: tuple[float, float] = (0.95, 0.95)
    elif fp in _canfd_suv_cars:
      self.angle_gain = (1.00, 1.05)
    else:
      self.angle_gain = (1.00, 1.15)

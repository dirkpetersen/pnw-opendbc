"""
angle2pnw-faithful2 — on-road tuning overlay for the bp-7.0 angle-primary lateral strategy
(lateral_angle_pnw.LateralAngleExt).

WHY THIS EXISTS: Alan Polk's tuning surface (his own written spec — Low/High Speed Factor, Lane
Change Factor, plus the port's blend-ratio/VLT/gain-table knobs) is normally driven by BluePilot's
own params.json UI. This tree does not have that UI wired up yet, and re-flashing the whole car to
change one gain constant during on-road tuning is a bad workflow. This module lets the driver edit
a JSON file on the device and have it take effect within a few seconds, with NO rebuild/reboot.

FAITHFUL DEFAULTS ARE LAW (see drives/2026-07-18/lightning-angle-steering/ALAN-POLK-PORT-DEVIATIONS.md):
every value this loader can return defaults to Alan Polk's exact bp-7.0 constant. With the overlay
file absent, unreadable, empty, malformed, or any individual key missing/wrong-typed/out-of-range,
that key falls back to ITS OWN default (never all-or-nothing) and the loader never raises. With no
file present at all, runtime behavior is BYTE-IDENTICAL to the pure faithful port -- this module
only ever narrows values toward the defaults it was given, never invents new behavior.

Only tuning-relevant, non-safety constants are exposed here (see the driver's brief, 2026-07-19):
  - gain_lowC_highV / gain_highC_highV     (bp-7.0 _GAIN_CANFD_BOF etc, platform-selected pair)
  - low_speed_curv_factor / high_speed_curv_factor / lane_change_factor_high_ang
      (bp-7.0's user-tunable "feel" multipliers; same clip ranges bp-7.0 itself enforces)
  - path_angle_blend_ratio                 (_FORD_PATH_ANGLE_BLEND_RATIO_DEFAULT, 0.50)
  - vlt_extra_max                          (_VLT_T_EXTRA_MAX, 0.10)
  - gain_speed_lo_ms / gain_speed_hi_ms    (the [13.5, 26.82] gain-interp speed breakpoints)
  - low_speed_boost                        (the 1.30 low-speed-curvature boost, line 446)
  - curvature_factor_bp_lo / curvature_factor_bp_hi  (the [0.0007, 0.001] gain-boost breakpoints)

Deliberately NOT exposed (safety-relevant or structural -- see the brief): the soft ROC table
(mirrored in panda's ford.h FORD_PATH_ANGLE_LIMITS_ANGLE -- changing it desyncs the safety
backstop), FORD_DBC_PATH_ANGLE_MIN/MAX, _PSCM_SAT_UNWIND_RATE, the stall-blip constants, the
current-curvature deviation clip / CarControllerParams.CURVATURE_ERROR, and the PSCM d_ref table.
"""
import json
import os
from dataclasses import dataclass, fields

from opendbc.car.carlog import carlog

ANGLE_TUNING_PATH = "/data/pnw/angle_tuning.json"

# (min, max) sane-range clamp per key. A file value outside its range is dropped (default used)
# -- see _clamp_or_default. Ranges mirror the brief's spec, which mirrors Alan Polk's own UI clips
# where he has one (low/high speed factor 0.5-1.5, lane change factor 0.85-1.50).
_RANGES = {
  "gain_lowC_highV": (0.5, 1.5),
  "gain_highC_highV": (0.5, 1.5),
  "low_speed_curv_factor": (0.5, 1.5),
  "high_speed_curv_factor": (0.5, 1.5),
  "lane_change_factor_high_ang": (0.85, 1.50),
  "path_angle_blend_ratio": (0.0, 1.0),
  "vlt_extra_max": (0.0, 0.5),
  "gain_speed_lo_ms": (0.0, 50.0),
  "gain_speed_hi_ms": (0.0, 50.0),
  "low_speed_boost": (1.0, 2.0),
  "curvature_factor_bp_lo": (0.0001, 0.01),
  "curvature_factor_bp_hi": (0.0001, 0.01),
}

# Keys that form an ordered pair for np.interp breakpoints -- if the loaded (and individually
# clamped) values would invert the order, BOTH keys in the pair revert to their defaults rather
# than feeding interp a decreasing breakpoint list. This is the one place a "per-key" fallback
# widens to a pair, and it is a deterministic, documented safety-of-tuning rule, not a crash path.
_ORDERED_PAIRS = (
  ("gain_speed_lo_ms", "gain_speed_hi_ms"),
  ("curvature_factor_bp_lo", "curvature_factor_bp_hi"),
)


@dataclass
class AngleTuning:
  gain_lowC_highV: float
  gain_highC_highV: float
  low_speed_curv_factor: float
  high_speed_curv_factor: float
  lane_change_factor_high_ang: float
  path_angle_blend_ratio: float
  vlt_extra_max: float
  gain_speed_lo_ms: float
  gain_speed_hi_ms: float
  low_speed_boost: float
  curvature_factor_bp_lo: float
  curvature_factor_bp_hi: float


def _clamp_or_default(key: str, raw, default: float) -> tuple[float, bool]:
  """Returns (value, overridden). Any failure (missing, wrong type, NaN, out of range) -> default."""
  lo, hi = _RANGES[key]
  try:
    val = float(raw)
  except (TypeError, ValueError):
    return default, False
  if not (val == val):  # NaN check without importing math for one use
    return default, False
  if not (lo <= val <= hi):
    return default, False
  return val, (val != default)


def load_angle_tuning(defaults: AngleTuning, path: str = ANGLE_TUNING_PATH) -> AngleTuning:
  """Build an AngleTuning by overlaying ``path`` (if present and valid) onto ``defaults``.

  ``defaults`` must already carry Alan Polk's exact bp-7.0 constants (gain_lowC_highV/highV
  pre-resolved for this platform via PnwVehicle.angle_gain -- this loader does not know about
  carFingerprint, per the capability-view rule). Never raises; logs once via carlog what, if
  anything, was overridden, so a drive report can state which numbers ran.
  """
  result = {f.name: getattr(defaults, f.name) for f in fields(AngleTuning)}
  overridden: dict[str, float] = {}

  raw = None
  try:
    if os.path.isfile(path):
      with open(path) as f:
        raw = json.load(f)
  except Exception as e:
    carlog.warning(f"angle_tuning_pnw: failed to read/parse {path} ({e}) — using bp-7.0 defaults")
    raw = None

  if isinstance(raw, dict):
    for key in result:
      if key not in raw:
        continue
      entry = raw[key]
      # Accept either {"value": X, "_doc": "..."} (the shipped reference-file shape) or a bare
      # scalar X, so a driver's minimal hand-edit (just the number) also works.
      value_raw = entry.get("value") if isinstance(entry, dict) else entry
      val, was_overridden = _clamp_or_default(key, value_raw, result[key])
      result[key] = val
      if was_overridden:
        overridden[key] = val

    # Pairwise ordering sanity check -- see _ORDERED_PAIRS docstring above.
    for lo_key, hi_key in _ORDERED_PAIRS:
      if result[lo_key] >= result[hi_key]:
        if lo_key in overridden or hi_key in overridden:
          carlog.warning(
            f"angle_tuning_pnw: {lo_key}={result[lo_key]} >= {hi_key}={result[hi_key]} after overlay — reverting both to bp-7.0 defaults")
        result[lo_key] = getattr(defaults, lo_key)
        result[hi_key] = getattr(defaults, hi_key)
        overridden.pop(lo_key, None)
        overridden.pop(hi_key, None)

  if overridden:
    carlog.warning(f"angle_tuning_pnw: loaded overrides from {path}: {overridden}")
  else:
    carlog.info(f"angle_tuning_pnw: source=default (no valid overrides from {path})")

  return AngleTuning(**result)

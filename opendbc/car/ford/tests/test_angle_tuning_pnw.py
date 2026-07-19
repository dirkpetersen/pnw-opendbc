"""
angle2pnw-faithful2 — unit tests for the on-road JSON tuning overlay (angle_tuning_pnw.py).

Proves the "faithful defaults are law" guarantee from the brief: no file (or an unreadable/
malformed one) must reproduce Alan Polk's exact bp-7.0 defaults, and a file overriding one key
must change ONLY that key, clamped to its documented range.
"""
import json
import os

from opendbc.car.ford.angle_tuning_pnw import AngleTuning, load_angle_tuning

_DEFAULTS = AngleTuning(
  gain_lowC_highV=0.95,
  gain_highC_highV=0.95,
  low_speed_curv_factor=1.0,
  high_speed_curv_factor=1.0,
  lane_change_factor_high_ang=1.0,
  path_angle_blend_ratio=0.50,
  vlt_extra_max=0.10,
  gain_speed_lo_ms=13.5,
  gain_speed_hi_ms=26.82,
  low_speed_boost=1.30,
  curvature_factor_bp_lo=0.0007,
  curvature_factor_bp_hi=0.001,
)


def test_no_file_matches_defaults(tmp_path):
  missing = tmp_path / "angle_tuning.json"
  assert not missing.exists()
  result = load_angle_tuning(_DEFAULTS, path=str(missing))
  assert result == _DEFAULTS


def test_empty_file_matches_defaults(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text("")
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result == _DEFAULTS


def test_malformed_json_matches_defaults(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text("{not valid json,,,")
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result == _DEFAULTS


def test_wrong_top_level_type_matches_defaults(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps([1, 2, 3]))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result == _DEFAULTS


def test_single_key_override_changes_only_that_key(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({"path_angle_blend_ratio": {"value": 0.35, "_doc": "test"}}))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.path_angle_blend_ratio == 0.35
  for field in _DEFAULTS.__dataclass_fields__:
    if field != "path_angle_blend_ratio":
      assert getattr(result, field) == getattr(_DEFAULTS, field), f"{field} changed unexpectedly"


def test_bare_scalar_value_also_accepted(tmp_path):
  """A driver's minimal hand-edit (just the number, no {"value": ...} wrapper) also works."""
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({"vlt_extra_max": 0.20}))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.vlt_extra_max == 0.20


def test_out_of_range_value_falls_back_to_default(tmp_path):
  f = tmp_path / "angle_tuning.json"
  # path_angle_blend_ratio range is [0.0, 1.0] -- 5.0 must be rejected, not clamped-in-place.
  f.write_text(json.dumps({"path_angle_blend_ratio": {"value": 5.0}}))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.path_angle_blend_ratio == _DEFAULTS.path_angle_blend_ratio


def test_wrong_type_value_falls_back_to_default(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({"vlt_extra_max": {"value": "banana"}}))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.vlt_extra_max == _DEFAULTS.vlt_extra_max


def test_unknown_key_ignored(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({"totally_made_up_key": {"value": 42.0}}))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result == _DEFAULTS


def test_inverted_speed_breakpoint_pair_reverts_both(tmp_path):
  """gain_speed_lo_ms must stay below gain_speed_hi_ms; an inverted pair reverts BOTH to default
  rather than feeding np.interp a decreasing breakpoint list."""
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({
    "gain_speed_lo_ms": {"value": 40.0},
    "gain_speed_hi_ms": {"value": 10.0},
  }))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.gain_speed_lo_ms == _DEFAULTS.gain_speed_lo_ms
  assert result.gain_speed_hi_ms == _DEFAULTS.gain_speed_hi_ms


def test_inverted_curvature_breakpoint_pair_reverts_both(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({
    "curvature_factor_bp_lo": {"value": 0.001},
    "curvature_factor_bp_hi": {"value": 0.0007},
  }))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.curvature_factor_bp_lo == _DEFAULTS.curvature_factor_bp_lo
  assert result.curvature_factor_bp_hi == _DEFAULTS.curvature_factor_bp_hi


def test_valid_ordered_pair_override_both_apply(tmp_path):
  f = tmp_path / "angle_tuning.json"
  f.write_text(json.dumps({
    "gain_speed_lo_ms": {"value": 10.0},
    "gain_speed_hi_ms": {"value": 30.0},
  }))
  result = load_angle_tuning(_DEFAULTS, path=str(f))
  assert result.gain_speed_lo_ms == 10.0
  assert result.gain_speed_hi_ms == 30.0


def test_reference_file_loads_to_exact_defaults():
  """The committed reference file (all keys at Alan Polk's defaults) must round-trip to the exact
  same defaults it documents -- a drift here means the reference file and the code disagree."""
  ref_path = os.path.join(os.path.dirname(__file__), "..", "angle_tuning.reference.json")
  assert os.path.isfile(ref_path), "reference file missing"
  result = load_angle_tuning(_DEFAULTS, path=ref_path)
  assert result == _DEFAULTS

# vinfp2pnw: tests for the VIN-decode identity fallback registry (opendbc/car/vin_fallbacks.py)
# and its wiring into car_helpers.py::fingerprint(). See docs/VIN-FINGERPRINT2PNW.md for the design.
#
# A note on VIN validity: `opendbc.car.vin.is_valid_vin()` is a REGEX-ONLY gate - 17 characters from
# the standard VIN charset (`[A-HJ-NPR-Z0-9]`, i.e. no I/O/Q). It does NOT verify the NHTSA check
# digit (position 9) despite the design doc's step 1 describing "length + charset + check-digit" -
# the actual shipped `is_valid_vin` only does length+charset. This means every synthetic VIN below
# only needs to be 17 chars drawn from that charset; no check-digit computation is required, and a
# wrong check digit does NOT make a test VIN fail the validity gate (confirmed by inspection of
# opendbc/car/vin.py and by running these tests).
import os
from types import SimpleNamespace
from unittest import mock

from opendbc.car import car_helpers, vin_fallbacks
from opendbc.car.structs import CarParams
from opendbc.car.vin import VIN_UNKNOWN, is_valid_vin
from opendbc.car.vin_fallbacks import VinFallbackEntry, decode_vin_platform

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"

# The actual reference truck (CLAUDE.md / design doc §3.4): 2025 F-150 Lightning Flash, ER 131 kWh,
# built at Dearborn/Rouge EV Center. wmi=1FT, pos8='7' (ER 131kWh NMC), pos10='S' (2025).
REFERENCE_VIN = "1FT6W3L78SWG05094"


def _vin(wmi='1FT', pos4='6', pos8='7', pos10='S', filler='A'):
  """Build a 17-char synthetic test VIN with an explicit WMI (positions 1-3), body/GVWR code
  (position 4), engine/battery code (position 8), and model-year code (position 10); every other
  position is a charset-legal filler character. All positions are 1-indexed to match the design doc
  / vin_fallbacks.py convention.

  pos4 defaults to '6' (a valid Lightning body code, design doc §3.1/§3.4) rather than the generic
  filler so every EXISTING "expect a Lightning match" test keeps passing after the B1 fix added a
  pos4 gate to the shipped registry row - callers that want to test the pos4 gate itself (e.g. an
  E-Transit/Super-Duty negative case) pass an explicit pos4.
  """
  chars = [filler] * 17
  chars[0:3] = list(wmi)
  chars[3] = pos4
  chars[7] = pos8
  chars[9] = pos10
  vin = ''.join(chars)
  assert is_valid_vin(vin), f"test helper produced an invalid VIN: {vin}"
  return vin


class TestVinFallbacksSanity:
  def test_reference_vin_is_actually_valid(self):
    # sanity check on the fixture itself, and a demonstration that is_valid_vin is regex-only
    # (no check-digit enforcement) - see module docstring.
    assert is_valid_vin(REFERENCE_VIN)

  def test_decode_year_table_matches_reference_vin(self):
    assert vin_fallbacks.POSITION_10_TO_YEAR[REFERENCE_VIN[9]] == 2025


class TestLightningRegistryRow:
  def test_positive_reference_truck_matches_lightning(self):
    assert decode_vin_platform(REFERENCE_VIN) == LIGHTNING

  def test_negative_etransit_same_wmi_and_shared_electric_engine_code(self):
    # B1 (adversarial-review-caught, real safety defect, fixed 2026-08-10): a REAL 2023 Ford
    # E-Transit VIN. The 1FT WMI covers every Ford truck/van (F-150, Super Duty, Transit, E-Transit,
    # E-Series), and the E-Transit's BEV variant SHARES the Lightning's electric position-8 code 'K'
    # - so position 8 alone is provably not enough to isolate the Lightning within the 1FT family.
    # Before the fix this decoded to FORD_F_150_LIGHTNING_MK1 (wrong car interface / wrong panda
    # safety expectations for an E-Transit). Must now be None.
    e_transit_vin = "1FTBW3XKXPKB78450"
    assert is_valid_vin(e_transit_vin)
    assert e_transit_vin[3] == 'B'  # E-Transit (van) body code - NOT 'V'/'6' (Lightning)
    assert e_transit_vin[7] == 'K'  # shares the Lightning's electric position-8 code
    assert decode_vin_platform(e_transit_vin) is None

  def test_negative_super_duty_style_same_wmi_and_electric_engine_code_wrong_body_code(self):
    # Same B1 mechanism, isolating the pos4 gate specifically: position 8 is deliberately set to a
    # VALID Lightning electric code ('7') so the ONLY reason either VIN below must fail to match is
    # the position-4 body-code gate (Super Duty uses '7'/'8', not the Lightning's 'V'/'6').
    for super_duty_pos4 in ('7', '8'):
      vin = _vin(wmi='1FT', pos4=super_duty_pos4, pos8='7', pos10='S')
      assert decode_vin_platform(vin) is None, f"pos4={super_duty_pos4} (Super-Duty-style) must not match"

  def test_positive_both_lightning_generation_body_codes_still_match(self):
    # The pos4 gate must not be so tight it excludes real Lightnings: both generations' body codes
    # (2022-23 'V', 2024-25 '6' - design doc §3.1/§3.4) must still match.
    for pos4 in ('V', '6'):
      vin = _vin(wmi='1FT', pos4=pos4, pos8='7', pos10='S')
      assert decode_vin_platform(vin) == LIGHTNING, f"body code {pos4} should match"

  def test_negative_ice_f150_same_wmi_non_electric_engine_code(self):
    # Safety-critical case (design doc §4.2/§4.3/§7): same WMI (1FT) and same model year (S=2025)
    # as the reference truck, but position 8 = 'T', which is NOT in the Lightning's electric code
    # set {L,V,K,S,7,M}. Must NOT match - this is what stops a gas F-150 from being mapped onto the
    # Lightning platform (wrong panda safety expectations).
    vin = _vin(wmi='1FT', pos8='T', pos10='S')
    assert decode_vin_platform(vin) is None

  def test_negative_2026_model_year_never_produced(self):
    # Otherwise-identical Lightning-looking VIN, but position 10 = 'T' (2026). The Lightning row is
    # year-bounded to 2022-2025 because 2026 was never built (design doc §3.1, §4.3) - a T-year VIN
    # must not be assigned MK1 even though the electric engine code matches.
    vin = _vin(wmi='1FT', pos8='7', pos10='T')
    assert decode_vin_platform(vin) is None

  def test_negative_tesla_vin_never_matches(self):
    # §4.4 hard boundary: the registry must never contain (or accidentally match) a Tesla entry.
    # A real-shaped Tesla Model S WMI (5YJ) simply has no row in the registry to match against.
    vin = _vin(wmi='5YJ', pos8='2', pos10='F')
    assert decode_vin_platform(vin) is None

  def test_edge_unknown_position8_code(self):
    # A plausible-but-unlisted future battery/engine letter must fail closed, not guess.
    vin = _vin(wmi='1FT', pos8='Z', pos10='S')
    assert decode_vin_platform(vin) is None

  def test_all_four_produced_model_years_match(self):
    # N=2022, P=2023, R=2024, S=2025 - the Lightning's actual production span.
    for pos10, year in (('N', 2022), ('P', 2023), ('R', 2024), ('S', 2025)):
      vin = _vin(wmi='1FT', pos8='7', pos10=pos10)
      assert decode_vin_platform(vin) == LIGHTNING, f"model year {year} ({pos10}) should match"

  def test_all_electric_engine_codes_match(self):
    for pos8 in ('L', 'V', 'K', 'S', '7', 'M'):
      vin = _vin(wmi='1FT', pos8=pos8, pos10='S')
      assert decode_vin_platform(vin) == LIGHTNING, f"engine code {pos8} should match"

  def test_invalid_vin_never_matches(self):
    # Validity gate (design doc §5 step 1): too short / bad charset -> skip the whole layer.
    assert decode_vin_platform("1FT6W3L78SWG0509") is None  # 16 chars
    assert decode_vin_platform("1FT6W3L78SWG050I4") is None  # 18 chars, contains 'I'


class TestGenericMatcherMechanics:
  """Exercise the generic matcher (§5) against synthetic registry rows, independent of the shipped
  Lightning row, so these tests keep working even if the real registry changes shape later."""

  def test_omitted_year_matches_every_model_year(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      for pos10 in ('N', 'S', 'V', '1'):  # spans multiple decades - omitted year is unconstrained
        vin = _vin(wmi='9TE', pos8='X', pos10=pos10)
        assert decode_vin_platform(vin) == 'TEST_WIDGET'

  def test_year_as_list_membership(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=[2022, 2024],
                              match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='N')) == 'TEST_WIDGET'  # 2022: in list
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='R')) == 'TEST_WIDGET'  # 2024: in list
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='P')) is None            # 2023: not in list

  def test_year_as_range_string_membership(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year='2023-2025',
                              match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='N')) is None            # 2022: before range
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='P')) == 'TEST_WIDGET'  # 2023: range start
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='S')) == 'TEST_WIDGET'  # 2025: range end
      assert decode_vin_platform(_vin(wmi='9TE', pos8='X', pos10='T')) is None            # 2026: after range

  def test_span_field_matches_a_substring_range(self):
    # match.span isn't used by the shipped Lightning row, but the generic matcher must still
    # support it (design doc §2.1) for future rows.
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'span': {'5-7': ['ABC']}})
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      vin_match = '9TE' + 'X' + 'ABC' + 'X' * 10
      vin_nomatch = '9TE' + 'X' + 'XYZ' + 'X' * 10
      assert len(vin_match) == 17 and len(vin_nomatch) == 17
      assert decode_vin_platform(vin_match) == 'TEST_WIDGET'
      assert decode_vin_platform(vin_nomatch) is None

  def test_more_specific_entry_wins_over_looser_entry(self):
    # design doc §5 "most-specific wins": a year-scoped, otherwise-identical row outranks an
    # all-years row when both match the same VIN.
    loose = VinFallbackEntry(make='Test', model='Widget', platform='TEST_LOOSE', year=None,
                              match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    specific = VinFallbackEntry(make='Test', model='Widget2', platform='TEST_SPECIFIC', year=2024,
                                 match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='R')  # R = 2024, satisfies both
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [loose, specific]):
      assert decode_vin_platform(vin) == 'TEST_SPECIFIC'
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [specific, loose]):  # order-independent
      assert decode_vin_platform(vin) == 'TEST_SPECIFIC'

  def test_equal_specificity_conflict_assigns_nothing_and_logs(self):
    # Fail-safe ambiguity (design doc §5, §7): two equally-specific entries disagree on platform ->
    # must return None (never guess) AND log the conflict loudly.
    entry_a = VinFallbackEntry(make='Test', model='WidgetA', platform='TEST_A', year=None,
                                match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    entry_b = VinFallbackEntry(make='Test', model='WidgetB', platform='TEST_B', year=None,
                                match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry_a, entry_b]), \
         mock.patch.object(vin_fallbacks.carlog, 'error') as log_mock:
      result = decode_vin_platform(vin)
    assert result is None
    assert log_mock.call_count == 1
    logged = log_mock.call_args[0][0]
    assert logged["event"] == "VIN decode registry AMBIGUOUS - equally-specific entries disagree, assigning nothing"
    assert logged["vin"] == vin

  def test_equal_specificity_same_platform_is_not_a_conflict(self):
    # Two entries at the same rank that happen to agree on platform is fine - not ambiguous.
    entry_a = VinFallbackEntry(make='Test', model='WidgetA', platform='TEST_SAME', year=None,
                                match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    entry_b = VinFallbackEntry(make='Test', model='WidgetB', platform='TEST_SAME', year=None,
                                match={'wmi': ['9TZ'], 'pos': {8: ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry_a, entry_b]):
      assert decode_vin_platform(vin) == 'TEST_SAME'

  def test_no_matching_entry_returns_none(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    vin = _vin(wmi='ZZZ', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) is None


class TestMalformedRegistryEntriesFailSafe:
  """N2 (adversarial-review should-fix, fixed 2026-08-10): a malformed registry row (a future data
  typo) must never crash `fingerprint()` - it must be SKIPPED (treated as non-matching) instead.
  Each test below installs ONE row as the whole registry, on a VIN that would otherwise satisfy
  every OTHER gate, so the only thing under test is whether the malformed field is handled safely
  (or, for the string-pos-key case, handled CORRECTLY - that shape is explicitly supported, not
  malformed)."""

  def test_string_pos_key_is_normalized_not_treated_as_malformed(self):
    # The design doc's own JSONC illustration (§2.3) writes `pos` keys as strings: {"8": [...]}.
    # A string key must be NORMALIZED and MATCH exactly like an int key - this is a documented,
    # supported shape, not a malformed one.
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'pos': {'8': ['X']}})  # string key '8'
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) == 'TEST_WIDGET'

  def test_pos_key_out_of_range_skips_entry_instead_of_raising(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'pos': {18: ['X']}})  # VIN position 18 doesn't exist
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) is None  # must not raise IndexError

  def test_dash_less_span_key_skips_entry_instead_of_raising(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'span': {'5': ['ABC']}})  # missing the 'a-b' separator
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) is None  # must not raise ValueError

  def test_single_year_string_skips_entry_instead_of_raising(self):
    # "2025" (no dash) is NOT one of the three documented `year` shapes (single int / list of ints /
    # "start-end" range string) - must fail safe, not crash trying to parse it as a range.
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year="2025",
                              match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) is None  # must not raise ValueError

  def test_non_dict_match_skips_entry_instead_of_raising(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None, match=None)
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) is None  # must not raise AttributeError

  def test_non_numeric_pos_key_skips_entry_instead_of_raising(self):
    entry = VinFallbackEntry(make='Test', model='Widget', platform='TEST_WIDGET', year=None,
                              match={'wmi': ['9TE'], 'pos': {'not-a-number': ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]):
      assert decode_vin_platform(vin) is None  # must not raise ValueError

  def test_one_malformed_entry_does_not_poison_a_good_entry_elsewhere_in_the_registry(self):
    bad = VinFallbackEntry(make='Test', model='Bad', platform='TEST_BAD', year=None,
                            match={'wmi': ['9TE'], 'pos': {'not-a-number': ['X']}})
    good = VinFallbackEntry(make='Test', model='Good', platform='TEST_GOOD', year=None,
                             match={'wmi': ['9TE'], 'pos': {8: ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [bad, good]):
      assert decode_vin_platform(vin) == 'TEST_GOOD'

  def test_malformed_entry_is_logged(self):
    entry = VinFallbackEntry(make='Test', model='Bad', platform='TEST_BAD', year=None,
                              match={'wmi': ['9TE'], 'pos': {18: ['X']}})
    vin = _vin(wmi='9TE', pos8='X', pos10='S')
    with mock.patch.object(vin_fallbacks, 'VIN_FALLBACK_REGISTRY', [entry]), \
         mock.patch.object(vin_fallbacks.carlog, 'error') as log_mock:
      assert decode_vin_platform(vin) is None
    assert log_mock.call_count == 1
    logged = log_mock.call_args[0][0]
    assert logged["event"] == "VIN decode registry entry malformed - skipping entry (fail-safe)"


class TestRegistryShapeValidation:
  """A structural sanity check over the REAL shipped registry (not synthetic rows) - independent of
  whether the matcher happens to fail safe on a bad row at runtime, this catches a malformed shipped
  row directly (registry-shape validation, per the task's N2 requirement)."""

  def test_every_shipped_entry_has_well_formed_match_spec(self):
    assert len(vin_fallbacks.VIN_FALLBACK_REGISTRY) > 0, "registry should not be empty"
    for entry in vin_fallbacks.VIN_FALLBACK_REGISTRY:
      assert isinstance(entry.match, dict), f"{entry!r} .match must be a dict"

      wmi_list = entry.match.get('wmi')
      if wmi_list is not None:
        assert all(isinstance(w, str) and len(w) == 3 for w in wmi_list), f"{entry!r} wmi entries must be 3-char strings"

      pos_map = entry.match.get('pos', {})
      for position, allowed in pos_map.items():
        position_int = int(position)  # must be int-parseable (accepts int or numeric string)
        assert 1 <= position_int <= 17, f"{entry!r} pos key {position!r} out of the 1-17 VIN range"
        assert len(allowed) > 0, f"{entry!r} pos key {position!r} has an empty allowed-chars list"

      span_map = entry.match.get('span', {})
      for span, allowed in span_map.items():
        assert '-' in str(span), f"{entry!r} span key {span!r} missing the required 'a-b' separator"
        start_str, end_str = str(span).split('-', 1)
        start, end = int(start_str), int(end_str)
        assert 1 <= start <= end <= 17, f"{entry!r} span key {span!r} out of the 1-17 VIN range"
        assert len(allowed) > 0, f"{entry!r} span key {span!r} has an empty allowed-substrings list"

      if entry.year is not None:
        # must be parseable by the shared expander without raising, and non-empty
        years = vin_fallbacks._expand_years(entry.year)
        assert len(years) > 0, f"{entry!r} year {entry.year!r} expanded to an empty set"

      assert entry.platform, f"{entry!r} platform must be a non-empty string"


# ---------------------------------------------------------------------------------------------
# Integration: verify car_helpers.py::fingerprint() actually reaches decode_vin_platform() at the
# fallback slot, with the documented precedence (fleet_vins.json "vins" > decode registry >
# "no_vin_platform"), mirroring the mocking style of test_fpcache_requery.py.
def _cached_params(fw_count=10, vin=REFERENCE_VIN, brand="ford"):
  return SimpleNamespace(brand=brand, carFw=[object()] * fw_count, carVin=vin)


def _run_fingerprint(live_vin, fleet_cfg, decode_mock=None):
  for k in ("FINGERPRINT", "SKIP_FW_QUERY", "DISABLE_FW_CACHE"):
    os.environ.pop(k, None)

  get_vin_mock = mock.Mock(return_value=(2024, 0, live_vin))
  get_fw_mock = mock.Mock(return_value=[])
  match_mock = mock.Mock(return_value=(True, set()))  # FW match always empty -> forces the fallback slot
  fleet_mock = mock.Mock(return_value=fleet_cfg)
  if decode_mock is None:
    decode_mock = mock.Mock(wraps=decode_vin_platform)

  with mock.patch.object(car_helpers, "get_vin", get_vin_mock), \
       mock.patch.object(car_helpers, "get_present_ecus", mock.Mock(return_value=set())), \
       mock.patch.object(car_helpers, "get_fw_versions_ordered", get_fw_mock), \
       mock.patch.object(car_helpers, "match_fw_to_car", match_mock), \
       mock.patch.object(car_helpers, "can_fingerprint", mock.Mock(return_value=(None, {}))), \
       mock.patch.object(car_helpers, "pnw_fleet_config", fleet_mock), \
       mock.patch.object(car_helpers, "decode_vin_platform", decode_mock):
    result = car_helpers.fingerprint(mock.Mock(return_value=[]), mock.Mock(), mock.Mock(),
                                     cached_params=None, num_pandas=1)
  return result, decode_mock


class TestCarHelpersIntegration:
  def test_decode_registry_fires_when_no_exact_vin_entry(self):
    # No cached params -> not cached, so the fallback slot is live. fleet "vins" doesn't know this
    # VIN, so decode_vin_platform() must be consulted and its hit must be assigned.
    (candidate, _, vin, _, source, exact_match), decode_mock = _run_fingerprint(
      live_vin=REFERENCE_VIN, fleet_cfg={"vins": {}})
    assert candidate == LIGHTNING
    assert source == CarParams.FingerprintSource.fixed
    assert exact_match is True
    assert vin == REFERENCE_VIN
    decode_mock.assert_called_once_with(REFERENCE_VIN)

  def test_exact_vin_entry_wins_over_decode_registry(self):
    # fleet "vins" has an explicit (possibly different) mapping for this exact VIN -> the exact-VIN
    # safety net must win, and decode_vin_platform must NOT even be consulted (design doc §6: exact
    # VIN is the most-specific / first-checked layer).
    decode_mock = mock.Mock(wraps=decode_vin_platform)
    (candidate, _, _, _, source, _), decode_mock = _run_fingerprint(
      live_vin=REFERENCE_VIN, fleet_cfg={"vins": {REFERENCE_VIN: "SOME_OTHER_PLATFORM_NOT_IN_INTERFACES"}},
      decode_mock=decode_mock)
    # exact-VIN fallback returned an unsupported platform, so it's ignored -> candidate stays None,
    # but the important assertion is that decode_vin_platform was never called (exact-VIN short-circuits).
    assert candidate is None
    decode_mock.assert_not_called()

  def test_no_vin_platform_used_when_vin_unknown_not_decode_registry(self):
    # VIN_UNKNOWN (unreadable over CAN, e.g. the Raven) must go straight to no_vin_platform and must
    # NEVER be handed to decode_vin_platform (design doc §4.4/§8 - decode can't run without a VIN).
    decode_mock = mock.Mock(wraps=decode_vin_platform)
    (candidate, _, vin, _, source, _), decode_mock = _run_fingerprint(
      live_vin=VIN_UNKNOWN, fleet_cfg={"no_vin_platform": "TESLA_MODEL_S_HW3"}, decode_mock=decode_mock)
    assert candidate == "TESLA_MODEL_S_HW3"
    assert vin == VIN_UNKNOWN
    decode_mock.assert_not_called()

  def test_decode_miss_falls_through_to_mock(self):
    # Live VIN is valid but matches nothing in the registry (e.g. a Tesla or ICE F-150) and fleet
    # "vins" doesn't know it either -> candidate stays None (caller maps that to MOCK).
    tesla_vin = "5YJSA1E26FF123456"
    (candidate, _, vin, _, _, _), decode_mock = _run_fingerprint(
      live_vin=tesla_vin, fleet_cfg={"vins": {}})
    assert candidate is None
    assert vin == tesla_vin
    decode_mock.assert_called_once_with(tesla_vin)

  def test_decode_registry_raising_falls_through_to_mock_not_crash(self):
    # N2 (adversarial-review should-fix, fixed 2026-08-10): even if decode_vin_platform() itself
    # raises (e.g. a future bug bypasses vin_fallbacks.py's own internal fail-safe), car_helpers.py's
    # own try/except around the call must catch it and treat it as "no match" - fingerprint() must
    # NEVER crash on this fallback path. If car_helpers.py's guard were missing or broken, this test
    # would fail with the RuntimeError propagating out of car_helpers.fingerprint() instead of being
    # caught here.
    raising_mock = mock.Mock(side_effect=RuntimeError("simulated vin_fallbacks bug"))
    (candidate, _, vin, _, source, _), decode_mock = _run_fingerprint(
      live_vin=REFERENCE_VIN, fleet_cfg={"vins": {}}, decode_mock=raising_mock)
    assert candidate is None  # falls through toward MOCK (caller maps None -> mock), no crash
    assert vin == REFERENCE_VIN
    decode_mock.assert_called_once_with(REFERENCE_VIN)

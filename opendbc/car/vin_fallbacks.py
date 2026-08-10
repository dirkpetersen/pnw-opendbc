"""
vinfp2pnw — VIN-decode identity fallback registry.

See `docs/VIN-FINGERPRINT2PNW.md` (repo `pnw-pilot`, mirrored to the workbench root `docs/` folder)
for the full design. This module implements design-doc §2 (declarative schema + shared year table)
and §5 (generic matcher). It is consumed by `opendbc/car/car_helpers.py :: fingerprint()` per §6.

WHAT / WHY
----------
openpilot identifies a car primarily by matching ECU firmware-version strings queried over UDS
(`match_fw_to_car`). A vendor software update (dealer service, an OTA) commonly REWRITES those
version strings, silently breaking exact-FW matching — the car falls back to MOCK/dashcam even
though nothing about the physical vehicle changed. The 2025 Ford F-150 Lightning is the worst case:
its EPS does not answer the Ford platform-code UDS query at all, so there is nothing for *fuzzy*
FW matching to key on either — it is exact-FW-match-only, and exact FW is exactly what an update
destroys (see design doc §1.2).

The **VIN is immutable** for the life of the vehicle and is already queried at every boot
(`opendbc.car.vin.get_vin`), independent of FW state. This module decodes it against a small,
declarative, IN-REPO table of `(make, model, year?) -> openpilot platform` rows, each backed by a
match spec over fixed VIN character positions (WMI, engine code, etc.) per design doc §2.1-§2.2.
Because the table holds no actual VINs — only class-level (make/model/year) predicates — it is NOT
personal data and can ship in the repo, unlike the on-device `/data/pnw/fleet_vins.json` exact-VIN
file (which stays in place as a safety net; see car_helpers.py).

THIS IS AN IDENTITY-ESTABLISHING FALLBACK, NOT A FEATURE. It only ever fires from
`car_helpers.py :: fingerprint()` at the point where FW matching AND CAN fingerprinting have BOTH
already failed to produce a candidate (see design doc §6, §7 "never overrides a good fingerprint").
A hit here selects `car_fingerprint`, which in turn selects the car interface and panda
`safetyConfigs` — so mapping the wrong platform is a SAFETY issue, not a cosmetic one. The matcher
below is intentionally conservative: any ambiguity resolves to "assign nothing" (== stays MOCK),
never a guess (design doc §5, §7).

PRECEDENCE (design doc §6) — established by the caller, not this module:
  1. Cached CarParams
  2. Exact FW match         (unchanged, primary, never overridden)
  3. Fuzzy FW match         (unchanged)
  4. CAN fingerprint        (unchanged)
  5. fleet_vins.json "vins" exact-VIN map   (KEPT as a manual per-VIN safety net / escape hatch)
  6. >>> decode_vin_platform() (THIS MODULE) <<<  — class-level, in-repo, matches any instance of a
     known make/model/year by decoding the live VIN
  7. fleet_vins.json "no_vin_platform"      (KEPT — the only path for a car whose platform-relevant
     identity is NOT encoded in the VIN at all; see the Tesla boundary below)
  8. FINGERPRINT env override
  9. MOCK

THE §4.4 TESLA / RAVEN BOUNDARY — READ BEFORE ADDING A ROW
------------------------------------------------------------
A Tesla VIN does NOT encode the Autopilot compute generation (HW2 / HW3 / HW4). openpilot's Tesla
platform split (the Raven is `TESLA_MODEL_S_HW3`) is EXACTLY that distinction — a hardware
generation that is invisible to the VIN standard's fixed character positions (make/model/body/
engine/model-year). No amount of VIN decoding can ever recover it, and the Raven's VIN is not even
reliably readable over CAN. This is a hard boundary of the whole approach, not a gap this table will
one day fill in: VIN decode only ever covers vehicles whose openpilot-platform split is fully
determined by fields the VIN standard actually encodes. THE REGISTRY BELOW MUST NEVER CONTAIN A
TESLA ENTRY. The Raven keeps identifying via CAN fingerprint / the on-device `no_vin_platform`
two-car inference (kept, unchanged, in car_helpers.py) — never via this module.

When adding a future row for a different make/model (design doc §2.4), the same test applies: is the
platform split something the VIN standard's fixed positions actually encode (body style, engine/
battery code, model year, ...)? If the split is instead a hardware/software generation that happens
to be invisible to the VIN (as with Tesla HW), do NOT add a row — use CAN fingerprint or
`no_vin_platform` instead, exactly as today.
"""

from opendbc.car.carlog import carlog
from opendbc.car.vin import is_valid_vin

# ---------------------------------------------------------------------------------------------
# §2.2 — shared, make-agnostic model-year table.
#
# VIN position 10 (1-indexed) encodes model year identically for EVERY manufacturer under the
# ISO-3779 / NHTSA standard cycle. Letters I, O, Q, U, Z and the digit 0 are never used (I/O/Q are
# excluded from the whole VIN charset to avoid confusion with 1/0; U and Z are skipped by
# convention in the year cycle; 0 is reserved). The cycle repeats every 30 years, which is
# unambiguous for any realistic vehicle-support window (we do not attempt to disambiguate which
# lap of the cycle a VIN belongs to — nothing we support is anywhere near 30 years old).
#
# This table decodes position 10 -> the SET of years it could represent is a single year (not a
# set) per design doc §2.2: one position-10 character maps to exactly one model year. It is entries'
# `year` fields (§2.1) that expand to a *set* of years, each looked up here in reverse to build the
# set of acceptable position-10 characters.
POSITION_10_TO_YEAR: dict[str, int] = {
  # ... cycle continues both directions; only the span actually needed by shipped rows is listed.
  # Add codes here (never invent new ones) as new model years are added to the registry.
  'N': 2022,
  'P': 2023,
  'R': 2024,
  'S': 2025,
  'T': 2026,
  'V': 2027,
  'W': 2028,
  'X': 2029,
  'Y': 2030,
  '1': 2031,
}
# Reverse lookup (year -> position-10 char), built once. If a future row's `year` requests a year
# not present in POSITION_10_TO_YEAR, that year is simply never satisfiable (fail-safe: the entry
# will never match on it) rather than raising - extend the table above instead of relying on this
# to error loudly for a typo'd/unshipped year.
YEAR_TO_POSITION_10: dict[int, str] = {year: code for code, year in POSITION_10_TO_YEAR.items()}


# ---------------------------------------------------------------------------------------------
# §2.1 — entry schema (declarative; VinFallbackEntry rows are pure data, never per-model code).
#
# Fields:
#   make      human label only (e.g. "Ford"); not consumed by the matcher, kept for readability/docs.
#   model     human label only (e.g. "F-150 Lightning"); not consumed by the matcher.
#   year      optional. One of:
#               - a single int model year, e.g. 2025
#               - a list of int model years, e.g. [2022, 2023, 2024, 2025]
#               - a "start-end" inclusive range string, e.g. "2022-2025"
#             Omitted (None) => unconstrained, matches every model year (§2.1, §5 step 5).
#   platform  the openpilot `CAR` enum VALUE STRING to assign (e.g. "FORD_F_150_LIGHTNING_MK1").
#             Kept as a plain string (not an import of the brand's CAR enum) so this module stays
#             import-light and brand-agnostic; car_helpers.py support-gates it against `interfaces`
#             exactly like the existing fleet_vins.json fallback does (§5 step 6) — NOT here.
#   match     the declarative VIN predicate (the machine form of make/model). Sub-fields, every
#             PRESENT one must pass (AND across sub-fields); each sub-field is an any-of list
#             (OR within the sub-field):
#               wmi   list[str]            - VIN[0:3] (chars 1-3, 1-indexed) must start with one of
#                                             these prefixes. Manufacturer/type/country code.
#               pos   dict[int, list[str]] - map of 1-INDEXED VIN position -> allowed single chars.
#                                             VIN[position - 1] must be one of the allowed chars.
#               span  dict[str, list[str]] - map of "a-b" (1-indexed, inclusive) -> allowed
#                                             substrings. VIN[a-1:b] must be one of the allowed
#                                             substrings. Present for future rows (e.g. a series/trim
#                                             code); unused by the one shipped row.
#
# NOTE on indexing: every position above is 1-INDEXED per the VIN standard (position 1 is the first
# character), matching the design doc and the human-facing VIN references it cites. This module
# converts to Python's 0-indexed strings internally (see _char_at / _span_at) so no caller needs to
# remember the off-by-one.
class VinFallbackEntry:
  def __init__(self, make: str, model: str, platform: str, match: dict, year=None):
    self.make = make
    self.model = model
    self.platform = platform
    self.match = match
    self.year = year

  def __repr__(self) -> str:
    return f"VinFallbackEntry(make={self.make!r}, model={self.model!r}, year={self.year!r}, platform={self.platform!r})"


def _expand_years(year) -> set[int]:
  """Expand an entry's `year` field (single int / list / "start-end" string) to a set of ints."""
  if year is None:
    return set()
  if isinstance(year, int):
    return {year}
  if isinstance(year, str):
    start_str, _, end_str = year.partition('-')
    return set(range(int(start_str), int(end_str) + 1))
  # list/tuple/set of ints
  return set(year)


# N2 (adversarial-review should-fix, fixed 2026-08-10): a registry row is DATA — a future typo
# (a string-vs-int position key, an out-of-range position, a dash-less span key, an unparseable
# `year`) must never crash `fingerprint()` at car startup. `_normalize_position` below is the one
# place a `pos`/`span` position is turned into a bounds-checked int; it's used by BOTH `pos` (via
# `_char_at`) and `span` (via `_span_at`), and it deliberately RAISES ValueError on anything it
# can't make sense of rather than silently guessing — `_entry_matches`'s try/except (below) is what
# turns that raise into a safe "skip this one malformed entry" instead of a propagated exception.
def _normalize_position(position) -> int:
  """Normalize a 1-indexed VIN position key to int, accepting either a native int or a numeric
  string — the design doc's own JSONC illustration (§2.3) writes `pos` keys as strings
  (e.g. `{"8": [...]}`), so `8` and `"8"` must be treated identically, not as a malformed shape.
  Raises ValueError/TypeError if the key isn't int-parseable or falls outside the valid 1-17 VIN
  character range; the caller is always wrapped by `_entry_matches`'s fail-safe try/except, so this
  raising is what marks "this registry entry is malformed" — it is never allowed to escape this
  module."""
  position_int = int(position)
  if not (1 <= position_int <= 17):
    raise ValueError(f"VIN position {position_int} is outside the valid 1-17 range")
  return position_int


def _char_at(vin: str, position_1_indexed) -> str:
  return vin[_normalize_position(position_1_indexed) - 1]


def _span_at(vin: str, span_1_indexed: str) -> str:
  start_str, sep, end_str = str(span_1_indexed).partition('-')
  if not sep:
    raise ValueError(f"span key {span_1_indexed!r} is missing the required 'a-b' separator")
  start, end = _normalize_position(start_str), _normalize_position(end_str)
  if start > end:
    raise ValueError(f"span key {span_1_indexed!r} has start > end")
  return vin[start - 1:end]


# ---------------------------------------------------------------------------------------------
# §2.3 — the registry itself. Adding a make/model/year is a DATA change (one row) — the matcher
# (§5, below) is generic and never changes for a new row.
#
# Decode strictly by POSITION, never by a bare letter scan: several letters mean different things
# at different VIN positions (e.g. on the Lightning, 'S' is BOTH an engine/battery code at
# position 8 AND the 2025-model-year code at position 10; '7' is an engine code and also appears
# as a plain digit elsewhere). Every predicate below is keyed by explicit position.
VIN_FALLBACK_REGISTRY: list[VinFallbackEntry] = [
  # ---- Ford F-150 Lightning (any trim, any battery), model years 2022-2025 --------------------
  # openpilot platform: FORD_F_150_LIGHTNING_MK1 (a distinct CAR enum from the ICE FORD_F_150_MK14
  # — design doc §4.1: "F-150 Lightning" is its own model, not an F-150 trim).
  #
  # wmi: "1FT" = Ford Motor Co., truck, USA (design doc §3.1). Covers both the 2022-23 "1FTVW..."
  #   and 2024-25 "1FT6W..." generations, but ALSO every other Ford truck/van built off the same
  #   WMI family — F-150 (ICE), Super Duty, Transit, E-Transit, E-Series. wmi alone is NOT
  #   sufficient to isolate the Lightning; see the span{5-7} + pos{8} gates below (B1).
  #
  # span {5-7: [...]} + pos {8: [...]}: TWO INDEPENDENT, SAFETY-LOAD-BEARING DISCRIMINATORS are
  #   required together to isolate "F-150 Lightning" from the rest of the 1FT family — neither one
  #   alone is sufficient, and a wrong match here selects the wrong car interface / panda safety
  #   expectations:
  #
  #   span 5-7 (series/trim code) = the MODEL-LINE discriminator. It identifies "this is an F-150"
  #   and rejects the REST of the 1FT family: the E-Transit's series is 'W3X' (not in this set),
  #   and Super Duty / Transit / E-Series use their own series codes — none are F-150 codes. Series
  #   set (design doc §3.1/§9): 'W1E' = ALL 2022-23 Lightning trims (single code, no per-trim
  #   split); 2024-25 splits by trim: 'W1B'=Pro, 'W3L'=XLT and Flash, 'W5L'=Lariat, 'W7L'=Platinum.
  #
  #   pos 8 (engine/battery code) = the EV discriminator. Series ALONE is not enough (B1,
  #   adversarial-review-caught 2026-08-10, revised 2026-08-10 per driver feedback): a GAS F-150
  #   XLT is ALSO series 'W3L' — the ICE and EV F-150 literally share several series letters
  #   ("W3L"/"W5L"/"W7L" appear on both an ICE XLT/Lariat/Platinum and an EV Flash/Lariat/Platinum,
  #   design doc §4.2) — so pos8's electric-only code set is what separates the Lightning from a
  #   gas F-150 sharing the same series code. Position 8's codes are electric-only for this model:
  #   L/V (2022-23 SR/ER), K/S (2024-25 SR/SR-LFP), 7/M (2024-25 ER retail/fleet). See design doc
  #   §3.2 for the full per-code battery/chemistry table.
  #
  #   Together: span 5-7 rejects the rest of the 1FT family (E-Transit, Super Duty, Transit,
  #   E-Series); pos 8 rejects a gas F-150 sharing an F-150 series code. Either gate alone
  #   under-constrains; both together isolate exactly the Lightning.
  #
  #   ⚠ TODO(§9, design doc): the series set {'W1E','W1B','W3L','W5L','W7L'} is NOT yet
  #   independently verified against the Ford Pro VIN guide (Rev 11) — it is transcribed from the
  #   owner-provided reference table + forum sources (design doc §3.1). Like the ⚠ R/U pos8 codes
  #   below, treat it as the best-known set, not a guaranteed-complete one, until checked against
  #   the Ford VIN guide + more real Lightning VINs (both generations, multiple trims). If a real,
  #   valid Lightning VIN is ever seen with a series code outside this set, the set is incomplete
  #   and must be updated — until then an unrecognized series code fails safe (no match -> MOCK,
  #   not a wrong assignment), never a wrong assignment.
  #
  #   NOT INCLUDED: position-8 codes 'R' and 'U'. The design doc (§3.2, §9.1) flags these as
  #   ⚠ UNVERIFIED — sourced only from forum/decoder threads and conflicting with the
  #   owner-provided reference table. TODO(§9): verify R/U against the Ford Pro VIN guide (Rev 11)
  #   and additional real ER/Flash VINs before adding them here. Until verified, a VIN carrying an
  #   unlisted position-8 code simply fails to match (fail-safe -> MOCK, not a wrong assignment) —
  #   see design doc §8 "unknown/absent position-8 code".
  #
  # year "2022-2025": the Lightning was produced ONLY in these four model years; 2026 was never
  #   built (design doc §3.1, §4.3). Bounding the year here is a VALIDITY gate, not a platform
  #   split (there is only one Lightning platform) — it exists specifically so a would-be 2026 VIN
  #   (position 10 = 'T') does NOT get mapped to MK1. Every other Lightning field (trim, battery
  #   size, fleet-vs-retail) is irrelevant to the openpilot platform choice; all map to the same
  #   FORD_F_150_LIGHTNING_MK1 (design doc §4.3, §8 "fleet ER-Pro / retail-upsold trucks").
  VinFallbackEntry(
    make='Ford',
    model='F-150 Lightning',
    year='2022-2025',
    platform='FORD_F_150_LIGHTNING_MK1',
    match={
      'wmi': ['1FT'],
      'span': {'5-7': ['W1E', 'W1B', 'W3L', 'W5L', 'W7L']},
      'pos': {8: ['L', 'V', 'K', 'S', '7', 'M']},
    },
  ),
]


def _entry_matches_unsafe(vin: str, entry: VinFallbackEntry) -> bool:
  """§5 steps 2-5: evaluate one entry's declarative `match` spec (+ year) against a valid VIN.

  "Unsafe" = this function may raise (TypeError/ValueError/AttributeError/IndexError/KeyError) on a
  malformed entry (bad `match` shape, unparseable `pos`/`span` keys, an unparseable `year`). It is
  never called directly outside this module — `_entry_matches` (below) is the fail-safe wrapper
  every caller actually uses.
  """
  match = entry.match

  wmi_list = match.get('wmi')
  if wmi_list is not None and not any(vin.startswith(prefix) for prefix in wmi_list):
    return False

  pos_map = match.get('pos')
  if pos_map is not None:
    for position, allowed_chars in pos_map.items():
      if _char_at(vin, position) not in allowed_chars:
        return False

  span_map = match.get('span')
  if span_map is not None:
    for span, allowed_substrings in span_map.items():
      if _span_at(vin, span) not in allowed_substrings:
        return False

  if entry.year is not None:
    years = _expand_years(entry.year)
    allowed_position_10_chars = {YEAR_TO_POSITION_10[y] for y in years if y in YEAR_TO_POSITION_10}
    if _char_at(vin, 10) not in allowed_position_10_chars:
      return False

  return True


def _entry_matches(vin: str, entry: VinFallbackEntry) -> bool:
  """N2 (adversarial-review should-fix, fixed 2026-08-10): fail-safe wrapper around
  `_entry_matches_unsafe`. The registry is DATA reviewed as data, not code — a future typo (a `year`
  that isn't one of the three documented shapes, a dash-less `span` key, a `pos` key outside 1-17,
  a non-dict `match`, ...) must never propagate out of this module and crash `fingerprint()` at car
  startup (car_helpers.py calls `decode_vin_platform()` with no guard of its own beyond the one this
  wrapper provides — see also car_helpers.py's own try/except around that call, defense in depth).
  A malformed entry is treated as simply not matching (skipped), exactly like a well-formed entry
  that legitimately doesn't match this VIN — it never poisons evaluation of the OTHER entries in the
  registry. Every skip is logged loudly so a real typo gets caught and fixed."""
  try:
    return _entry_matches_unsafe(vin, entry)
  except Exception as e:
    carlog.error({"event": "VIN decode registry entry malformed - skipping entry (fail-safe)",
                  "entry": repr(entry), "error": repr(e)})
    return False


def _specificity(entry: VinFallbackEntry) -> int:
  """§5 "most-specific wins": count of constrained positions (pos keys + span chars) + a year bonus.

  wmi is intentionally NOT counted — every shipped/likely row constrains wmi, so it doesn't help
  rank one row over another, and the design doc's ranking language ("more `match` constraints + a
  `year` outranks an all-years/looser one") is illustrated entirely in terms of pos/span/year.
  """
  match = entry.match
  score = 0
  score += len(match.get('pos', {}))
  for allowed_substrings in match.get('span', {}).values():
    # use the length of the first allowed substring as the "chars constrained" contribution;
    # all alternatives for a given span are expected to be the same length (same character range).
    if allowed_substrings:
      score += len(allowed_substrings[0])
  if entry.year is not None:
    score += 1
  return score


def decode_vin_platform(vin: str) -> str | None:
  """§5 — the one generic matcher. Decodes `vin` against VIN_FALLBACK_REGISTRY and returns the
  winning platform string, or None if there is no match / the match is ambiguous.

  Deliberately does NOT check `platform in interfaces` here (design doc §5 step 6 is delegated to
  the caller) — car_helpers.py reuses its existing `fallback in interfaces` support gate, the same
  one the fleet_vins.json exact-VIN fallback already goes through, so both fallback layers are
  gated identically and any support-gate logging/behavior stays in one place.

  Safety posture (design doc §7): conservative-or-nothing. Any ambiguity - no matching entry, or a
  tie between entries that disagree on platform - returns None (=> caller falls through toward
  MOCK). This function NEVER guesses.
  """
  if not is_valid_vin(vin):
    return None

  candidates = [entry for entry in VIN_FALLBACK_REGISTRY if _entry_matches(vin, entry)]
  if len(candidates) == 0:
    return None

  ranked = sorted(candidates, key=_specificity, reverse=True)
  top_score = _specificity(ranked[0])
  top_entries = [entry for entry in ranked if _specificity(entry) == top_score]

  top_platforms = {entry.platform for entry in top_entries}
  if len(top_platforms) > 1:
    # Fail-safe ambiguity (§5, §7): equally-specific entries disagree on platform. Assign nothing,
    # never guess. Logged loudly (mirrors car_helpers.py's existing carlog.error style) so the
    # conflicting rows can be reviewed and one of them tightened.
    carlog.error({"event": "VIN decode registry AMBIGUOUS - equally-specific entries disagree, assigning nothing",
                  "vin": vin, "candidates": [repr(e) for e in top_entries]})
    return None

  return top_entries[0].platform

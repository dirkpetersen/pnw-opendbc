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


def _char_at(vin: str, position_1_indexed: int) -> str:
  return vin[position_1_indexed - 1]


def _span_at(vin: str, span_1_indexed: str) -> str:
  start_str, _, end_str = span_1_indexed.partition('-')
  start, end = int(start_str), int(end_str)
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
  #   and 2024-25 "1FT6W..." generations — position 4 (body/GVWR code) differs by generation and is
  #   deliberately NOT constrained here; only the 3-char WMI prefix is required.
  #
  # pos {8: [...]}: THE EV-VS-ICE DISCRIMINATOR. Position 8 is the engine/battery code, and this is
  #   the ONLY reliable way to tell a Lightning from a gas F-150 by VIN (design doc §4.2-§4.3): the
  #   series/trim code at positions 5-7 is NOT usable for this because the ICE F-150 and the
  #   Lightning literally SHARE several series letters (e.g. "W3L"/"W5L"/"W7L" appear on both an ICE
  #   XLT/Lariat/Platinum and an EV Flash/Lariat/Platinum) — keying EV detection on the series code
  #   would risk mis-assigning a GAS truck to the Lightning platform, which is a safety-relevant
  #   fingerprinting error (wrong actuator messages / wrong panda safety expectations). Position 8's
  #   codes are electric-only for this model: L/V (2022-23 SR/ER), K/S (2024-25 SR/SR-LFP), 7/M
  #   (2024-25 ER retail/fleet). See design doc §3.2 for the full per-code battery/chemistry table.
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
      'pos': {8: ['L', 'V', 'K', 'S', '7', 'M']},
    },
  ),
]


def _entry_matches(vin: str, entry: VinFallbackEntry) -> bool:
  """§5 steps 2-5: evaluate one entry's declarative `match` spec (+ year) against a valid VIN."""
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

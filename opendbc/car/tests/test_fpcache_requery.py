# fpcache2pnw: tests for the cached-miss live-requery fallback in car_helpers.fingerprint()
# (2026-07-11 poisoned-cache incident: a partial sleepy-bus FW set persisted under a Lightning
# label made every cached-path fingerprint of the session fall to MOCK, even with the truck on).
import os
from types import SimpleNamespace
from unittest import mock

from opendbc.car import car_helpers
from opendbc.car.structs import CarParams
from opendbc.car.vin import VIN_UNKNOWN

LIGHTNING = "FORD_F_150_LIGHTNING_MK1"
LIVE_VIN = "1FT6W3L78SWG05094"


def _cached_params(fw_count=10, vin=LIVE_VIN, brand="ford"):
  # fingerprint() only reads .brand, .carFw and .carVin from cached_params
  return SimpleNamespace(brand=brand, carFw=[object()] * fw_count, carVin=vin)


def _run(cached_params, match_side_effect, live_vin=LIVE_VIN, fleet_cfg=None, live_fw_count=9):
  for k in ("FINGERPRINT", "SKIP_FW_QUERY", "DISABLE_FW_CACHE"):
    os.environ.pop(k, None)

  get_vin_mock = mock.Mock(return_value=(2024, 0, live_vin))
  get_fw_mock = mock.Mock(return_value=[object()] * live_fw_count)
  match_mock = mock.Mock(side_effect=match_side_effect)
  fleet_mock = mock.Mock(return_value=fleet_cfg if fleet_cfg is not None else {})

  with mock.patch.object(car_helpers, "get_vin", get_vin_mock), \
       mock.patch.object(car_helpers, "get_present_ecus", mock.Mock(return_value=set())), \
       mock.patch.object(car_helpers, "get_fw_versions_ordered", get_fw_mock), \
       mock.patch.object(car_helpers, "match_fw_to_car", match_mock), \
       mock.patch.object(car_helpers, "can_fingerprint", mock.Mock(return_value=(None, {}))), \
       mock.patch.object(car_helpers, "pnw_fleet_config", fleet_mock):
    result = car_helpers.fingerprint(mock.Mock(return_value=[]), mock.Mock(), mock.Mock(),
                                     cached_params, num_pandas=1)
  return result, get_vin_mock, get_fw_mock, match_mock


class TestFpCacheRequery:
  def test_poisoned_cache_live_requery_fleet_fallback_rescues(self):
    # Cached FW unmatchable AND live FW unmatchable (post-OTA / sleepy-bus) -> requery must happen,
    # and the fleet fallback must fire on the LIVE-queried VIN, never committing MOCK.
    (candidate, _, vin, _, source, _), get_vin, get_fw, match = _run(
      _cached_params(), match_side_effect=lambda *a: (True, set()),
      fleet_cfg={"vins": {LIVE_VIN: LIGHTNING}})
    assert candidate == LIGHTNING
    assert source == CarParams.FingerprintSource.fixed
    assert vin == LIVE_VIN
    assert get_vin.call_count == 1  # fell back to exactly one live query (no retry loop)
    assert get_fw.call_count == 1
    assert match.call_count == 2    # once on cached FW, once on live FW

  def test_cached_match_success_no_requery(self):
    # Healthy cache: FW matches on the first (cached) attempt -> the live query must NOT run.
    (candidate, _, _, _, source, _), get_vin, get_fw, _ = _run(
      _cached_params(fw_count=22), match_side_effect=lambda *a: (True, {LIGHTNING}))
    assert candidate == LIGHTNING
    assert source == CarParams.FingerprintSource.fw
    assert get_vin.call_count == 0
    assert get_fw.call_count == 0

  def test_cached_miss_live_fw_match_succeeds(self):
    # Cache poisoned but the car is awake: the live requery's FW set matches exactly.
    (candidate, _, _, _, source, _), get_vin, _, match = _run(
      _cached_params(), match_side_effect=[(True, set()), (True, {LIGHTNING})])
    assert candidate == LIGHTNING
    assert source == CarParams.FingerprintSource.fw
    assert get_vin.call_count == 1
    assert match.call_count == 2

  def test_swap_guard_stale_cached_vin_never_labels_the_car(self):
    # Device-swap guard: the fleet fallback must key off the LIVE VIN. With the live VIN unreadable
    # and the fleet file only knowing the (stale) cached VIN, the result must be None (-> MOCK),
    # never the cached VIN's platform.
    (candidate, _, _, _, _, _), get_vin, _, _ = _run(
      _cached_params(vin=LIVE_VIN), match_side_effect=lambda *a: (True, set()),
      live_vin=VIN_UNKNOWN, fleet_cfg={"vins": {LIVE_VIN: LIGHTNING}})
    assert candidate is None
    assert get_vin.call_count == 1

  def test_requery_runs_at_most_once(self):
    # Nothing matches and no fleet config: session ends at None (MOCK) after ONE requery.
    # vinfp2pnw: the live-requeried VIN must be one the vin_fallbacks decode registry does NOT
    # recognize either (LIVE_VIN is the real Lightning reference truck, which the registry now
    # correctly resolves by class - see test_vin_fallbacks.py - so it's no longer a "nothing
    # matches" fixture on its own; a non-Ford VIN keeps this test's original intent).
    (candidate, _, _, _, _, _), get_vin, get_fw, match = _run(
      _cached_params(), match_side_effect=lambda *a: (True, set()), fleet_cfg={},
      live_vin="5YJSA1E26FF123456")
    assert candidate is None
    assert get_vin.call_count == 1
    assert get_fw.call_count == 1
    assert match.call_count == 2

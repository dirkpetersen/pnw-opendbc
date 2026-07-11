import json
import os
import time

from opendbc.car import gen_empty_fingerprint
from opendbc.car.can_definitions import CanRecvCallable, CanSendCallable
from opendbc.car.carlog import carlog
from opendbc.car.structs import CarParams, CarParamsT
from opendbc.car.fingerprints import eliminate_incompatible_cars, all_legacy_fingerprint_cars
from opendbc.car.fw_versions import ObdCallback, get_fw_versions_ordered, get_present_ecus, match_fw_to_car
from opendbc.car.mock.values import CAR as MOCK
from opendbc.car.values import BRANDS
from opendbc.car.vin import get_vin, is_valid_vin, VIN_UNKNOWN

FRAME_FINGERPRINT = 100  # 1s


def load_interfaces(brand_names):
  ret = {}
  for brand_name in brand_names:
    path = f'opendbc.car.{brand_name}'
    CarInterface = __import__(path + '.interface', fromlist=['CarInterface']).CarInterface
    for model_name in brand_names[brand_name]:
      ret[model_name] = CarInterface
  return ret


def _get_interface_names() -> dict[str, list[str]]:
  # returns a dict of brand name and its respective models
  brand_names = {}
  for brand in BRANDS:
    brand_name = brand.__module__.split('.')[-2]
    brand_names[brand_name] = [model.value for model in brand]

  return brand_names


# imports from directory opendbc/car/<name>/
interface_names = _get_interface_names()
interfaces = load_interfaces(interface_names)


# pnw: fleet identity fallback — config lives ON-DEVICE ONLY (VINs are personal data, never in the
# repo): /data/pnw/fleet_vins.json (mode 600), shape:
#   {"vins": {"<17-char VIN>": "<PLATFORM>"}, "no_vin_platform": "<PLATFORM>"}
# "vins": VIN -> platform. A vendor OTA reflashes modules and changes their FW part-number strings,
#   breaking exact FW matching (the 2025 Lightning is exact-match-only: its EPS answers no Ford
#   platform-code query, so fuzzy matching can never rescue it). The VIN never changes.
# "no_vin_platform": two-car-fleet inference — when everything failed AND the live-queried VIN reads
#   UNKNOWN, assume the fleet car whose VIN is unreadable over CAN (the Tesla Raven).
# Missing/unparseable file, or a platform name not in `interfaces` -> stock behavior (MOCK).
PNW_FLEET_FILE = "/data/pnw/fleet_vins.json"


def pnw_fleet_config() -> dict:
  try:
    with open(PNW_FLEET_FILE) as f:
      cfg = json.load(f)
    return cfg if isinstance(cfg, dict) else {}
  except Exception:
    return {}


def can_fingerprint(can_recv: CanRecvCallable) -> tuple[str | None, dict[int, dict]]:
  finger = gen_empty_fingerprint()
  candidate_cars = {i: all_legacy_fingerprint_cars() for i in [0, 1]}  # attempt fingerprint on both bus 0 and 1
  frame = 0
  car_fingerprint = None
  done = False

  while not done:
    # can_recv(wait_for_one=True) may return zero or multiple packets, so we increment frame for each one we receive
    can_packets = can_recv(wait_for_one=True)
    for can_packet in can_packets:
      for can in can_packet:
        # The fingerprint dict is generated for all buses, this way the car interface
        # can use it to detect a (valid) multipanda setup and initialize accordingly
        if can.src < 128:
          if can.src not in finger:
            finger[can.src] = {}
          finger[can.src][can.address] = len(can.dat)

        for b in candidate_cars:
          # Ignore extended messages and VIN query response.
          if can.src == b and can.address < 0x800 and can.address not in (0x7df, 0x7e0, 0x7e8):
            candidate_cars[b] = eliminate_incompatible_cars(can, candidate_cars[b])

      # if we only have one car choice and the time since we got our first
      # message has elapsed, exit
      for b in candidate_cars:
        if len(candidate_cars[b]) == 1 and frame > FRAME_FINGERPRINT:
          # fingerprint done
          car_fingerprint = candidate_cars[b][0]

      # bail if no cars left or we've been waiting for more than 2s
      failed = (all(len(cc) == 0 for cc in candidate_cars.values()) and frame > FRAME_FINGERPRINT) or frame > 200
      succeeded = car_fingerprint is not None
      done = failed or succeeded

      frame += 1

  return car_fingerprint, finger


# **** for use live only ****
def fingerprint(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback,
                cached_params: CarParamsT | None, num_pandas: int = 1) -> tuple[str | None, dict, str, list[CarParams.CarFw], CarParams.FingerprintSource, bool]:
  fixed_fingerprint = os.environ.get('FINGERPRINT', "")
  skip_fw_query = os.environ.get('SKIP_FW_QUERY', False)
  disable_fw_cache = os.environ.get('DISABLE_FW_CACHE', False)
  ecu_rx_addrs = set()

  start_time = time.monotonic()
  if not skip_fw_query:
    if cached_params is not None and cached_params.brand != "mock" and len(cached_params.carFw) > 0 and \
       cached_params.carVin is not VIN_UNKNOWN and not disable_fw_cache:
      carlog.warning("Using cached CarParams")
      vin_rx_addr, vin_rx_bus, vin = -1, -1, cached_params.carVin
      car_fw = list(cached_params.carFw)
      cached = True
    else:
      carlog.warning("Getting VIN & FW versions")
      # enable OBD multiplexing for VIN query
      # NOTE: this takes ~0.1s and is relied on to allow sendcan subscriber to connect in time
      set_obd_multiplexing(True)
      # VIN query only reliably works through OBDII
      vin_rx_addr, vin_rx_bus, vin = get_vin(can_recv, can_send, (0, 1))
      ecu_rx_addrs = get_present_ecus(can_recv, can_send, set_obd_multiplexing, num_pandas=num_pandas)
      car_fw = get_fw_versions_ordered(can_recv, can_send, set_obd_multiplexing, vin, ecu_rx_addrs, num_pandas=num_pandas)
      cached = False

    exact_fw_match, fw_candidates = match_fw_to_car(car_fw, vin)
  else:
    vin_rx_addr, vin_rx_bus, vin = -1, -1, VIN_UNKNOWN
    exact_fw_match, fw_candidates, car_fw = True, set(), []
    cached = False

  if not is_valid_vin(vin):
    carlog.error({"event": "Malformed VIN", "vin": vin})
    vin = VIN_UNKNOWN
  carlog.warning("VIN %s", vin)

  # disable OBD multiplexing for CAN fingerprinting and potential ECU knockouts
  set_obd_multiplexing(False)

  fw_query_time = time.monotonic() - start_time

  # CAN fingerprint
  # drain CAN socket so we get the latest messages
  can_recv()
  car_fingerprint, finger = can_fingerprint(can_recv)

  exact_match = True
  source = CarParams.FingerprintSource.can

  # If FW query returns exactly 1 candidate, use it
  if len(fw_candidates) == 1:
    car_fingerprint = list(fw_candidates)[0]
    source = CarParams.FingerprintSource.fw
    exact_match = exact_fw_match

  # pnw: fleet identity fallback — fires only when FW matching AND CAN fingerprinting both came up
  # empty (post-vendor-OTA FW churn), and only on a LIVE-queried VIN: a cached VIN could be stale
  # from the other fleet car after a device swap and must never label this one (Gemini review catch;
  # CarParamsCache is CLEAR_ON_MANAGER_START so cache can't cross a swap anyway — belt and braces).
  # Logged loudly so a fallback hit is visible and the new FW strings get captured + added to
  # fingerprints.py (see pnw-pilot-deploy skill, vendor-OTA recipe).
  if car_fingerprint is None and not cached:
    fleet = pnw_fleet_config()
    fallback = fleet.get("vins", {}).get(vin) if vin != VIN_UNKNOWN else fleet.get("no_vin_platform")
    if fallback is not None and fallback in interfaces:
      car_fingerprint = fallback
      source = CarParams.FingerprintSource.fixed
      exact_match = True
      carlog.error({"event": "PNW fleet identity fallback", "vin": vin, "car_fingerprint": car_fingerprint,
                    "fw_count": len(car_fw)})
    elif fallback is not None:
      carlog.error({"event": "PNW fleet fallback IGNORED - unknown platform in fleet_vins.json", "platform": str(fallback)})

  if fixed_fingerprint:
    car_fingerprint = fixed_fingerprint
    source = CarParams.FingerprintSource.fixed

  carlog.error({"event": "fingerprinted", "car_fingerprint": str(car_fingerprint), "source": source, "fuzzy": not exact_match,
                "cached": cached, "fw_count": len(car_fw), "ecu_responses": list(ecu_rx_addrs), "vin_rx_addr": vin_rx_addr,
                "vin_rx_bus": vin_rx_bus, "fingerprints": repr(finger), "fw_query_time": fw_query_time})

  return car_fingerprint, finger, vin, car_fw, source, exact_match


def get_car(can_recv: CanRecvCallable, can_send: CanSendCallable, set_obd_multiplexing: ObdCallback, alpha_long_allowed: bool,
            is_release: bool, cached_params: CarParamsT | None = None, num_pandas: int = 1):
  candidate, fingerprints, vin, car_fw, source, exact_match = fingerprint(can_recv, can_send, set_obd_multiplexing, cached_params, num_pandas=num_pandas)

  if candidate is None:
    carlog.error({"event": "car doesn't match any fingerprints", "fingerprints": repr(fingerprints)})
    candidate = "MOCK"

  CarInterface = interfaces[candidate]
  CP: CarParams = CarInterface.get_params(candidate, fingerprints, car_fw, alpha_long_allowed, is_release, docs=False)
  CP.carVin = vin
  CP.carFw = car_fw
  CP.fingerprintSource = source
  CP.fuzzyFingerprint = not exact_match

  return interfaces[CP.carFingerprint](CP)


def get_demo_car_params():
  platform = MOCK.MOCK
  CarInterface = interfaces[platform]
  CP = CarInterface.get_non_essential_params(platform)
  return CP

"""bp71items2pnw: the five F-150 Lightning firmware strings from BluePilot (ford-op/opendbc 2abe9990,
sunnypilot/car/ford/fingerprints_ext.py). A truck reporting any of them must fingerprint as the Lightning (exact
and fuzzy), and the Tesla Raven's firmware match must not change.
"""
import pytest

from opendbc.car.ford.fingerprints import FW_VERSIONS as FORD_FW
from opendbc.car.ford.values import CAR as FORD
from opendbc.car.fw_versions import match_fw_to_car
from opendbc.car.structs import CarParams
from opendbc.car.tesla.fingerprints import FW_VERSIONS as TESLA_FW
from opendbc.car.tesla.values import CAR as TESLA

Ecu = CarParams.Ecu
LIGHTNING = FORD.FORD_F_150_LIGHTNING_MK1
NEW = [
  (Ecu.abs, 0x760, b'NL38-2D053-AF' + b'\x00' * 11),
  (Ecu.abs, 0x760, b'RL38-2D053-BC' + b'\x00' * 11),
  (Ecu.fwdRadar, 0x764, b'ML3T-14D049-AH' + b'\x00' * 10),
  (Ecu.eps, 0x730, b'NL38-14D003-AA' + b'\x00' * 10),
  (Ecu.eps, 0x730, b'NL38-14D003-AC' + b'\x00' * 10),
]


def _live(table, brand, override=None):
  """One CarFw per ECU of `table`, the first listed version, with `override` = (ecu, addr, version) swapped in."""
  fw = []
  for (ecu, addr, sub), versions in table.items():
    version = versions[0]
    if override is not None and (ecu, addr) == override[:2]:
      version = override[2]
    fw.append(CarParams.CarFw(ecu=ecu, fwVersion=version, brand=brand, address=addr, subAddress=sub or 0))
  return fw


@pytest.mark.parametrize("ecu,addr,version", NEW)
def test_new_lightning_fw_matches_lightning_only(ecu, addr, version):
  assert version in FORD_FW[LIGHTNING][(ecu, addr, None)]
  live = _live(FORD_FW[LIGHTNING], "ford", (ecu, addr, version))
  exact, matches = match_fw_to_car(live, "", allow_fuzzy=False, log=False)
  assert exact and matches == {LIGHTNING}
  _, fuzzy = match_fw_to_car(live, "", allow_exact=False, log=False)
  assert fuzzy <= {LIGHTNING}  # fuzzy may decline, but must never pick another Ford


def test_raven_fw_match_unchanged():
  live = _live(TESLA_FW[TESLA.TESLA_MODEL_S_HW3], "tesla")
  exact, matches = match_fw_to_car(live, "", allow_fuzzy=False, log=False)
  assert exact and matches == {TESLA.TESLA_MODEL_S_HW3}

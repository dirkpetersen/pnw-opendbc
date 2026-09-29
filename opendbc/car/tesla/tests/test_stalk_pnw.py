"""teslastalk2pnw: the Raven's speed-control stalk (0x45 STW_ACTN_RQ) as ButtonEvents, and the EPS-refusal detector.

Frames are built by hand (byte 0 = 0x40 idle | SpdCtrlLvr_Stat, exactly what the 166 rlogs in
drives/2026-09-28/tesla-stalk-decode/ show: 0x40 idle, 0x41 FWD, 0x42 RWD, ...) and pushed through the real DBC,
the real CANParser and the real Tesla CarInterface.update -- the path card runs.
"""
import pytest

from opendbc.car import ButtonType, gen_empty_fingerprint
from opendbc.car.can_definitions import CanData
from opendbc.car.car_helpers import interfaces
from opendbc.car.pnw_vehicle import PnwVehicle
from opendbc.car.tesla.carstate import EPS_REFUSAL_FRAMES, next_eps_refusal, stalk_button_code
from opendbc.car.tesla.values import CANBUS, CAR

DT = 10_000_000
IDLE, FWD, RWD, UP_1ST, UP_2ND, DN_1ST, DN_2ND = 0, 1, 2, 16, 4, 32, 8


@pytest.fixture
def canbus(monkeypatch):
  for k in ("party", "vehicle", "radar", "autopilot_party", "powertrain", "chassis", "autopilot_powertrain"):
    monkeypatch.setattr(CANBUS, k, getattr(CANBUS, k))


def stalk_frame(code: int, inv: int = 0) -> bytes:
  return bytes([0x40 | code | (inv << 7), 0, 0, 0x30, 0, 0, 0, 0])


def epas_frame(eac: int, err: int = 0) -> bytes:
  """EPAS_sysStatus 0x370: eacStatus = bits 55..53 (byte 6 top 3 bits), eacErrorCode = byte 2 high nibble."""
  d = bytearray(8)
  d[2] = err << 4
  d[6] = eac << 5
  return bytes(d)


class Car:
  def __init__(self, platform):
    CarInterface = interfaces[platform]
    self.CI = CarInterface(CarInterface.get_params(platform, gen_empty_fingerprint(), [], False, False, False))
    self.chassis = self.CI.can_parsers["chassis"].bus
    self.party = self.CI.can_parsers["party"].bus
    self.t = 0
    self.CI.update([(self.t, [])])

  def tick(self, stalk=None, epas=None):
    self.t += DT
    frames = []
    if stalk is not None:
      frames.append(CanData(0x45, stalk, self.chassis))
    if epas is not None:
      frames.append(CanData(0x370, epas, self.party))
    return self.CI.update([(self.t, frames)])


def presses(cs):
  return [(b.type, b.pressed) for b in cs.buttonEvents]


# --- decode ---------------------------------------------------------------------------------------------------

@pytest.mark.parametrize("raw,want", [(0, 0), (1, 1), (2, 2), (4, 0), (8, 0), (16, 0), (32, 0), (63, 0), (3, 0)])
def test_only_fwd_and_rwd_become_buttons(raw, want):
  """UP/DN detents are speed adjusts and were never proven to be anything else; SNA (63) or a two-bit combination
  must read as no button, not as a guessed one."""
  assert stalk_button_code(raw) == want


def test_fwd_push_is_a_mainCruise_press_then_release(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  car.tick(stalk_frame(IDLE))
  assert presses(car.tick(stalk_frame(FWD))) == [(ButtonType.mainCruise, True)]
  assert presses(car.tick(stalk_frame(FWD))) == []          # held: no repeat event
  assert presses(car.tick(stalk_frame(IDLE))) == [(ButtonType.mainCruise, False)]


def test_rwd_pull_is_a_resumeCruise_press(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  car.tick(stalk_frame(IDLE))
  assert presses(car.tick(stalk_frame(RWD))) == [(ButtonType.resumeCruise, True)]


@pytest.mark.parametrize("code", [UP_1ST, UP_2ND, DN_1ST, DN_2ND])
def test_speed_detents_emit_nothing(canbus, code):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  car.tick(stalk_frame(IDLE))
  assert presses(car.tick(stalk_frame(code))) == []
  assert presses(car.tick(stalk_frame(IDLE))) == []


def test_switching_from_fwd_to_a_detent_releases_fwd(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  car.tick(stalk_frame(FWD))
  assert presses(car.tick(stalk_frame(UP_2ND))) == [(ButtonType.mainCruise, False)]


@pytest.mark.parametrize("platform", [CAR.TESLA_MODEL_S_HW1, CAR.TESLA_MODEL_S_HW2, CAR.TESLA_MODEL_X_HW1,
                                      CAR.TESLA_MODEL_X_HW2])
def test_only_the_measured_raven_emits(canbus, platform):
  """The stalk was decoded on the HW3 Raven only; nothing changes on any other Tesla (or the Lightning)."""
  car = Car(platform)
  car.tick(stalk_frame(IDLE))
  assert presses(car.tick(stalk_frame(FWD))) == []


def test_capability_is_raven_only():
  class CP:
    def __init__(self, fp):
      self.carFingerprint = fp
      self.openpilotLongitudinalControl = False
  assert PnwVehicle(CP("TESLA_MODEL_S_HW3")).stalk_cruise_buttons
  assert PnwVehicle(CP("TESLA_MODEL_S_HW3")).eps_refusal_alert
  for fp in ("FORD_F_150_LIGHTNING_MK1", "TESLA_MODEL_Y", "TESLA_MODEL_S_HW1"):
    assert not PnwVehicle(CP(fp)).stalk_cruise_buttons
    assert not PnwVehicle(CP(fp)).eps_refusal_alert


# --- EPS refusal detector -------------------------------------------------------------------------------------

def _run(seq):
  out, cnt = [], 0
  for commanded, active in seq:
    fired, cnt = next_eps_refusal(cnt, commanded, active)
    out.append(fired)
  return out


def test_fires_exactly_at_the_threshold():
  fired = _run([(True, False)] * (EPS_REFUSAL_FRAMES + 5))
  assert fired.index(True) == EPS_REFUSAL_FRAMES - 1
  assert all(fired[EPS_REFUSAL_FRAMES - 1:])


def test_the_longest_observed_handover_never_fires():
  """21 frames (0.21 s) is the longest lat-active-but-not-ACTIVE stretch in 482,584 recorded frames."""
  assert not any(_run([(True, False)] * 21 + [(True, True)] * 100))


def test_one_good_frame_resets_the_run():
  assert not any(_run(([(True, False)] * (EPS_REFUSAL_FRAMES - 1) + [(True, True)]) * 4))


def test_not_commanding_never_fires():
  """Fully engaged, driver override, or lateral off: the detector judges only the steering-only command."""
  assert not any(_run([(False, False)] * 500))


def test_clears_when_the_eps_recovers():
  fired = _run([(True, False)] * EPS_REFUSAL_FRAMES + [(True, True)])
  assert fired[-2] and not fired[-1]


def _feed(car, n, eac, commanded=True):
  cs = None
  for _ in range(n):
    car.CI.CS.lat_only_commanded = commanded
    cs = car.tick(epas=epas_frame(eac))
  return cs


def test_carstate_raises_steerFaultTemporary_when_the_eps_is_not_active(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  cs = _feed(car, 5, eac=2)
  assert not cs.steerFaultTemporary
  cs = _feed(car, EPS_REFUSAL_FRAMES - 1, eac=1)      # AVAILABLE: the EPS is not applying our command
  assert not cs.steerFaultTemporary
  cs = _feed(car, 3, eac=1)
  assert cs.steerFaultTemporary and car.CI.CS.eps_refused
  assert (car.CI.CS.eac_status_raw, car.CI.CS.eac_error_raw) == (1, 0)
  cs = _feed(car, 1, eac=2)
  assert not cs.steerFaultTemporary and not car.CI.CS.eps_refused


def test_carstate_does_not_judge_a_car_that_is_not_lateral_only(canbus):
  car = Car(CAR.TESLA_MODEL_S_HW3)
  cs = _feed(car, EPS_REFUSAL_FRAMES * 3, eac=1, commanded=False)
  assert not cs.steerFaultTemporary and not car.CI.CS.eps_refused


def _apply(car, lat_active, enabled, hands_on=0.0):
  from opendbc.car import structs
  car.CI.CS.hands_on_level = hands_on
  CC = structs.CarControl()
  CC.latActive, CC.enabled = lat_active, enabled
  car.CI.apply(CC.as_reader(), car.t)
  return car.CI.CS.lat_only_commanded


def test_carcontroller_tells_carstate_when_lateral_is_commanded_without_full_engagement(canbus):
  """The detector's `commanded` input is set by CarController.update (carstate cannot see carControl)."""
  car = Car(CAR.TESLA_MODEL_S_HW3)
  assert _apply(car, lat_active=True, enabled=False) is True       # steering-only
  assert _apply(car, lat_active=True, enabled=True) is False       # normal engagement: not judged
  assert _apply(car, lat_active=False, enabled=False) is False     # nothing commanded
  assert _apply(car, lat_active=True, enabled=False, hands_on=3.0) is False   # driver override: the EPS is SUPPOSED to inhibit


def test_the_stalk_frame_is_alive_checked_on_the_raven(canbus):
  """Rule 2: a lost 0x45 must surface as a CAN error, not as a silently dead "off" button. Reading vl["STW_ACTN_RQ"] in
  update_legacy registers the message (alive-checked) on the first update, so no explicit subscription is needed."""
  assert 0x45 in Car(CAR.TESLA_MODEL_S_HW3).CI.can_parsers["chassis"].addresses

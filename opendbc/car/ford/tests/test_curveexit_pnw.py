"""
curveexit2pnw -- angle-mode curve-exit fixes (drives/2026-10-21/seattle-left-curve-overshoot).

Fix 2 (after the Gemini 4 Argon review): the deviation clip's speed gate STAYS at bp-7.0's 9 m/s. ford.h checks the
shadow curvature only above 10.0 m/s, but the shadow it judges is the one computed one steer frame earlier, and
Veh_V_ActlBrk carries single-sample spikes of up to 0.53 m/s. These tests pin the panda numbers to ford.h, pin the
shadow age to the real CarController's send order, and run the worst observed case through the real libsafety.

Run: pytest opendbc/car/ford/tests/test_curveexit_pnw.py -q   (needs cereal -- pnw-pilot venv)
"""
import importlib.util
import re
from pathlib import Path

import numpy as np
import pytest

from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.values import CAR as FORD, CarControllerParams

HAVE_CEREAL = importlib.util.find_spec("cereal") is not None
pytestmark = pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH -- run from the pnw-pilot venv")

FORD_H = Path(__file__).resolve().parents[3] / "safety" / "modes" / "ford.h"


def _ford_limits_field(name: str) -> float:
  """A field of ford.h's FORD_LIMITS macro, the struct FORD_CANFD_STEERING_LIMITS is built from."""
  text = FORD_H.read_text()
  macro = text[text.index("#define FORD_LIMITS("):text.index("FORD_PATH_ANGLE_LIMITS = {")]
  m = re.search(r"\." + name + r"\s*=\s*([0-9.]+)", macro)
  assert m is not None, f"{name} not found in ford.h FORD_LIMITS -- the safety file changed shape, re-check fix 2"
  return float(m.group(1))


# Worst case the gate must cover, measured on route 00000322--40000d276d (rlog seg 27/31/32 at 100 Hz, qlogs of all 36
# segments) and derived from carcontroller's send order (test_lmc2_is_judged_against_the_previous_steer_frames_shadow).
SHADOW_AGE_S = 0.15         # 50 ms (one steer step) + up to 20 ms speed-sample staleness + host/USB latency, with margin
SPEED_SPIKE = 0.53          # m/s, largest single-sample step of Veh_V_ActlBrk seen near 7-13 m/s
A_MAX_OBSERVED = 2.6        # m/s2, largest aEgo seen near the gate in those logs (4 m/s2 would need a gate of 8.87)


def test_gate_constants_match_ford_h():
  from opendbc.car.ford.lateral_angle_pnw import _PANDA_SHADOW_MIN_SPEED, _CLIP_MIN_SPEED
  assert _ford_limits_field("angle_error_min_speed") == _PANDA_SHADOW_MIN_SPEED
  # the tolerance is NOT widened: CURVATURE_ERROR already equals the panda's max_angle_error
  assert _ford_limits_field("max_angle_error") / _ford_limits_field("angle_deg_to_can") == CarControllerParams.CURVATURE_ERROR
  # the gate may never be higher than what the observed worst case allows
  assert _CLIP_MIN_SPEED <= _PANDA_SHADOW_MIN_SPEED - (SPEED_SPIKE + A_MAX_OBSERVED * SHADOW_AGE_S)


def test_gate_decisions():
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  gate = LateralAngleExt.deviation_clip_active
  assert not gate(9.0)
  assert gate(9.01) and gate(9.3) and gate(10.0)


def test_lmc2_is_judged_against_the_previous_steer_frames_shadow():
  """Pins SHADOW_AGE_S's 50 ms term to the real CarController: replay its can_sends in order through the panda's latch
  rule (Lane_Assist_Data1 bytes 5-6 latch the shadow; LateralMotionControl2 is judged against the latch) and check that
  every LMC2 is judged against the shadow computed exactly one steer step (5 frames) earlier, never older."""
  from opendbc.car import DT_CTRL, structs
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  CI = interfaces[FORD.FORD_F_150_LIGHTNING_MK1]
  ci = CI(CI.get_params(FORD.FORD_F_150_LIGHTNING_MK1, {b: {} for b in range(7)}, [], alpha_long=False,
                        is_release=False, docs=False).as_reader())
  ci.update([])
  ci.CC._latext = None
  ci.CC._latext_angle = LateralAngleExt(ci.CP)
  ci.CS.out.vEgo = ci.CS.out.vEgoRaw = 5.0          # below the clip: the shadow is the plan, distinct every steer frame
  shadow_frame = {}                                  # wire raw value -> control frame that computed it
  latched = None
  ages = []
  now = 0
  for i in range(120):
    cc = structs.CarControl()
    cc.enabled = cc.latActive = True
    cc.actuators.curvature = 0.0001 * (1 + i // CarControllerParams.STEER_STEP)
    frame = ci.CC.frame
    _, sends = ci.apply(cc.as_reader(), now)
    now += int(DT_CTRL * 1e9)
    if frame % CarControllerParams.STEER_STEP == 0:
      shadow_frame.setdefault(int(round(-ci.CC._latext_angle.bp_kappa_cmd / 1e-6)), frame)
    for addr, dat, _bus in sends:
      if addr == 0x3CA:                              # Lane_Assist_Data1: the panda latches the shadow here
        raw = (dat[5] << 8) | dat[6]
        latched = raw - 0x10000 if raw & 0x8000 else raw
      elif addr == 0x3D6 and latched is not None and latched in shadow_frame:
        ages.append(frame - shadow_frame[latched])
  assert len(ages) > 15, ages
  assert max(ages) == CarControllerParams.STEER_STEP, ages


def _ext():
  CI = interfaces[FORD.FORD_F_150_LIGHTNING_MK1]
  CP = CI.get_params(FORD.FORD_F_150_LIGHTNING_MK1, {b: {} for b in range(7)}, [], alpha_long=False, is_release=False,
                     docs=False).as_reader()
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  ext = LateralAngleExt(CP)
  ext.path_angle_blend_ratio = 0.0
  ext._apply_tuning = lambda: None
  return ext, CP


class _Out:
  steeringPressed = False
  steeringAngleDeg = 0.0


class _CS:
  lat_ctl_lim_stat = 0

  def __init__(self, v, a, yaw):
    self.out = _Out()
    self.out.vEgo = self.out.vEgoRaw = v
    self.out.aEgo = a
    self.out.yawRate = yaw


class _CC:
  latActive = True


class _Act:
  def __init__(self, k):
    self.curvature = k


@pytest.fixture
def panda():
  from opendbc.safety.tests.test_ford import TestFordCANFDStockSafety
  t = TestFordCANFDStockSafety()
  t.setUp()
  t._drain_reset_latch()
  yield t
  t.tearDown()


def _panda_accepts(t, v_panda, yaw, shadow_wire) -> bool:
  for _ in range(6):
    t._rx(t._speed_msg(v_panda))
    t._rx(t._yaw_rate_msg(yaw / v_panda, v_panda))   # VehYaw_W_Actl = yaw
  t.safety.set_controls_allowed(True)
  t._tx(t._lka_angle_msg(True, shadow_curvature=shadow_wire))
  return t._tx(t._lat_ctl_msg(True, 0, 0, 0, 0))


def test_harness_has_teeth(panda):
  """Above the panda gate a shadow 0.005 off the measurement is blocked; below it the same shadow passes."""
  assert not _panda_accepts(panda, 10.3, 0.0, 0.005)
  assert _panda_accepts(panda, 9.9, 0.0, 0.005)


def test_every_unclipped_shadow_passes_the_panda_at_worst_timing(panda):
  """Every shadow the gate leaves unclipped is judged by the real libsafety with the panda's speed sample SHADOW_AGE_S
  newer, the truck accelerating at A_MAX_OBSERVED, plus a SPEED_SPIKE up-spike on that sample. All must pass. Clipped
  shadows are judged at the same speed sample: the clip itself sits inside the panda band (tolerance unchanged)."""
  ext, CP = _ext()
  n_free = n_clip = 0
  for v in np.arange(8.0, 11.51, 0.05):
    for meas in (-0.025, -0.004, 0.0, 0.004, 0.025):
      for des in (-0.025, -0.008, 0.0, 0.008, 0.025):
        yaw = -meas * v
        ext.update(_CC(), _CS(v, 0.0, yaw), _Act(des), CP)
        shadow_wire = -ext.bp_kappa_cmd              # carcontroller negates onto the wire
        if ext.deviation_clip_active(v):
          n_clip += 1
          ok = _panda_accepts(panda, v, yaw, shadow_wire)
        else:
          n_free += 1
          ok = _panda_accepts(panda, v + A_MAX_OBSERVED * SHADOW_AGE_S + SPEED_SPIKE, yaw, shadow_wire)
        assert ok, f"panda blocked: v={v:.2f} meas={meas} des={des} kappa={ext.bp_kappa_cmd:.5f}"
  assert n_free > 400 and n_clip > 400, (n_free, n_clip)


# --- fix 3: unwind lead, SHADOW ONLY ------------------------------------------------------------------------------

from types import SimpleNamespace as _NS


def _model(kappa_now, kappa_far, v):
  """A modelV2 stub whose curvature goes linearly from kappa_now (t=0) to kappa_far (t=1 s), then holds."""
  from opendbc.car.ford.lateral_angle_pnw import ModelConstants
  k = np.interp(ModelConstants.T_IDXS, [0.0, 1.0], [kappa_now, kappa_far])
  return _NS(orientationRate=_NS(z=list(k * v)), meta=_NS(laneChangeState=0, laneChangeDirection=0))


def _unwind(ext, CP, v, frames=40, pressed=False, start=-0.03, end=0.002):
  """A left curve unwinding: desired falls from `start` toward `end` while the plan's future is further along."""
  leads, pas = [], []
  for i in range(frames):
    des = start + (end - start) * i / frames
    ext.model = _model(des, des + (end - start) * 0.5, v)
    cs = _CS(v, 0.0, -des * v)
    cs.out.steeringPressed = pressed
    res = ext.update(_CC(), cs, _Act(des), CP)
    leads.append((des, ext.unwind_lead_shadow))
    pas.append(res.path_angle)
  return leads, pas


def test_unwind_lead_shadow_bounds():
  ext, CP = _ext()
  leads, _ = _unwind(ext, CP, 8.0)
  assert max(abs(lead) for _, lead in leads) > 0.003, "the shadow never engaged on a plain unwind -- fixture is wrong"
  prev = 0.0
  for des, lead in leads:
    assert lead * des <= 0.0, "lead must point toward centre"
    assert abs(lead) <= min(abs(des), 0.010) + 1e-12, "lead past zero or past its cap"
    assert abs(lead - prev) <= 0.001 + 1e-12, "lead not slewed"
    prev = lead


@pytest.mark.parametrize("v,pressed", [(16.0, False), (8.0, True), (2.0, False)])
def test_unwind_lead_shadow_off(v, pressed):
  ext, CP = _ext()
  leads, _ = _unwind(ext, CP, v, pressed=pressed)
  assert all(lead == 0.0 for _, lead in leads)


def test_unwind_lead_is_never_applied(monkeypatch):
  """The same unwind with the shadow computation stubbed out must put the identical path angle on the wire."""
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  ext, CP = _ext()
  leads, with_shadow = _unwind(ext, CP, 8.0)
  assert max(abs(lead) for _, lead in leads) > 0.003
  monkeypatch.setattr(LateralAngleExt, "_update_unwind_lead_shadow", lambda *a, **k: None)
  ext, CP = _ext()
  _, without = _unwind(ext, CP, 8.0)
  assert with_shadow == without


def test_unwind_lead_shadow_bounds_under_a_fast_plan():
  """The slew lags a plan that moves fast. The hard bounds must still hold on every frame: cap, never past zero,
  and zero once the plan has crossed to the other side."""
  ext, CP = _ext()
  v = 8.0

  def step(des, fut):
    ext.model = _model(des, des + 2 * (fut - des), v)   # the model's value at now + ~0.6 s is about `fut`
    ext.update(_CC(), _CS(v, 0.0, -des * v), _Act(des), CP)
    return ext.unwind_lead_shadow

  for _ in range(20):
    lead = step(-0.025, 0.0)                            # big unwind gap: the cap binds
  assert lead == pytest.approx(0.010)
  lead = step(-0.002, -0.002)                           # plan snaps near straight in one frame
  assert 0.0 <= lead <= 0.002 + 1e-12
  lead = step(+0.003, +0.003)                           # plan crossed to the right
  assert lead == 0.0


@pytest.mark.parametrize("end", ["inactive", "human_turn"])
def test_unwind_episode_is_logged_when_lateral_ends_mid_episode(monkeypatch, end):
  """Gemini review finding 2: a disengage or a human-turn override in the middle of an unwind must still log the
  episode, at the speed of its peak, and leave nothing to leak into the next engagement."""
  import opendbc.car.ford.lateral_angle_pnw as lap
  lines = []
  monkeypatch.setattr(lap.carlog, "info", lambda msg, *a, **k: lines.append(msg))
  ext, CP = _ext()
  _unwind(ext, CP, 8.0, end=-0.01)                    # stops mid-unwind, lead still non-zero
  assert ext.unwind_lead_shadow != 0.0 and ext.unwind_lead_peak >= 0.002
  lines.clear()
  if end == "inactive":
    ext.update(_NS(latActive=False), _CS(20.0, 0.0, 0.0), _Act(0.0), CP)
  else:
    ext._end_unwind_episode("driver turning the wheel (human-turn override)")   # the exact call that path makes
  shadow_lines = [m for m in lines if "unwind-lead SHADOW" in m]
  assert len(shadow_lines) == 1 and "at 8.0 m/s" in shadow_lines[0], lines
  assert ext.unwind_lead_frames == 0 and ext.unwind_lead_peak == 0.0 and ext.unwind_lead_shadow == 0.0

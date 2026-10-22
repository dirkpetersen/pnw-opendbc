"""
curveexit2pnw -- angle-mode curve-exit fixes (drives/2026-10-21/seattle-left-curve-overshoot).

Fix 2: the deviation clip's speed gate now follows what the PANDA enforces (ford.h ford_shadow_curvature_error_check,
active only above FORD_LIMITS .angle_error_min_speed = 10.0 m/s, tolerance 0.002 1/m) instead of bp-7.0's 9 m/s.
These tests pin the constants to ford.h's text and run the gate's decisions through the real libsafety: every shadow
the relaxed gate lets through unclipped must pass the panda even when the panda's speed sample is newer and higher.

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


def test_gate_constants_match_ford_h():
  from opendbc.car.ford.lateral_angle_pnw import _PANDA_SHADOW_MIN_SPEED, _CLIP_SPEED_MARGIN
  assert _ford_limits_field("angle_error_min_speed") == _PANDA_SHADOW_MIN_SPEED
  # the tolerance is NOT widened: CURVATURE_ERROR already equals the panda's max_angle_error
  assert _ford_limits_field("max_angle_error") / _ford_limits_field("angle_deg_to_can") == CarControllerParams.CURVATURE_ERROR
  assert 0.0 < _CLIP_SPEED_MARGIN < _PANDA_SHADOW_MIN_SPEED - 9.0 + 1e-9  # never looser than the panda, never below bp-7.0


def test_gate_decisions():
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  gate = LateralAngleExt.deviation_clip_active
  assert not gate(9.0, 0.0)          # bp-7.0 clipped here; the panda does not check
  assert not gate(9.3, 0.0)          # 2026-10-21 22:39:21.34-21.64 PT
  assert gate(9.6, 0.0)
  assert gate(10.0, 0.0) and gate(12.0, 0.0)
  assert gate(9.1, 4.0)              # hard acceleration toward the panda gate: clip early
  assert not gate(9.3, -3.0)         # braking does not move the gate


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


def test_every_unclipped_command_passes_the_panda(panda):
  """Every frame the relaxed gate leaves unclipped is sent to the panda with a speed sample 0.10 s newer at the same
  acceleration (one stale steer step + LKA latch + CAN latency) plus 0.3 m/s of extra jitter. All must pass. Clipped
  frames are checked at the same speed sample: the clip itself must sit inside the panda band (unchanged tolerance)."""
  ext, CP = _ext()
  n_free = n_clip = 0
  for v in np.arange(8.5, 11.51, 0.1):
    for a in (-2.0, 0.0, 1.0, 3.0, 5.0):
      for meas in (-0.025, -0.004, 0.0, 0.004, 0.025):
        for des in (-0.025, -0.008, 0.0, 0.008, 0.025):
          yaw = -meas * v
          ext.update(_CC(), _CS(v, a, yaw), _Act(des), CP)
          shadow_wire = -ext.bp_kappa_cmd              # carcontroller negates onto the wire
          if ext.deviation_clip_active(v, a):
            n_clip += 1
            ok = _panda_accepts(panda, v, yaw, shadow_wire)
          else:
            n_free += 1
            ok = _panda_accepts(panda, v + max(a, 0.0) * 0.10 + 0.3, yaw, shadow_wire)
          assert ok, f"panda blocked: v={v:.2f} a={a} meas={meas} des={des} kappa={ext.bp_kappa_cmd:.5f}"
  assert n_free > 500 and n_clip > 500, (n_free, n_clip)


# 2026-10-21 22:39:21.34-21.64 PT, rlog seg 31: (vEgoRaw, aEgo, yawRate, actuators.curvature). The plan asks RIGHT (+),
# the truck still yaws LEFT. bp-7.0's 9 m/s clip sent a LEFT path angle (-0.013 to -0.028 rad) on every one of these.
_LEFT_WHILE_PLAN_RIGHT = [
  (9.014, 0.78, 0.0390, 0.004498), (9.078, 0.93, 0.0402, 0.003430), (9.078, 0.92, 0.0390, 0.003538),
  (9.156, 0.90, 0.0358, 0.003617), (9.217, 1.16, 0.0328, 0.003992), (9.264, 1.16, 0.0282, 0.005156),
  (9.281, 0.87, 0.0266, 0.005829),
]


def test_2239_exit_command_follows_the_plan():
  ext, CP = _ext()
  for v, a, yaw, des in _LEFT_WHILE_PLAN_RIGHT:
    res = ext.update(_CC(), _CS(v, a, yaw), _Act(des), CP)
    assert not ext.bp_curvature_deviation_limited
    assert ext.bp_kappa_cmd == pytest.approx(des)
    assert res.path_angle > 0.0, f"v={v}: path angle {res.path_angle} is LEFT while the plan asks RIGHT"


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

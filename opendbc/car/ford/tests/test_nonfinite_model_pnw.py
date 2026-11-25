"""bp71items2pnw: a NaN/inf modelV2 frame must never reach the Ford CAN packer (BluePilot
bluepilot-lateral 5a31711). The packer raises on NaN/inf (int(floor(nan))), which kills card while
steering; test_packer_raises_on_nonfinite pins that failure, the engaged tests prove the gate stops it
on every Ford lateral path (4-signal, angle, pc-blend fallback).

Run from the pnw-pilot venv (needs cereal).
"""
import importlib.util
import math
from types import SimpleNamespace

import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford.fordlat_pnw import NONFINITE_HOLD_FRAMES, FiniteModelGate, model_lateral_finite
from opendbc.car.ford.values import CAR as FORD

HAVE_CEREAL = importlib.util.find_spec("cereal") is not None
NAN = float("nan")


def _fake_model(bad=None, value=NAN):
  m = SimpleNamespace(orientationRate=SimpleNamespace(z=[0.01] * 33), position=SimpleNamespace(y=[0.0] * 33),
                      laneLineProbs=[0.9] * 4, laneLines=[SimpleNamespace(y=[1.0] * 33) for _ in range(4)])
  if bad == "orientationRate":
    m.orientationRate.z[5] = value
  elif bad == "laneLineProbs":
    m.laneLineProbs[2] = value
  elif bad == "laneLines":
    m.laneLines[1].y[0] = value
  elif bad == "position":
    m.position.y[3] = value
  return m


@pytest.mark.parametrize("bad", ["orientationRate", "laneLineProbs", "laneLines", "position"])
@pytest.mark.parametrize("value", [NAN, float("inf"), -float("inf")])
def test_every_read_field_is_checked(bad, value):
  assert model_lateral_finite(_fake_model())
  assert not model_lateral_finite(_fake_model(bad, value))


def test_gate_holds_last_good_then_drops_to_none():
  g = FiniteModelGate()
  good = _fake_model()
  assert g.filter(good) is good
  for _ in range(NONFINITE_HOLD_FRAMES):
    assert g.filter(_fake_model("orientationRate")) is good   # short glitch: last good frame held
  assert g.filter(_fake_model("orientationRate")) is None     # persistent: no model terms, never stale
  assert g.dropped == NONFINITE_HOLD_FRAMES + 1
  good2 = _fake_model()
  assert g.filter(good2) is good2 and g.bad_run == 0           # recovers on the next finite frame


def test_gate_with_no_good_frame_yet_returns_none():
  assert FiniteModelGate().filter(_fake_model("laneLineProbs")) is None


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH")
def test_packer_raises_on_nonfinite():
  """The failure this guards against: the Ford packer raises on NaN/inf instead of clipping."""
  from opendbc.can import CANPacker
  from opendbc.car.ford import fordcan_pnw
  from opendbc.car.ford.values import DBC
  packer = CANPacker(DBC[FORD.FORD_F_150_LIGHTNING_MK1]["pt"])
  CAN = SimpleNamespace(main=0, camera=2)
  with pytest.raises(ValueError):
    fordcan_pnw.create_lat_ctl2_msg(packer, CAN, 1, 0, 1, 0.0, NAN, 0.0, 0.0, 0)
  with pytest.raises(OverflowError):
    fordcan_pnw.create_lat_ctl2_msg(packer, CAN, 1, 0, 1, 0.0, float("inf"), 0.0, 0.0, 0)


def _nan_model_msg():
  import cereal.messaging as messaging
  msg = messaging.new_message("modelV2")
  m = msg.modelV2
  m.orientationRate.z = [NAN] * 33
  m.orientation.x = [0.0] * 33
  m.position.y = [NAN] * 33
  ll = m.init("laneLines", 4)
  for i in range(4):
    ll[i].y = [NAN] * 33
  m.laneLineProbs = [NAN] * 4
  return msg.as_reader().modelV2


def _feed(sm, model):
  """Make a real SubMaster report `model` as a new modelV2 frame on every update()."""
  sm.data["modelV2"] = model

  def update(_timeout=0):
    sm.updated = dict.fromkeys(sm.services, False)
    sm.updated["modelV2"] = True
  sm.update = update


def _make_interface():
  CI = interfaces[FORD.FORD_F_150_LIGHTNING_MK1]
  cp = CI.get_params(FORD.FORD_F_150_LIGHTNING_MK1, {b: {} for b in range(7)}, [], alpha_long=False,
                     is_release=False, docs=False)
  ci = CI(cp.as_reader())
  ci.update([])
  ci.CS.out.vEgo = ci.CS.out.vEgoRaw = 25.0
  ci.CS.out.canValid = True
  return ci


def _engaged():
  cc = structs.CarControl()
  cc.enabled = cc.latActive = True
  cc.actuators.curvature = 0.003
  return cc.as_reader()


def _run(ci, frames=100):
  now = 0
  for _ in range(frames):
    na, _ = ci.apply(_engaged(), now)  # pre-fix: raises ValueError in the packer
    now += int(DT_CTRL * 1e9)
    assert math.isfinite(na.curvature)


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH")
def test_nan_model_4signal_path_keeps_steering():
  ci = _make_interface()
  assert ci.CC._latext is not None
  _feed(ci.CC._latext.sm, _nan_model_msg())
  _run(ci)
  assert ci.CC._latext is not None, "4-signal path fell back to stock: the NaN reached its math"
  assert ci.CC._latext.model is None and ci.CC._latext._model_gate.dropped > 0


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH")
def test_nan_model_angle_path_keeps_steering():
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  ci = _make_interface()
  ci.CC._latext = None
  ci.CC._latext_angle = LateralAngleExt(ci.CP)
  _feed(ci.CC._latext_angle.sm, _nan_model_msg())
  _run(ci)
  assert ci.CC._latext_angle is not None, "angle path fell back to stock: the NaN reached its math"
  assert ci.CC._latext_angle.model is None and ci.CC._latext_angle._model_gate.dropped > 0
  assert math.isfinite(ci.CC._shadow_curvature)


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH")
def test_nan_model_pcblend_fallback_path():
  import cereal.messaging as messaging
  from opendbc.car.ford.fordlat_pnw import FiniteModelGate as Gate
  ci = _make_interface()
  ci.CC._latext = None
  ci.CC._pcblend_enabled = True
  ci.CC._pcblend_sm = messaging.SubMaster(["modelV2"])
  ci.CC._pcblend_tidxs = [0.0] * 33
  ci.CC._pcblend_gate = Gate()
  ci.CC._pcblend_model = None
  _feed(ci.CC._pcblend_sm, _nan_model_msg())
  ci.CC._pcblend_sm.alive["modelV2"] = True
  _run(ci)
  assert ci.CC._pcblend_gate.dropped > 0


def _short(field):
  m = _fake_model()
  if field == "orientationRate":
    m.orientationRate.z = m.orientationRate.z[:17]  # np.interp against the 33 T_IDXS raises on this
  elif field == "position":
    m.position.y = []
  elif field == "laneLines":
    m.laneLines = m.laneLines[:2]
  elif field == "laneLineY":
    m.laneLines[2].y = []
  elif field == "laneLineProbs":
    m.laneLineProbs = m.laneLineProbs[:2]
  return m


@pytest.mark.parametrize("field", ["orientationRate", "position", "laneLines", "laneLineY", "laneLineProbs"])
def test_short_frames_are_rejected(field):
  assert not model_lateral_finite(_short(field))


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH")
def test_model_idx_n_matches_modeld():
  from openpilot.selfdrive.modeld.constants import ModelConstants
  from opendbc.car.ford.fordlat_pnw import MODEL_IDX_N
  assert MODEL_IDX_N == ModelConstants.IDX_N == len(ModelConstants.T_IDXS)


class _SM:
  def __init__(self):
    self.updated, self.alive, self.data = {"modelV2": False}, {"modelV2": True}, {}

  def __getitem__(self, k):
    return self.data[k]


def test_step_drops_the_model_when_modelv2_dies(monkeypatch):
  import opendbc.car.ford.fordlat_pnw as fl
  logged = []
  monkeypatch.setattr(fl.carlog, "error", lambda msg, *a, **k: logged.append(msg))
  g, sm, good = FiniteModelGate(), _SM(), _fake_model()
  sm.updated["modelV2"], sm.data["modelV2"] = True, good
  cur = g.step(sm, None)
  assert cur is good
  sm.updated["modelV2"] = False
  assert g.step(sm, cur) is good                  # alive, no new frame: keep using it
  sm.alive["modelV2"] = False
  assert g.step(sm, cur) is None                  # dead: never steer on a frozen frame
  assert any("not alive" in m for m in logged)
  assert g.filter(_fake_model("position")) is None  # and no pre-outage frame is held afterwards


@pytest.mark.skipif(not HAVE_CEREAL, reason="cereal not on PYTHONPATH")
@pytest.mark.parametrize("path", ["4signal", "angle"])
def test_dead_modelv2_clears_the_held_model(path):
  from opendbc.car.ford.lateral_angle_pnw import LateralAngleExt
  ci = _make_interface()
  if path == "angle":
    ci.CC._latext = None
    ci.CC._latext_angle = LateralAngleExt(ci.CP)
  ext = ci.CC._latext or ci.CC._latext_angle
  ext.model = _fake_model()  # a good frame from before the outage
  _run(ci, 5)                 # real SubMaster, no publisher: modelV2 is not alive
  assert ext.model is None

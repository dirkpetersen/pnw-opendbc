"""toggles2pnw: "Disable Ford Convenience Features" (param DisableFordConvenience) gates every non-driving CAN write the comma
makes on the Ford -- today only the Pro Power Onboard re-arm (0x455).

Part 1: ConvenienceGate in isolation (fail-open, logged, ~1 Hz, change-logged).
Part 2: the real Lightning CarController with a fake Params: the toggle ON transmits NOTHING on 0x455 and constructs no armer;
OFF is today's behaviour; a mid-run flip takes effect without a restart; an unreadable param keeps the feature RUNNING and logs.
Run from the pnw-pilot venv (needs cereal + openpilot.common.params), like the other Ford engaged tests.
"""
import importlib.util
import time

import pytest

from opendbc.car import DT_CTRL, structs
from opendbc.car.car_helpers import interfaces
from opendbc.car.ford import lightning_extra_pnw as lx
from opendbc.car.ford.values import CAR as FORD

HAVE_CEREAL = importlib.util.find_spec("cereal") is not None and importlib.util.find_spec("openpilot") is not None


# ---- part 1: the gate -------------------------------------------------------------------------------------------------
class Clock:
  def __init__(self):
    self.t = 100.0

  def __call__(self):
    return self.t


def make_gate(values):
  """values: dict with 'v' (the param) -- or an Exception instance to raise."""
  clk, logs, errs = Clock(), [], []

  def get_bool(k):
    assert k == "DisableFordConvenience"
    if isinstance(values["v"], Exception):
      raise values["v"]
    return values["v"]
  return lx.ConvenienceGate(get_bool, clk, logs.append, errs.append), clk, logs, errs


def test_default_is_off_features_run():
  g, _, logs, errs = make_gate({"v": False})
  assert g.disabled() is False and logs == [] and errs == []


def test_on_is_reported_and_logged_once():
  v = {"v": True}
  g, clk, logs, errs = make_gate(v)
  assert g.disabled() is True
  for _ in range(5):
    clk.t += 1.0
    assert g.disabled() is True
  assert len(logs) == 1 and "DISABLED" in logs[0] and "transmit nothing" in logs[0] and errs == []


def test_read_is_throttled_to_about_1hz():
  v = {"v": False}
  g, clk, _, _ = make_gate(v)
  assert g.disabled() is False
  v["v"] = True
  clk.t += lx.CONVENIENCE_READ_S / 2
  assert g.disabled() is False, "no re-read inside the interval"
  clk.t += lx.CONVENIENCE_READ_S
  assert g.disabled() is True


def test_flip_back_off_is_logged_and_features_resume():
  v = {"v": True}
  g, clk, logs, _ = make_gate(v)
  g.disabled()
  v["v"] = False
  clk.t += 2
  assert g.disabled() is False
  assert len(logs) == 2 and "ENABLED again" in logs[1]


def test_unreadable_param_fails_open_and_is_logged_not_spammed():
  g, clk, logs, errs = make_gate({"v": RuntimeError("UnknownKeyName")})
  for _ in range(30):
    clk.t += 1.0
    assert g.disabled() is False, "a failed read keeps today's behaviour: the features run"
  assert len(errs) == 1 and "unreadable" in errs[0] and "keep RUNNING" in errs[0], errs
  clk.t += lx.CONVENIENCE_ERR_LOG_S
  g.disabled()
  assert len(errs) == 2, "and says so again after a minute"


def test_stale_on_is_dropped_when_the_param_becomes_unreadable():
  v = {"v": True}
  g, clk, _, errs = make_gate(v)
  assert g.disabled() is True
  v["v"] = RuntimeError("store gone")
  clk.t += 2
  assert g.disabled() is False and errs


# ---- part 2: the real CarController -----------------------------------------------------------------------------------
class FakeParams:
  store = {}          # class-level: the carcontroller builds its own instance
  fail = None

  def __init__(self, *a, **k):
    pass

  def get_bool(self, k):
    if FakeParams.fail is not None and k == "DisableFordConvenience":
      raise FakeParams.fail
    return bool(FakeParams.store.get(k, False))

  def get(self, k, *a, **kw):
    return None


class Rig:
  """A Lightning CarController at standstill in Park with Pro Power reading OFF, on a fake monotonic clock."""

  def __init__(self, monkeypatch, disabled=False, fail=None):
    if not HAVE_CEREAL:
      pytest.skip("cereal/openpilot not importable -- run from the pnw-pilot venv")
    import openpilot.common.params as params_mod
    FakeParams.store = {"DisableFordConvenience": disabled}
    FakeParams.fail = fail
    monkeypatch.setattr(params_mod, "Params", FakeParams)
    self.now = 1000.0
    monkeypatch.setattr(time, "monotonic", lambda: self.now)   # the armer and the gate both read this clock
    from opendbc.car.carlog import carlog
    self.warns, self.errs = [], []
    monkeypatch.setattr(carlog, "warning", lambda m, *a, **k: self.warns.append(m % a if a else m))
    monkeypatch.setattr(carlog, "error", lambda m, *a, **k: self.errs.append(m % a if a else m))
    monkeypatch.setattr(carlog, "exception", lambda m, *a, **k: self.errs.append(m % a if a else m))
    CarInterface = interfaces[FORD.FORD_F_150_LIGHTNING_MK1]
    cp = CarInterface.get_params(FORD.FORD_F_150_LIGHTNING_MK1, {b: {} for b in range(7)}, [], alpha_long=False,
                                 is_release=False, docs=False)
    self.ci = CarInterface(cp.as_reader())
    self.ci.update([])
    out = self.ci.CS.out
    out.standstill, out.canValid, out.vEgo, out.vEgoRaw = True, True, 0.0, 0.0
    out.gearShifter = structs.CarState.GearShifter.park
    self.ci.CS.ppo_valid, self.ci.CS.ppo_on = True, False
    self.nanos = 0
    self.tx = []          # (time, payload) of every 0x455 frame handed out

  def run(self, secs):
    cc = structs.CarControl().as_reader()
    for _ in range(int(secs / DT_CTRL)):
      _, sends = self.ci.apply(cc, self.nanos)
      self.tx += [(self.now, bytes(s[1])) for s in sends if s[0] == lx.PPO_ADDR]
      self.nanos += int(DT_CTRL * 1e9)
      self.now += DT_CTRL


SETTLE = lx.PPO_SETTLE_S + lx.PPO_PRESS_S + 2.0


def test_default_off_the_armer_runs_and_presses_pro_power(monkeypatch):
  r = Rig(monkeypatch, disabled=False)
  assert r.ci.CC._ppo_armer is not None
  r.run(SETTLE)
  assert r.tx and all(p == lx.PPO_PRESS_ON for _, p in r.tx), "today's behaviour: the ON press goes out"


def test_on_constructs_nothing_and_transmits_nothing(monkeypatch):
  r = Rig(monkeypatch, disabled=True)
  assert r.ci.CC._ppo_armer is None, "ON must not even construct the armer"
  r.run(SETTLE + lx.PPO_REARM_S / 20)
  assert r.tx == [], "ON: not one 0x455 frame"
  assert any("DISABLED" in m for m in r.warns), "and it says so"
  assert r.ci.CC._ppo_armer is None


def test_flip_on_mid_run_stops_transmitting_without_a_restart(monkeypatch):
  r = Rig(monkeypatch, disabled=False)
  r.run(lx.PPO_SETTLE_S + 0.2)               # the armer is mid-settle / about to press
  FakeParams.store["DisableFordConvenience"] = True
  r.run(1.5)                                 # > the 1 s read interval
  n = len(r.tx)
  assert r.ci.CC._ppo_armer is None
  r.run(SETTLE)
  assert len(r.tx) == n, "no frame after the flip"


def test_flip_back_off_resumes(monkeypatch):
  r = Rig(monkeypatch, disabled=True)
  r.run(2.0)
  FakeParams.store["DisableFordConvenience"] = False
  r.run(1.5)
  assert r.ci.CC._ppo_armer is not None, "a fresh armer once re-enabled"
  r.run(SETTLE)
  assert r.tx, "and it presses again"


def test_unreadable_param_keeps_the_feature_running_and_logs(monkeypatch):
  r = Rig(monkeypatch, disabled=True, fail=RuntimeError("UnknownKeyName"))
  assert r.ci.CC._ppo_armer is not None, "fail-open: the feature runs"
  r.run(SETTLE)
  assert r.tx, "today's behaviour"
  assert any("unreadable" in m and "keep" in m for m in r.errs), r.errs


def test_a_gate_that_cannot_be_built_keeps_the_feature_running_and_logs(monkeypatch):
  def boom(*a, **k):
    raise RuntimeError("no params")
  import openpilot.common.params as params_mod
  if not HAVE_CEREAL:
    pytest.skip("cereal/openpilot not importable")
  monkeypatch.setattr(params_mod, "Params", boom)
  from opendbc.car.carlog import carlog
  errs = []
  monkeypatch.setattr(carlog, "exception", lambda m, *a, **k: errs.append(m))
  CarInterface = interfaces[FORD.FORD_F_150_LIGHTNING_MK1]
  cp = CarInterface.get_params(FORD.FORD_F_150_LIGHTNING_MK1, {b: {} for b in range(7)}, [], alpha_long=False,
                               is_release=False, docs=False)
  ci = CarInterface(cp.as_reader())
  assert ci.CC._conv_gate is None and ci.CC._ppo_armer is not None
  assert any("convenience gate could not be built" in m for m in errs), errs


def test_driving_tx_is_not_behind_the_convenience_gate():
  """The gate may only ever cover convenience TX: the carcontroller's driving frames must not reference it."""
  import inspect
  from opendbc.car.ford import carcontroller
  src = inspect.getsource(carcontroller)
  assert src.count("_conv_gate") == src.count("self._conv_gate"), "one object, referenced as an attribute only"
  head, _, tail = src.partition("### acc buttons ###")
  assert "_conv_gate" in head and "_conv_gate" not in tail, "the gate is consulted only before the driving/acc section"

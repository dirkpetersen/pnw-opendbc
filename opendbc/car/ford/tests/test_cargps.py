"""cargps2pnw: the Lightning's own GPS fix, decoded from the GWM's APIMGPS messages."""
import pytest
from opendbc.car.ford.carstate import _dm_to_deg


class TestDegreeMinuteCombination:
  """The one place this is easy to get catastrophically wrong."""

  def test_western_longitude_combines_sign_first(self):
    """GPS_Longitude_Degrees is scaled (1,-179) so it arrives ALREADY NEGATIVE, while minutes are
    UNSIGNED magnitudes. Real frame from the 2026-09-05 capture: deg=-122.0, min=21.0, dec=0.904.
    Naive deg + min/60 gives -121.635 -- 57 km east of the truth, and plausible enough to believe."""
    assert _dm_to_deg(-122.0, 21.0, 0.904) == pytest.approx(-122.365067, abs=1e-6)
    assert _dm_to_deg(-122.0, 21.0, 0.904) != pytest.approx(-121.635, abs=1e-3)

  def test_northern_latitude(self):
    """Same frame: deg=47.0, min=40.0, dec=0.3771 -> 47.672952, which matched the comma's own GPS
    to 5.4 m on the vehicle."""
    assert _dm_to_deg(47.0, 40.0, 0.3771) == pytest.approx(47.672952, abs=1e-6)

  def test_southern_and_eastern_hemispheres(self):
    assert _dm_to_deg(-33.0, 30.0, 0.0) == pytest.approx(-33.5)
    assert _dm_to_deg(151.0, 12.0, 0.6) == pytest.approx(151.21)

  def test_zero_degrees_keeps_minutes_positive(self):
    """deg == 0 is not negative, so the minutes must add, not subtract."""
    assert _dm_to_deg(0.0, 30.0, 0.0) == pytest.approx(0.5)

  def test_exact_degree_boundary(self):
    assert _dm_to_deg(-122.0, 0.0, 0.0) == pytest.approx(-122.0)


class TestGpsBusRegistration:
  """cargpsbus2pnw: APIMGPS is on the POWERTRAIN bus, not the camera bus.

  Shipped 2026-09-05 registered on Bus.cam and logged a FROZEN position for an entire drive: the
  panda relays the frame onto bus 2, but a relayed frame carries the TX flag (src = 2 + 128 = 130)
  and CANParser.update() drops anything where `src != self.bus`. Nothing errored -- the last decode
  simply persisted while the published `ts` kept advancing, so it read as live.
  """

  @staticmethod
  def _CP():
    from opendbc.car.ford.values import CAR
    from opendbc.car import structs
    CP = structs.CarParams()
    CP.carFingerprint = CAR.FORD_F_150_LIGHTNING_MK1
    CP.safetyConfigs = [structs.CarParams.SafetyConfig()]
    return CP

  def test_gps_msgs_registered_on_pt_not_cam(self):
    from opendbc.car.ford.carstate import CarState
    from opendbc.car import Bus
    from opendbc.car.ford.fordcan import CanBus
    CP = self._CP()
    parsers = CarState.get_can_parsers(CP)
    pt, cam = parsers[Bus.pt], parsers[Bus.cam]
    assert pt.bus == CanBus(CP).main == 0
    assert cam.bus == CanBus(CP).camera == 2
    for name in CarState.GPS_MSGS:
      addr = pt.dbc.name_to_msg[name].address
      assert addr in pt.addresses, f"{name} must be registered on the POWERTRAIN parser"
      assert addr not in cam.addresses, f"{name} on the camera parser can never decode (src=130)"

  def test_registered_with_nan_frequency_so_can_valid_cannot_fail(self):
    """On Bus.pt this is load-bearing: interfaces.py ANDs every parser's can_valid into
    ret.canValid, so an alive-checked GPS message would take the truck offroad outright."""
    from opendbc.car.ford.carstate import CarState
    from opendbc.car import Bus
    pt = CarState.get_can_parsers(self._CP())[Bus.pt]
    for name in CarState.GPS_MSGS:
      st = pt.message_states[pt.dbc.name_to_msg[name].address]
      assert st.ignore_alive, f"{name} must be ignore_alive"
    assert pt.can_valid, "a parser that has received nothing at all must still be valid"

  def test_relayed_frame_on_bus2_does_not_decode(self):
    """The actual bug, reproduced: same frame, src=0 decodes, src=130 (relay TX) does not."""
    import time
    from opendbc.can.parser import CANParser
    from opendbc.can.packer import CANPacker
    from opendbc.car.ford.carstate import CarState, _dm_to_deg
    from opendbc.car.ford.values import CAR, DBC
    from opendbc.car import Bus
    dbc = DBC[CAR.FORD_F_150_LIGHTNING_MK1][Bus.pt]
    msgs = [(m, float("nan")) for m in CarState.GPS_MSGS]
    pt, cam = CANParser(dbc, msgs, 0), CANParser(dbc, msgs, 2)
    dat = CANPacker(dbc).make_can_msg("APIMGPS_Data_Nav_1_FD1", 0, {
      "GPS_Latitude_Degrees": 47.0, "GPS_Latitude_Minutes": 40.0, "GPS_Latitude_Min_dec": 0.3771,
      "GPS_Longitude_Degrees": -122.0, "GPS_Longitude_Minutes": 21.0, "GPS_Longitude_Min_dec": 0.904,
    })[1]
    t = time.monotonic_ns()
    pt.update([[t, [(0x462, dat, 0)]]])          # as broadcast, on the powertrain bus
    cam.update([[t, [(0x462, dat, 130)]]])       # as relayed by the panda, TX flag set
    assert _dm_to_deg(pt.vl["APIMGPS_Data_Nav_1_FD1"]["GPS_Latitude_Degrees"],
                      pt.vl["APIMGPS_Data_Nav_1_FD1"]["GPS_Latitude_Minutes"],
                      pt.vl["APIMGPS_Data_Nav_1_FD1"]["GPS_Latitude_Min_dec"]) == pytest.approx(47.672952, abs=1e-5)
    assert cam.vl["APIMGPS_Data_Nav_1_FD1"]["GPS_Latitude_Degrees"] == 0.0, \
      "a bus-2 parser must see nothing -- this is what froze the telemetry"

  def test_gps_silence_cannot_take_the_truck_offroad(self):
    """The failure that would matter: interfaces.py ANDs can_valid into ret.canValid, so if a silent
    GPS message could invalidate the POWERTRAIN parser the truck would go undriveable for a
    telemetry field. Drive a real 100 Hz message while APIMGPS never arrives, past the bus-timeout
    horizon, and assert the parser stays valid -- with a control proving it can still go False."""
    import time
    from opendbc.can.parser import CANParser
    from opendbc.can.packer import CANPacker
    from opendbc.car.ford.carstate import CarState
    from opendbc.car.ford.values import CAR, DBC
    from opendbc.car import Bus
    dbc = DBC[CAR.FORD_F_150_LIGHTNING_MK1][Bus.pt]
    msgs = [("BrakeSnData_4", 50)] + [(m, float("nan")) for m in CarState.GPS_MSGS]
    cp = CANParser(dbc, msgs, 0)
    addr, real, _ = CANPacker(dbc).make_can_msg("BrakeSnData_4", 0, {})
    t = time.monotonic_ns()
    for i in range(600):                       # 6 s of real traffic, zero GPS frames
      t += 10_000_000
      cp.update([[t, [(addr, real, 0)]]])
      assert cp.can_valid, f"GPS silence invalidated the powertrain parser at tick {i}"
    # Control: the real message stops -> must go invalid. can_invalid_cnt only advances when the
    # property is READ (parser.py:212), so it has to be evaluatedevery tick, exactly as card.py does.
    for _i in range(600):
      t += 10_000_000
      cp.update([[t, []]])
      last = cp.can_valid
    assert not last, "control failed: parser must still enforce its REAL messages"

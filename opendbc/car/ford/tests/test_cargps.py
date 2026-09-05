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

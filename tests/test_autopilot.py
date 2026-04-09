"""
Tests for the autopilot system.
"""

import math
import pytest
from src.systems.autopilot import (
    AutopilotSystem, LateralMode, VerticalMode, SpeedMode, _heading_error
)


class TestHeadingError:
    def test_zero_error(self):
        assert _heading_error(90, 90) == 0.0

    def test_right_turn(self):
        assert _heading_error(100, 80) == pytest.approx(20.0, abs=0.1)

    def test_left_turn(self):
        assert _heading_error(80, 100) == pytest.approx(-20.0, abs=0.1)

    def test_wrap_360(self):
        # Turning from 350 to 010 (shortest = +20)
        assert _heading_error(10, 350) == pytest.approx(20.0, abs=0.1)

    def test_wrap_left(self):
        # Turning from 010 to 350 (shortest = -20)
        assert _heading_error(350, 10) == pytest.approx(-20.0, abs=0.1)

    def test_max_range(self):
        err = _heading_error(270, 90)
        assert abs(err) == pytest.approx(180.0, abs=1.0)


class TestAutopilotEngage:
    def test_engage_stores_targets(self):
        ap = AutopilotSystem()
        ap.engage(heading=90, altitude=10668, cas_kts=250, vs_fpm=0)
        assert ap.engaged
        assert ap.targets.heading_deg == 90
        assert ap.targets.altitude_m  == 10668

    def test_disengage(self):
        ap = AutopilotSystem()
        ap.engage(heading=90, altitude=5000, cas_kts=250, vs_fpm=0)
        ap.disengage()
        assert not ap.engaged
        assert ap.lateral_mode  == LateralMode.OFF
        assert ap.vertical_mode == VerticalMode.OFF


class TestAutopilotCompute:
    def _make_ap(self) -> AutopilotSystem:
        ap = AutopilotSystem()
        ap.engage(heading=90, altitude=5000, cas_kts=250, vs_fpm=0)
        ap.lateral_mode  = LateralMode.HDG_SEL
        ap.vertical_mode = VerticalMode.ALT_HLD
        ap.speed_mode    = SpeedMode.SPEED
        return ap

    def test_heading_select_turns_toward_target(self):
        ap = self._make_ap()
        ap.targets.heading_deg = 120
        # Current heading = 90, need to turn right → bank rate > 0
        out = ap.compute(0.1, heading_deg=90, altitude_m=5000,
                         cas_kts=250, vs_fpm=0, pitch_deg=2, bank_deg=0,
                         throttle_current=0.75)
        assert out.bank_rate_dps > 0

    def test_altitude_hold_no_pitch_when_on_target(self):
        ap = self._make_ap()
        ap.targets.altitude_m = 5000
        out = ap.compute(0.1, heading_deg=90, altitude_m=5000,
                         cas_kts=250, vs_fpm=0, pitch_deg=2, bank_deg=0,
                         throttle_current=0.75)
        # Pitch rate should be small when exactly on target altitude
        assert abs(out.pitch_rate_dps) < 1.0

    def test_altitude_hold_climbs_when_below(self):
        ap = self._make_ap()
        ap.targets.altitude_m = 6000
        ap.vertical_mode = VerticalMode.VS
        ap.targets.vs_fpm = 1000
        out = ap.compute(0.1, heading_deg=90, altitude_m=5000,
                         cas_kts=250, vs_fpm=0, pitch_deg=2, bank_deg=0,
                         throttle_current=0.75)
        # Should pitch up to climb
        assert out.pitch_rate_dps >= 0

    def test_speed_hold_increases_throttle_when_slow(self):
        ap = self._make_ap()
        ap.targets.cas_kts = 280
        out = ap.compute(0.1, heading_deg=90, altitude_m=5000,
                         cas_kts=240, vs_fpm=0, pitch_deg=2, bank_deg=0,
                         throttle_current=0.6)
        # Should increase throttle (current speed 240 < target 280)
        assert out.throttle > 0.6

    def test_speed_hold_decreases_throttle_when_fast(self):
        ap = self._make_ap()
        ap.targets.cas_kts = 220
        out = ap.compute(0.1, heading_deg=90, altitude_m=5000,
                         cas_kts=270, vs_fpm=0, pitch_deg=2, bank_deg=0,
                         throttle_current=0.8)
        assert out.throttle < 0.8

    def test_disengaged_returns_zero_commands(self):
        ap = AutopilotSystem()   # not engaged
        out = ap.compute(0.1, heading_deg=90, altitude_m=5000,
                         cas_kts=250, vs_fpm=0, pitch_deg=2, bank_deg=0,
                         throttle_current=0.75)
        assert out.pitch_rate_dps == 0.0
        assert out.bank_rate_dps  == 0.0
        assert out.throttle       == 0.75

    def test_mode_line_string(self):
        ap = self._make_ap()
        line = ap.mode_line()
        assert "HDG_SEL" in line
        assert "ALT_HLD" in line
        assert "SPEED" in line

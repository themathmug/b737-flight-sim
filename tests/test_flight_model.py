"""
Tests for the 3-DOF flight model.
"""

import math
import pytest
from src.flight_model import FlightModel, PilotInputs, FlightState, OEW_KG, DEFAULT_FUEL_KG, DEFAULT_PAYLOAD_KG
from src.physics.atmosphere import G


def make_inputs(throttle: float = 0.0,
                pitch_rate: float = 0.0,
                bank_rate: float  = 0.0) -> PilotInputs:
    return PilotInputs(
        throttle1=throttle,
        throttle2=throttle,
        pitch_rate_cmd_dps=pitch_rate,
        bank_rate_cmd_dps=bank_rate,
    )


class TestFlightModelInit:
    def test_default_init(self):
        fm = FlightModel()
        assert fm.state.altitude_m == 0.0
        assert fm.state.on_ground

    def test_airborne_init(self):
        fm = FlightModel(initial_altitude_m=10668, initial_speed_cas_ms=200)
        assert not fm.state.on_ground
        assert fm.state.altitude_m == pytest.approx(10668, abs=1)

    def test_mass_correct(self):
        fm = FlightModel(fuel_kg=14000, payload_kg=15000)
        expected = OEW_KG + 14000 + 15000
        assert fm.state.mass_kg == pytest.approx(expected, rel=0.001)


class TestGroundRoll:
    def test_aircraft_accelerates_on_throttle(self):
        fm = FlightModel()
        fm.state.pitch_deg = 0.0
        inp = make_inputs(throttle=0.95)
        for _ in range(100):   # 10 seconds
            fm.update(0.1, inp)
        # Should have accelerated (engines need ~4s spool)
        assert fm.state.tas_ms > 10.0

    def test_aircraft_stays_on_ground_at_idle(self):
        fm = FlightModel()
        inp = make_inputs(throttle=0.22)
        for _ in range(10):
            fm.update(0.1, inp)
        # At idle with no initial speed, should stay on ground
        assert fm.state.altitude_m == pytest.approx(0.0, abs=0.1)


class TestClimb:
    def test_positive_pitch_causes_climb(self):
        """Aircraft should climb when pitched up with sufficient speed."""
        fm = FlightModel(initial_altitude_m=0, initial_speed_cas_ms=150)
        fm.state.on_ground = False
        fm.state.pitch_deg = 10.0
        inp = make_inputs(throttle=0.9)
        initial_alt = fm.state.altitude_m
        for _ in range(50):   # 5 seconds
            fm.update(0.1, inp)
        assert fm.state.altitude_m > initial_alt

    def test_negative_pitch_causes_descent(self):
        """Aircraft should descend when pitched down from altitude."""
        fm = FlightModel(initial_altitude_m=5000, initial_speed_cas_ms=250)
        fm.state.on_ground = False
        fm.state.pitch_deg = -5.0
        inp = make_inputs(throttle=0.5)
        initial_alt = fm.state.altitude_m
        for _ in range(100):
            fm.update(0.1, inp)
        assert fm.state.altitude_m < initial_alt


class TestTurn:
    def test_bank_changes_heading(self):
        fm = FlightModel(initial_altitude_m=5000, initial_speed_cas_ms=250)
        fm.state.on_ground = False
        fm.state.bank_deg = 20.0
        fm.state.pitch_deg = 2.0
        initial_hdg = fm.state.heading_deg
        inp = make_inputs(throttle=0.75)
        for _ in range(100):   # 10 seconds
            fm.update(0.1, inp)
        # Heading should have changed
        assert fm.state.heading_deg != pytest.approx(initial_hdg, abs=5)

    def test_left_bank_turns_left(self):
        fm = FlightModel(initial_altitude_m=5000, initial_speed_cas_ms=250)
        fm.state.on_ground = False
        fm.state.heading_deg = 90.0
        fm.state.bank_deg = -25.0
        fm.state.pitch_deg = 2.0
        inp = make_inputs(throttle=0.75)
        for _ in range(200):
            fm.update(0.1, inp)
        # With left bank we should turn toward 0/360
        hdg_err = (fm.state.heading_deg - 90.0) % 360.0
        assert hdg_err > 180.0 or hdg_err < 10.0   # heading decreased


class TestFuelBurn:
    def test_fuel_decreases_over_time(self):
        fm = FlightModel(initial_altitude_m=10000, initial_speed_cas_ms=200)
        fm.state.on_ground = False
        initial_fuel = fm.state.fuel_kg
        inp = make_inputs(throttle=0.85)
        for _ in range(100):
            fm.update(0.1, inp)
        assert fm.state.fuel_kg < initial_fuel

    def test_mass_decreases_with_fuel(self):
        fm = FlightModel(initial_altitude_m=10000, initial_speed_cas_ms=200)
        fm.state.on_ground = False
        initial_mass = fm.state.mass_kg
        inp = make_inputs(throttle=0.85)
        for _ in range(100):
            fm.update(0.1, inp)
        assert fm.state.mass_kg < initial_mass


class TestDerivedValues:
    def test_mach_computed(self):
        fm = FlightModel(initial_altitude_m=10668, initial_speed_cas_ms=260)
        fm.state.on_ground = False
        inp = make_inputs(throttle=0.8, pitch_rate=0)
        fm.update(0.1, inp)
        assert fm.state.mach > 0

    def test_vs_fpm_and_vs_ms_consistent(self):
        fm = FlightModel(initial_altitude_m=5000, initial_speed_cas_ms=250)
        fm.state.on_ground = False
        fm.state.pitch_deg = 5.0
        inp = make_inputs(throttle=0.8)
        fm.update(0.1, inp)
        assert fm.state.vs_fpm == pytest.approx(fm.state.vs_ms * 196.85, rel=0.01)

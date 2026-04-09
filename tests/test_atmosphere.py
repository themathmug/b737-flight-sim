"""
Tests for the ISA atmosphere model.
"""

import math
import pytest
from src.physics.atmosphere import (
    temperature, pressure, density, speed_of_sound,
    tas_to_mach, mach_to_tas, cas_to_tas, tas_to_cas,
    dynamic_pressure, T0, P0, RHO0, H_TROPO, T_TROPO
)


class TestTemperature:
    def test_sea_level(self):
        assert temperature(0) == pytest.approx(288.15, abs=0.01)

    def test_10000ft(self):
        # 10 000 ft = 3 048 m → T ≈ 268.34 K
        assert temperature(3048) == pytest.approx(268.34, abs=0.1)

    def test_tropopause(self):
        assert temperature(11000) == pytest.approx(T_TROPO, abs=0.01)

    def test_isothermal_stratosphere(self):
        # Above 11 km temperature should be constant
        assert temperature(15000) == pytest.approx(T_TROPO, abs=0.01)
        assert temperature(20000) == pytest.approx(T_TROPO, abs=0.01)

    def test_negative_altitude_clamped(self):
        # Below sea level should return sea-level value
        assert temperature(-100) == pytest.approx(288.15, abs=0.01)


class TestPressure:
    def test_sea_level(self):
        assert pressure(0) == pytest.approx(101325.0, rel=1e-4)

    def test_decreasing_with_altitude(self):
        assert pressure(5000) < pressure(0)
        assert pressure(10000) < pressure(5000)

    def test_tropopause_pressure(self):
        # At 11 km: ~22632 Pa
        assert pressure(11000) == pytest.approx(22632, rel=0.01)

    def test_fl350(self):
        # FL350 = 10 668 m → ~23 842 Pa
        assert 20000 < pressure(10668) < 26000


class TestDensity:
    def test_sea_level(self):
        assert density(0) == pytest.approx(1.225, abs=0.001)

    def test_decreasing_with_altitude(self):
        assert density(5000) < density(0)
        assert density(10000) < density(5000)

    def test_fl350(self):
        # ~0.380 kg/m³ at FL350
        assert 0.30 < density(10668) < 0.45


class TestSpeedOfSound:
    def test_sea_level(self):
        # Should be ~340.3 m/s
        assert speed_of_sound(0) == pytest.approx(340.3, abs=0.5)

    def test_fl350(self):
        # Should be ~295 m/s
        assert 290 < speed_of_sound(10668) < 300


class TestConversions:
    def test_mach_tas_roundtrip(self):
        tas = 230.0  # m/s
        alt = 10668.0
        mach = tas_to_mach(tas, alt)
        assert mach_to_tas(mach, alt) == pytest.approx(tas, abs=0.01)

    def test_cas_tas_sl(self):
        # At sea level CAS ≈ TAS
        cas = 100.0
        tas = cas_to_tas(cas, 0.0)
        assert tas == pytest.approx(cas, rel=0.005)

    def test_cas_tas_altitude(self):
        # At altitude TAS > CAS
        cas = 150.0
        tas = cas_to_tas(cas, 10668.0)
        assert tas > cas

    def test_cas_tas_roundtrip(self):
        cas = 200.0
        alt = 6000.0
        assert tas_to_cas(cas_to_tas(cas, alt), alt) == pytest.approx(cas, abs=0.1)

    def test_dynamic_pressure_positive(self):
        q = dynamic_pressure(200, 5000)
        assert q > 0

    def test_dynamic_pressure_sl(self):
        # q = 0.5 * 1.225 * 100^2 = 6125 Pa
        q = dynamic_pressure(100, 0)
        assert q == pytest.approx(6125, abs=10)

"""
Tests for the 737-800 aerodynamics model.
"""

import math
import pytest
from src.physics.aerodynamics import (
    AerodynamicsModel, AeroState,
    CL_MAX_CLEAN, CL_MAX_FLAPS40,
    ALPHA_STALL_CLEAN_DEG, S_REF, K_INDUCED,
    ground_effect_factor, CD0_CLEAN
)


class TestGroundEffect:
    def test_on_ground(self):
        ke = ground_effect_factor(0)
        assert ke < 0.5

    def test_far_from_ground(self):
        ke = ground_effect_factor(10000)
        assert ke == pytest.approx(1.0, abs=0.01)

    def test_intermediate(self):
        ke_low = ground_effect_factor(5)
        ke_high = ground_effect_factor(100)
        assert ke_low < ke_high


class TestLiftModel:
    def setup_method(self):
        self.model = AerodynamicsModel()
        self.model.flap_position = 0

    def test_positive_cl_at_positive_alpha(self):
        cl, stall = self.model.cl_from_alpha(5.0)
        assert cl > 0
        assert not stall

    def test_cl_increases_with_alpha(self):
        cl_low, _ = self.model.cl_from_alpha(2.0)
        cl_high, _ = self.model.cl_from_alpha(10.0)
        assert cl_high > cl_low

    def test_stall_at_high_alpha(self):
        cl, stall = self.model.cl_from_alpha(ALPHA_STALL_CLEAN_DEG + 2)
        assert stall

    def test_no_stall_below_stall_angle(self):
        cl, stall = self.model.cl_from_alpha(ALPHA_STALL_CLEAN_DEG - 2)
        assert not stall

    def test_flaps_increase_cl(self):
        self.model.flap_position = 0
        cl_clean, _ = self.model.cl_from_alpha(5.0)
        self.model.flap_position = 30
        cl_flaps, _ = self.model.cl_from_alpha(5.0)
        assert cl_flaps > cl_clean

    def test_cl_max_flaps_greater_than_clean(self):
        self.model.flap_position = 40
        cl_max_f40 = self.model._cl_max()
        self.model.flap_position = 0
        cl_max_clean = self.model._cl_max()
        assert cl_max_f40 > cl_max_clean


class TestDragModel:
    def setup_method(self):
        self.model = AerodynamicsModel()
        self.model.flap_position = 0

    def test_drag_positive(self):
        cd = self.model.cd_from_cl(1.0)
        assert cd > 0

    def test_drag_increases_with_cl(self):
        cd_low  = self.model.cd_from_cl(0.3)
        cd_high = self.model.cd_from_cl(1.5)
        assert cd_high > cd_low

    def test_gear_increases_drag(self):
        self.model.gear_down = False
        cd_clean = self.model.cd_from_cl(0.5)
        self.model.gear_down = True
        cd_gear = self.model.cd_from_cl(0.5)
        assert cd_gear > cd_clean

    def test_flaps_increase_drag(self):
        self.model.flap_position = 0
        cd_clean = self.model.cd_from_cl(0.5)
        self.model.flap_position = 40
        cd_flaps = self.model.cd_from_cl(0.5)
        assert cd_flaps > cd_clean


class TestAeroCompute:
    def setup_method(self):
        self.model = AerodynamicsModel()

    def test_lift_scales_with_speed(self):
        aero1 = self.model.compute(100, 0, 5)
        aero2 = self.model.compute(200, 0, 5)
        assert aero2.lift_n > aero1.lift_n

    def test_drag_scales_with_speed(self):
        aero1 = self.model.compute(100, 0, 5)
        aero2 = self.model.compute(200, 0, 5)
        assert aero2.drag_n > aero1.drag_n

    def test_l_over_d_sensible(self):
        aero = self.model.compute(230, 10668, 3.0)
        ld = aero.lift_n / aero.drag_n
        assert 8 < ld < 20   # typical 737 L/D ~15-17 at cruise

    def test_returns_aerostate(self):
        result = self.model.compute(200, 5000, 5)
        assert isinstance(result, AeroState)
        assert result.lift_n > 0
        assert result.drag_n > 0


class TestStallSpeed:
    def setup_method(self):
        self.model = AerodynamicsModel()

    def test_stall_speed_decreases_with_flaps(self):
        weight = 75000 * 9.80665   # N
        self.model.flap_position = 0
        vs_clean = self.model.v_stall_ms(weight, 0)
        self.model.flap_position = 40
        vs_flaps = self.model.v_stall_ms(weight, 0)
        assert vs_flaps < vs_clean

    def test_stall_speed_increases_with_altitude(self):
        weight = 70000 * 9.80665
        self.model.flap_position = 0
        vs_sl  = self.model.v_stall_ms(weight, 0)
        vs_alt = self.model.v_stall_ms(weight, 10668)
        assert vs_alt > vs_sl

    def test_stall_speed_plausible_at_sl(self):
        # 737-800 Vst clean ~155 kt ≈ 80 m/s at MTOW
        weight = 79016 * 9.80665
        self.model.flap_position = 0
        vs = self.model.v_stall_ms(weight, 0)
        assert 60 < vs < 110

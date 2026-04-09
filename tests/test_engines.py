"""
Tests for the CFM56-7B27 engine model.
"""

import math
import pytest
from src.physics.engines import (
    EngineModel, DualEngineSystem, EngineState,
    THRUST_SL_MAX_N, N1_IDLE, N1_MAX, FF_IDLE_KGH, FF_MAX_SL_KGH
)


class TestEngineModel:
    def setup_method(self):
        self.eng = EngineModel()

    def test_initial_state_idle(self):
        assert self.eng.state.n1_pct == pytest.approx(N1_IDLE, abs=0.1)
        assert self.eng.state.running

    def test_throttle_to_n1_idle(self):
        n1 = EngineModel.throttle_to_n1_target(0.0)
        assert n1 == pytest.approx(N1_IDLE, abs=0.1)

    def test_throttle_to_n1_max(self):
        n1 = EngineModel.throttle_to_n1_target(1.0)
        assert n1 == pytest.approx(N1_MAX, abs=0.1)

    def test_throttle_to_n1_midrange(self):
        n1 = EngineModel.throttle_to_n1_target(0.5)
        assert N1_IDLE < n1 < N1_MAX

    def test_thrust_at_sl_max(self):
        thrust = EngineModel.n1_to_thrust(N1_MAX, 0, 0)
        assert thrust == pytest.approx(THRUST_SL_MAX_N, rel=0.05)

    def test_thrust_at_idle_near_zero(self):
        thrust = EngineModel.n1_to_thrust(N1_IDLE, 0, 0)
        assert thrust < 1000   # idle thrust ≈ 0

    def test_thrust_decreases_with_altitude(self):
        t_sl  = EngineModel.n1_to_thrust(90, 0)
        t_alt = EngineModel.n1_to_thrust(90, 10668)
        assert t_alt < t_sl

    def test_egt_increases_with_n1(self):
        egt_low  = EngineModel.n1_to_egt(N1_IDLE, 0)
        egt_high = EngineModel.n1_to_egt(N1_MAX,  0)
        assert egt_high > egt_low

    def test_fuel_flow_increases_with_n1(self):
        ff_idle = EngineModel.n1_to_fuel_flow(N1_IDLE, 0)
        ff_max  = EngineModel.n1_to_fuel_flow(N1_MAX, 0)
        assert ff_max > ff_idle

    def test_fuel_flow_idle_plausible(self):
        ff = EngineModel.n1_to_fuel_flow(N1_IDLE, 0)
        assert ff == pytest.approx(FF_IDLE_KGH, rel=0.05)

    def test_spool_response(self):
        """N1 should increase toward target after throttle increase."""
        self.eng.state.n1_pct = N1_IDLE
        for _ in range(50):   # 5 seconds
            self.eng.update(0.1, 1.0, 0)
        assert self.eng.state.n1_pct > N1_IDLE + 20

    def test_engine_shutdown(self):
        self.eng.start()
        self.eng.shutdown()
        # After shutdown, running flag is False
        assert not self.eng.state.running
        # After one step, thrust should be 0
        self.eng.update(1.0, 0.5, 0)
        assert self.eng.state.thrust_n == 0.0


class TestDualEngineSystem:
    def setup_method(self):
        self.system = DualEngineSystem()

    def test_total_thrust_both_engines(self):
        for _ in range(10):
            self.system.update(0.1, 0.8, 0.8, 0)
        assert self.system.total_thrust_n > 0

    def test_independent_throttle(self):
        """Each engine responds independently."""
        for _ in range(50):
            self.system.update(0.1, 1.0, 0.0, 0)
        assert self.system.eng1.state.n1_pct > self.system.eng2.state.n1_pct

    def test_total_fuel_flow(self):
        for _ in range(5):
            self.system.update(0.1, 0.5, 0.5, 0)
        assert self.system.total_fuel_flow_kgh > 0

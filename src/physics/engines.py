"""
CFM56-7B27 Engine Model for Boeing 737-800.

Each engine delivers up to 121.4 kN (27,300 lbf) of thrust at sea level.
The model covers:
  - Throttle → N1 relationship
  - N1 → Thrust at altitude (with altitude / Mach corrections)
  - Engine temperatures (EGT proxy)
  - Fuel flow estimation
  - Engine spool rates (transient response)

Reference data from publicly available 737-800 FCOM/QRH performance tables.
"""

import math
from dataclasses import dataclass, field

from src.physics.atmosphere import (
    density, temperature, speed_of_sound, RHO0, T0, G
)

# ── Engine reference data ──────────────────────────────────────────────────────
THRUST_SL_MAX_N: float = 121_400.0   # N per engine at sea level, ISA, static
N1_IDLE: float = 22.0                # % N1 at ground idle
N1_MAX: float  = 100.0               # % N1 at max continuous thrust
N2_PER_N1: float = 0.988             # N2 closely follows N1

# Fuel flow at max thrust SL: ~2 000 kg/h per engine
# At typical cruise (N1 ≈ 90 %): ~2 750 kg/h per engine
FF_IDLE_KGH: float = 180.0           # kg/h – idle fuel flow per engine
FF_MAX_SL_KGH: float = 4_000.0      # kg/h – max thrust fuel flow at SL per engine

# Spool rates (time constants in seconds)
SPOOL_UP_TAU: float   = 4.0    # seconds to go from idle to max
SPOOL_DOWN_TAU: float = 6.0    # seconds to go from max to idle

# ── EGT model (simplified) ────────────────────────────────────────────────────
EGT_IDLE_C: float    =  380.0
EGT_MAX_C: float     =  900.0
EGT_LIMIT_C: float   =  950.0   # typical EGT red-line
EGT_TAKEOFF_C: float =  870.0   # typical takeoff EGT


@dataclass
class EngineState:
    """Mutable state of one CFM56-7B engine."""
    n1_pct: float     = N1_IDLE    # % N1 (fan speed)
    n2_pct: float     = N1_IDLE * N2_PER_N1  # % N2 (core speed)
    egt_c: float      = EGT_IDLE_C  # exhaust gas temperature, °C
    thrust_n: float   = 0.0         # current net thrust, N
    fuel_flow_kgh: float = FF_IDLE_KGH   # fuel flow, kg/h
    throttle_lever: float = 0.0     # physical lever 0–1
    running: bool     = True
    fire: bool        = False
    oil_pressure_psi: float = 65.0


class EngineModel:
    """Simulation model of a single CFM56-7B27 engine."""

    def __init__(self) -> None:
        self.state = EngineState()

    def prespool(self, throttle: float, altitude_m: float = 0.0) -> None:
        """Instantly set engine state to match *throttle* – used for in-flight init."""
        self.state.throttle_lever = max(0.0, min(1.0, throttle))
        self.state.n1_pct = self.throttle_to_n1_target(self.state.throttle_lever)
        self.state.n2_pct = self.state.n1_pct * N2_PER_N1
        self.state.thrust_n = self.n1_to_thrust(self.state.n1_pct, altitude_m)
        self.state.egt_c = self.n1_to_egt(self.state.n1_pct, altitude_m)
        self.state.fuel_flow_kgh = self.n1_to_fuel_flow(self.state.n1_pct, altitude_m)

    # ── Throttle → target N1 ─────────────────────────────────────────────────
    @staticmethod
    def throttle_to_n1_target(throttle: float) -> float:
        """Map normalised throttle lever (0–1) to target %N1.

        At throttle = 0 → N1_IDLE (22 %)
        At throttle = 1 → N1_MAX (100 %)
        The relationship is slightly non-linear (quadratic knee at low power).
        """
        throttle = max(0.0, min(1.0, throttle))
        return N1_IDLE + (N1_MAX - N1_IDLE) * throttle

    # ── N1 → thrust with altitude / Mach correction ──────────────────────────
    @staticmethod
    def n1_to_thrust(n1_pct: float, altitude_m: float, mach: float = 0.0) -> float:
        """Return net thrust (N) for a given %N1 at altitude.

        Thrust correction with altitude:
            T ≈ T_SL_max · (ρ/ρ₀)^0.9 · f(N1)
        Ram drag correction for Mach:
            T_net ≈ T_gross · (1 – 0.25·M²)  (simplified inlet ram drag)
        """
        rho = density(altitude_m)
        altitude_factor = (rho / RHO0) ** 0.9
        # N1 fraction relative to max (above idle)
        n1_fraction = max(0.0, (n1_pct - N1_IDLE) / (N1_MAX - N1_IDLE))
        # Thrust goes roughly as N1^2 relative to max
        thrust_gross = THRUST_SL_MAX_N * altitude_factor * (n1_fraction ** 2)
        # Ram recovery drag (compressible effect)
        ram_correction = max(0.0, 1.0 - 0.15 * mach * mach)
        return thrust_gross * ram_correction

    # ── EGT model ─────────────────────────────────────────────────────────────
    @staticmethod
    def n1_to_egt(n1_pct: float, altitude_m: float) -> float:
        """Return estimated EGT (°C) for given N1 and altitude.

        EGT decreases with altitude due to lower OAT and pressure.
        """
        t_ratio = temperature(altitude_m) / T0
        n1_frac = (n1_pct - N1_IDLE) / (N1_MAX - N1_IDLE)
        n1_frac = max(0.0, min(1.0, n1_frac))
        egt = EGT_IDLE_C + (EGT_MAX_C - EGT_IDLE_C) * n1_frac ** 1.3
        return egt * (0.7 + 0.3 * t_ratio)   # slightly lower at altitude

    # ── Fuel flow model ───────────────────────────────────────────────────────
    @staticmethod
    def n1_to_fuel_flow(n1_pct: float, altitude_m: float) -> float:
        """Return fuel flow (kg/h) for given N1 and altitude.

        Fuel flow scales approximately with N1^3 (power ~ N1^3).
        At altitude, density correction reduces absolute fuel flow.
        """
        rho = density(altitude_m)
        density_factor = (rho / RHO0) ** 0.7
        n1_frac = max(0.0, (n1_pct - N1_IDLE) / (N1_MAX - N1_IDLE))
        ff = FF_IDLE_KGH + (FF_MAX_SL_KGH - FF_IDLE_KGH) * n1_frac ** 2.5
        return ff * density_factor

    # ── Update (integrate over dt) ────────────────────────────────────────────
    def update(
        self,
        dt: float,
        throttle: float,
        altitude_m: float,
        mach: float = 0.0,
    ) -> EngineState:
        """Advance engine state by *dt* seconds.

        Implements first-order lag spool response to throttle changes.
        """
        if not self.state.running:
            self.state.thrust_n = 0.0
            self.state.n1_pct = max(0.0, self.state.n1_pct - 5.0 * dt)
            self.state.n2_pct = self.state.n1_pct * N2_PER_N1
            self.state.egt_c = 15.0
            self.state.fuel_flow_kgh = 0.0
            return self.state

        self.state.throttle_lever = max(0.0, min(1.0, throttle))
        n1_target = self.throttle_to_n1_target(self.state.throttle_lever)

        # First-order spool lag
        n1_err = n1_target - self.state.n1_pct
        tau = SPOOL_UP_TAU if n1_err > 0 else SPOOL_DOWN_TAU
        dn1 = (n1_err / tau) * dt
        self.state.n1_pct += dn1
        self.state.n1_pct = max(N1_IDLE, min(N1_MAX, self.state.n1_pct))
        self.state.n2_pct = self.state.n1_pct * N2_PER_N1

        self.state.thrust_n = self.n1_to_thrust(self.state.n1_pct, altitude_m, mach)
        self.state.egt_c = self.n1_to_egt(self.state.n1_pct, altitude_m)
        self.state.fuel_flow_kgh = self.n1_to_fuel_flow(self.state.n1_pct, altitude_m)

        return self.state

    def start(self) -> None:
        """Begin engine start sequence (instant for simulation)."""
        self.state.running = True
        self.state.n1_pct = N1_IDLE
        self.state.n2_pct = N1_IDLE * N2_PER_N1

    def shutdown(self) -> None:
        """Cut fuel – begin engine winddown."""
        self.state.running = False
        self.state.throttle_lever = 0.0


class DualEngineSystem:
    """Manages both CFM56 engines on the 737-800."""

    def __init__(self) -> None:
        self.eng1 = EngineModel()
        self.eng2 = EngineModel()

    def update(
        self,
        dt: float,
        throttle1: float,
        throttle2: float,
        altitude_m: float,
        mach: float = 0.0,
    ) -> tuple[EngineState, EngineState]:
        e1 = self.eng1.update(dt, throttle1, altitude_m, mach)
        e2 = self.eng2.update(dt, throttle2, altitude_m, mach)
        return e1, e2

    @property
    def total_thrust_n(self) -> float:
        return self.eng1.state.thrust_n + self.eng2.state.thrust_n

    @property
    def total_fuel_flow_kgh(self) -> float:
        return self.eng1.state.fuel_flow_kgh + self.eng2.state.fuel_flow_kgh

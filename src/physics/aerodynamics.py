"""
Boeing 737-800 Aerodynamic Model.

Implements a simplified but study-accurate aerodynamic model based on publicly
available 737-800 performance data.

Wing geometry (737-800):
    Reference area  S   = 125.08 m²
    Wingspan        b   = 35.79 m
    Aspect ratio    AR  = b²/S ≈ 10.24
    Taper ratio     λ   = 0.278
    Sweep (¼-chord) Λ   = 25.0°
    Mean chord      c̄   = S/b ≈ 3.495 m

Aerodynamic coefficients are derived from the DATCOM empirical method and
cross-checked against published 737 performance tables (FCOM-style values).
"""

import math
from dataclasses import dataclass

from src.physics.atmosphere import dynamic_pressure, density, RHO0

# ── Wing reference geometry ────────────────────────────────────────────────────
S_REF: float = 125.08          # m²  – reference wing area
WINGSPAN: float = 35.79        # m
ASPECT_RATIO: float = WINGSPAN**2 / S_REF   # ≈ 10.24
SWEEP_RAD: float = math.radians(25.0)       # quarter-chord sweep

# ── Lift model ─────────────────────────────────────────────────────────────────
# Low-speed lift-slope ∂CL/∂α (rad⁻¹), corrected for sweep and aspect ratio
# Using Helmbold/DATCOM formula: CL_alpha ≈ 2πAR / (2 + sqrt(4 + AR²·β²·(1+tan²Λ)/β²))
# Approximated here for a clean 737-800 wing:
CL_ALPHA: float = 5.5     # /rad
CL_0: float = 0.10        # zero-angle lift (cambered wing)
ALPHA_0: float = math.radians(-2.0)  # zero-lift angle of attack

# Stall angles (geometric AoA, degrees)
ALPHA_STALL_CLEAN_DEG: float = 16.0
ALPHA_STALL_FLAPS_DEG: float = 14.0  # stall angle with flaps extended

# CL max values
CL_MAX_CLEAN: float = 1.55
CL_MAX_FLAPS10: float = 1.90
CL_MAX_FLAPS25: float = 2.15
CL_MAX_FLAPS30: float = 2.35
CL_MAX_FLAPS40: float = 2.55

# ── Drag model ─────────────────────────────────────────────────────────────────
# Oswald efficiency
OSWALD_E: float = 0.82
# k factor in CD = CD0 + k·CL²
K_INDUCED: float = 1.0 / (math.pi * ASPECT_RATIO * OSWALD_E)

# Parasite drag (CD0) for various configurations
CD0_CLEAN: float = 0.0175
CD0_GEAR_DOWN: float = 0.0175 + 0.0200   # gear lowers CD0 by ~0.020
CD0_SPOILERS: float = 0.0175 + 0.0100

# Flap drag increments (added to CD0_CLEAN)
CD_FLAP = {
    0:  0.0000,
    1:  0.0010,   # Flaps 1
    5:  0.0020,   # Flaps 5
    10: 0.0040,   # Flaps 10
    15: 0.0060,   # Flaps 15
    25: 0.0100,   # Flaps 25
    30: 0.0140,   # Flaps 30
    40: 0.0200,   # Flaps 40
}

# Flap CL increment at a given AoA (added to clean CL)
DCL_FLAP = {
    0:  0.00,
    1:  0.10,
    5:  0.25,
    10: 0.50,
    15: 0.70,
    25: 0.90,
    30: 1.00,
    40: 1.20,
}

# ── Ground effect ──────────────────────────────────────────────────────────────
# Ground effect modifies induced drag when h/b < 1
# Approximation from Raymer: Ke = 1 – 33·(h/b)^0.7 / (1 + 33·(h/b)^0.7)
# but for simplicity we use the classical factor:
# Ke = (16·h_wr/b)^2 / (1 + (16·h_wr/b)^2)  where h_wr is height above ground


def ground_effect_factor(height_agl_m: float) -> float:
    """Return the ground-effect induced-drag reduction factor (0 to 1).

    A value of 1 means no ground effect (far from ground).
    A value < 1 reduces induced drag.
    """
    if height_agl_m <= 0:
        return 0.3
    ratio = 16.0 * height_agl_m / WINGSPAN
    return ratio**2 / (1.0 + ratio**2)


# ── Flap configuration helpers ─────────────────────────────────────────────────
_FLAP_POSITIONS = sorted(CD_FLAP.keys())


def _nearest_flap(flap_deg: float) -> int:
    """Return the nearest valid flap position (degrees)."""
    return min(_FLAP_POSITIONS, key=lambda f: abs(f - flap_deg))


@dataclass
class AeroState:
    """Snapshot of all aerodynamic forces / moments for one simulation step."""
    lift_n: float          # Lift force, N
    drag_n: float          # Drag force, N
    cl: float              # Lift coefficient
    cd: float              # Drag coefficient
    alpha_deg: float       # Angle of attack, deg
    q_pa: float            # Dynamic pressure, Pa
    stall: bool            # True if AoA exceeds stall


class AerodynamicsModel:
    """737-800 aerodynamics suitable for a 3-DOF point-mass simulation."""

    def __init__(self) -> None:
        self.flap_position: int = 0    # degrees (0, 1, 5, 10, 15, 25, 30, 40)
        self.gear_down: bool = False
        self.speedbrakes: bool = False

    # ── CL curve ──────────────────────────────────────────────────────────────
    def cl_from_alpha(self, alpha_deg: float) -> tuple[float, bool]:
        """Return (CL, stall_flag) for a given angle of attack (degrees).

        Uses a linear rise up to the stall angle, then a gentle post-stall drop.
        """
        fp = _nearest_flap(self.flap_position)
        dcl = DCL_FLAP[fp]
        cl_max = self._cl_max()

        alpha_rad = math.radians(alpha_deg)
        # Linear portion
        cl_linear = CL_0 + dcl + CL_ALPHA * (alpha_rad - ALPHA_0)

        stall_angle = (ALPHA_STALL_FLAPS_DEG if fp > 0
                       else ALPHA_STALL_CLEAN_DEG)
        stall = alpha_deg >= stall_angle

        if stall:
            # Post-stall drop: parabolic reduction from CL_max
            past = alpha_deg - stall_angle
            cl = cl_max - 0.08 * past * past
            cl = max(cl, 0.2)   # floor – never goes to zero instantly
        else:
            cl = min(cl_linear, cl_max)

        return cl, stall

    def _cl_max(self) -> float:
        fp = _nearest_flap(self.flap_position)
        mapping = {
            0:  CL_MAX_CLEAN,
            1:  CL_MAX_FLAPS10,   # flaps 1 similar to flaps 10 (leading edge)
            5:  CL_MAX_FLAPS10,
            10: CL_MAX_FLAPS10,
            15: CL_MAX_FLAPS25,
            25: CL_MAX_FLAPS25,
            30: CL_MAX_FLAPS30,
            40: CL_MAX_FLAPS40,
        }
        return mapping.get(fp, CL_MAX_CLEAN)

    # ── CD polar ──────────────────────────────────────────────────────────────
    def cd_from_cl(self, cl: float, height_agl_m: float = 5000.0) -> float:
        """Return CD from the polar CD = CD0 + Ke·k·CL²."""
        fp = _nearest_flap(self.flap_position)
        cd0 = CD0_CLEAN + CD_FLAP[fp]
        if self.gear_down:
            cd0 += 0.0200
        if self.speedbrakes:
            cd0 += 0.0120
        ke = ground_effect_factor(height_agl_m)
        cd_induced = ke * K_INDUCED * cl * cl
        return cd0 + cd_induced

    # ── Main forces computation ───────────────────────────────────────────────
    def compute(
        self,
        tas_ms: float,
        altitude_m: float,
        alpha_deg: float,
        height_agl_m: float = 5000.0,
    ) -> AeroState:
        """Compute aerodynamic lift and drag.

        Parameters
        ----------
        tas_ms:
            True airspeed in m/s.
        altitude_m:
            MSL altitude for density calculation.
        alpha_deg:
            Angle of attack in degrees.
        height_agl_m:
            Height above ground level for ground-effect calculation.
        """
        q = dynamic_pressure(tas_ms, altitude_m)
        cl, stall = self.cl_from_alpha(alpha_deg)
        cd = self.cd_from_cl(cl, height_agl_m)

        lift = q * S_REF * cl
        drag = q * S_REF * cd

        return AeroState(
            lift_n=lift,
            drag_n=drag,
            cl=cl,
            cd=cd,
            alpha_deg=alpha_deg,
            q_pa=q,
            stall=stall,
        )

    # ── Stall speed ───────────────────────────────────────────────────────────
    def v_stall_ms(self, weight_n: float, altitude_m: float) -> float:
        """Return the stall speed (TAS, m/s) for the current flap setting."""
        rho = density(altitude_m)
        cl_max = self._cl_max()
        if cl_max <= 0 or rho <= 0:
            return 0.0
        return math.sqrt(2.0 * weight_n / (rho * S_REF * cl_max))

    def v_stall_cas_ms(self, weight_n: float, altitude_m: float) -> float:
        """Return stall speed as Calibrated Air Speed (m/s)."""
        tas = self.v_stall_ms(weight_n, altitude_m)
        rho = density(altitude_m)
        return tas * math.sqrt(rho / RHO0)

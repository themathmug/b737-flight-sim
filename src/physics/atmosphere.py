"""
International Standard Atmosphere (ISA) model.

Implements the ICAO standard atmosphere up to 20,000 m (65,600 ft), covering
the troposphere (0–11,000 m) and lower stratosphere (11,000–20,000 m).

Reference: ICAO Doc 7488-CD, Manual of the ICAO Standard Atmosphere, 3rd ed.
"""

import math

# ── Sea-level reference values ─────────────────────────────────────────────────
T0: float = 288.15   # K  – sea-level standard temperature
P0: float = 101325.0 # Pa – sea-level standard pressure
RHO0: float = 1.225  # kg/m³ – sea-level standard density

# ── Physical constants ─────────────────────────────────────────────────────────
R_AIR: float = 287.058   # J/(kg·K) – specific gas constant for dry air
GAMMA: float = 1.4        # – specific heat ratio
G: float = 9.80665        # m/s² – standard gravity

# ── Lapse rates ────────────────────────────────────────────────────────────────
L_TROPO: float = 0.0065   # K/m – temperature lapse rate (troposphere)
H_TROPO: float = 11000.0  # m – tropopause altitude
T_TROPO: float = T0 - L_TROPO * H_TROPO  # 216.65 K at tropopause


def temperature(altitude_m: float) -> float:
    """Return ISA temperature in Kelvin at the given altitude (metres).

    Parameters
    ----------
    altitude_m:
        Geometric altitude in metres (MSL).  Clamped to [0, 20 000].
    """
    alt = max(0.0, min(altitude_m, 20000.0))
    if alt <= H_TROPO:
        return T0 - L_TROPO * alt
    return T_TROPO  # isothermal stratosphere up to 20 km


def pressure(altitude_m: float) -> float:
    """Return ISA static pressure in Pascals at the given altitude."""
    alt = max(0.0, min(altitude_m, 20000.0))
    if alt <= H_TROPO:
        t = T0 - L_TROPO * alt
        return P0 * (t / T0) ** (G / (R_AIR * L_TROPO))
    # Isothermal stratosphere
    p_tropo = pressure(H_TROPO)
    return p_tropo * math.exp(-G * (alt - H_TROPO) / (R_AIR * T_TROPO))


def density(altitude_m: float) -> float:
    """Return ISA air density in kg/m³ at the given altitude."""
    t = temperature(altitude_m)
    p = pressure(altitude_m)
    return p / (R_AIR * t)


def speed_of_sound(altitude_m: float) -> float:
    """Return the speed of sound in m/s at the given altitude."""
    t = temperature(altitude_m)
    return math.sqrt(GAMMA * R_AIR * t)


def tas_to_mach(tas_ms: float, altitude_m: float) -> float:
    """Convert True Air Speed (m/s) to Mach number."""
    a = speed_of_sound(altitude_m)
    return tas_ms / a if a > 0 else 0.0


def mach_to_tas(mach: float, altitude_m: float) -> float:
    """Convert Mach number to True Air Speed (m/s)."""
    return mach * speed_of_sound(altitude_m)


def cas_to_tas(cas_ms: float, altitude_m: float) -> float:
    """Convert Calibrated Air Speed (m/s) to True Air Speed (m/s).

    Uses the compressible Bernoulli correction.  Valid for subsonic flight.
    """
    rho = density(altitude_m)
    # Bernoulli (incompressible): TAS = CAS * sqrt(rho0/rho)
    return cas_ms * math.sqrt(RHO0 / rho)


def tas_to_cas(tas_ms: float, altitude_m: float) -> float:
    """Convert True Air Speed (m/s) to Calibrated Air Speed (m/s)."""
    rho = density(altitude_m)
    return tas_ms * math.sqrt(rho / RHO0)


def tas_to_eas(tas_ms: float, altitude_m: float) -> float:
    """Convert TAS (m/s) to Equivalent Air Speed (m/s)."""
    rho = density(altitude_m)
    return tas_ms * math.sqrt(rho / RHO0)


def dynamic_pressure(tas_ms: float, altitude_m: float) -> float:
    """Return dynamic pressure q = 0.5·ρ·V² in Pascals."""
    rho = density(altitude_m)
    return 0.5 * rho * tas_ms * tas_ms


def oat_celsius(altitude_m: float, deviation_c: float = 0.0) -> float:
    """Outside Air Temperature in °C with optional ISA deviation."""
    return temperature(altitude_m) - 273.15 + deviation_c


def pressure_altitude_from_qnh(altitude_msl_m: float, qnh_pa: float) -> float:
    """Return pressure altitude (m) given MSL altitude and QNH (Pa).

    Pressure altitude = altitude corrected to standard pressure (1013.25 hPa).
    Simple approximation: ΔAlt ≈ 30 · (1013.25 - QNH_hPa) ft → converted to m.
    """
    delta_hpa = (101325.0 - qnh_pa) / 100.0
    delta_ft = 30.0 * delta_hpa
    return altitude_msl_m + delta_ft * 0.3048

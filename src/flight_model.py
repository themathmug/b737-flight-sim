"""
Boeing 737-800 Three-Degree-of-Freedom (3-DOF) Flight Model.

State vector
------------
    x_m         East position, metres (relative to starting point)
    y_m         North position, metres
    altitude_m  MSL altitude, metres
    tas_ms      True airspeed, m/s
    gamma_deg   Flight-path angle (positive = climb), degrees
    heading_deg Magnetic heading, degrees
    pitch_deg   Pitch attitude, degrees
    bank_deg    Bank angle, degrees
    mass_kg     Current aircraft mass, kg

Equations of motion (point-mass, no rotational inertia)
---------------------------------------------------------
Longitudinal:
    m·V̇  = T·cos α – D – W·sin γ
    m·V·γ̇ = L + T·sin α – W·cos γ

Lateral:
    ḣdg = g·tan(φ) / V   (coordinated turn, no sideslip)

Position:
    ẋ    = V·cos γ·sin(hdg)
    ẏ    = V·cos γ·cos(hdg)
    ż    = V·sin γ

Attitude:
    Pitch = γ + α         (simplified, no thrust vector separation)
    Bank  driven by pilot/autopilot input (rate controlled)

Weight/fuel:
    ṁ    = −ṁ_fuel        (total fuel flow in kg/s)

Aircraft data (737-800)
-----------------------
    OEW         41 413 kg
    Max payload  20 000 kg (typical)
    Max fuel     26 022 kg  (both tanks)
    MTOW         79 016 kg
    MLW          66 361 kg
"""

import math
from dataclasses import dataclass, field
from typing import Optional

from src.physics.atmosphere import (
    density, speed_of_sound, dynamic_pressure,
    cas_to_tas, tas_to_cas, G
)
from src.physics.aerodynamics import AerodynamicsModel, AeroState
from src.physics.engines import DualEngineSystem

# ── 737-800 weight data ────────────────────────────────────────────────────────
OEW_KG: float     = 41_413.0   # operating empty weight
MAX_FUEL_KG: float = 26_022.0  # max fuel (both tanks)
MTOW_KG: float    = 79_016.0   # max take-off weight
MLW_KG: float     = 66_361.0   # max landing weight
MZFW_KG: float    = 61_688.0   # max zero fuel weight

# Typical training scenario starting weight
DEFAULT_FUEL_KG: float   = 14_000.0
DEFAULT_PAYLOAD_KG: float = 15_000.0

# ── Limits ────────────────────────────────────────────────────────────────────
V_MO_MS: float    = 162.0   # VMO = 340 kt CAS ≈ 175 m/s (approx at altitude)
MMO: float        = 0.820   # MMO Mach limit
ALT_MAX_M: float  = 12_497.0  # service ceiling ~41 000 ft

# Control surface rate limits (deg/s) – realistic values
MAX_BANK_RATE_DPS: float  = 15.0   # aileron
MAX_PITCH_RATE_DPS: float = 3.0    # elevator


@dataclass
class FlightState:
    """Complete aircraft state at a single point in time."""
    # Position
    x_m: float = 0.0
    y_m: float = 0.0
    altitude_m: float = 0.0

    # Kinematics
    tas_ms: float = 0.0
    gamma_deg: float = 0.0      # flight-path angle
    heading_deg: float = 0.0
    pitch_deg: float = 0.0
    bank_deg: float = 0.0

    # Weight
    mass_kg: float = OEW_KG + DEFAULT_PAYLOAD_KG + DEFAULT_FUEL_KG
    fuel_kg: float = DEFAULT_FUEL_KG

    # Derived (computed each step)
    alpha_deg: float = 0.0
    cas_ms: float = 0.0
    mach: float = 0.0
    vs_ms: float = 0.0          # vertical speed, m/s
    vs_fpm: float = 0.0         # vertical speed, ft/min
    gnd_speed_ms: float = 0.0   # ground speed, m/s
    weight_n: float = 0.0
    load_factor: float = 1.0    # g
    on_ground: bool = True
    stall: bool = False

    # Simulation time
    sim_time_s: float = 0.0

    # Wind (set externally)
    wind_speed_ms: float = 0.0
    wind_dir_deg: float = 0.0   # direction wind is FROM

    # Flags
    gear_down: bool = True
    flap_position: int = 0     # degrees (0,1,5,10,15,25,30,40)
    speedbrakes: bool = False


@dataclass
class PilotInputs:
    """Control inputs supplied by pilot or autopilot this step."""
    throttle1: float = 0.22    # 0–1 per engine
    throttle2: float = 0.22
    elevator: float  = 0.0     # –1 to +1 (+ = pitch up)
    aileron: float   = 0.0     # –1 to +1 (+ = right bank)
    # Rates controlled by these inputs
    pitch_rate_cmd_dps: float = 0.0    # commanded pitch rate (elevator → pitch)
    bank_rate_cmd_dps: float  = 0.0    # commanded bank rate


class FlightModel:
    """3-DOF Boeing 737-800 flight dynamics model."""

    # ── Construction ──────────────────────────────────────────────────────────
    def __init__(
        self,
        initial_altitude_m: float = 0.0,
        initial_heading_deg: float = 0.0,
        initial_speed_cas_ms: float = 0.0,
        fuel_kg: float = DEFAULT_FUEL_KG,
        payload_kg: float = DEFAULT_PAYLOAD_KG,
    ) -> None:
        mass = OEW_KG + payload_kg + fuel_kg
        self.state = FlightState(
            altitude_m=initial_altitude_m,
            heading_deg=initial_heading_deg,
            mass_kg=mass,
            fuel_kg=fuel_kg,
        )
        if initial_speed_cas_ms > 0:
            self.state.tas_ms = cas_to_tas(initial_speed_cas_ms, initial_altitude_m)
            self.state.on_ground = False

        self.aero = AerodynamicsModel()
        self.engines = DualEngineSystem()

        self._wind_x: float = 0.0   # m/s, eastward component
        self._wind_y: float = 0.0   # m/s, northward component

        # Pilot pitch/bank command integrators
        self._target_pitch: float = initial_altitude_m > 0 and 2.0 or 0.0
        self._target_bank: float  = 0.0

    # ── Wind helper ───────────────────────────────────────────────────────────
    def set_wind(self, speed_ms: float, direction_from_deg: float) -> None:
        """Set steady wind.  direction_from_deg: direction wind is blowing FROM."""
        dir_rad = math.radians(direction_from_deg)
        self._wind_x = -speed_ms * math.sin(dir_rad)  # eastward component
        self._wind_y = -speed_ms * math.cos(dir_rad)  # northward component
        self.state.wind_speed_ms = speed_ms
        self.state.wind_dir_deg = direction_from_deg

    def add_turbulence(self, intensity_ms: float, dt: float) -> None:
        """Add a random turbulence gust to the wind field."""
        import random
        gust_x = random.gauss(0, intensity_ms * 0.5)
        gust_y = random.gauss(0, intensity_ms * 0.5)
        self._wind_x += gust_x * dt
        self._wind_y += gust_y * dt

    # ── Main update ───────────────────────────────────────────────────────────
    def update(self, dt: float, inputs: PilotInputs) -> FlightState:
        """Advance the flight model by *dt* seconds with the given pilot inputs."""
        s = self.state
        s.sim_time_s += dt

        # ── 1. Engine update ──────────────────────────────────────────────────
        mach = speed_of_sound(s.altitude_m)
        mach_n = s.tas_ms / mach if mach > 0 else 0.0
        e1, e2 = self.engines.update(
            dt, inputs.throttle1, inputs.throttle2, s.altitude_m, mach_n
        )
        thrust_total = e1.thrust_n + e2.thrust_n

        # ── 2. Fuel burn ──────────────────────────────────────────────────────
        ff_kgs = (e1.fuel_flow_kgh + e2.fuel_flow_kgh) / 3600.0
        fuel_burned = ff_kgs * dt
        s.fuel_kg = max(0.0, s.fuel_kg - fuel_burned)
        s.mass_kg = OEW_KG + DEFAULT_PAYLOAD_KG + s.fuel_kg
        weight_n = s.mass_kg * G
        s.weight_n = weight_n

        # ── 3. Pilot control inputs → attitude ────────────────────────────────
        # Pitch: elevator input drives pitch rate
        pitch_rate = inputs.pitch_rate_cmd_dps
        pitch_rate = max(-MAX_PITCH_RATE_DPS, min(MAX_PITCH_RATE_DPS, pitch_rate))
        s.pitch_deg += pitch_rate * dt
        s.pitch_deg = max(-30.0, min(30.0, s.pitch_deg))

        # Bank: aileron input drives bank rate
        bank_rate = inputs.bank_rate_cmd_dps
        bank_rate = max(-MAX_BANK_RATE_DPS, min(MAX_BANK_RATE_DPS, bank_rate))
        s.bank_deg += bank_rate * dt
        s.bank_deg = max(-67.0, min(67.0, s.bank_deg))   # 737 bank limit

        # ── 4. Aerodynamics ───────────────────────────────────────────────────
        self.aero.flap_position = s.flap_position
        self.aero.gear_down = s.gear_down
        self.aero.speedbrakes = s.speedbrakes

        # AoA ≈ pitch – flight_path_angle
        s.alpha_deg = s.pitch_deg - s.gamma_deg
        height_agl = s.altitude_m  # simplified – assume flat earth

        aero: AeroState = self.aero.compute(
            max(1.0, s.tas_ms), s.altitude_m, s.alpha_deg, height_agl
        )
        s.stall = aero.stall

        # ── 5. Equations of motion (longitudinal) ─────────────────────────────
        alpha_rad  = math.radians(s.alpha_deg)
        gamma_rad  = math.radians(s.gamma_deg)
        bank_rad   = math.radians(s.bank_deg)

        cos_bank   = math.cos(bank_rad)
        lift_eff   = aero.lift_n * cos_bank   # effective upward lift component

        # Speed change
        v_dot = (
            thrust_total * math.cos(alpha_rad)
            - aero.drag_n
            - weight_n * math.sin(gamma_rad)
        ) / s.mass_kg

        # Flight-path angle change
        if s.tas_ms > 10.0:
            gamma_dot_rad = (
                lift_eff + thrust_total * math.sin(alpha_rad) - weight_n * math.cos(gamma_rad)
            ) / (s.mass_kg * s.tas_ms)
        else:
            gamma_dot_rad = 0.0

        # ── 6. Ground collision / runway ──────────────────────────────────────
        if s.altitude_m <= 0.0:
            s.on_ground  = True
            s.altitude_m = 0.0
            s.gamma_deg  = 0.0   # no flight-path angle on ground

            # Rolling friction (μ ≈ 0.02 smooth runway, 0.40 brakes)
            if s.tas_ms > 0.1:
                v_dot -= 0.02 * G   # rolling resistance deceleration
            elif v_dot <= 0.0:
                # Aircraft is truly stopped and no net forward acceleration
                v_dot    = 0.0
                s.tas_ms = 0.0
                s.vs_ms  = 0.0
                s.vs_fpm = 0.0
                self._update_derived(s, mach_n)
                return s
        else:
            s.on_ground = False

        # ── 7. Integrate ──────────────────────────────────────────────────────
        s.tas_ms += v_dot * dt
        s.tas_ms = max(0.0, s.tas_ms)  # no negative airspeed

        gamma_dot_deg = math.degrees(gamma_dot_rad)
        s.gamma_deg += gamma_dot_deg * dt
        s.gamma_deg = max(-30.0, min(30.0, s.gamma_deg))

        # Clamp on ground
        if s.on_ground and s.gamma_deg < 0.0:
            s.gamma_deg = 0.0

        # ── 8. Heading update (coordinated turn) ──────────────────────────────
        if s.tas_ms > 10.0:
            hdg_rate_rads = G * math.tan(bank_rad) / s.tas_ms
            s.heading_deg += math.degrees(hdg_rate_rads) * dt
            s.heading_deg %= 360.0

        # ── 9. Position integration ───────────────────────────────────────────
        hdg_rad = math.radians(s.heading_deg)
        gnd_speed = s.tas_ms * math.cos(math.radians(s.gamma_deg))
        # Ground track includes wind effect
        gnd_vx = gnd_speed * math.sin(hdg_rad) + self._wind_x
        gnd_vy = gnd_speed * math.cos(hdg_rad) + self._wind_y
        s.gnd_speed_ms = math.sqrt(gnd_vx**2 + gnd_vy**2)

        s.x_m += gnd_vx * dt
        s.y_m += gnd_vy * dt
        vs = s.tas_ms * math.sin(math.radians(s.gamma_deg))
        s.altitude_m += vs * dt
        s.altitude_m = max(0.0, min(ALT_MAX_M, s.altitude_m))
        s.vs_ms = vs

        # ── 10. Derived values ────────────────────────────────────────────────
        self._update_derived(s, mach_n)
        return s

    # ── Helpers ───────────────────────────────────────────────────────────────
    @staticmethod
    def _update_derived(s: FlightState, mach_n: float) -> None:
        s.vs_fpm = s.vs_ms * 196.85
        s.cas_ms = tas_to_cas(s.tas_ms, s.altitude_m)
        a = speed_of_sound(s.altitude_m)
        s.mach = s.tas_ms / a if a > 0 else 0.0
        s.weight_n = s.mass_kg * G
        # Load factor n = L/W (approximated as 1/cos(bank) in level flight)
        bank_rad = math.radians(s.bank_deg)
        s.load_factor = 1.0 / math.cos(bank_rad) if abs(bank_rad) < math.pi / 2.2 else 1.0

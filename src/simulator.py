"""
Main simulation controller – ties together all sub-systems.

The Simulator runs an update loop at a fixed dt (default 0.1 s) and
can be driven either by a live UI or headlessly for testing/logging.
"""

import time
import math
from dataclasses import dataclass, field
from typing import Optional, Callable

from src.flight_model import FlightModel, PilotInputs, FlightState, DEFAULT_FUEL_KG, DEFAULT_PAYLOAD_KG
from src.physics.atmosphere import tas_to_cas, G
from src.systems.aircraft_systems import AircraftSystems
from src.systems.autopilot import AutopilotSystem, LateralMode, VerticalMode, SpeedMode
from src.systems.fms import FMS
from src.instruments.pfd import build_pfd, PFDData
from src.instruments.mfd import build_mfd_nav, MFDNavData
from src.instruments.eicas import build_eicas, EICASData

MS_TO_KTS = 1.94384
KTS_TO_MS = 1.0 / MS_TO_KTS
M_TO_FT = 3.28084


@dataclass
class SimulatorConfig:
    """Simulator startup configuration."""
    dt: float = 0.1                       # simulation step, seconds
    initial_altitude_m: float = 0.0
    initial_heading_deg: float = 360.0
    initial_speed_cas_kts: float = 0.0
    fuel_kg: float = DEFAULT_FUEL_KG
    payload_kg: float = DEFAULT_PAYLOAD_KG
    gear_down: bool = True
    flap_position: int = 0
    throttle: float = 0.22
    ap_engaged: bool = False
    ap_target_alt_m: float = 0.0
    ap_target_hdg: float = 0.0
    ap_target_cas_kts: float = 250.0
    wind_speed_kts: float = 0.0
    wind_dir_deg: float = 0.0
    route: list = field(default_factory=list)
    # Starting lat/lon (for FMS)
    origin_lat: float = 47.449
    origin_lon: float = -122.309


@dataclass
class SimulatorState:
    """Aggregated state exposed to the UI each step."""
    flight: FlightState = field(default_factory=FlightState)
    pfd: Optional[PFDData] = None
    mfd: Optional[MFDNavData] = None
    eicas: Optional[EICASData] = None
    # Pilot throttle (both engines same for normal ops)
    throttle: float = 0.22
    # Alert messages
    aural_alerts: list = field(default_factory=list)
    # Sim timing
    sim_time_s: float = 0.0
    real_fps: float = 0.0


class Simulator:
    """Boeing 737-800 study-level flight simulator."""

    def __init__(self, config: Optional[SimulatorConfig] = None) -> None:
        if config is None:
            config = SimulatorConfig()
        self.cfg = config
        self.dt: float = config.dt

        # Pilot controls
        self._throttle: float = config.throttle
        self._pitch_rate_cmd: float = 0.0    # deg/s
        self._bank_rate_cmd: float  = 0.0    # deg/s

        # Flight model
        self.model = FlightModel(
            initial_altitude_m=config.initial_altitude_m,
            initial_heading_deg=config.initial_heading_deg,
            initial_speed_cas_ms=config.initial_speed_cas_kts * KTS_TO_MS,
            fuel_kg=config.fuel_kg,
            payload_kg=config.payload_kg,
        )
        self.model.state.gear_down     = config.gear_down
        self.model.state.flap_position = config.flap_position

        # Aircraft systems
        self.systems = AircraftSystems(initial_fuel_kg=config.fuel_kg)
        self.systems.gear.gear_state = (
            __import__("src.systems.aircraft_systems", fromlist=["GearState"]).GearState.DOWN
            if config.gear_down else
            __import__("src.systems.aircraft_systems", fromlist=["GearState"]).GearState.UP
        )
        self.systems.flaps.lever_position = config.flap_position

        # Pre-spool engines to match initial throttle if starting in-flight
        if config.initial_altitude_m > 0.0:
            self.model.engines.eng1.prespool(config.throttle, config.initial_altitude_m)
            self.model.engines.eng2.prespool(config.throttle, config.initial_altitude_m)

        # Wind
        self.model.set_wind(
            config.wind_speed_kts * KTS_TO_MS, config.wind_dir_deg
        )

        # Autopilot
        self.autopilot = AutopilotSystem()
        if config.ap_engaged:
            cas_kts = config.initial_speed_cas_kts
            self.autopilot.engage(
                heading=config.ap_target_hdg,
                altitude=config.ap_target_alt_m,
                cas_kts=config.ap_target_cas_kts,
                vs_fpm=0.0,
                throttle_trim=config.throttle,   # pre-load integrator near trim
            )
            self.autopilot.lateral_mode  = LateralMode.HDG_SEL
            self.autopilot.vertical_mode = VerticalMode.ALT_HLD
            self.autopilot.speed_mode    = SpeedMode.SPEED

        # FMS
        self.fms = FMS()
        if config.route:
            self.fms.set_route(config.route)
            self.fms.activate()

        # State snapshot
        self.state = SimulatorState()
        self._fuel_start = config.fuel_kg

        # Turbulence intensity (m/s)
        self._turbulence: float = 0.0

        # Stall warning flag
        self._stall_warning_timer: float = 0.0

        # Timing
        self._last_real_time: float = time.time()
        self._steps: int = 0

    # ── Pilot command interface ───────────────────────────────────────────────
    def set_throttle(self, value: float) -> None:
        """Set throttle for both engines (0.0–1.0)."""
        self._throttle = max(0.0, min(1.0, value))

    def adjust_throttle(self, delta: float) -> None:
        self._throttle = max(0.0, min(1.0, self._throttle + delta))

    def set_pitch_rate(self, rate_dps: float) -> None:
        self._pitch_rate_cmd = rate_dps

    def set_bank_rate(self, rate_dps: float) -> None:
        self._bank_rate_cmd = rate_dps

    def toggle_gear(self) -> str:
        cas_kts = self.model.state.cas_ms * MS_TO_KTS
        msg = self.systems.gear.toggle(cas_kts)
        return msg

    def extend_flaps(self) -> str:
        return self.systems.flaps.extend()

    def retract_flaps(self) -> str:
        return self.systems.flaps.retract()

    def toggle_speedbrakes(self) -> None:
        self.model.state.speedbrakes = not self.model.state.speedbrakes

    def toggle_autopilot(self) -> str:
        if self.autopilot.engaged:
            self.autopilot.disengage()
            return "AUTOPILOT OFF"
        s = self.model.state
        cas_kts = s.cas_ms * MS_TO_KTS
        self.autopilot.engage(
            heading=s.heading_deg,
            altitude=s.altitude_m,
            cas_kts=cas_kts,
            vs_fpm=s.vs_fpm,
        )
        self.autopilot.lateral_mode  = LateralMode.HDG_SEL
        self.autopilot.vertical_mode = VerticalMode.ALT_HLD
        self.autopilot.speed_mode    = SpeedMode.SPEED
        return "AUTOPILOT ON – HDG/ALT/SPD"

    def set_ap_heading(self, hdg: float) -> None:
        self.autopilot.select_heading(hdg)

    def set_ap_altitude(self, alt_ft: float) -> None:
        self.autopilot.select_altitude(alt_ft / M_TO_FT)

    def set_ap_vs(self, vs_fpm: float) -> None:
        self.autopilot.select_vs(vs_fpm)

    def set_ap_speed(self, cas_kts: float) -> None:
        self.autopilot.select_speed(cas_kts)

    def set_turbulence(self, intensity: float) -> None:
        """Set turbulence intensity (0 = off, 1 = light, 2 = moderate, 3 = severe)."""
        self._turbulence = intensity * 0.5  # m/s per unit

    def set_wind(self, speed_kts: float, dir_deg: float) -> None:
        self.model.set_wind(speed_kts * KTS_TO_MS, dir_deg)

    # ── Main update step ──────────────────────────────────────────────────────
    def step(self) -> SimulatorState:
        """Advance simulation by one dt.  Returns the updated state snapshot."""
        dt = self.dt
        s = self.model.state

        # Sync flap/gear to flight model state
        s.flap_position = self.systems.flaps.lever_position
        s.gear_down     = self.systems.gear.is_down

        # Turbulence
        if self._turbulence > 0:
            self.model.add_turbulence(self._turbulence, dt)

        # ── Autopilot ────────────────────────────────────────────────────────
        cas_kts = s.cas_ms * MS_TO_KTS
        ap_out = self.autopilot.compute(
            dt=dt,
            heading_deg=s.heading_deg,
            altitude_m=s.altitude_m,
            cas_kts=cas_kts,
            vs_fpm=s.vs_fpm,
            pitch_deg=s.pitch_deg,
            bank_deg=s.bank_deg,
            throttle_current=self._throttle,
        )

        if self.autopilot.engaged:
            self._pitch_rate_cmd = ap_out.pitch_rate_dps
            self._bank_rate_cmd  = ap_out.bank_rate_dps
            self._throttle       = ap_out.throttle

        # ── Assemble pilot inputs ─────────────────────────────────────────────
        inputs = PilotInputs(
            throttle1=self._throttle,
            throttle2=self._throttle,
            pitch_rate_cmd_dps=self._pitch_rate_cmd,
            bank_rate_cmd_dps=self._bank_rate_cmd,
        )

        # ── Advance flight model ──────────────────────────────────────────────
        flight = self.model.update(dt, inputs)

        # ── Aircraft systems update ───────────────────────────────────────────
        alt_ft = flight.altitude_m * M_TO_FT
        ff_total = (
            self.model.engines.eng1.state.fuel_flow_kgh
            + self.model.engines.eng2.state.fuel_flow_kgh
        )
        self.systems.fuel.burn(
            (self.model.engines.eng1.state.fuel_flow_kgh / 3600.0) * dt,
            (self.model.engines.eng2.state.fuel_flow_kgh / 3600.0) * dt,
        )
        self.systems.gear.update(dt)
        self.systems.flaps.update(dt)
        self.systems.pressurisation.update(alt_ft, dt)
        self.systems._check_hydraulics(
            self.model.engines.eng1.state.running,
            self.model.engines.eng2.state.running,
        )
        self.systems._check_electrical(
            self.model.engines.eng1.state.running,
            self.model.engines.eng2.state.running,
        )
        self.systems._cautions = []
        self.systems._warnings = []
        self.systems._generate_alerts(alt_ft, cas_kts)

        # ── FMS navigation update ─────────────────────────────────────────────
        lat, lon = FMS.xy_to_latlon(
            flight.x_m, flight.y_m, self.cfg.origin_lat, self.cfg.origin_lon
        )
        self.fms.update(lat, lon, flight.gnd_speed_ms, flight.heading_deg)

        # LNAV heading update
        if (self.autopilot.engaged and
                self.autopilot.lateral_mode == LateralMode.LNAV):
            cmd_hdg = self.fms.lnav_heading_cmd()
            self.autopilot.targets.heading_deg = cmd_hdg

        # ── Aural alerts ──────────────────────────────────────────────────────
        alerts: list[str] = []
        if flight.stall:
            self._stall_warning_timer += dt
            if self._stall_warning_timer > 0.5:
                alerts.append("STALL  STALL")
        else:
            self._stall_warning_timer = 0.0

        if alt_ft < 2500 and not flight.on_ground and not self.systems.gear.is_down:
            alerts.append("TOO LOW GEAR")
        if alt_ft < 500 and not flight.on_ground and self.systems.flaps.lever_position < 15:
            alerts.append("TOO LOW FLAPS")

        # ── Build display packets ─────────────────────────────────────────────
        e1 = self.model.engines.eng1.state
        e2 = self.model.engines.eng2.state
        ap_mode = self.autopilot.mode_line()

        pfd = build_pfd(
            state=flight,
            ap_lat=self.autopilot.lateral_mode.name,
            ap_vert=self.autopilot.vertical_mode.name,
            ap_spd=self.autopilot.speed_mode.name,
            ap_engaged=self.autopilot.engaged,
        )
        mfd = build_mfd_nav(
            state=flight,
            fms=self.fms,
            origin_lat=self.cfg.origin_lat,
            origin_lon=self.cfg.origin_lon,
        )
        eicas = build_eicas(
            eng1=e1, eng2=e2,
            systems=self.systems,
            altitude_m=flight.altitude_m,
            fuel_used_kg=self._fuel_start - self.systems.fuel.total_kg,
        )

        # ── Timing ────────────────────────────────────────────────────────────
        now = time.time()
        elapsed = now - self._last_real_time
        fps = 1.0 / elapsed if elapsed > 0 else 0.0
        self._last_real_time = now
        self._steps += 1

        self.state = SimulatorState(
            flight=flight,
            pfd=pfd,
            mfd=mfd,
            eicas=eicas,
            throttle=self._throttle,
            aural_alerts=alerts,
            sim_time_s=flight.sim_time_s,
            real_fps=fps,
        )
        return self.state

    # ── Convenience ──────────────────────────────────────────────────────────
    @classmethod
    def from_scenario(cls, scenario) -> "Simulator":
        """Create a Simulator from a ScenarioConfig object."""
        cfg = SimulatorConfig(
            initial_altitude_m=scenario.altitude_m,
            initial_heading_deg=scenario.heading_deg,
            initial_speed_cas_kts=scenario.speed_cas_kts,
            fuel_kg=scenario.fuel_kg,
            payload_kg=scenario.payload_kg,
            gear_down=scenario.gear_down,
            flap_position=scenario.flap_position,
            throttle=scenario.throttle,
            ap_engaged=scenario.ap_engaged,
            ap_target_alt_m=scenario.ap_target_alt_m,
            ap_target_hdg=scenario.ap_target_hdg,
            ap_target_cas_kts=scenario.ap_target_cas_kts,
            wind_speed_kts=scenario.wind_speed_kts,
            wind_dir_deg=scenario.wind_dir_deg,
            route=list(scenario.route),
        )
        return cls(cfg)

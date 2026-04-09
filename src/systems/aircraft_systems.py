"""
Boeing 737-800 Aircraft Systems.

Covers:
  - Hydraulic systems (Systems A, B, Standby)
  - Electrical systems (AC/DC buses)
  - Landing gear with extension/retraction timing
  - Flap / slat control with detent logic
  - Pressurisation
  - Fuel system (two wing tanks + centre tank)
"""

import math
from dataclasses import dataclass, field
from enum import Enum, auto


# ── Enumerations ──────────────────────────────────────────────────────────────
class GearState(Enum):
    UP       = auto()
    DOWN     = auto()
    TRANSIT  = auto()


class HydSystemState(Enum):
    NORMAL   = auto()
    LOW_PRES = auto()
    FAILED   = auto()


class ElecBusState(Enum):
    POWERED  = auto()
    UNPOWERED = auto()


# ── Flap configuration table (737-800 FCOM-style) ─────────────────────────────
# Maps flap lever position → (slat position °, flap position °)
FLAP_DETENTS = [0, 1, 5, 10, 15, 25, 30, 40]
FLAP_SLAT_ANGLE = {0: 0, 1: 15, 5: 15, 10: 15, 15: 15, 25: 15, 30: 15, 40: 15}

# Speeds limits for each flap setting (CAS, knots) – Vfe
VFE_KT = {0: 340, 1: 260, 5: 250, 10: 230, 15: 215, 25: 195, 30: 185, 40: 162}

# Gear speed limits
VLO_KT = 270   # max speed for gear operation
VLE_KT = 270   # max speed with gear extended

# Gear transit time (seconds)
GEAR_TRANSIT_S = 10.0


@dataclass
class HydraulicSystem:
    """Dual-system (A and B) hydraulics + standby pump."""
    sys_a: HydSystemState = HydSystemState.NORMAL
    sys_b: HydSystemState = HydSystemState.NORMAL
    standby: HydSystemState = HydSystemState.NORMAL
    pressure_a_psi: float = 3000.0
    pressure_b_psi: float = 3000.0

    def any_normal(self) -> bool:
        return (self.sys_a == HydSystemState.NORMAL or
                self.sys_b == HydSystemState.NORMAL)


@dataclass
class ElectricalSystem:
    """Simple electrical bus model."""
    ac_bus1: ElecBusState = ElecBusState.POWERED
    ac_bus2: ElecBusState = ElecBusState.POWERED
    dc_bus1: ElecBusState = ElecBusState.POWERED
    dc_bus2: ElecBusState = ElecBusState.POWERED
    battery: bool = True

    def any_ac(self) -> bool:
        return (self.ac_bus1 == ElecBusState.POWERED or
                self.ac_bus2 == ElecBusState.POWERED)


@dataclass
class FuelSystem:
    """Two wing tanks + centre tank."""
    left_tank_kg: float    = 7_000.0
    right_tank_kg: float   = 7_000.0
    centre_tank_kg: float  = 0.0
    boost_pump_l: bool     = True
    boost_pump_r: bool     = True
    crossfeed_open: bool   = False

    @property
    def total_kg(self) -> float:
        return self.left_tank_kg + self.right_tank_kg + self.centre_tank_kg

    def burn(self, kg_left: float, kg_right: float) -> None:
        """Burn fuel from each side.  Centre tank drains first."""
        # Simplified: centre drains first, then wings
        for eng_burn, side in [(kg_left, "left"), (kg_right, "right")]:
            centre_burn = min(eng_burn * 0.5, self.centre_tank_kg)
            self.centre_tank_kg = max(0.0, self.centre_tank_kg - centre_burn)
            wing_burn = eng_burn - centre_burn
            if side == "left":
                self.left_tank_kg = max(0.0, self.left_tank_kg - wing_burn)
            else:
                self.right_tank_kg = max(0.0, self.right_tank_kg - wing_burn)

    @property
    def imbalance_kg(self) -> float:
        return abs(self.left_tank_kg - self.right_tank_kg)


@dataclass
class PressSystem:
    """Cabin pressurisation (simplified)."""
    cabin_alt_ft: float = 0.0
    diff_pressure_psi: float = 0.0
    auto: bool = True

    def update(self, aircraft_alt_ft: float, dt: float) -> None:
        """Target cabin altitude: ~8 000 ft at cruise."""
        target_cabin = min(8_000.0, aircraft_alt_ft * 0.26)
        lag = 60.0  # seconds
        self.cabin_alt_ft += (target_cabin - self.cabin_alt_ft) / lag * dt
        # Diff pressure = (ISA(cabin_alt) - ISA(aircraft_alt)) / 144 (psi)
        from src.physics.atmosphere import pressure
        p_cabin = pressure(self.cabin_alt_ft * 0.3048)
        p_outside = pressure(aircraft_alt_ft * 0.3048)
        self.diff_pressure_psi = (p_cabin - p_outside) / 6894.76


class LandingGearSystem:
    """Landing gear with transit simulation."""

    def __init__(self) -> None:
        self.gear_state: GearState = GearState.DOWN
        self._transit_timer: float = 0.0
        self._target_state: GearState = GearState.DOWN
        self.nose_steering_angle: float = 0.0  # degrees

    @property
    def is_down(self) -> bool:
        return self.gear_state == GearState.DOWN

    @property
    def is_up(self) -> bool:
        return self.gear_state == GearState.UP

    @property
    def in_transit(self) -> bool:
        return self.gear_state == GearState.TRANSIT

    def command_up(self, cas_kts: float) -> str:
        """Retract gear.  Returns advisory string."""
        if self.gear_state == GearState.UP:
            return "GEAR ALREADY UP"
        if cas_kts > VLO_KT:
            return f"GEAR RETRACT LIMIT {VLO_KT} KT"
        self._target_state = GearState.UP
        self.gear_state = GearState.TRANSIT
        self._transit_timer = GEAR_TRANSIT_S
        return "GEAR RETRACTING"

    def command_down(self, cas_kts: float) -> str:
        """Extend gear.  Returns advisory string."""
        if self.gear_state == GearState.DOWN:
            return "GEAR ALREADY DOWN"
        if cas_kts > VLE_KT:
            return f"GEAR EXTEND LIMIT {VLE_KT} KT"
        self._target_state = GearState.DOWN
        self.gear_state = GearState.TRANSIT
        self._transit_timer = GEAR_TRANSIT_S
        return "GEAR EXTENDING"

    def toggle(self, cas_kts: float) -> str:
        """Toggle gear between up and down."""
        if self.gear_state in (GearState.DOWN, GearState.TRANSIT):
            return self.command_up(cas_kts)
        return self.command_down(cas_kts)

    def update(self, dt: float) -> None:
        if self.gear_state == GearState.TRANSIT:
            self._transit_timer -= dt
            if self._transit_timer <= 0.0:
                self.gear_state = self._target_state
                self._transit_timer = 0.0


class FlapsSystem:
    """Flap / slat control with scheduling."""

    _DETENTS = FLAP_DETENTS

    def __init__(self) -> None:
        self.lever_position: int = 0     # selected detent (degrees)
        self.actual_position: float = 0.0  # actual position (slews to lever)
        self._rate_dps: float = 3.0       # flap slew rate deg/s

    @property
    def vfe_kts(self) -> float:
        return VFE_KT.get(self.lever_position, 340)

    def extend(self) -> str:
        """Select next flap detent."""
        idx = self._DETENTS.index(self.lever_position)
        if idx < len(self._DETENTS) - 1:
            self.lever_position = self._DETENTS[idx + 1]
        return f"FLAPS {self.lever_position}"

    def retract(self) -> str:
        """Select previous flap detent."""
        idx = self._DETENTS.index(self.lever_position)
        if idx > 0:
            self.lever_position = self._DETENTS[idx - 1]
        return f"FLAPS {self.lever_position}"

    def update(self, dt: float) -> None:
        target = float(self.lever_position)
        diff = target - self.actual_position
        if abs(diff) < self._rate_dps * dt:
            self.actual_position = target
        else:
            self.actual_position += math.copysign(self._rate_dps * dt, diff)


class AircraftSystems:
    """Aggregator for all aircraft systems."""

    def __init__(self, initial_fuel_kg: float = 14_000.0) -> None:
        half = initial_fuel_kg / 2.0
        self.hydraulics  = HydraulicSystem()
        self.electrical  = ElectricalSystem()
        self.fuel        = FuelSystem(
            left_tank_kg=half, right_tank_kg=half, centre_tank_kg=0.0
        )
        self.pressurisation = PressSystem()
        self.gear        = LandingGearSystem()
        self.flaps       = FlapsSystem()

        # Caution/Warning flags
        self.master_caution: bool = False
        self.master_warning: bool = False
        self._cautions: list[str] = []
        self._warnings: list[str] = []

    def update(
        self,
        dt: float,
        altitude_ft: float,
        cas_kts: float,
        fuel_flow_kgh_total: float,
        engine1_running: bool,
        engine2_running: bool,
    ) -> None:
        """Update all systems for one simulation step."""
        # Gear transit
        self.gear.update(dt)

        # Flap slew
        self.flaps.update(dt)

        # Fuel burn (from engine fuel-flow, split equally)
        ff_per_engine_kgs = (fuel_flow_kgh_total / 2.0) / 3600.0
        self.fuel.burn(ff_per_engine_kgs * dt, ff_per_engine_kgs * dt)

        # Pressurisation
        self.pressurisation.update(altitude_ft, dt)

        # Hydraulic health check (very simplified)
        self._check_hydraulics(engine1_running, engine2_running)

        # Electrical health
        self._check_electrical(engine1_running, engine2_running)

        # Generate alerts
        self._cautions = []
        self._warnings = []
        self._generate_alerts(altitude_ft, cas_kts)
        self.master_caution = bool(self._cautions)
        self.master_warning = bool(self._warnings)

    def _check_hydraulics(self, eng1: bool, eng2: bool) -> None:
        self.hydraulics.sys_a = (HydSystemState.NORMAL
                                 if eng1 else HydSystemState.FAILED)
        self.hydraulics.sys_b = (HydSystemState.NORMAL
                                 if eng2 else HydSystemState.FAILED)
        self.hydraulics.pressure_a_psi = 3000.0 if eng1 else 0.0
        self.hydraulics.pressure_b_psi = 3000.0 if eng2 else 0.0

    def _check_electrical(self, eng1: bool, eng2: bool) -> None:
        self.electrical.ac_bus1 = (ElecBusState.POWERED
                                   if eng1 else ElecBusState.UNPOWERED)
        self.electrical.ac_bus2 = (ElecBusState.POWERED
                                   if eng2 else ElecBusState.UNPOWERED)

    def _generate_alerts(self, altitude_ft: float, cas_kts: float) -> None:
        # Fuel imbalance
        if self.fuel.imbalance_kg > 454:   # 1 000 lbs
            self._cautions.append("FUEL IMBALANCE")
        # Low fuel
        if self.fuel.total_kg < 2_000:
            self._warnings.append("LOW FUEL")
        # Hydraulic failure
        if self.hydraulics.sys_a == HydSystemState.FAILED:
            self._warnings.append("HYD SYS A FAIL")
        if self.hydraulics.sys_b == HydSystemState.FAILED:
            self._warnings.append("HYD SYS B FAIL")
        # Gear not down for approach
        if altitude_ft < 1000 and cas_kts < 180 and not self.gear.is_down:
            self._warnings.append("GEAR NOT DOWN")
        # Flap/speed limits
        if cas_kts > self.flaps.vfe_kts and self.flaps.lever_position > 0:
            self._warnings.append(f"FLAP OVERSPEED")

    @property
    def cautions(self) -> list[str]:
        return list(self._cautions)

    @property
    def warnings(self) -> list[str]:
        return list(self._warnings)

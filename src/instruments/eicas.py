"""
Engine Indicating and Crew Alerting System (EICAS) data helpers.
"""

from dataclasses import dataclass, field
from src.physics.engines import EngineState, EGT_LIMIT_C
from src.systems.aircraft_systems import AircraftSystems
from src.physics.atmosphere import oat_celsius


@dataclass
class EICASData:
    """All values shown on the EICAS display."""
    # Engine 1
    n1_1: float = 0.0
    n2_1: float = 0.0
    egt_1: float = 0.0
    ff_1: float  = 0.0   # kg/h
    oil_1: float = 65.0  # psi
    eng1_running: bool = True

    # Engine 2
    n1_2: float = 0.0
    n2_2: float = 0.0
    egt_2: float = 0.0
    ff_2: float  = 0.0
    oil_2: float = 65.0
    eng2_running: bool = True

    # Total fuel / weight
    total_fuel_kg: float = 0.0
    fuel_used_kg: float  = 0.0

    # Systems status
    hyd_a_ok: bool = True
    hyd_b_ok: bool = True
    elec_ok: bool  = True

    # Pressurisation
    cabin_alt_ft: float = 0.0
    diff_press_psi: float = 8.6

    # Flaps / gear
    flap_pos: int   = 0
    gear_down: bool = True

    # OAT
    oat_c: float = 15.0

    # Cautions / warnings
    cautions: list = field(default_factory=list)
    warnings: list = field(default_factory=list)

    # EGT limit exceeded
    egt_1_exceeded: bool = False
    egt_2_exceeded: bool = False


def build_eicas(
    eng1: EngineState,
    eng2: EngineState,
    systems: AircraftSystems,
    altitude_m: float,
    fuel_used_kg: float = 0.0,
) -> EICASData:
    """Assemble an :class:`EICASData` snapshot."""
    from src.systems.aircraft_systems import HydSystemState, ElecBusState

    hyd_a_ok = systems.hydraulics.sys_a == HydSystemState.NORMAL
    hyd_b_ok = systems.hydraulics.sys_b == HydSystemState.NORMAL
    elec_ok  = systems.electrical.any_ac()

    return EICASData(
        n1_1=round(eng1.n1_pct, 1),
        n2_1=round(eng1.n2_pct, 1),
        egt_1=round(eng1.egt_c),
        ff_1=round(eng1.fuel_flow_kgh),
        oil_1=eng1.oil_pressure_psi,
        eng1_running=eng1.running,

        n2_2=round(eng2.n2_pct, 1),
        n1_2=round(eng2.n1_pct, 1),
        egt_2=round(eng2.egt_c),
        ff_2=round(eng2.fuel_flow_kgh),
        oil_2=eng2.oil_pressure_psi,
        eng2_running=eng2.running,

        total_fuel_kg=round(systems.fuel.total_kg),
        fuel_used_kg=round(fuel_used_kg),

        hyd_a_ok=hyd_a_ok,
        hyd_b_ok=hyd_b_ok,
        elec_ok=elec_ok,

        cabin_alt_ft=round(systems.pressurisation.cabin_alt_ft),
        diff_press_psi=round(systems.pressurisation.diff_pressure_psi, 2),

        flap_pos=systems.flaps.lever_position,
        gear_down=systems.gear.is_down,

        oat_c=round(oat_celsius(altitude_m), 1),

        cautions=systems.cautions,
        warnings=systems.warnings,

        egt_1_exceeded=eng1.egt_c > EGT_LIMIT_C,
        egt_2_exceeded=eng2.egt_c > EGT_LIMIT_C,
    )

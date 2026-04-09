"""
Normal Procedures – pre-defined scenario setups for standard 737-800 operations.

Each procedure returns a dict that can be used to initialise a Simulator instance.
"""

from dataclasses import dataclass
from typing import Any


@dataclass
class ScenarioConfig:
    """Configuration for a simulator training scenario."""
    name: str
    description: str
    # Aircraft initial state
    altitude_m: float
    heading_deg: float
    speed_cas_kts: float   # calibrated airspeed in knots
    fuel_kg: float
    payload_kg: float
    # Gear / flaps
    gear_down: bool
    flap_position: int
    # Throttle
    throttle: float
    # Autopilot
    ap_engaged: bool
    ap_target_alt_m: float
    ap_target_hdg: float
    ap_target_cas_kts: float
    # Wind
    wind_speed_kts: float
    wind_dir_deg: float
    # Route (list of waypoint idents)
    route: list
    # Training notes
    notes: list


# ── Scenario definitions ──────────────────────────────────────────────────────

SCENARIO_TAKEOFF = ScenarioConfig(
    name="Normal Takeoff – KSEA RWY 16C",
    description=(
        "Standard departure from Seattle-Tacoma. Aircraft is lined up on runway "
        "16C, ready for takeoff at FLEX thrust. Flaps 5, V1=138kt, VR=143kt, "
        "V2=148kt. Climb to 5 000 ft and maintain heading 160."
    ),
    altitude_m=0.0,
    heading_deg=160.0,
    speed_cas_kts=0.0,
    fuel_kg=14_000.0,
    payload_kg=15_000.0,
    gear_down=True,
    flap_position=5,
    throttle=0.22,       # idle – pilot applies thrust
    ap_engaged=False,
    ap_target_alt_m=1_524.0,   # 5 000 ft
    ap_target_hdg=160.0,
    ap_target_cas_kts=250.0,
    wind_speed_kts=10.0,
    wind_dir_deg=180.0,   # 10 kt headwind
    route=["KSEA", "BEAVR", "SUMMA"],
    notes=[
        "Apply TOGA thrust – N1 should reach ~96%",
        "V1 = 138 kt – no abort after this speed",
        "Rotate at VR = 143 kt – pitch to 15°",
        "Positive climb: GEAR UP",
        "At 400 ft AGL: engage autopilot (P key)",
        "Clean up flaps on schedule",
        "At 10 000 ft: limit speed to 250 kt CAS",
    ],
)

SCENARIO_CRUISE = ScenarioConfig(
    name="Cruise – FL350 KSEA→KJFK",
    description=(
        "Aircraft is established in cruise at FL350, Mach 0.785 on a "
        "transcontinental route. Practice autopilot management, fuel monitoring, "
        "and weather deviation."
    ),
    altitude_m=10_668.0,   # 35 000 ft
    heading_deg=95.0,
    speed_cas_kts=270.0,   # approximately Mach 0.785 at FL350
    fuel_kg=18_000.0,
    payload_kg=15_000.0,
    gear_down=False,
    flap_position=0,
    throttle=0.82,
    ap_engaged=True,
    ap_target_alt_m=10_668.0,
    ap_target_hdg=95.0,
    ap_target_cas_kts=270.0,
    wind_speed_kts=50.0,
    wind_dir_deg=270.0,   # 50 kt westerly (tailwind)
    route=["KSEA", "BEAVR", "SUMMA", "OAL", "BOI", "BURHL", "SLC", "ELORE", "DRABS", "KUBBS", "KORD", "KJFK"],
    notes=[
        "Monitor fuel burn – typical cruise is ~5 500 kg/h total",
        "Check MACH number – maintain M0.785 (speed mode ON)",
        "Use MFD nav page to verify route progress",
        "Top of Descent: approximately 120 nm before destination",
        "Begin descent at ~-2 000 fpm, decelerate to 250 kt below FL100",
    ],
)

SCENARIO_APPROACH = ScenarioConfig(
    name="ILS Approach – KJFK RWY 31L",
    description=(
        "Aircraft is on final approach to JFK runway 31L. Established on the "
        "ILS at 3 000 ft, 180 kt. Configure for landing and fly precision approach."
    ),
    altitude_m=914.4,    # 3 000 ft
    heading_deg=310.0,
    speed_cas_kts=180.0,
    fuel_kg=4_000.0,
    payload_kg=15_000.0,
    gear_down=False,
    flap_position=15,
    throttle=0.55,
    ap_engaged=True,
    ap_target_alt_m=0.0,
    ap_target_hdg=310.0,
    ap_target_cas_kts=145.0,
    wind_speed_kts=8.0,
    wind_dir_deg=300.0,
    route=["KJFK"],
    notes=[
        "Check: ATIS, altimeter set, approach briefed",
        "Flaps schedule: Flaps 15 → 25 at glideslope alive",
        "Gear down – three greens",
        "Flaps 30 or 40 as required",
        "Vref + 5 = approach speed (~142 kt for landing weight)",
        "At 50 ft: RETARD throttle, flare 2–3°",
        "Reverse thrust and autobrake on landing",
    ],
)

SCENARIO_SHORT_HOP = ScenarioConfig(
    name="Short Hop – KSEA → KPDX (Portland)",
    description=(
        "A 30-minute flight from Seattle to Portland. Practice a complete "
        "departure, climb, cruise and approach cycle in a short time."
    ),
    altitude_m=0.0,
    heading_deg=180.0,
    speed_cas_kts=0.0,
    fuel_kg=6_000.0,
    payload_kg=12_000.0,
    gear_down=True,
    flap_position=5,
    throttle=0.22,
    ap_engaged=False,
    ap_target_alt_m=3_048.0,   # 10 000 ft
    ap_target_hdg=180.0,
    ap_target_cas_kts=250.0,
    wind_speed_kts=5.0,
    wind_dir_deg=180.0,
    route=["KSEA", "BEAVR"],
    notes=[
        "Takeoff, clean up, climb to 10 000 ft",
        "Cruise at 10 000 ft to save time",
        "Start descent early – Portland is only 150 nm away",
        "Full ILS approach to runway 28L",
    ],
)

ALL_SCENARIOS = [
    SCENARIO_TAKEOFF,
    SCENARIO_CRUISE,
    SCENARIO_APPROACH,
    SCENARIO_SHORT_HOP,
]

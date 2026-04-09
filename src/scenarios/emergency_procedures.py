"""
Emergency Procedures – abnormal / emergency training scenarios for 737-800.

Each scenario starts the aircraft in a situation requiring an emergency response.
Study notes accompany each scenario with QRH reference checklist items.
"""

from src.scenarios.normal_procedures import ScenarioConfig


SCENARIO_ENGINE_FAILURE_TAKEOFF = ScenarioConfig(
    name="Engine Failure at V1",
    description=(
        "Engine 2 fails at exactly V1 (138 kt) during takeoff roll. "
        "Continue the takeoff on one engine, maintain directional control, "
        "climb to MSA and run the Engine Failure checklist."
    ),
    altitude_m=0.0,
    heading_deg=160.0,
    speed_cas_kts=71.0,   # ~138 kt / 2 for initial display (scaled for demo)
    fuel_kg=14_000.0,
    payload_kg=15_000.0,
    gear_down=True,
    flap_position=5,
    throttle=0.98,    # TOGA on eng1 (eng2 will be shut down by simulator)
    ap_engaged=False,
    ap_target_alt_m=610.0,    # 2 000 ft
    ap_target_hdg=160.0,
    ap_target_cas_kts=250.0,
    wind_speed_kts=5.0,
    wind_dir_deg=160.0,
    route=["KSEA"],
    notes=[
        "MAINTAIN directional control – apply rudder toward operating engine",
        "Rotate at VR – positive rate of climb confirmed",
        "GEAR UP after positive rate",
        "Climb at V2+15 (approximately 163 kt)",
        "At acceleration altitude (1 500 ft): FLCH, accelerate",
        "Checklist: ENGINE FAILURE NON-NORMAL",
        "Consider SINGLE ENGINE approach and landing",
    ],
)

SCENARIO_ENGINE_FAILURE_CRUISE = ScenarioConfig(
    name="Engine Failure in Cruise – FL350",
    description=(
        "Engine 1 flames out in cruise at FL350 over mountainous terrain. "
        "Drift-down to single-engine cruise altitude, divert to nearest airport."
    ),
    altitude_m=10_668.0,
    heading_deg=95.0,
    speed_cas_kts=270.0,
    fuel_kg=12_000.0,
    payload_kg=15_000.0,
    gear_down=False,
    flap_position=0,
    throttle=0.50,    # eng2 only at half power at start
    ap_engaged=True,
    ap_target_alt_m=7_620.0,   # drift-down to ~FL250
    ap_target_hdg=95.0,
    ap_target_cas_kts=255.0,
    wind_speed_kts=40.0,
    wind_dir_deg=270.0,
    route=["KDEN"],
    notes=[
        "Checklist: ENGINE FAILURE/SHUTDOWN NON-NORMAL",
        "Drift-down: descend to single-engine cruise altitude (≈FL250)",
        "Reduce speed to green dot / long-range cruise speed",
        "Declare MAYDAY – request direct routing to nearest suitable airport",
        "Check fuel: single engine burns ~3 000 kg/h vs ~5 500 kg/h dual",
        "Brief crew for single-engine approach",
    ],
)

SCENARIO_RAPID_DECOMPRESSION = ScenarioConfig(
    name="Rapid Decompression at FL350",
    description=(
        "A window seal failure causes rapid decompression at FL350. "
        "Don oxygen masks, initiate emergency descent to 10 000 ft."
    ),
    altitude_m=10_668.0,
    heading_deg=90.0,
    speed_cas_kts=270.0,
    fuel_kg=15_000.0,
    payload_kg=15_000.0,
    gear_down=False,
    flap_position=0,
    throttle=0.98,
    ap_engaged=True,
    ap_target_alt_m=3_048.0,   # emergency descent to 10 000 ft
    ap_target_hdg=90.0,
    ap_target_cas_kts=320.0,   # VMO during emergency descent
    wind_speed_kts=30.0,
    wind_dir_deg=270.0,
    route=[],
    notes=[
        "DON OXYGEN MASKS immediately",
        "Declare MAYDAY",
        "Emergency descent: thrust IDLE, speedbrakes EXT, pitch -15°",
        "Target speed: MMO/VMO (M0.82/340kt CAS)",
        "Turn away from terrain if necessary",
        "Level at 10 000 ft (or MEA + 1 000 ft)",
        "Monitor cabin altitude – target below 14 000 ft",
    ],
)

SCENARIO_GO_AROUND = ScenarioConfig(
    name="Missed Approach / Go-Around",
    description=(
        "Runway incursion at decision height – execute missed approach. "
        "Aircraft is on final at 1 000 ft, configured for landing."
    ),
    altitude_m=304.8,   # 1 000 ft
    heading_deg=280.0,
    speed_cas_kts=145.0,
    fuel_kg=4_500.0,
    payload_kg=15_000.0,
    gear_down=True,
    flap_position=30,
    throttle=0.45,
    ap_engaged=False,
    ap_target_alt_m=609.6,   # go-around altitude 2 000 ft
    ap_target_hdg=280.0,
    ap_target_cas_kts=210.0,
    wind_speed_kts=10.0,
    wind_dir_deg=270.0,
    route=[],
    notes=[
        "TOGA thrust – announce 'Go Around, Flaps 15'",
        "Pitch to 15° initially to arrest descent",
        "Positive rate: GEAR UP",
        "At acceleration altitude: Flaps schedule UP",
        "Engage autopilot, fly published missed approach procedure",
        "Declare intentions to ATC – expect vectors for another approach",
    ],
)

SCENARIO_WINDSHEAR = ScenarioConfig(
    name="Windshear Encounter on Approach",
    description=(
        "Microburst windshear encountered at 800 ft on approach. "
        "Airspeed drops 30 kt suddenly. Execute windshear escape manoeuvre."
    ),
    altitude_m=243.8,   # 800 ft
    heading_deg=280.0,
    speed_cas_kts=145.0,
    fuel_kg=5_000.0,
    payload_kg=15_000.0,
    gear_down=True,
    flap_position=15,
    throttle=0.55,
    ap_engaged=True,
    ap_target_alt_m=609.6,
    ap_target_hdg=280.0,
    ap_target_cas_kts=200.0,
    wind_speed_kts=30.0,
    wind_dir_deg=100.0,   # severe headwind component initially (then reversal)
    route=[],
    notes=[
        "WINDSHEAR alert: 'WINDSHEAR AHEAD'",
        "Windshear escape: TOGA thrust, pitch to 15° (stick-shaker = pitch no further)",
        "DO NOT change flap/gear configuration",
        "Monitor airspeed trend – maintain attitude even if speed drops",
        "EGPWS 'PULL UP' call: add up to stick-shaker",
        "Continue climb until clear of windshear",
        "Declare MAYDAY if unable to maintain terrain clearance",
    ],
)

ALL_EMERGENCY_SCENARIOS = [
    SCENARIO_ENGINE_FAILURE_TAKEOFF,
    SCENARIO_ENGINE_FAILURE_CRUISE,
    SCENARIO_RAPID_DECOMPRESSION,
    SCENARIO_GO_AROUND,
    SCENARIO_WINDSHEAR,
]

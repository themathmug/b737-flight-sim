#!/usr/bin/env python3
"""
Boeing 737-800 Study-Level Flight Simulator
============================================

Entry point.  Run with:

    python main.py                    # interactive cockpit (curses)
    python main.py --scenario 2       # start on scenario 2 (cruise)
    python main.py --list-scenarios   # print available scenarios
    python main.py --headless 120     # headless run for 120 sim-seconds
"""

import argparse
import sys
import time


def _list_scenarios() -> None:
    from src.scenarios.normal_procedures import ALL_SCENARIOS
    from src.scenarios.emergency_procedures import ALL_EMERGENCY_SCENARIOS

    print("\n=== NORMAL PROCEDURES ===")
    for i, s in enumerate(ALL_SCENARIOS, 1):
        print(f"  {i}. {s.name}")
    print("\n=== EMERGENCY PROCEDURES ===")
    for i, s in enumerate(ALL_EMERGENCY_SCENARIOS, 6):
        print(f"  {i}. {s.name}")
    print()


def _run_headless(scenario_idx: int, duration_s: float, dt: float = 0.1) -> None:
    """Run the simulator headlessly and print a periodic status."""
    from src.scenarios.normal_procedures import ALL_SCENARIOS
    from src.scenarios.emergency_procedures import ALL_EMERGENCY_SCENARIOS
    from src.simulator import Simulator

    all_sc = ALL_SCENARIOS + ALL_EMERGENCY_SCENARIOS
    if scenario_idx < 1 or scenario_idx > len(all_sc):
        scenario_idx = 1

    sc = all_sc[scenario_idx - 1]
    print(f"\nRunning: {sc.name}")
    print(f"Duration: {duration_s:.0f} s  dt={dt} s\n")

    sim = Simulator.from_scenario(sc)
    steps = int(duration_s / dt)
    report_every = max(1, int(10.0 / dt))   # report every 10 sim-seconds

    for i in range(steps):
        state = sim.step()
        if i % report_every == 0:
            f = state.flight
            print(
                f"T={f.sim_time_s:6.0f}s  ALT={f.altitude_m*3.28084:7.0f}ft  "
                f"CAS={f.cas_ms*1.94384:5.0f}kt  HDG={f.heading_deg:5.1f}°  "
                f"VS={f.vs_fpm:+6.0f}fpm  "
                f"N1={sim.model.engines.eng1.state.n1_pct:5.1f}%  "
                f"FUEL={f.fuel_kg:6.0f}kg"
            )

    f = state.flight
    print(f"\n--- Final state ---")
    print(f"  Altitude : {f.altitude_m*3.28084:.0f} ft")
    print(f"  CAS      : {f.cas_ms*1.94384:.0f} kt")
    print(f"  Heading  : {f.heading_deg:.1f}°")
    print(f"  Fuel left: {f.fuel_kg:.0f} kg")
    print(f"  On ground: {f.on_ground}")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Boeing 737-800 Study-Level Flight Simulator"
    )
    parser.add_argument(
        "--scenario", "-s", type=int, default=0,
        help="Scenario number to load (default: 0 = default state)"
    )
    parser.add_argument(
        "--list-scenarios", "-l", action="store_true",
        help="List available training scenarios and exit"
    )
    parser.add_argument(
        "--headless", "-r", type=float, default=0.0,
        metavar="DURATION",
        help="Run headless for DURATION simulation seconds then exit"
    )
    args = parser.parse_args()

    if args.list_scenarios:
        _list_scenarios()
        return

    if args.headless > 0:
        _run_headless(args.scenario or 1, args.headless)
        return

    # Interactive cockpit
    from src.ui.cockpit import launch
    from src.simulator import Simulator
    from src.scenarios.normal_procedures import ALL_SCENARIOS
    from src.scenarios.emergency_procedures import ALL_EMERGENCY_SCENARIOS

    if args.scenario > 0:
        all_sc = ALL_SCENARIOS + ALL_EMERGENCY_SCENARIOS
        if 1 <= args.scenario <= len(all_sc):
            sim = Simulator.from_scenario(all_sc[args.scenario - 1])
        else:
            print(f"Unknown scenario {args.scenario}; use --list-scenarios to see options.")
            sys.exit(1)
    else:
        sim = Simulator()

    import curses
    try:
        curses.wrapper(lambda stdscr: __import__("src.ui.cockpit", fromlist=["run_cockpit"]).run_cockpit(stdscr, sim))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

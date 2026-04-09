# Boeing 737-800 Study-Level Flight Simulator

A comprehensive, terminal-based Boeing 737-800 flight simulator written in
Python.  Designed for study and understanding of 737 systems, flight dynamics,
and procedures.

```
╔═══════════════════════════════════════════════════════════════════════╗
║  ██ BOEING 737-800 FLIGHT SIMULATOR ██          SIM 00:12:34  FPS:20 ║
╠═══════════════════════════════╦═══════════════════╦═══════════════════╣
║ ─── PRIMARY FLIGHT DISPLAY ── ║ ─── EICAS ─────── ║ ─── NAVIGATION ── ║
║ CAS    250 kt                 ║        ENG 1 ENG 2║ LAT  47.4490°     ║
║ TAS    300 kt                 ║ N1 %   88.2  88.2 ║ LON -122.3090°    ║
║ GS     295 kt                 ║ N2 %   87.2  87.2 ║ HDG   095.0°      ║
║ MACH  0.785                   ║ EGT °C  780   780 ║ GS    295 kt      ║
║ Vref   137 kt                 ║ FF kg/h 2750 2750 ║ WIND 270° /  50 kt║
║                               ║ ─ SYSTEMS ──────  ║ ─ FMS ROUTE ─────║
║ PITCH  +2.1°                  ║ HYD A: NORM       ║ KSEA → KORD       ║
║ BANK    0.0°                  ║ HYD B: NORM       ║ NEXT: KORD        ║
║                               ║ ELEC:  NORM       ║ DIST:  1 234 nm   ║
║ ALT  35 000 ft                ║ FUEL: 15 400 kg   ║ ETE:    04:12     ║
║ V/S    +100 fpm               ║ CABIN: 8 000 ft   ║ XTK:   +0.02 nm  ║
║ HDG    095.0°                 ║ OAT:  -52.0 °C    ║                   ║
║ AP ON  HDG_SEL ALT_HLD SPEED  ║                   ║                   ║
╠═══════════════════════════════╩═══════════════════╩═══════════════════╣
║ AP:ON  THR  82%  GEAR UP  FLP  0  PITCH +2.1°  BANK +0.0°  MASS 71t  ║
╠═══════════════════════════════════════════════════════════════════════╣
║ [W/S]pitch  [A/D]bank  [Z/X]thr  [G]gear  [F/V]flaps  [P]AP  [Q]quit║
╚═══════════════════════════════════════════════════════════════════════╝
```

## Features

### Flight Dynamics & Physics
- **ISA atmosphere** – temperature, pressure, density, speed of sound up to FL650
- **737-800 aerodynamic polar** – CL/CD curves, stall, ground effect
- **CFM56-7B27 engine model** – N1/N2, EGT, fuel flow, spool lag
- **3-DOF point-mass equations of motion** – speed, flight-path angle, heading, position
- **Wind and turbulence** – steady wind + random gust generation

### Aircraft Systems
- **Autopilot** – Heading Select, Altitude Hold, Vertical Speed, Speed (A/THR), Approach
- **Flight Management System** – waypoint database, route planning, LNAV, ETE
- **Landing gear** – extension/retraction with transit timing and speed limits
- **Flaps / slats** – 8 detent positions (0, 1, 5, 10, 15, 25, 30, 40) with VFE limits
- **Hydraulic systems** – Systems A & B tied to engine health
- **Electrical system** – AC buses, battery backup
- **Fuel system** – wing tanks + centre tank, imbalance detection
- **Pressurisation** – cabin altitude with first-order lag model

### Instrument Panel
- **Primary Flight Display (PFD)** – speed tape, attitude, altitude, V/S, heading, AP modes
- **EICAS** – N1/N2/EGT/FF per engine, systems status, cautions/warnings
- **Navigation display** – position, route, next waypoint, ETE, cross-track error

### Training Scenarios

| # | Name | Type |
|---|---|---|
| 1 | Normal Takeoff – KSEA RWY 16C | Normal |
| 2 | Cruise – FL350 KSEA→KJFK | Normal |
| 3 | ILS Approach – KJFK RWY 31L | Normal |
| 4 | Short Hop – KSEA→KPDX | Normal |
| 6 | Engine Failure at V1 | Emergency |
| 7 | Engine Failure in Cruise | Emergency |
| 8 | Rapid Decompression FL350 | Emergency |
| 9 | Missed Approach / Go-Around | Emergency |
| 10 | Windshear on Approach | Emergency |

## Quick Start

```bash
git clone https://github.com/themathmug/b737-flight-sim.git
cd b737-flight-sim
pip3 install -r requirements.txt

# Interactive curses cockpit (requires a real terminal)
python3 main.py                       # starts at default takeoff scenario
python3 main.py --scenario 2          # cruise FL350 KSEA→KJFK  ← best for study

# Headless (no terminal needed)
python3 main.py --headless 300 --scenario 2

# List all scenarios
python3 main.py --list-scenarios
```

See [`docs/quick_start.md`](docs/quick_start.md) for a full tutorial.

## Documentation

| Document | Description |
|---|---|
| [`docs/quick_start.md`](docs/quick_start.md) | Installation and first flight tutorial |
| [`docs/architecture.md`](docs/architecture.md) | System architecture and data flow |
| [`docs/aerodynamic_model.md`](docs/aerodynamic_model.md) | Aerodynamic equations and parameters |
| [`docs/training_scenarios.md`](docs/training_scenarios.md) | All scenario descriptions with procedures |

## Running Tests

```bash
python3 -m pytest tests/ -v
```

## Requirements

- Python 3.10+
- numpy ≥ 1.21
- A terminal ≥ 80 × 24 (120 × 40 recommended for full cockpit layout)

Happy flying! ✈️
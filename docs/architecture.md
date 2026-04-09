# Boeing 737-800 Flight Simulator – Architecture

## Overview

The simulator is structured as a layered Python application:

```
main.py                  ← Entry-point / CLI
src/
├── simulator.py         ← Master controller (ties subsystems together)
├── flight_model.py      ← 3-DOF equations of motion
├── physics/
│   ├── atmosphere.py    ← ISA atmosphere model
│   ├── aerodynamics.py  ← 737-800 lift/drag polar
│   └── engines.py       ← CFM56-7B27 thrust/fuel/EGT
├── systems/
│   ├── autopilot.py     ← FD / AP / A/THR controllers
│   ├── fms.py           ← Flight Management System (routes, perf)
│   └── aircraft_systems.py ← Gear, flaps, hydraulics, electrical, fuel
├── instruments/
│   ├── pfd.py           ← Primary Flight Display data builder
│   ├── mfd.py           ← Multi-Function Display (nav page)
│   └── eicas.py         ← Engine Indicating and Crew Alerting
├── scenarios/
│   ├── normal_procedures.py   ← Takeoff, cruise, approach scenarios
│   └── emergency_procedures.py ← Engine failure, decompression, etc.
└── ui/
    └── cockpit.py       ← Curses terminal cockpit renderer
```

---

## Data Flow

```
Keyboard/CLI
    │
    ▼
Simulator.step()
    │
    ├─▶ AutopilotSystem.compute()  ─▶ pitch/bank/throttle commands
    │
    ├─▶ FlightModel.update(PilotInputs)
    │       │
    │       ├─▶ DualEngineSystem.update()   → thrust, fuel-flow, EGT
    │       ├─▶ AerodynamicsModel.compute() → lift, drag
    │       └─▶ Integrate EOM              → new state (alt, speed, hdg…)
    │
    ├─▶ AircraftSystems.update()   → gear, flaps, hyd, elec, pressurisation
    │
    ├─▶ FMS.update()               → XTK error, next-waypoint distance
    │
    ├─▶ build_pfd() / build_mfd() / build_eicas()  → display packets
    │
    └─▶ SimulatorState             → returned to UI
```

---

## Module Descriptions

### `src/physics/atmosphere.py`
Implements the **ICAO International Standard Atmosphere (ISA)** up to 20 km.

Key exports:
| Function | Description |
|---|---|
| `temperature(alt_m)` | ISA temperature (K) |
| `pressure(alt_m)` | ISA pressure (Pa) |
| `density(alt_m)` | Air density (kg/m³) |
| `speed_of_sound(alt_m)` | Speed of sound (m/s) |
| `cas_to_tas(cas, alt)` | CAS → TAS conversion |
| `dynamic_pressure(tas, alt)` | q = ½ρV² (Pa) |

### `src/physics/aerodynamics.py`
Implements the **737-800 aerodynamic polar** using:
- Helmbold lift-slope formula corrected for wing sweep/AR
- Oswald efficiency (e = 0.82)
- Flap/slat CL increment tables
- Ground-effect induced-drag correction
- Post-stall CL model with a smooth drop-off

### `src/physics/engines.py`
Models the **CFM56-7B27** turbofan:
- Throttle → N1 target (linear)
- N1 → thrust with altitude correction ∝ (ρ/ρ₀)^0.9 and Mach ram drag
- First-order spool lag (τ_up = 4 s, τ_down = 6 s)
- EGT and fuel-flow empirical models

### `src/flight_model.py`
**3-DOF point-mass equations of motion**:
```
m·V̇  = T·cosα – D – W·sinγ          (speed)
m·V·γ̇ = L·cosφ + T·sinα – W·cosγ    (flight-path angle)
ḣdg  = g·tanφ / V                    (coordinated turn)
ẋ    = V·cosγ·sin(hdg)               (x position)
ẏ    = V·cosγ·cos(hdg)               (y position)
ż    = V·sinγ                         (altitude)
```
where α is AoA, γ is flight-path angle, φ is bank angle.

### `src/systems/autopilot.py`
PID-based controllers for:
| Mode | Controller |
|---|---|
| HDG SEL | Proportional heading error → bank command |
| ALT HLD | Proportional alt error → VS command → pitch |
| VS | Proportional VS error → pitch command |
| SPEED | PI speed error → throttle |
| APPROACH | Fixed –1° pitch target with GS deviation |

### `src/systems/fms.py`
- Built-in waypoint database (20+ ICAOs / enroute fixes)
- Haversine great-circle distance and initial bearing
- Cross-track error via projection
- Waypoint sequencing (capture radius 5 km)
- Performance: fuel required, top-of-descent estimate

### `src/systems/aircraft_systems.py`
- **Gear**: transit simulation (10 s), speed limits
- **Flaps**: slew to selected detent at 3°/s, VFE table
- **Hydraulics**: Systems A & B tied to engine 1 & 2
- **Electrical**: AC bus 1 & 2 tied to engine 1 & 2
- **Fuel**: Two wing tanks + centre, imbalance check
- **Pressurisation**: first-order cabin altitude lag model
- **EICAS alerts**: low fuel, hydraulic failure, gear-not-down, flap overspeed

### `src/ui/cockpit.py`
Terminal UI using Python's built-in `curses` library.  Renders:
- PFD (speed, attitude, altitude, V/S, heading, AP modes)
- EICAS (N1/N2/EGT/FF per engine, systems status, fuel, cabin alt)
- Navigation page (position, route, next waypoint, ETE, XTK)
- Status bar (AP targets, throttle, gear, flaps, pitch, bank)
- Keyboard help overlay

---

## Coordinate System

- **x**: East (metres from starting point)
- **y**: North (metres from starting point)
- **altitude**: MSL in metres
- **heading**: magnetic degrees, 0 = North, 90 = East
- **bank**: positive = right wing down
- **pitch**: positive = nose up
- **flight-path angle (γ)**: positive = climb

Flat-earth approximation is used for local position integration.  The FMS
converts local x/y to lat/lon using a simple spherical-Earth projection for
route guidance.

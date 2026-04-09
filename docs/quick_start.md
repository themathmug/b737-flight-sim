# Quick Start Guide

## Prerequisites

- Python 3.10 or later
- A terminal at least **120 columns × 40 rows** (for the cockpit display)
- `pip` (to install dependencies)

---

## Installation

```bash
git clone https://github.com/themathmug/b737-flight-sim.git
cd b737-flight-sim
pip install -r requirements.txt
```

---

## Running the Simulator

### Interactive cockpit (recommended)
```bash
python main.py
```

### Start on a specific scenario
```bash
python main.py --scenario 2   # cruise at FL350
```

### List all available scenarios
```bash
python main.py --list-scenarios
```

### Headless mode (for testing / logging)
```bash
python main.py --headless 300 --scenario 1   # run 300 sim-seconds, scenario 1
```

---

## Keyboard Controls

| Key | Action |
|---|---|
| `W` | Pitch nose up |
| `S` | Pitch nose down |
| `A` | Bank left |
| `D` | Bank right |
| `Z` | Decrease throttle |
| `X` | Increase throttle |
| `P` | Toggle autopilot (HDG + ALT + SPD) |
| `G` | Toggle landing gear |
| `F` | Extend flaps (next detent) |
| `V` | Retract flaps (previous detent) |
| `B` | Toggle speedbrakes |
| `H` | AP heading −10° |
| `J` | AP heading +10° |
| `+` / `=` | AP altitude +1 000 ft |
| `-` | AP altitude −1 000 ft |
| `1`–`5` | Load normal procedure scenario |
| `6`–`9` | Load emergency scenario |
| `?` or `/` | Show help overlay |
| `Q` or `Esc` | Quit |

---

## Cockpit Layout

```
╔═══════════════════════════════════════════════════════════════════════╗
║                     B737-800 FLIGHT SIMULATOR                        ║
╠═══════════════════════════════╦═══════════════════╦═══════════════════╣
║  PRIMARY FLIGHT DISPLAY       ║  EICAS            ║  NAVIGATION       ║
║  CAS  250 kt                  ║  ENG 1   ENG 2    ║  LAT / LON        ║
║  TAS  300 kt                  ║  N1%     N1%      ║  HDG / GS         ║
║  GS   295 kt                  ║  N2%     N2%      ║  WIND             ║
║  MACH 0.785                   ║  EGT °C  EGT °C   ║  ─ FMS ROUTE ─   ║
║                               ║  FF kg/h FF kg/h  ║  KSEA→KORD→KJFK  ║
║  PITCH  +2.1°                 ║  ─ SYSTEMS ─      ║  NEXT: KORD       ║
║  BANK    0.0°                 ║  HYD A/B: NORM    ║  DIST: 1 234 nm   ║
║                               ║  ELEC:    NORM    ║  ETE:  04:12      ║
║  ALT  35 000 ft               ║  FUEL: 15 400 kg  ║  XTK:  +0.02 nm  ║
║  V/S    +100 fpm              ║  CABIN: 8 000 ft  ║                   ║
║                               ║  OAT:  -52.0 °C   ║                   ║
║  HDG    095.0°                ║                   ║                   ║
║  AP ON  HDG_SEL ALT_HLD SPEED ║                   ║                   ║
╠═══════════════════════════════╩═══════════════════╩═══════════════════╣
║  AP:ON  THR  82%  GEAR UP  FLP  0  PITCH +2.1°  BANK +0.0°  MASS 71t ║
╠═══════════════════════════════════════════════════════════════════════╣
║  [W/S]pitch  [A/D]bank  [Z/X]thr  [G]gear  [F/V]flaps  [P]AP  [Q]quit║
╚═══════════════════════════════════════════════════════════════════════╝
```

---

## First Flight Tutorial (Scenario 1 – Takeoff)

1. **Start the simulator**  
   ```bash
   python main.py --scenario 1
   ```

2. **Apply takeoff thrust**  
   Press `X` repeatedly (or hold) until throttle reaches ~95%.  
   Watch N1 spool up to ~96%.

3. **Rotate**  
   At CAS ≈ 143 kt, press `W` to pitch to 15°.

4. **Gear up**  
   Once positive rate of climb shows, press `G`.

5. **Engage autopilot**  
   Press `P`. The AP will hold heading 160° and climb.  
   Use `+` to set a target altitude of 15 000 ft.

6. **Flap retraction schedule**  
   - Flaps 5 → retract to 1 at V2+15 (~163 kt)
   - Flaps 1 → UP at acceleration altitude (1 500 ft)
   Press `V` at the appropriate speeds.

7. **Cruise**  
   At 10 000 ft, speed is limited to 250 kt (CAS) below FL100.  
   Above FL100 you can accelerate to 280 kt / Mach 0.78.

---

## Autopilot Quick Reference

| Mode | How to arm |
|---|---|
| Heading Select | Press `P` (AP on); use `H`/`J` to change target |
| Altitude Hold | Press `P`; use `+`/`-` to set target |
| Speed (A/THR) | Enabled automatically with `P` |
| LNAV (route) | Set a route in FMS, then activate – currently heading-select fallback |
| Approach | Select scenario 3 (ILS approach) |

---

## Running Tests

```bash
pytest tests/ -v
```

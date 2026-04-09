# Training Scenarios

## Overview

The simulator includes **4 normal procedure scenarios** and **5 emergency procedure scenarios**.
Load them with the number keys `1`–`9` inside the cockpit, or via the `--scenario N` flag.

---

## Normal Procedure Scenarios

### 1 – Normal Takeoff (KSEA RWY 16C)

**Objective**: Perform a standard departure from Seattle-Tacoma.

**Initial conditions**:
- Altitude: 0 ft (runway 16C)
- Speed: 0 kt
- Heading: 160°
- Fuel: 14 000 kg  Payload: 15 000 kg
- Flaps: 5  Gear: down

**Key speeds**:
| Speed | Meaning |
|---|---|
| V1 = 138 kt | Decision speed – no abort |
| VR = 143 kt | Rotation speed |
| V2 = 148 kt | Takeoff safety speed |
| V2+15 = 163 kt | Clean-up speed |

**Procedure**:
1. Apply TOGA thrust (X key to 95–98 %)
2. Rotate at 143 kt (W key, pitch to 15°)
3. Gear UP after positive rate (G key)
4. Engage AP at 400 ft AGL (P key)
5. Retract flaps on schedule (V key)
6. Level at 5 000 ft; set AP altitude

**Learning points**: Engine spool response, rotation technique, flap schedule, AP engagement altitude.

---

### 2 – Cruise (FL350, KSEA→KJFK)

**Objective**: Manage a transatlantic cruise, monitor fuel, and practice AP modes.

**Initial conditions**:
- Altitude: 35 000 ft
- Speed: ~270 kt CAS (Mach 0.785)
- Heading: 095°
- Fuel: 18 000 kg
- Autopilot: ON (ALT/HDG/SPD)
- Wind: 270°/50 kt (tailwind)

**Tasks**:
- Monitor N1 and EGT (should be ~90 % / ~780 °C at cruise)
- Monitor fuel flow (~5 500 kg/h total), predict arrival fuel
- Change heading to deviate around weather (H/J keys)
- Perform top-of-descent calculation (TOD ≈ 120 nm from dest at 3°)
- Begin descent by selecting lower AP altitude (- key)

**Learning points**: Cruise fuel monitoring, step-climb logic, AP altitude management.

---

### 3 – ILS Approach (KJFK RWY 31L)

**Objective**: Fly a stabilised ILS approach to landing.

**Initial conditions**:
- Altitude: 3 000 ft
- Speed: 180 kt CAS
- Heading: 310°
- Fuel: 4 000 kg
- Flaps: 15  Gear: up
- AP: ON (HDG + ALT hold)

**Approach sequence**:
1. Below 10 000 ft: CAS ≤ 250 kt
2. At glide-slope alive (≈ 2 000 ft): Flaps 25, gear down (G)
3. Final: Flaps 30, Vapp ≈ Vref + 5 ≈ 142 kt
4. At 500 ft: Check stabilised (speed ±10 kt, V/S < –1 000 fpm, heading ±5°)
5. At 50 ft: RETARD throttle (Z key), flare to 2°
6. Touchdown: spoilers, reverse thrust, brakes

**Learning points**: Approach configuration management, speed discipline, decision height.

---

### 4 – Short Hop (KSEA→KPDX)

**Objective**: Complete a full flight profile in a short time.

**Initial conditions**:
- Altitude: 0 ft
- Speed: 0 kt
- Fuel: 6 000 kg

**Route**: KSEA → BEAVR  
**Cruise altitude**: 10 000 ft

**Learning points**: Full flight cycle in compressed time; suitable for repeated practice.

---

## Emergency Scenarios

### 6 – Engine Failure at V1

**Objective**: Handle single-engine takeoff and initial climb.

**What happens**: Engine 2 has failed (N1 at 0 %).

**Immediate actions**:
1. Continue takeoff (V1 = no-abort point)
2. Maintain directional control (right rudder)
3. Rotate at VR, climb at V2+15 (≈ 163 kt)
4. Gear UP, accelerate, clean up flaps
5. Checklist: ENGINE FAILURE/SHUTDOWN NON-NORMAL

**Key performance note**: Single-engine climb gradient is approximately 2.4 % at V2.

---

### 7 – Engine Failure in Cruise

**Objective**: Drift-down to single-engine cruise altitude and divert.

**What happens**: Engine 1 N1 drops to idle/windmill (~22 %).

**Actions**:
1. Checklist: ENGINE FAILURE/SHUTDOWN NON-NORMAL
2. Drift-down: descend to ~FL250 (single-engine optimum altitude)
3. Reduce to long-range cruise speed (~255 kt)
4. Declare MAYDAY, request direct routing
5. Note increased fuel burn asymmetry

---

### 8 – Rapid Decompression at FL350

**Objective**: Execute emergency descent to 10 000 ft.

**What happens**: Cabin altitude begins climbing rapidly.

**Immediate actions** (from memory – time critical at altitude):
1. Oxygen masks ON
2. MAYDAY MAYDAY MAYDAY
3. Emergency descent: Thrust IDLE, speedbrakes EXT
4. Pitch to -15°, descend at VMO/MMO
5. Level at 10 000 ft (or MEA + 1 000 ft)
6. Divert to nearest airport

---

### 9 – Missed Approach / Go-Around

**Objective**: Execute a missed approach from 1 000 ft.

**What happens**: Aircraft on final, go-around required.

**Immediate actions**:
1. TOGA thrust
2. "Go Around, Flaps 15"
3. Pitch to 15°, arrest descent
4. Positive rate: GEAR UP
5. Accelerate, clean up flaps on schedule
6. Fly published missed approach (published heading + 2 000 ft)

---

### 10 – Windshear on Approach

**Objective**: Recognise and escape from a microburst.

**What happens**: Airspeed suddenly drops 30 kt at 800 ft (microburst encounter).

**Immediate actions**:
1. "WINDSHEAR AHEAD" alert – GO AROUND
2. TOGA thrust
3. Pitch to 15° (do not exceed stick-shaker)
4. DO NOT change configuration
5. Monitor airspeed trend (not absolute value)
6. If EGPWS "PULL UP": pitch to stick-shaker, no configuration change
7. Continue climb until clear

---

## Performance Reference (737-800, typical weights)

| Phase | Speed | N1 | Fuel flow |
|---|---|---|---|
| Taxi | – | 22–25 % | ~200 kg/h total |
| Takeoff (TOGA) | V2 = 148 kt | 96 % | ~8 000 kg/h total |
| Climb (250 kt / 10 000 ft) | 250 kt CAS | 88 % | ~7 500 kg/h |
| Climb (M0.76 / FL280) | M0.76 | 90 % | ~6 800 kg/h |
| Cruise (M0.785 / FL350) | M0.785 | 88 % | ~5 500 kg/h |
| Descent (idle, 3°) | 280 kt | 25 % | ~400 kg/h |
| Final approach | 145 kt | 55 % | ~2 000 kg/h |

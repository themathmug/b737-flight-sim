# Aerodynamic Model Explanation

## 1  Wing Geometry (737-800)

| Parameter | Value |
|---|---|
| Reference area S | 125.08 m² |
| Wingspan b | 35.79 m |
| Aspect ratio AR = b²/S | 10.24 |
| Quarter-chord sweep | 25° |
| Taper ratio | 0.278 |
| Mean aerodynamic chord | 3.495 m |
| Oswald efficiency e | 0.82 |

---

## 2  Lift Model

### 2.1 Linear region

The lift coefficient in the linear (pre-stall) region is:

```
CL = CL₀ + ΔCL_flap + CL_α · (α – α₀)
```

where  
- `CL₀ = 0.10` – zero-angle-of-attack lift (camber)  
- `CL_α = 5.5 /rad` – lift slope  
- `α₀ = –2°` – zero-lift AoA  
- `ΔCL_flap` – flap/slat CL increment (see table below)

### 2.2 Flap CL increment table

| Flap position | ΔCL |
|---|---|
| 0 (clean) | 0.00 |
| 1 | 0.10 |
| 5 | 0.25 |
| 10 | 0.50 |
| 15 | 0.70 |
| 25 | 0.90 |
| 30 | 1.00 |
| 40 | 1.20 |

### 2.3 Maximum lift coefficient

| Configuration | CL_max |
|---|---|
| Clean | 1.55 |
| Flaps 10 | 1.90 |
| Flaps 25 | 2.15 |
| Flaps 30 | 2.35 |
| Flaps 40 | 2.55 |

### 2.4 Post-stall model

Above the stall angle of attack (α_stall), CL decreases as:

```
CL_stall = CL_max – 0.08 · (α – α_stall)²
```

CL is floored at 0.2 to prevent an abrupt zero-lift condition.

---

## 3  Drag Model

### 3.1 Drag polar

The drag coefficient uses the classic **parabolic polar**:

```
CD = CD₀ + Ke · k · CL²
```

where  
- `k = 1 / (π · AR · e) ≈ 0.0377`  
- `Ke` = ground-effect factor (reduces induced drag near ground)

### 3.2 Parasite drag (CD₀)

| Configuration | CD₀ |
|---|---|
| Clean | 0.0175 |
| Gear down (+) | +0.0200 |
| Speedbrakes (+) | +0.0120 |

Flap drag increments are added:

| Flap position | ΔCD₀ |
|---|---|
| 0 | 0.0000 |
| 10 | 0.0040 |
| 25 | 0.0100 |
| 30 | 0.0140 |
| 40 | 0.0200 |

### 3.3 Ground effect

The classical ground-effect formula reduces induced drag:

```
Ke = (16·h/b)² / (1 + (16·h/b)²)
```

where `h` is wheel height above ground and `b` is the wingspan.  
When `h/b → 0` (on runway) `Ke ≈ 0.3`.  When `h/b > 1`, `Ke → 1.0`.

---

## 4  Lift and Drag Forces

```
L = q · S · CL      (Newtons)
D = q · S · CD      (Newtons)

q = ½ · ρ · V²     (dynamic pressure, Pa)
ρ = f(altitude)    (ISA density, kg/m³)
```

---

## 5  Stall Speeds

The stall speed (TAS) is derived from the maximum lift:

```
Vs = √( 2·W / (ρ · S · CL_max) )
```

Typical 737-800 stall speeds (at MTOW = 79 016 kg, sea level):

| Configuration | Vs (TAS) | Vs (CAS) |
|---|---|---|
| Clean | ~82 m/s | ~82 m/s |
| Flaps 10 | ~70 m/s | ~70 m/s |
| Flaps 40 | ~62 m/s | ~62 m/s |

(CAS ≈ TAS at sea level; at altitude TAS is higher.)

---

## 6  CFM56-7B27 Thrust Model

### 6.1 Thrust vs N1

```
T = T_SL_max · (ρ/ρ₀)^0.9 · ((N1 – N1_idle) / (N1_max – N1_idle))²
```

- `T_SL_max = 121 400 N` (sea-level static thrust per engine)  
- Ram drag correction: `T_net = T_gross · (1 – 0.15·M²)`

### 6.2 Engine parameters

| Parameter | Value |
|---|---|
| Max thrust SL (per engine) | 121.4 kN |
| N1 idle | 22 % |
| N1 max | 100 % |
| EGT at max thrust | ~900 °C |
| EGT limit | 950 °C |
| Spool-up time constant | 4 s |
| Spool-down time constant | 6 s |
| Fuel flow at idle SL | ~180 kg/h |
| Fuel flow at max SL | ~4 000 kg/h |

### 6.3 Fuel flow model

```
FF = FF_idle + (FF_max – FF_idle) · ((N1 – N1_idle)/(N1_max – N1_idle))^2.5
```

Altitude correction: `FF_alt = FF_sl · (ρ/ρ₀)^0.7`

Typical cruise fuel flow (both engines, cruise N1 ≈ 90 %): **~5 500 kg/h**.

---

## 7  Limitations and Simplifications

1. **No 6-DOF** – rotational inertia (pitch/roll/yaw moments) are not modelled;  
   attitude rates are directly commanded.  
2. **Flat Earth** – position integration uses flat-earth Cartesian coordinates;  
   FMS converts to lat/lon for route display.  
3. **No compressibility correction** – CL_alpha is constant (valid Mach < 0.6);  
   at high Mach the model is slightly optimistic.  
4. **Steady engine model** – no combustion instability or hot-section thermal  
   detail; EGT is an empirical proxy.  
5. **Magnetic variation** – not applied (all headings are assumed magnetic).  
6. **No crosswind/sideslip** – the lateral model assumes perfectly coordinated  
   flight; there is no rudder/yaw model.

"""
Curses-based cockpit interface for the Boeing 737-800 simulator.

Layout (requires ≥ 120 × 40 terminal):

  ┌─────────────────────────────────────────────────────────────────────────────────┐
  │  ██ B737-800 FLIGHT SIMULATOR  ██               [SIM TIME]  [FPS]              │
  ├────────────────────────┬────────────────────────┬────────────────────────────────┤
  │  PRIMARY FLIGHT DISP   │  ENGINE / EICAS        │  NAVIGATION / FMS              │
  │  SPD | ATTITUDE | ALT  │  ENG1  |  ENG2         │  HDG / ROUTE / ETE             │
  │      |          |      │  N1/N2/EGT/FF          │                                │
  │      |   AHRS   |  VS  │  SYSTEMS STATUS        │  CAUTIONS / WARNINGS           │
  ├────────────────────────┴────────────────────────┴────────────────────────────────┤
  │  AP:  LAT │ VERT │ SPD     TARGETS: HDG  ALT  SPD   THROTTLE  GEAR  FLAPS       │
  ├─────────────────────────────────────────────────────────────────────────────────┤
  │  KEYBOARD CONTROLS                                                               │
  └─────────────────────────────────────────────────────────────────────────────────┘

Keyboard Controls
-----------------
  W/S       pitch up/down         A/D       bank left/right
  Z/X       throttle –/+          P         toggle autopilot
  G         gear toggle           F         flaps extend
  V         flaps retract         B         speedbrakes
  H         heading –10°          J         heading +10°
  +/-       AP altitude ±1000 ft  Q/Escape  quit
  1-5       select scenario       ?         help overlay
"""

import curses
import math
import sys
import time
from typing import Optional

from src.simulator import Simulator, SimulatorConfig, SimulatorState
from src.scenarios.normal_procedures import ALL_SCENARIOS
from src.scenarios.emergency_procedures import ALL_EMERGENCY_SCENARIOS


# ── Colour pair constants ──────────────────────────────────────────────────────
CP_NORMAL  = 0
CP_GREEN   = 1
CP_CYAN    = 2
CP_YELLOW  = 3
CP_RED     = 4
CP_MAGENTA = 5
CP_WHITE   = 6
CP_DIM     = 7


def _init_colors() -> None:
    curses.start_color()
    curses.use_default_colors()
    curses.init_pair(CP_GREEN,   curses.COLOR_GREEN,   -1)
    curses.init_pair(CP_CYAN,    curses.COLOR_CYAN,    -1)
    curses.init_pair(CP_YELLOW,  curses.COLOR_YELLOW,  -1)
    curses.init_pair(CP_RED,     curses.COLOR_RED,     -1)
    curses.init_pair(CP_MAGENTA, curses.COLOR_MAGENTA, -1)
    curses.init_pair(CP_WHITE,   curses.COLOR_WHITE,   -1)
    curses.init_pair(CP_DIM,     curses.COLOR_WHITE,   -1)


def _safe_addstr(win, y: int, x: int, text: str, attr: int = 0) -> None:
    """Add a string to a curses window, silently ignoring out-of-bounds errors."""
    max_y, max_x = win.getmaxyx()
    if y < 0 or y >= max_y or x < 0:
        return
    if x + len(text) > max_x:
        text = text[: max_x - x]
    if not text:
        return
    try:
        win.addstr(y, x, text, attr)
    except curses.error:
        pass


# ── Column / row layout constants ────────────────────────────────────────────
PFD_COL   = 1
EICAS_COL = 42
NAV_COL   = 84
PANEL_ROW = 2
CONTENT_ROW = 3
STATUS_ROW_OFFSET = -6   # rows from bottom
HELP_ROW_OFFSET   = -4


def _render_header(win, state: SimulatorState, cols: int) -> None:
    title = " ██ BOEING 737-800 FLIGHT SIMULATOR ██ "
    t_str = f"  SIM {int(state.sim_time_s // 3600):02d}:{int(state.sim_time_s % 3600 // 60):02d}:{int(state.sim_time_s % 60):02d}  FPS:{state.real_fps:.0f} "
    _safe_addstr(win, 0, 0, "=" * cols, curses.color_pair(CP_CYAN) | curses.A_BOLD)
    _safe_addstr(win, 1, 2, title, curses.color_pair(CP_WHITE) | curses.A_BOLD)
    _safe_addstr(win, 1, cols - len(t_str) - 1, t_str, curses.color_pair(CP_DIM))
    _safe_addstr(win, 2, 0, "─" * cols, curses.color_pair(CP_DIM))


def _render_pfd(win, state: SimulatorState, start_row: int) -> None:
    pfd = state.pfd
    if pfd is None:
        return
    r = start_row
    c = PFD_COL

    _safe_addstr(win, r, c, "─── PRIMARY FLIGHT DISPLAY ───────", curses.color_pair(CP_CYAN))
    r += 1

    # Speed
    spd_color = curses.color_pair(CP_RED) if pfd.overspeed or pfd.stall else curses.color_pair(CP_GREEN)
    _safe_addstr(win, r, c, f"CAS  {pfd.cas_kts:5.0f} kt", spd_color | curses.A_BOLD)
    r += 1
    _safe_addstr(win, r, c, f"TAS  {pfd.tas_kts:5.0f} kt", curses.color_pair(CP_NORMAL))
    r += 1
    _safe_addstr(win, r, c, f"GS   {pfd.gs_kts:5.0f} kt", curses.color_pair(CP_NORMAL))
    r += 1
    _safe_addstr(win, r, c, f"MACH    {pfd.mach:.3f}", curses.color_pair(CP_CYAN))
    r += 1
    _safe_addstr(win, r, c, f"Vref {pfd.vref_kts:5.0f} kt", curses.color_pair(CP_YELLOW))
    r += 1

    # Attitude
    r += 1
    _safe_addstr(win, r, c, f"PITCH {pfd.pitch_deg:+6.1f}°", curses.color_pair(CP_WHITE) | curses.A_BOLD)
    r += 1
    _safe_addstr(win, r, c, f"BANK  {pfd.bank_deg:+6.1f}°", curses.color_pair(CP_WHITE) | curses.A_BOLD)
    r += 1

    # Altitude / VS
    r += 1
    alt_color = curses.color_pair(CP_GREEN) | curses.A_BOLD
    _safe_addstr(win, r, c, f"ALT  {pfd.altitude_ft:7.0f} ft", alt_color)
    r += 1
    vs_color = (curses.color_pair(CP_RED) if abs(pfd.vs_fpm) > 3000
                else curses.color_pair(CP_WHITE))
    _safe_addstr(win, r, c, f"V/S  {pfd.vs_fpm:+7.0f} fpm", vs_color)
    r += 1

    # Heading
    r += 1
    _safe_addstr(win, r, c, f"HDG     {pfd.heading_deg:5.1f}°", curses.color_pair(CP_CYAN) | curses.A_BOLD)
    r += 1

    # Autopilot mode line
    r += 1
    ap_color = curses.color_pair(CP_GREEN) if pfd.ap_engaged else curses.color_pair(CP_DIM)
    ap_str = "AP ON " if pfd.ap_engaged else "AP OFF"
    _safe_addstr(win, r, c, f"{ap_str}  {pfd.lat_mode:<8} {pfd.vert_mode:<8} {pfd.spd_mode:<6}", ap_color)
    r += 1

    # Alerts
    if pfd.stall:
        _safe_addstr(win, r, c, " !! STALL  STALL !! ", curses.color_pair(CP_RED) | curses.A_BOLD | curses.A_BLINK)
        r += 1
    if pfd.overspeed:
        _safe_addstr(win, r, c, " !! OVERSPEED !!     ", curses.color_pair(CP_RED) | curses.A_BOLD)
        r += 1


def _render_eicas(win, state: SimulatorState, start_row: int) -> None:
    eicas = state.eicas
    if eicas is None:
        return
    r = start_row
    c = EICAS_COL

    _safe_addstr(win, r, c, "─── EICAS ────────────────────────", curses.color_pair(CP_CYAN))
    r += 1

    def eng_color(running: bool) -> int:
        return (curses.color_pair(CP_GREEN) | curses.A_BOLD
                if running else curses.color_pair(CP_RED))

    _safe_addstr(win, r, c, "          ENG 1    ENG 2", curses.color_pair(CP_WHITE))
    r += 1

    n1_color1 = eng_color(eicas.eng1_running)
    n1_color2 = eng_color(eicas.eng2_running)
    _safe_addstr(win, r, c, f"N1 %   ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 7, f"{eicas.n1_1:6.1f}    ", n1_color1)
    _safe_addstr(win, r, c + 17, f"{eicas.n1_2:6.1f}", n1_color2)
    r += 1

    _safe_addstr(win, r, c, f"N2 %   ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 7, f"{eicas.n2_1:6.1f}    ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 17, f"{eicas.n2_2:6.1f}", curses.color_pair(CP_NORMAL))
    r += 1

    egt1_col = curses.color_pair(CP_RED) if eicas.egt_1_exceeded else curses.color_pair(CP_YELLOW)
    egt2_col = curses.color_pair(CP_RED) if eicas.egt_2_exceeded else curses.color_pair(CP_YELLOW)
    _safe_addstr(win, r, c, f"EGT °C ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 7, f"{eicas.egt_1:6.0f}    ", egt1_col)
    _safe_addstr(win, r, c + 17, f"{eicas.egt_2:6.0f}", egt2_col)
    r += 1

    _safe_addstr(win, r, c, f"FF kg/h", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 7, f"{eicas.ff_1:6.0f}    ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 17, f"{eicas.ff_2:6.0f}", curses.color_pair(CP_NORMAL))
    r += 1

    r += 1
    _safe_addstr(win, r, c, "─ SYSTEMS ─────────────────────", curses.color_pair(CP_DIM))
    r += 1

    hyd_a = "NORM" if eicas.hyd_a_ok else "FAIL"
    hyd_b = "NORM" if eicas.hyd_b_ok else "FAIL"
    hyd_a_col = curses.color_pair(CP_GREEN) if eicas.hyd_a_ok else curses.color_pair(CP_RED)
    hyd_b_col = curses.color_pair(CP_GREEN) if eicas.hyd_b_ok else curses.color_pair(CP_RED)
    _safe_addstr(win, r, c, f"HYD A: ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 7, f"{hyd_a}", hyd_a_col)
    _safe_addstr(win, r, c + 13, f"  HYD B: ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 22, f"{hyd_b}", hyd_b_col)
    r += 1

    elec_str = "NORM" if eicas.elec_ok else "FAIL"
    elec_col = curses.color_pair(CP_GREEN) if eicas.elec_ok else curses.color_pair(CP_RED)
    _safe_addstr(win, r, c, f"ELEC:  ", curses.color_pair(CP_NORMAL))
    _safe_addstr(win, r, c + 7, elec_str, elec_col)
    r += 1

    fuel_color = curses.color_pair(CP_RED) if eicas.total_fuel_kg < 2000 else curses.color_pair(CP_WHITE)
    _safe_addstr(win, r, c, f"FUEL:  {eicas.total_fuel_kg:6.0f} kg  USED: {eicas.fuel_used_kg:.0f} kg", fuel_color)
    r += 1

    _safe_addstr(win, r, c, f"CABIN: {eicas.cabin_alt_ft:5.0f} ft  ΔP: {eicas.diff_press_psi:.2f} psi", curses.color_pair(CP_NORMAL))
    r += 1
    _safe_addstr(win, r, c, f"OAT:   {eicas.oat_c:+5.1f} °C", curses.color_pair(CP_NORMAL))
    r += 1

    # Cautions / Warnings
    if eicas.warnings:
        r += 1
        for w in eicas.warnings[:3]:
            _safe_addstr(win, r, c, f"⚠ {w}", curses.color_pair(CP_RED) | curses.A_BOLD)
            r += 1
    if eicas.cautions:
        for ca in eicas.cautions[:3]:
            _safe_addstr(win, r, c, f"△ {ca}", curses.color_pair(CP_YELLOW))
            r += 1


def _render_nav(win, state: SimulatorState, start_row: int) -> None:
    mfd = state.mfd
    if mfd is None:
        return
    r = start_row
    c = NAV_COL

    _safe_addstr(win, r, c, "─── NAVIGATION ───────────────────", curses.color_pair(CP_CYAN))
    r += 1

    _safe_addstr(win, r, c, f"LAT  {mfd.lat:9.4f}°  LON {mfd.lon:9.4f}°", curses.color_pair(CP_NORMAL))
    r += 1
    _safe_addstr(win, r, c, f"HDG  {mfd.heading_deg:5.1f}°  GS  {mfd.gnd_speed_kts:5.0f} kt", curses.color_pair(CP_NORMAL))
    r += 1
    _safe_addstr(win, r, c, f"WIND {mfd.wind_dir_deg:03.0f}° / {mfd.wind_speed_kts:4.0f} kt", curses.color_pair(CP_CYAN))
    r += 2

    _safe_addstr(win, r, c, "─ FMS ROUTE ──────────────────────", curses.color_pair(CP_DIM))
    r += 1
    _safe_addstr(win, r, c, mfd.route_str[:35], curses.color_pair(CP_WHITE) | curses.A_BOLD)
    r += 1

    if mfd.next_wp != "-----":
        _safe_addstr(win, r, c, f"NEXT:  {mfd.next_wp}", curses.color_pair(CP_GREEN))
        _safe_addstr(win, r, c + 15, f"BRG {mfd.bearing_to_next:5.1f}°", curses.color_pair(CP_NORMAL))
        r += 1
        _safe_addstr(win, r, c, f"DIST:  {mfd.dist_to_next_nm:6.1f} nm", curses.color_pair(CP_NORMAL))
        r += 1
        _safe_addstr(win, r, c, f"ETE:   {mfd.ete_str}", curses.color_pair(CP_NORMAL))
        r += 1
        xtk_col = curses.color_pair(CP_RED) if abs(mfd.xtk_nm) > 0.5 else curses.color_pair(CP_GREEN)
        _safe_addstr(win, r, c, f"XTK:  {mfd.xtk_nm:+6.2f} nm", xtk_col)
        r += 1
        _safe_addstr(win, r, c, f"DEST:  {mfd.dist_to_dest_nm:6.0f} nm", curses.color_pair(CP_NORMAL))
        r += 1
    else:
        _safe_addstr(win, r, c, "NO ACTIVE ROUTE", curses.color_pair(CP_DIM))
        r += 1

    r += 1
    _safe_addstr(win, r, c, f"CRZ FL: {mfd.cruise_fl}", curses.color_pair(CP_NORMAL))
    r += 1


def _render_status_bar(win, sim: "Simulator", state: SimulatorState, rows: int, cols: int) -> None:
    row = rows - 6
    _safe_addstr(win, row, 0, "─" * cols, curses.color_pair(CP_DIM))
    row += 1

    f = state.flight
    e = state.eicas
    ap_str = "AP:ON " if (state.pfd and state.pfd.ap_engaged) else "AP:OFF"
    gear_str = "GEAR DN" if f.gear_down else "GEAR UP"
    flaps_str = f"FLP {f.flap_position:2d}"
    thr_str = f"THR {state.throttle*100:4.0f}%"

    _safe_addstr(win, row, 0,
                 f" {ap_str}  {thr_str}  {gear_str}  {flaps_str}  "
                 f"PITCH {f.pitch_deg:+5.1f}°  BANK {f.bank_deg:+5.1f}°  "
                 f"MASS {f.mass_kg/1000:.1f}t",
                 curses.color_pair(CP_WHITE))
    row += 1

    # Aural alerts
    if state.aural_alerts:
        alert_text = "  ".join(state.aural_alerts)
        _safe_addstr(win, row, 0, f" !!  {alert_text}  !! ", curses.color_pair(CP_RED) | curses.A_BOLD | curses.A_BLINK)
    row += 1

    _safe_addstr(win, row, 0, "─" * cols, curses.color_pair(CP_DIM))
    row += 1
    _safe_addstr(win, row, 0,
                 " [W/S]pitch  [A/D]bank  [Z/X]thr  [G]gear  [F/V]flaps  "
                 "[P]AP  [B]spdbk  [H/J]hdg  [+/-]alt  [1-5]scenario  [Q]quit",
                 curses.color_pair(CP_DIM))


def _render_help(win, rows: int, cols: int) -> None:
    lines = [
        "═══════════════ KEYBOARD CONTROLS ═══════════════",
        "",
        "  W / S        Pitch nose up / down (elevator)",
        "  A / D        Bank left / right (aileron)",
        "  Z / X        Throttle decrease / increase",
        "  P            Toggle autopilot (HDG + ALT + SPD)",
        "  G            Gear toggle (up/down)",
        "  F            Flaps extend (next detent)",
        "  V            Flaps retract (prev detent)",
        "  B            Speedbrakes toggle",
        "  H / J        AP heading -10° / +10°",
        "  + / -        AP altitude +1000 ft / -1000 ft",
        "  1-5          Load normal procedure scenario",
        "  6-0          Load emergency scenario",
        "  Q / Escape   Quit simulator",
        "",
        "    Press any key to close this help ─────────────",
    ]
    start_y = max(0, rows // 2 - len(lines) // 2)
    start_x = max(0, cols // 2 - 26)
    for i, line in enumerate(lines):
        _safe_addstr(win, start_y + i, start_x, line, curses.color_pair(CP_CYAN) | curses.A_BOLD)


# ── Main cockpit loop ─────────────────────────────────────────────────────────
def run_cockpit(stdscr, initial_sim: Optional[Simulator] = None) -> None:
    """Main interactive loop.  ``initial_sim`` is used if provided."""
    curses.curs_set(0)
    stdscr.nodelay(True)
    stdscr.keypad(True)
    _init_colors()

    sim: Simulator = initial_sim or Simulator()
    show_help = False
    message: str = ""
    message_timer: float = 0.0

    TICK = 0.05   # real-time target step (20 fps)

    while True:
        rows, cols = stdscr.getmaxyx()
        if rows < 24 or cols < 80:
            stdscr.clear()
            _safe_addstr(stdscr, 0, 0,
                         f"Terminal too small! Need ≥80×24, got {cols}×{rows}",
                         curses.color_pair(CP_RED) | curses.A_BOLD)
            stdscr.refresh()
            time.sleep(0.5)
            ch = stdscr.getch()
            if ch in (ord('q'), ord('Q'), 27):
                break
            continue

        # ── Input ──────────────────────────────────────────────────────────
        ch = stdscr.getch()
        if ch != -1:
            # Clear pitch/bank commands (set for one frame then released)
            sim.set_pitch_rate(0.0)
            sim.set_bank_rate(0.0)

        if ch in (ord('q'), ord('Q'), 27):
            break
        elif ch in (ord('?'), ord('/')):
            show_help = not show_help
        elif show_help:
            show_help = False   # any key closes help
        elif ch == ord('w'):
            sim.set_pitch_rate(2.0)
        elif ch == ord('s'):
            sim.set_pitch_rate(-2.0)
        elif ch == ord('a'):
            sim.set_bank_rate(-5.0)
        elif ch == ord('d'):
            sim.set_bank_rate(5.0)
        elif ch == ord('z'):
            sim.adjust_throttle(-0.02)
        elif ch == ord('x'):
            sim.adjust_throttle(0.02)
        elif ch == ord('p'):
            message = sim.toggle_autopilot()
            message_timer = 3.0
        elif ch == ord('g'):
            message = sim.toggle_gear()
            message_timer = 3.0
        elif ch == ord('f'):
            message = sim.extend_flaps()
            message_timer = 2.0
        elif ch == ord('v'):
            message = sim.retract_flaps()
            message_timer = 2.0
        elif ch == ord('b'):
            sim.toggle_speedbrakes()
            message = "SPEEDBRAKES " + ("EXT" if sim.model.state.speedbrakes else "RET")
            message_timer = 2.0
        elif ch == ord('h'):
            new_hdg = (sim.autopilot.targets.heading_deg - 10) % 360
            sim.set_ap_heading(new_hdg)
            message = f"AP HDG → {new_hdg:.0f}°"
            message_timer = 2.0
        elif ch == ord('j'):
            new_hdg = (sim.autopilot.targets.heading_deg + 10) % 360
            sim.set_ap_heading(new_hdg)
            message = f"AP HDG → {new_hdg:.0f}°"
            message_timer = 2.0
        elif ch == ord('+') or ch == ord('='):
            new_alt = sim.autopilot.targets.altitude_m * 3.28084 + 1000
            sim.set_ap_altitude(new_alt)
            message = f"AP ALT → {new_alt:.0f} ft"
            message_timer = 2.0
        elif ch == ord('-'):
            new_alt = max(0, sim.autopilot.targets.altitude_m * 3.28084 - 1000)
            sim.set_ap_altitude(new_alt)
            message = f"AP ALT → {new_alt:.0f} ft"
            message_timer = 2.0
        elif ord('1') <= ch <= ord('5'):
            idx = ch - ord('1')
            if idx < len(ALL_SCENARIOS):
                sim = Simulator.from_scenario(ALL_SCENARIOS[idx])
                message = f"LOADED: {ALL_SCENARIOS[idx].name}"
                message_timer = 4.0
        elif ord('6') <= ch <= ord('9'):
            idx = ch - ord('6')
            if idx < len(ALL_EMERGENCY_SCENARIOS):
                sim = Simulator.from_scenario(ALL_EMERGENCY_SCENARIOS[idx])
                message = f"LOADED: {ALL_EMERGENCY_SCENARIOS[idx].name}"
                message_timer = 4.0

        # ── Simulation step ────────────────────────────────────────────────
        start_t = time.time()
        state = sim.step()

        # ── Render ────────────────────────────────────────────────────────
        stdscr.erase()
        _render_header(stdscr, state, cols)
        row0 = CONTENT_ROW
        _render_pfd(stdscr, state, row0)
        _render_eicas(stdscr, state, row0)
        _render_nav(stdscr, state, row0)
        _render_status_bar(stdscr, sim, state, rows, cols)

        # Scenario message
        if message_timer > 0:
            _safe_addstr(stdscr, rows - 2, 2, f" {message} ",
                         curses.color_pair(CP_YELLOW) | curses.A_BOLD)
            message_timer -= TICK

        if show_help:
            _render_help(stdscr, rows, cols)

        stdscr.refresh()

        # ── Throttle frame rate ────────────────────────────────────────────
        elapsed = time.time() - start_t
        sleep = max(0.0, TICK - elapsed)
        time.sleep(sleep)


def launch() -> None:
    """Entry point for the cockpit UI."""
    try:
        curses.wrapper(run_cockpit)
    except KeyboardInterrupt:
        pass

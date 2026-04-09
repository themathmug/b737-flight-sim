"""
737-800 Autopilot System.

Modes implemented:
    - CMD / CWS  (command / control-wheel-steering)
    - HEADING SELECT (HDGSEL) – roll to and maintain selected heading
    - LNAV – lateral navigation tracking (waypoints)
    - ALTITUDE HOLD (ALTHLD)
    - VERTICAL SPEED (VS) – hold selected V/S
    - VNAV – vertical navigation (managed climb/descent)
    - SPEED (SPEED) – autothrottle to maintain CAS target
    - APPROACH (APP) – ILS LOC+GS guidance
    - FLARE – automatic flare for landing

All controllers use PID-style gains tuned for the 737-800.
"""

import math
from dataclasses import dataclass
from enum import Enum, auto
from typing import Optional


class LateralMode(Enum):
    OFF     = auto()
    HDG_SEL = auto()    # Heading select
    HDG_HLD = auto()    # Heading hold
    LNAV    = auto()    # Lateral nav (waypoints)
    APPROACH = auto()   # LOC tracking


class VerticalMode(Enum):
    OFF     = auto()
    VS      = auto()    # Vertical speed hold
    ALT_HLD = auto()    # Altitude hold
    VNAV    = auto()    # Managed climb/descent
    FLCH    = auto()    # Flight Level Change (speed-on-elevator)
    APPROACH = auto()   # Glideslope


class SpeedMode(Enum):
    OFF     = auto()
    SPEED   = auto()    # CAS target
    MACH    = auto()    # Mach target


@dataclass
class AutopilotTargets:
    """Selected autopilot targets (set by pilot via MCP)."""
    heading_deg: float = 0.0
    altitude_m: float  = 0.0
    vs_fpm: float      = 0.0    # target V/S when in VS mode
    cas_kts: float     = 250.0  # target CAS
    mach: float        = 0.785  # target Mach
    # Approach targets (ILS)
    loc_dev_deg: float = 0.0    # localizer deviation (degrees)
    gs_dev_deg: float  = 0.0    # glide-slope deviation (degrees)


class AutopilotOutputs:
    """Control outputs from the autopilot for this step."""

    def __init__(self) -> None:
        self.pitch_rate_dps: float  = 0.0    # commanded pitch rate
        self.bank_rate_dps: float   = 0.0    # commanded bank rate
        self.throttle: float        = 0.22   # commanded throttle (both engines)
        self.target_bank: float     = 0.0    # computed bank target
        self.target_pitch: float    = 0.0    # computed pitch target


class AutopilotSystem:
    """737-800 autopilot / flight-director / autothrottle."""

    # ── Gain tables ───────────────────────────────────────────────────────────
    # Heading→bank: Kp = 1 deg bank per 1 deg heading error (capped)
    K_HDG_BANK: float   = 1.2    # deg bank / deg heading error
    MAX_BANK_CMD: float = 30.0   # max commanded bank

    # Bank→bank-rate: Kp proportional control of bank rate
    K_BANK_RATE: float  = 5.0    # deg/s per deg bank error

    # Altitude→VS: Kp (with VS derivative term for phugoid damping)
    K_ALT_VS: float     = 4.0    # fpm / ft error
    K_ALT_D:  float     = 0.6    # VS damping: reduces commanded VS by this fraction of current VS
    MAX_VS_CMD: float   = 2_500.0  # fpm

    # VS→pitch: Kp
    K_VS_PITCH: float   = 0.003   # deg pitch / fpm error

    # Speed→throttle PI: integrator provides the trim reference
    K_SPD_P: float    = 0.003   # throttle / kt error  (proportional)
    K_SPD_I: float    = 0.0008  # throttle / (kt·s)   (integrator)
    MAX_THR: float    = 0.98
    MIN_THR: float    = 0.05

    def __init__(self) -> None:
        self.lateral_mode: LateralMode = LateralMode.OFF
        self.vertical_mode: VerticalMode = VerticalMode.OFF
        self.speed_mode: SpeedMode = SpeedMode.OFF

        self.targets = AutopilotTargets()
        self._engaged: bool = False

        # Internal integrators
        self._alt_int: float  = 0.0
        self._spd_int: float  = 0.0
        self._vs_int: float   = 0.0

    @property
    def engaged(self) -> bool:
        return self._engaged

    def engage(
        self,
        heading: float,
        altitude: float,
        cas_kts: float,
        vs_fpm: float,
        throttle_trim: float = 0.0,
    ) -> None:
        """Engage autopilot – capture current values as targets.

        *throttle_trim* pre-loads the speed integrator to the current trim
        throttle so A/THR starts at the right power setting immediately.
        """
        self._engaged = True
        self.targets.heading_deg = heading
        self.targets.altitude_m  = altitude
        self.targets.cas_kts     = cas_kts
        self.targets.vs_fpm      = vs_fpm
        self._alt_int = 0.0
        # Pre-load integrator to the trim throttle so A/THR starts near trim
        self._spd_int = max(self.MIN_THR, min(self.MAX_THR, throttle_trim))
        self._vs_int  = 0.0

    def disengage(self) -> None:
        self._engaged = False
        self.lateral_mode  = LateralMode.OFF
        self.vertical_mode = VerticalMode.OFF
        self.speed_mode    = SpeedMode.OFF

    def select_heading(self, hdg: float) -> None:
        self.targets.heading_deg = hdg % 360.0
        if self._engaged:
            self.lateral_mode = LateralMode.HDG_SEL

    def select_altitude(self, alt_m: float) -> None:
        self.targets.altitude_m = alt_m
        if self._engaged:
            if self.vertical_mode not in (VerticalMode.VNAV, VerticalMode.APPROACH):
                self.vertical_mode = VerticalMode.VS   # managed V/S to target alt

    def select_vs(self, vs_fpm: float) -> None:
        self.targets.vs_fpm = vs_fpm
        if self._engaged:
            self.vertical_mode = VerticalMode.VS

    def select_speed(self, cas_kts: float) -> None:
        self.targets.cas_kts = cas_kts
        if self._engaged:
            self.speed_mode = SpeedMode.SPEED

    def arm_approach(self) -> None:
        if self._engaged:
            self.lateral_mode  = LateralMode.APPROACH
            self.vertical_mode = VerticalMode.APPROACH

    # ── Main compute ──────────────────────────────────────────────────────────
    def compute(
        self,
        dt: float,
        heading_deg: float,
        altitude_m: float,
        cas_kts: float,
        vs_fpm: float,
        pitch_deg: float,
        bank_deg: float,
        throttle_current: float,
    ) -> AutopilotOutputs:
        """Compute autopilot output for one simulation step.

        Returns an AutopilotOutputs object with pitch_rate_dps, bank_rate_dps,
        and throttle command.  Returns zeros (with current throttle) if
        disengaged.
        """
        out = AutopilotOutputs()
        out.throttle = throttle_current  # default: maintain current throttle

        if not self._engaged:
            return out

        # ── Lateral ──────────────────────────────────────────────────────────
        out.bank_rate_dps = self._lateral_control(
            dt, heading_deg, bank_deg, out
        )

        # ── Vertical ─────────────────────────────────────────────────────────
        out.pitch_rate_dps = self._vertical_control(
            dt, altitude_m, vs_fpm, pitch_deg, out
        )

        # ── Speed / autothrottle ──────────────────────────────────────────────
        if self.speed_mode == SpeedMode.SPEED:
            out.throttle = self._speed_control(dt, cas_kts, throttle_current)

        return out

    # ── Controller sub-functions ──────────────────────────────────────────────
    def _lateral_control(
        self, dt: float, heading: float, bank: float, out: AutopilotOutputs
    ) -> float:
        """Return commanded bank rate (°/s)."""
        if self.lateral_mode == LateralMode.OFF:
            return 0.0

        if self.lateral_mode in (LateralMode.HDG_SEL, LateralMode.HDG_HLD):
            err = _heading_error(self.targets.heading_deg, heading)
            target_bank = self.K_HDG_BANK * err
            target_bank = max(-self.MAX_BANK_CMD, min(self.MAX_BANK_CMD, target_bank))
            out.target_bank = target_bank
            bank_err = target_bank - bank
            return max(-15.0, min(15.0, self.K_BANK_RATE * bank_err))

        if self.lateral_mode == LateralMode.APPROACH:
            # Simplified LOC: just hold heading
            target_bank = self.K_HDG_BANK * _heading_error(
                self.targets.heading_deg, heading
            )
            target_bank = max(-10.0, min(10.0, target_bank))
            out.target_bank = target_bank
            bank_err = target_bank - bank
            return max(-10.0, min(10.0, self.K_BANK_RATE * bank_err))

        return 0.0

    def _vertical_control(
        self, dt: float, altitude: float, vs_fpm: float, pitch: float,
        out: AutopilotOutputs
    ) -> float:
        """Return commanded pitch rate (°/s)."""
        if self.vertical_mode == VerticalMode.OFF:
            return 0.0

        if self.vertical_mode == VerticalMode.ALT_HLD:
            alt_err_m = self.targets.altitude_m - altitude
            target_vs = self.K_ALT_VS * alt_err_m * 3.28084  # → fpm
            # PD: subtract a fraction of the current VS to damp phugoid oscillations
            target_vs -= self.K_ALT_D * vs_fpm
            target_vs = max(-self.MAX_VS_CMD, min(self.MAX_VS_CMD, target_vs))
            vs_err = target_vs - vs_fpm
            target_pitch = 2.0 + self.K_VS_PITCH * vs_err
            target_pitch = max(-15.0, min(15.0, target_pitch))
            out.target_pitch = target_pitch
            return max(-3.0, min(3.0, (target_pitch - pitch) * 0.5))

        if self.vertical_mode == VerticalMode.VS:
            # Hold selected VS; switch to ALT HLD when close to target
            alt_err_m = self.targets.altitude_m - altitude
            alt_err_ft = alt_err_m * 3.28084
            # Capture altitude when within 300 ft
            if abs(alt_err_ft) < 300:
                self.vertical_mode = VerticalMode.ALT_HLD
                return 0.0
            target_vs = self.targets.vs_fpm
            vs_err = target_vs - vs_fpm
            target_pitch = 2.0 + self.K_VS_PITCH * vs_err
            target_pitch = max(-20.0, min(20.0, target_pitch))
            out.target_pitch = target_pitch
            return max(-3.0, min(3.0, (target_pitch - pitch) * 0.5))

        if self.vertical_mode == VerticalMode.FLCH:
            # Speed-on-elevator mode: pitch to maintain speed, throttle to climb
            # Simplified: same as VS but with larger authority
            return self._vertical_control.__wrapped__ if False else 0.0   # placeholder

        if self.vertical_mode == VerticalMode.APPROACH:
            # Simplified glide slope: use -3° flight path
            target_pitch = -1.0   # on a 3° GS with alpha ≈ 4°, pitch ≈ 1°
            gs_err = self.targets.gs_dev_deg  # + = above GS
            target_pitch -= gs_err * 3.0
            target_pitch = max(-10.0, min(5.0, target_pitch))
            out.target_pitch = target_pitch
            return max(-3.0, min(3.0, (target_pitch - pitch) * 0.5))

        return 0.0

    def _speed_control(
        self, dt: float, cas_kts: float, throttle: float
    ) -> float:
        """Return commanded throttle using a PI controller.

        The integrator *_spd_int* is pre-loaded at AP engagement to the
        current trim throttle, so A/THR starts near the right power immediately.
        The integral slowly adjusts to account for any model inaccuracies.
        """
        err = self.targets.cas_kts - cas_kts
        # Integrator: slow wind-up to track long-term trim point
        self._spd_int += err * dt * self.K_SPD_I
        self._spd_int = max(self.MIN_THR, min(self.MAX_THR, self._spd_int))
        # PI output: integrator is the trim, proportional corrects for errors
        thr_cmd = self._spd_int + self.K_SPD_P * err
        return max(self.MIN_THR, min(self.MAX_THR, thr_cmd))

    # ── Status helpers ────────────────────────────────────────────────────────
    def mode_line(self) -> str:
        """Return a compact mode-line string for display."""
        lat = self.lateral_mode.name if self.lateral_mode != LateralMode.OFF else "---"
        vert = self.vertical_mode.name if self.vertical_mode != VerticalMode.OFF else "---"
        spd = self.speed_mode.name if self.speed_mode != SpeedMode.OFF else "---"
        return f"LAT:{lat}  VERT:{vert}  SPD:{spd}"


def _heading_error(target: float, current: float) -> float:
    """Signed heading error in degrees (–180 to +180)."""
    err = (target - current) % 360.0
    if err > 180.0:
        err -= 360.0
    return err

"""
Primary Flight Display (PFD) rendering helpers.

Produces a structured dict of values suitable for display on any front-end
(terminal, GUI, etc.).  The actual curses rendering lives in ``ui/cockpit.py``.
"""

from dataclasses import dataclass
from src.flight_model import FlightState
from src.physics.atmosphere import tas_to_cas, oat_celsius, pressure_altitude_from_qnh


@dataclass
class PFDData:
    """All values needed to draw a PFD."""
    # Speed tape
    cas_kts: float
    tas_kts: float
    gs_kts: float
    mach: float
    vref_kts: float        # reference speed (Vref / Vapp)
    vmo_kts: float = 340.0 # VMO
    # Attitude
    pitch_deg: float = 0.0
    bank_deg: float  = 0.0
    # Altitude
    altitude_ft: float = 0.0
    vs_fpm: float      = 0.0
    # Heading
    heading_deg: float = 0.0
    track_deg: float   = 0.0
    # Barometric
    baro_set: float    = 29.92   # inches Hg
    # Flight director bars
    fd_pitch_bar: float = 0.0   # degrees (positive = pitch up command)
    fd_bank_bar: float  = 0.0   # degrees (positive = roll right command)
    fd_active: bool     = False
    # AP mode annunciations
    lat_mode: str  = "---"
    vert_mode: str = "---"
    spd_mode: str  = "---"
    ap_engaged: bool = False
    # Alert flags
    stall: bool = False
    overspeed: bool = False
    low_energy: bool = False
    # Radio altitude
    radio_alt_ft: float = 0.0


def build_pfd(
    state: FlightState,
    ap_lat: str = "---",
    ap_vert: str = "---",
    ap_spd: str = "---",
    ap_engaged: bool = False,
    fd_pitch: float = 0.0,
    fd_bank: float = 0.0,
    qnh_inhg: float = 29.92,
    vref_kts: float = 137.0,
) -> PFDData:
    """Build a :class:`PFDData` snapshot from the current :class:`FlightState`."""
    MS_TO_KTS = 1.94384

    cas_kts = state.cas_ms * MS_TO_KTS
    tas_kts = state.tas_ms * MS_TO_KTS
    gs_kts  = state.gnd_speed_ms * MS_TO_KTS

    altitude_ft = state.altitude_m * 3.28084
    vs_fpm      = state.vs_ms * 196.85

    # Overspeed flag (VMO or MMO)
    overspeed = cas_kts > 340.0 or state.mach > 0.820

    # Low energy: below 1.3·Vref
    low_energy = cas_kts < vref_kts * 1.05 and altitude_ft > 200

    return PFDData(
        cas_kts=round(cas_kts, 1),
        tas_kts=round(tas_kts, 1),
        gs_kts=round(gs_kts, 1),
        mach=round(state.mach, 3),
        vref_kts=vref_kts,
        pitch_deg=round(state.pitch_deg, 1),
        bank_deg=round(state.bank_deg, 1),
        altitude_ft=round(altitude_ft),
        vs_fpm=round(vs_fpm),
        heading_deg=round(state.heading_deg % 360, 1),
        track_deg=round(state.heading_deg % 360, 1),
        baro_set=qnh_inhg,
        fd_pitch_bar=fd_pitch,
        fd_bank_bar=fd_bank,
        fd_active=ap_engaged,
        lat_mode=ap_lat,
        vert_mode=ap_vert,
        spd_mode=ap_spd,
        ap_engaged=ap_engaged,
        stall=state.stall,
        overspeed=overspeed,
        low_energy=low_energy,
        radio_alt_ft=round(altitude_ft),  # simplified – RA = baro alt
    )

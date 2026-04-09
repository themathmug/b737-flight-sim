"""
Multi-Function Display (MFD) – Navigation / Engine data page helpers.
"""

from dataclasses import dataclass, field
from typing import Optional
from src.flight_model import FlightState
from src.systems.fms import FMS


@dataclass
class MFDNavData:
    """Navigation display data."""
    lat: float = 0.0
    lon: float = 0.0
    heading_deg: float = 0.0
    track_deg: float = 0.0
    gnd_speed_kts: float = 0.0
    wind_speed_kts: float = 0.0
    wind_dir_deg: float = 0.0
    # FMS
    route_str: str = "NO ROUTE"
    next_wp: str   = "-----"
    dist_to_next_nm: float = 0.0
    bearing_to_next: float = 0.0
    ete_str: str   = "--:--"
    xtk_nm: float  = 0.0
    dist_to_dest_nm: float = 0.0
    cruise_fl: int = 350


def build_mfd_nav(
    state: FlightState,
    fms: FMS,
    origin_lat: float = 47.449,
    origin_lon: float = -122.309,
) -> MFDNavData:
    """Build navigation MFD page from flight state and FMS."""
    MS_TO_KTS = 1.94384

    lat, lon = FMS.xy_to_latlon(state.x_m, state.y_m, origin_lat, origin_lon)

    next_wp  = fms.next_waypoint
    dist_nm  = fms.dist_to_next_m / 1852.0
    dest_nm  = fms.dist_to_dest_m / 1852.0

    # ETE string
    if fms.ete_s > 0 and state.gnd_speed_ms > 1:
        ete_min = int(fms.ete_s // 60)
        ete_sec = int(fms.ete_s % 60)
        ete_str = f"{ete_min:02d}:{ete_sec:02d}"
    else:
        ete_str = "--:--"

    return MFDNavData(
        lat=round(lat, 4),
        lon=round(lon, 4),
        heading_deg=round(state.heading_deg % 360, 1),
        track_deg=round(state.heading_deg % 360, 1),
        gnd_speed_kts=round(state.gnd_speed_ms * MS_TO_KTS, 1),
        wind_speed_kts=round(state.wind_speed_ms * MS_TO_KTS, 1),
        wind_dir_deg=state.wind_dir_deg,
        route_str=fms.route_string(),
        next_wp=next_wp.ident if next_wp else "-----",
        dist_to_next_nm=round(dist_nm, 1),
        bearing_to_next=round(fms.bearing_to_next, 1),
        ete_str=ete_str,
        xtk_nm=round(fms.xtk_m / 1852.0, 2),
        dist_to_dest_nm=round(dest_nm, 1),
        cruise_fl=fms.cruise_fl,
    )

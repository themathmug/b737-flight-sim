"""
Flight Management System (FMS) for Boeing 737-800.

Provides:
  - Waypoint database (a small built-in set of ICAOs + lat/lon)
  - Route management (origin → destination with intermediate waypoints)
  - Performance calculations (fuel required, TOD, etc.)
  - LNAV steering commands (cross-track error, bearing-to-waypoint)

Navigation uses the Haversine formula for great-circle distances on a
spherical Earth (R = 6371 km).  Bearings are magnetic-heading approximations
(no magnetic variation correction for simplicity).
"""

import math
from dataclasses import dataclass, field
from typing import Optional


EARTH_RADIUS_M: float = 6_371_000.0   # metres


# ── Waypoint ──────────────────────────────────────────────────────────────────
@dataclass
class Waypoint:
    ident: str
    lat: float   # decimal degrees, +N
    lon: float   # decimal degrees, +E
    altitude_constraint_m: Optional[float] = None  # VNAV constraint
    speed_constraint_kts: Optional[float]  = None


# ── Small built-in database (CONUS + a few international) ────────────────────
WAYPOINT_DB: dict[str, Waypoint] = {
    # Airports (runway threshold approximation)
    "KSEA": Waypoint("KSEA",  47.449,  -122.309),
    "KLAX": Waypoint("KLAX",  33.943,  -118.408),
    "KJFK": Waypoint("KJFK",  40.640,   -73.779),
    "KORD": Waypoint("KORD",  41.979,   -87.905),
    "KDFW": Waypoint("KDFW",  32.898,   -97.038),
    "KATL": Waypoint("KATL",  33.641,   -84.427),
    "KDEN": Waypoint("KDEN",  39.856,  -104.674),
    "KSFO": Waypoint("KSFO",  37.619,  -122.375),
    "KBOS": Waypoint("KBOS",  42.365,   -71.010),
    "KMIA": Waypoint("KMIA",  25.796,   -80.287),
    "EGLL": Waypoint("EGLL",  51.477,    -0.461),  # Heathrow
    "EDDF": Waypoint("EDDF",  50.037,     8.571),  # Frankfurt
    "RJTT": Waypoint("RJTT",  35.553,   139.781),  # Tokyo Haneda
    "YSSY": Waypoint("YSSY", -33.947,   151.177),  # Sydney
    # Enroute fixes
    "BEAVR": Waypoint("BEAVR",  47.735,  -122.657),
    "SUMMA": Waypoint("SUMMA",  45.943,  -121.133),
    "OAL":   Waypoint("OAL",    44.124,  -117.834),
    "BOI":   Waypoint("BOI",    43.564,  -116.223),
    "BURHL": Waypoint("BURHL",  42.190,  -112.478),
    "SLC":   Waypoint("SLC",    40.789,  -111.978),
    "ELORE": Waypoint("ELORE",  40.123,  -104.345),
    "DRABS": Waypoint("DRABS",  41.456,   -95.678),
    "KUBBS": Waypoint("KUBBS",  41.856,   -88.455),
}


def haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Great-circle distance in metres between two lat/lon points."""
    r = EARTH_RADIUS_M
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlam = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlam / 2) ** 2
    return 2 * r * math.asin(math.sqrt(a))


def bearing_deg(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Initial bearing (degrees) from point 1 to point 2."""
    phi1, phi2 = math.radians(lat1), math.radians(lat2)
    dlam = math.radians(lon2 - lon1)
    y = math.sin(dlam) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(dlam)
    return math.degrees(math.atan2(y, x)) % 360.0


@dataclass
class RouteSegment:
    from_wp: Waypoint
    to_wp: Waypoint
    distance_m: float = field(init=False)
    initial_bearing: float = field(init=False)

    def __post_init__(self) -> None:
        self.distance_m = haversine_m(
            self.from_wp.lat, self.from_wp.lon,
            self.to_wp.lat,   self.to_wp.lon,
        )
        self.initial_bearing = bearing_deg(
            self.from_wp.lat, self.from_wp.lon,
            self.to_wp.lat,   self.to_wp.lon,
        )


class FMS:
    """Flight Management System core."""

    # Waypoint capture radius (switch to next leg when this close)
    WP_CAPTURE_M: float = 5_000.0   # 5 km

    def __init__(self) -> None:
        self.origin: Optional[Waypoint]      = None
        self.destination: Optional[Waypoint] = None
        self.route: list[Waypoint]            = []    # full ordered list
        self.segments: list[RouteSegment]     = []
        self._leg_idx: int                    = 0

        # FMS perf page
        self.cruise_fl: int      = 350      # cruise flight level
        self.cost_index: int     = 45       # cost index
        self.reserve_kg: float   = 2_000.0  # fuel reserve
        self.zfw_kg: float       = 56_413.0 # zero-fuel weight

        # Active navigation
        self.active: bool        = False
        self.xtk_m: float        = 0.0      # cross-track error, metres
        self.dtk_deg: float      = 0.0      # desired track, degrees
        self.bearing_to_next: float = 0.0
        self.dist_to_next_m: float  = 0.0
        self.ete_s: float            = 0.0  # estimated time en-route to dest, sec

    # ── Route building ────────────────────────────────────────────────────────
    def set_route(self, idents: list[str]) -> list[str]:
        """Build route from list of ICAO/fix identifiers.

        Returns a list of unresolved identifiers (if any).
        """
        resolved = []
        unresolved = []
        for ident in idents:
            wp = WAYPOINT_DB.get(ident.upper())
            if wp:
                resolved.append(wp)
            else:
                unresolved.append(ident)

        if len(resolved) < 2:
            return unresolved

        self.route = resolved
        self.origin      = resolved[0]
        self.destination = resolved[-1]
        self._build_segments()
        self._leg_idx = 0
        self.active = False
        return unresolved

    def activate(self) -> None:
        """Activate the route for LNAV guidance."""
        if len(self.route) >= 2:
            self.active = True
            self._leg_idx = 0

    def _build_segments(self) -> None:
        self.segments = [
            RouteSegment(self.route[i], self.route[i + 1])
            for i in range(len(self.route) - 1)
        ]

    # ── Performance calculations ──────────────────────────────────────────────
    def total_distance_m(self) -> float:
        return sum(s.distance_m for s in self.segments)

    def fuel_required_kg(
        self,
        cruise_ff_kgh: float = 5_500.0,   # total at cruise
        tas_ms: float = 230.0,             # cruise TAS
    ) -> float:
        """Estimated fuel required for the route (simplistic)."""
        dist_m = self.total_distance_m()
        if tas_ms <= 0:
            return 0.0
        trip_h = dist_m / tas_ms / 3600.0
        return cruise_ff_kgh * trip_h + self.reserve_kg

    def top_of_descent_m(
        self, cruise_alt_m: float, target_alt_m: float = 0.0
    ) -> float:
        """Distance from destination to start descent at 3°."""
        delta_alt = cruise_alt_m - target_alt_m
        # 3° descent: for every 1 000 ft descent ≈ 3 nm horizontal
        dist_nm = (delta_alt * 3.28084 / 1000.0) * 3.0
        return dist_nm * 1852.0

    # ── Update (called each sim step with aircraft position) ──────────────────
    def update(
        self,
        lat: float,
        lon: float,
        gnd_speed_ms: float,
        heading_deg: float,
    ) -> None:
        """Update LNAV guidance based on current aircraft position."""
        if not self.active or self._leg_idx >= len(self.segments):
            return

        seg = self.segments[self._leg_idx]
        # Distance and bearing to next waypoint
        self.dist_to_next_m  = haversine_m(lat, lon, seg.to_wp.lat, seg.to_wp.lon)
        self.bearing_to_next = bearing_deg(lat, lon, seg.to_wp.lat, seg.to_wp.lon)
        self.dtk_deg         = seg.initial_bearing

        # Cross-track error (signed, right-of-track positive)
        dist_from   = haversine_m(lat, lon, seg.from_wp.lat, seg.from_wp.lon)
        angle_rad   = math.radians(self.bearing_to_next - self.dtk_deg)
        self.xtk_m  = dist_from * math.sin(angle_rad)

        # ETE to destination
        remaining_m = sum(
            self.segments[i].distance_m
            for i in range(self._leg_idx + 1, len(self.segments))
        ) + self.dist_to_next_m
        self.ete_s = remaining_m / gnd_speed_ms if gnd_speed_ms > 1.0 else 0.0

        # Waypoint sequencing
        if self.dist_to_next_m < self.WP_CAPTURE_M:
            self._leg_idx += 1
            if self._leg_idx >= len(self.segments):
                self.active = False  # arrived

    # ── LNAV heading command ──────────────────────────────────────────────────
    def lnav_heading_cmd(self) -> float:
        """Return the desired heading for LNAV (degrees).

        Uses a proportional cross-track correction on top of desired track.
        """
        if not self.active:
            return 0.0
        xtk_nm = self.xtk_m / 1852.0
        # Intercept angle proportional to XTK error (max 30°)
        intercept = max(-30.0, min(30.0, xtk_nm * -5.0))
        return (self.dtk_deg + intercept) % 360.0

    # ── Properties ────────────────────────────────────────────────────────────
    @property
    def current_leg(self) -> Optional[RouteSegment]:
        if 0 <= self._leg_idx < len(self.segments):
            return self.segments[self._leg_idx]
        return None

    @property
    def next_waypoint(self) -> Optional[Waypoint]:
        seg = self.current_leg
        return seg.to_wp if seg else None

    @property
    def dist_to_dest_m(self) -> float:
        if not self.segments:
            return 0.0
        return sum(
            s.distance_m for s in self.segments[self._leg_idx:]
        ) if self._leg_idx < len(self.segments) else 0.0

    def route_string(self) -> str:
        if not self.route:
            return "NO ROUTE"
        return " → ".join(wp.ident for wp in self.route)

    # ── Lat/lon from simulation x/y ──────────────────────────────────────────
    @staticmethod
    def xy_to_latlon(
        x_m: float, y_m: float, origin_lat: float, origin_lon: float
    ) -> tuple[float, float]:
        """Convert local flat-earth (x=E, y=N) metres to lat/lon."""
        delta_lat = math.degrees(y_m / EARTH_RADIUS_M)
        delta_lon = math.degrees(
            x_m / (EARTH_RADIUS_M * math.cos(math.radians(origin_lat)))
        )
        return origin_lat + delta_lat, origin_lon + delta_lon

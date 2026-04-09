"""
Tests for the FMS (Flight Management System).
"""

import math
import pytest
from src.systems.fms import (
    FMS, Waypoint, WAYPOINT_DB, haversine_m, bearing_deg, RouteSegment
)


class TestHaversine:
    def test_zero_distance(self):
        assert haversine_m(47.0, -122.0, 47.0, -122.0) == pytest.approx(0.0, abs=1.0)

    def test_known_distance(self):
        # Seattle to JFK ≈ 3 900 km
        dist = haversine_m(47.449, -122.309, 40.640, -73.779)
        assert 3_700_000 < dist < 4_100_000

    def test_positive_distance(self):
        dist = haversine_m(0, 0, 1, 1)
        assert dist > 0


class TestBearing:
    def test_north(self):
        # Point 2 is due north of point 1
        b = bearing_deg(0, 0, 10, 0)
        assert b == pytest.approx(0.0, abs=1.0)

    def test_east(self):
        b = bearing_deg(0, 0, 0, 10)
        assert b == pytest.approx(90.0, abs=1.0)

    def test_south(self):
        b = bearing_deg(10, 0, 0, 0)
        assert b == pytest.approx(180.0, abs=1.0)


class TestFMSRouteBuilding:
    def setup_method(self):
        self.fms = FMS()

    def test_set_route_valid(self):
        unresolved = self.fms.set_route(["KSEA", "KORD", "KJFK"])
        assert unresolved == []
        assert len(self.fms.route) == 3

    def test_set_route_unknown_waypoint(self):
        unresolved = self.fms.set_route(["KSEA", "XXXX", "KJFK"])
        assert "XXXX" in unresolved

    def test_segments_built(self):
        self.fms.set_route(["KSEA", "KORD", "KJFK"])
        assert len(self.fms.segments) == 2

    def test_total_distance_positive(self):
        self.fms.set_route(["KSEA", "KJFK"])
        assert self.fms.total_distance_m() > 0

    def test_route_string(self):
        self.fms.set_route(["KSEA", "KORD", "KJFK"])
        rs = self.fms.route_string()
        assert "KSEA" in rs
        assert "KJFK" in rs

    def test_no_route_string(self):
        assert self.fms.route_string() == "NO ROUTE"


class TestFMSNavigation:
    def setup_method(self):
        self.fms = FMS()
        self.fms.set_route(["KSEA", "KORD", "KJFK"])
        self.fms.activate()

    def test_active_after_activate(self):
        assert self.fms.active

    def test_next_waypoint_is_kord(self):
        assert self.fms.next_waypoint is not None
        assert self.fms.next_waypoint.ident == "KORD"

    def test_update_from_origin(self):
        # Aircraft at KSEA position
        self.fms.update(47.449, -122.309, 250, 95.0)
        assert self.fms.dist_to_next_m > 0

    def test_waypoint_sequencing(self):
        """When aircraft arrives at KORD, should sequence to KJFK."""
        kord = WAYPOINT_DB["KORD"]
        self.fms.update(kord.lat, kord.lon, 250, 95.0)
        # Should have sequenced past KORD
        # dist_to_next should now reference leg to KJFK or route finished
        assert self.fms.next_waypoint is None or self.fms.next_waypoint.ident == "KJFK"


class TestFMSPerformance:
    def setup_method(self):
        self.fms = FMS()
        self.fms.set_route(["KSEA", "KJFK"])

    def test_fuel_required_positive(self):
        fuel = self.fms.fuel_required_kg()
        assert fuel > 0

    def test_tod_positive(self):
        tod = self.fms.top_of_descent_m(10668, 0)
        assert tod > 0

    def test_xy_to_latlon_round_trip(self):
        origin_lat, origin_lon = 47.449, -122.309
        lat, lon = FMS.xy_to_latlon(0, 0, origin_lat, origin_lon)
        assert lat == pytest.approx(origin_lat, abs=0.001)
        assert lon == pytest.approx(origin_lon, abs=0.001)

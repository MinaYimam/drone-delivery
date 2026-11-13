import math

import pytest

from drone_delivery import NoFlyZone, Point, RouteNotFound, RoutePlanner
from drone_delivery.geo import path_length, point_segment_distance


def test_distance_and_towards():
    a, b = Point(0, 0), Point(3, 4)
    assert a.distance_to(b) == 5
    assert a.towards(b, 2.5) == Point(1.5, 2.0)
    assert a.towards(b, 99) == b  # never overshoots


def test_point_segment_distance():
    assert point_segment_distance(Point(0, 1), Point(-1, 0), Point(1, 0)) == pytest.approx(1)
    assert point_segment_distance(Point(5, 0), Point(-1, 0), Point(1, 0)) == pytest.approx(4)


def test_zone_intersection():
    zone = NoFlyZone(Point(0, 0), 1)
    assert zone.intersects_segment(Point(-2, 0), Point(2, 0))
    assert not zone.intersects_segment(Point(-2, 1.5), Point(2, 1.5))
    assert zone.contains(Point(0.5, 0))
    assert not zone.contains(Point(1.5, 0))


def test_direct_route_when_clear():
    planner = RoutePlanner([NoFlyZone(Point(10, 10), 1)])
    assert planner.plan(Point(0, 0), Point(5, 0)) == [Point(0, 0), Point(5, 0)]


def test_route_detours_around_zone():
    zone = NoFlyZone(Point(5, 0), 1.5)
    planner = RoutePlanner([zone])
    route = planner.plan(Point(0, 0), Point(10, 0))
    assert len(route) > 2
    for a, b in zip(route, route[1:]):
        assert not zone.intersects_segment(a, b)
    # Detour should be longer than straight line but not absurdly so.
    assert 10 < path_length(route) < 10 + 2 * math.pi * 1.5


def test_route_through_overlapping_zones():
    zones = [NoFlyZone(Point(5, 0), 1.5), NoFlyZone(Point(5, 2), 1.5)]
    planner = RoutePlanner(zones)
    route = planner.plan(Point(0, 1), Point(10, 1))
    for a, b in zip(route, route[1:]):
        assert planner.is_clear(a, b)


def test_endpoint_inside_zone_raises():
    planner = RoutePlanner([NoFlyZone(Point(0, 0), 2)])
    with pytest.raises(RouteNotFound):
        planner.plan(Point(0, 0), Point(5, 5))


def test_enclosed_goal_unreachable():
    # A goal completely ringed by zones cannot be reached.
    ring = [NoFlyZone(Point(math.cos(a) * 3, math.sin(a) * 3), 1.6) for a in [i * math.pi / 4 for i in range(8)]]
    planner = RoutePlanner(ring)
    with pytest.raises(RouteNotFound):
        planner.plan(Point(0, 0), Point(10, 10))

"""Route planning around no-fly zones.

Uses a visibility graph: every no-fly zone is wrapped in a polygon whose
edges stay safely outside the circle. Nodes are the start, the goal and
all polygon vertices; edges are straight segments that don't cross any
zone. Dijkstra over that graph gives the shortest safe path.
"""

from __future__ import annotations

import heapq
import math
from typing import Sequence

from .geo import NoFlyZone, Point, path_length


class RouteNotFound(Exception):
    """Raised when no safe path exists between two points."""


class RoutePlanner:
    def __init__(self, zones: Sequence[NoFlyZone] = (), margin: float = 0.15, sides: int = 12):
        if sides < 3:
            raise ValueError("sides must be >= 3")
        self.zones = tuple(zones)
        self.margin = margin
        self.sides = sides
        self._waypoints = self._build_waypoints()
        self._cache: dict[tuple[Point, Point], list[Point]] = {}

    # ------------------------------------------------------------------ setup
    def _build_waypoints(self) -> list[Point]:
        """Polygon vertices around each zone, dropping any inside another zone."""
        points: list[Point] = []
        # Circumradius such that each polygon edge clears radius*(1+margin).
        factor = (1 + self.margin) / math.cos(math.pi / self.sides)
        for zone in self.zones:
            r = zone.radius * factor
            for i in range(self.sides):
                angle = 2 * math.pi * i / self.sides
                p = Point(zone.center.x + r * math.cos(angle), zone.center.y + r * math.sin(angle))
                if not self.is_blocked(p):
                    points.append(p)
        return points

    # ---------------------------------------------------------------- queries
    def is_blocked(self, p: Point) -> bool:
        return any(z.contains(p) for z in self.zones)

    def is_clear(self, a: Point, b: Point) -> bool:
        return not any(z.intersects_segment(a, b) for z in self.zones)

    def plan(self, start: Point, goal: Point) -> list[Point]:
        """Shortest safe path from start to goal, inclusive of both endpoints."""
        key = (start, goal)
        if key in self._cache:
            return list(self._cache[key])

        for label, p in (("start", start), ("goal", goal)):
            if self.is_blocked(p):
                raise RouteNotFound(f"{label} {p} is inside a no-fly zone")

        if self.is_clear(start, goal):
            route = [start, goal]
        else:
            route = self._dijkstra(start, goal)

        self._cache[key] = route
        return list(route)

    def distance(self, start: Point, goal: Point) -> float:
        return path_length(self.plan(start, goal))

    # --------------------------------------------------------------- internals
    def _dijkstra(self, start: Point, goal: Point) -> list[Point]:
        nodes = [start, goal, *self._waypoints]
        goal_idx = 1
        dist = {0: 0.0}
        prev: dict[int, int] = {}
        heap = [(0.0, 0)]
        visited: set[int] = set()

        while heap:
            d, u = heapq.heappop(heap)
            if u in visited:
                continue
            visited.add(u)
            if u == goal_idx:
                break
            for v in range(len(nodes)):
                if v in visited or v == u:
                    continue
                if not self.is_clear(nodes[u], nodes[v]):
                    continue
                nd = d + nodes[u].distance_to(nodes[v])
                if nd < dist.get(v, math.inf):
                    dist[v] = nd
                    prev[v] = u
                    heapq.heappush(heap, (nd, v))

        if goal_idx not in dist:
            raise RouteNotFound(f"no safe path from {start} to {goal}")

        path = [goal_idx]
        while path[-1] != 0:
            path.append(prev[path[-1]])
        return [nodes[i] for i in reversed(path)]

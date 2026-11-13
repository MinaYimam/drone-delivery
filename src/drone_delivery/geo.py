"""Geometry primitives.

All coordinates are on a flat 2D plane measured in kilometres. For a
city-scale service area (a few tens of km across) this is accurate enough
and keeps the maths simple and fast.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Iterable, Sequence

EPSILON = 1e-9


@dataclass(frozen=True)
class Point:
    x: float
    y: float

    def distance_to(self, other: "Point") -> float:
        return math.hypot(self.x - other.x, self.y - other.y)

    def towards(self, other: "Point", step: float) -> "Point":
        """Return the point `step` km from self in the direction of `other`.

        Never overshoots: if `other` is closer than `step`, returns `other`.
        """
        d = self.distance_to(other)
        if d <= step or d < EPSILON:
            return other
        t = step / d
        return Point(self.x + (other.x - self.x) * t, self.y + (other.y - self.y) * t)

    def to_dict(self) -> dict:
        return {"x": round(self.x, 4), "y": round(self.y, 4)}

    @classmethod
    def from_dict(cls, data: dict | Sequence[float]) -> "Point":
        if isinstance(data, dict):
            return cls(float(data["x"]), float(data["y"]))
        return cls(float(data[0]), float(data[1]))


@dataclass(frozen=True)
class NoFlyZone:
    """A circular restricted area (airport, stadium, hospital helipad...)."""

    center: Point
    radius: float
    name: str = ""

    def contains(self, p: Point) -> bool:
        return p.distance_to(self.center) < self.radius - EPSILON

    def intersects_segment(self, a: Point, b: Point) -> bool:
        """True if the straight segment a->b passes through the zone interior."""
        return point_segment_distance(self.center, a, b) < self.radius - EPSILON

    def to_dict(self) -> dict:
        return {"name": self.name, "center": self.center.to_dict(), "radius": self.radius}

    @classmethod
    def from_dict(cls, data: dict) -> "NoFlyZone":
        return cls(Point.from_dict(data["center"]), float(data["radius"]), data.get("name", ""))


def point_segment_distance(p: Point, a: Point, b: Point) -> float:
    """Shortest distance from point p to the segment a-b."""
    dx, dy = b.x - a.x, b.y - a.y
    length_sq = dx * dx + dy * dy
    if length_sq < EPSILON:
        return p.distance_to(a)
    t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / length_sq
    t = max(0.0, min(1.0, t))
    return p.distance_to(Point(a.x + t * dx, a.y + t * dy))


def path_length(points: Iterable[Point]) -> float:
    total = 0.0
    prev = None
    for p in points:
        if prev is not None:
            total += prev.distance_to(p)
        prev = p
    return total

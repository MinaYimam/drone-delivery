"""Order-to-drone assignment.

Greedy dispatcher: orders are handled most-urgent first, and each one goes
to the available drone that can reach the pickup soonest while still having
enough battery to finish the delivery and fly back to a hub with a safety
reserve left over.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Iterable, Optional, Sequence

from .geo import Point, path_length
from .models import Drone, Hub, Order
from .planner import RouteNotFound, RoutePlanner


@dataclass
class Plan:
    drone: Drone
    order: Order
    to_pickup: list[Point]
    to_dropoff: list[Point]
    to_hub: list[Point]
    return_hub: Hub
    energy_wh: float
    eta_hours: float  # time until the package is dropped off


class Dispatcher:
    def __init__(self, hubs: Sequence[Hub], planner: RoutePlanner, reserve_fraction: float = 0.15):
        if not hubs:
            raise ValueError("at least one hub is required")
        if not 0 <= reserve_fraction < 1:
            raise ValueError("reserve_fraction must be in [0, 1)")
        self.hubs = list(hubs)
        self.planner = planner
        self.reserve_fraction = reserve_fraction

    def nearest_hub(self, p: Point) -> tuple[Hub, list[Point]]:
        best: Optional[tuple[float, Hub, list[Point]]] = None
        for hub in self.hubs:
            try:
                route = self.planner.plan(p, hub.location)
            except RouteNotFound:
                continue
            length = path_length(route)
            if best is None or length < best[0]:
                best = (length, hub, route)
        if best is None:
            raise RouteNotFound(f"no hub reachable from {p}")
        return best[1], best[2]

    def evaluate(self, drone: Drone, order: Order, battery_wh: Optional[float] = None) -> Optional[Plan]:
        """Return a feasible plan for `drone` to serve `order`, or None."""
        if order.weight_kg > drone.max_payload_kg:
            return None
        battery = drone.battery_wh if battery_wh is None else battery_wh
        try:
            leg1 = self.planner.plan(drone.position, order.pickup)
            leg2 = self.planner.plan(order.pickup, order.dropoff)
            hub, leg3 = self.nearest_hub(order.dropoff)
        except RouteNotFound:
            return None

        d1, d2, d3 = path_length(leg1), path_length(leg2), path_length(leg3)
        energy = drone.energy_for(d1) + drone.energy_for(d2, order.weight_kg) + drone.energy_for(d3)
        reserve = self.reserve_fraction * drone.battery_capacity_wh
        if battery - energy < reserve:
            return None
        return Plan(drone, order, leg1, leg2, leg3, hub, energy, (d1 + d2) / drone.speed_kmh)

    def is_ever_feasible(self, order: Order, drones: Iterable[Drone]) -> tuple[bool, str]:
        """Could any drone deliver this order with a full battery from its hub?"""
        drones = list(drones)
        if all(order.weight_kg > d.max_payload_kg for d in drones):
            return False, "too heavy for every drone in the fleet"
        if self.planner.is_blocked(order.pickup) or self.planner.is_blocked(order.dropoff):
            return False, "pickup or dropoff is inside a no-fly zone"
        hubs = {h.id: h for h in self.hubs}
        for d in drones:
            home = hubs.get(d.hub_id)
            if home is None:
                continue
            probe = Drone(**{**_drone_specs(d), "position": home.location})
            if self.evaluate(probe, order, battery_wh=d.battery_capacity_wh):
                return True, ""
        return False, "out of range for every drone"

    def assign(self, drones: Sequence[Drone], orders: Sequence[Order]) -> list[Plan]:
        free = [d for d in drones if d.available]
        plans: list[Plan] = []
        for order in sorted(orders, key=lambda o: (-o.priority, o.created_at, o.id)):
            if not free:
                break
            candidates = [p for p in (self.evaluate(d, order) for d in free) if p is not None]
            if not candidates:
                continue
            best = min(candidates, key=lambda p: (p.eta_hours, p.energy_wh, p.drone.id))
            plans.append(best)
            free.remove(best.drone)
        return plans


def _drone_specs(d: Drone) -> dict:
    return {
        "id": d.id,
        "hub_id": d.hub_id,
        "position": d.position,
        "speed_kmh": d.speed_kmh,
        "max_payload_kg": d.max_payload_kg,
        "battery_capacity_wh": d.battery_capacity_wh,
        "base_wh_per_km": d.base_wh_per_km,
        "payload_wh_per_km_per_kg": d.payload_wh_per_km_per_kg,
    }

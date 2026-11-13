"""Time-stepped fleet simulation."""

from __future__ import annotations

import json
import statistics
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional, Sequence

from .dispatcher import Dispatcher, Plan
from .geo import NoFlyZone, Point
from .models import Drone, DroneStatus, Hub, Order, OrderStatus
from .planner import RoutePlanner


@dataclass
class SimulationConfig:
    tick_seconds: float = 10.0
    charge_rate_w: float = 1500.0  # watts while docked at a hub
    reserve_fraction: float = 0.15
    max_duration_s: float = 24 * 3600.0
    record_every: int = 6  # ticks between recorded frames (0 disables)


@dataclass
class SimulationReport:
    duration_s: float
    orders_total: int
    delivered: int
    failed: int
    undelivered: int
    avg_delivery_min: Optional[float]
    p90_delivery_min: Optional[float]
    max_delivery_min: Optional[float]
    total_distance_km: float
    total_energy_wh: float
    per_drone: list[dict] = field(default_factory=list)

    def to_dict(self) -> dict:
        return self.__dict__.copy()

    def format(self) -> str:
        def fmt(v: Optional[float]) -> str:
            return "n/a" if v is None else f"{v:.1f} min"

        lines = [
            "Simulation report",
            "=" * 40,
            f"Simulated time      : {self.duration_s / 60:.1f} min",
            f"Orders              : {self.orders_total}",
            f"  delivered         : {self.delivered}",
            f"  failed            : {self.failed}",
            f"  still open        : {self.undelivered}",
            f"Avg delivery time   : {fmt(self.avg_delivery_min)}",
            f"P90 delivery time   : {fmt(self.p90_delivery_min)}",
            f"Max delivery time   : {fmt(self.max_delivery_min)}",
            f"Fleet distance      : {self.total_distance_km:.1f} km",
            f"Fleet energy        : {self.total_energy_wh / 1000:.2f} kWh",
            "",
            f"{'Drone':<8}{'Deliveries':>11}{'Distance km':>13}{'Battery %':>11}",
        ]
        for d in self.per_drone:
            lines.append(f"{d['id']:<8}{d['deliveries']:>11}{d['distance_km']:>13.1f}{d['battery_pct']:>11.0f}")
        return "\n".join(lines)


class Simulation:
    def __init__(
        self,
        hubs: Sequence[Hub],
        drones: Sequence[Drone],
        orders: Sequence[Order],
        zones: Sequence[NoFlyZone] = (),
        config: Optional[SimulationConfig] = None,
    ):
        self.config = config or SimulationConfig()
        self.hubs = list(hubs)
        self.hubs_by_id = {h.id: h for h in self.hubs}
        self.drones = list(drones)
        self.zones = list(zones)
        self.orders = sorted(orders, key=lambda o: o.created_at)
        self.planner = RoutePlanner(self.zones)
        self.dispatcher = Dispatcher(self.hubs, self.planner, self.config.reserve_fraction)

        self.now = 0.0
        self.tick = 0
        self._next_order = 0
        self.pending: list[Order] = []
        self.frames: list[dict] = []

        for d in self.drones:
            if d.hub_id not in self.hubs_by_id:
                raise ValueError(f"drone {d.id} references unknown hub {d.hub_id}")

    # ------------------------------------------------------------- main loop
    def step(self) -> None:
        self.now += self.config.tick_seconds
        self.tick += 1
        self._release_orders()
        for plan in self.dispatcher.assign(self.drones, self.pending):
            self._start(plan)
        for drone in self.drones:
            self._advance(drone)
        if self.config.record_every and self.tick % self.config.record_every == 0:
            self.frames.append(self.snapshot())

    def finished(self) -> bool:
        all_released = self._next_order >= len(self.orders)
        all_closed = all(o.status in (OrderStatus.DELIVERED, OrderStatus.FAILED) for o in self.orders)
        drones_home = all(d.status in (DroneStatus.IDLE, DroneStatus.CHARGING) for d in self.drones)
        return all_released and all_closed and drones_home

    def run(self) -> SimulationReport:
        while not self.finished() and self.now < self.config.max_duration_s:
            self.step()
        return self.report()

    # -------------------------------------------------------------- internals
    def _release_orders(self) -> None:
        while self._next_order < len(self.orders) and self.orders[self._next_order].created_at <= self.now:
            order = self.orders[self._next_order]
            self._next_order += 1
            ok, reason = self.dispatcher.is_ever_feasible(order, self.drones)
            if ok:
                self.pending.append(order)
            else:
                order.status = OrderStatus.FAILED
                order.failure_reason = reason

    def _start(self, plan: Plan) -> None:
        drone, order = plan.drone, plan.order
        self.pending.remove(order)
        order.status = OrderStatus.ASSIGNED
        order.assigned_to = drone.id
        drone.order = order
        drone.status = DroneStatus.TO_PICKUP
        drone.route = plan.to_pickup[1:]

    def _advance(self, drone: Drone) -> None:
        dt = self.config.tick_seconds
        if drone.status == DroneStatus.CHARGING:
            drone.battery_wh = min(
                drone.battery_capacity_wh, drone.battery_wh + self.config.charge_rate_w * dt / 3600
            )
            if drone.battery_wh >= drone.battery_capacity_wh:
                drone.status = DroneStatus.IDLE
            return
        if drone.status == DroneStatus.IDLE:
            return

        budget = drone.speed_kmh * dt / 3600  # km this tick
        while budget > 1e-12:
            if not drone.route:
                self._arrive(drone)
                if drone.status in (DroneStatus.IDLE, DroneStatus.CHARGING):
                    return
                continue
            target = drone.route[0]
            step = min(budget, drone.position.distance_to(target))
            drone.battery_wh -= drone.energy_for(step, drone.payload_kg)
            drone.energy_used_wh += drone.energy_for(step, drone.payload_kg)
            drone.odometer_km += step
            drone.position = drone.position.towards(target, step)
            budget -= step
            if drone.position.distance_to(target) < 1e-9:
                drone.position = target
                drone.route.pop(0)
        if not drone.route:
            self._arrive(drone)

    def _arrive(self, drone: Drone) -> None:
        order = drone.order
        if drone.status == DroneStatus.TO_PICKUP and order is not None:
            order.status = OrderStatus.IN_FLIGHT
            order.picked_up_at = self.now
            drone.status = DroneStatus.TO_DROPOFF
            drone.route = self.planner.plan(drone.position, order.dropoff)[1:]
        elif drone.status == DroneStatus.TO_DROPOFF and order is not None:
            order.status = OrderStatus.DELIVERED
            order.delivered_at = self.now
            drone.deliveries += 1
            drone.order = None
            hub, route = self.dispatcher.nearest_hub(drone.position)
            drone.hub_id = hub.id
            drone.status = DroneStatus.RETURNING
            drone.route = route[1:]
        elif drone.status == DroneStatus.RETURNING:
            drone.status = DroneStatus.CHARGING

    # ---------------------------------------------------------------- output
    def snapshot(self) -> dict:
        return {
            "t": self.now,
            "drones": [d.to_dict() for d in self.drones],
            "pending": [o.id for o in self.pending],
        }

    def report(self) -> SimulationReport:
        times = sorted(o.delivery_time / 60 for o in self.orders if o.delivery_time is not None)
        delivered = len(times)
        failed = sum(o.status == OrderStatus.FAILED for o in self.orders)
        p90 = times[min(len(times) - 1, int(0.9 * len(times)))] if times else None
        return SimulationReport(
            duration_s=self.now,
            orders_total=len(self.orders),
            delivered=delivered,
            failed=failed,
            undelivered=len(self.orders) - delivered - failed,
            avg_delivery_min=statistics.fmean(times) if times else None,
            p90_delivery_min=p90,
            max_delivery_min=times[-1] if times else None,
            total_distance_km=sum(d.odometer_km for d in self.drones),
            total_energy_wh=sum(d.energy_used_wh for d in self.drones),
            per_drone=[
                {
                    "id": d.id,
                    "deliveries": d.deliveries,
                    "distance_km": round(d.odometer_km, 2),
                    "battery_pct": round(d.battery_pct, 1),
                }
                for d in self.drones
            ],
        )

    def export(self, path: str | Path) -> None:
        """Write a JSON replay: map, orders, recorded frames and the report."""
        data = {
            "hubs": [h.to_dict() for h in self.hubs],
            "zones": [z.to_dict() for z in self.zones],
            "orders": [o.to_dict() for o in self.orders],
            "frames": self.frames,
            "report": self.report().to_dict(),
        }
        Path(path).write_text(json.dumps(data, indent=2))

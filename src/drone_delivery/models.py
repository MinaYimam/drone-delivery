"""Core domain objects: hubs, drones and orders."""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional

from .geo import Point


class DroneStatus(str, Enum):
    IDLE = "idle"
    TO_PICKUP = "to_pickup"
    TO_DROPOFF = "to_dropoff"
    RETURNING = "returning"
    CHARGING = "charging"


class OrderStatus(str, Enum):
    PENDING = "pending"
    ASSIGNED = "assigned"
    IN_FLIGHT = "in_flight"
    DELIVERED = "delivered"
    FAILED = "failed"


@dataclass
class Hub:
    id: str
    location: Point
    name: str = ""

    def to_dict(self) -> dict:
        return {"id": self.id, "name": self.name, "location": self.location.to_dict()}


@dataclass
class Order:
    id: str
    pickup: Point
    dropoff: Point
    weight_kg: float
    created_at: float = 0.0  # simulation seconds
    priority: int = 1  # higher = more urgent

    status: OrderStatus = OrderStatus.PENDING
    assigned_to: Optional[str] = None
    picked_up_at: Optional[float] = None
    delivered_at: Optional[float] = None
    failure_reason: Optional[str] = None

    @property
    def delivery_time(self) -> Optional[float]:
        """Seconds from order creation to delivery."""
        if self.delivered_at is None:
            return None
        return self.delivered_at - self.created_at

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "pickup": self.pickup.to_dict(),
            "dropoff": self.dropoff.to_dict(),
            "weight_kg": self.weight_kg,
            "created_at": self.created_at,
            "priority": self.priority,
            "status": self.status.value,
            "assigned_to": self.assigned_to,
            "delivered_at": self.delivered_at,
            "failure_reason": self.failure_reason,
        }


@dataclass
class Drone:
    id: str
    hub_id: str
    position: Point
    speed_kmh: float = 60.0
    max_payload_kg: float = 5.0
    battery_capacity_wh: float = 500.0
    base_wh_per_km: float = 12.0
    payload_wh_per_km_per_kg: float = 2.0
    battery_wh: Optional[float] = None

    status: DroneStatus = DroneStatus.IDLE
    route: list[Point] = field(default_factory=list)
    order: Optional[Order] = None
    odometer_km: float = 0.0
    energy_used_wh: float = 0.0
    deliveries: int = 0

    def __post_init__(self) -> None:
        if self.battery_wh is None:
            self.battery_wh = self.battery_capacity_wh

    @property
    def battery_pct(self) -> float:
        return 100.0 * self.battery_wh / self.battery_capacity_wh

    @property
    def payload_kg(self) -> float:
        if self.order is not None and self.status == DroneStatus.TO_DROPOFF:
            return self.order.weight_kg
        return 0.0

    @property
    def available(self) -> bool:
        """Drones can only be dispatched while sitting at a hub."""
        return self.status in (DroneStatus.IDLE, DroneStatus.CHARGING)

    def energy_for(self, distance_km: float, payload_kg: float = 0.0) -> float:
        return distance_km * (self.base_wh_per_km + payload_kg * self.payload_wh_per_km_per_kg)

    def to_dict(self) -> dict:
        return {
            "id": self.id,
            "hub_id": self.hub_id,
            "position": self.position.to_dict(),
            "status": self.status.value,
            "battery_pct": round(self.battery_pct, 1),
            "order_id": self.order.id if self.order else None,
        }

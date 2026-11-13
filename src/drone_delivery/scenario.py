"""Build simulations from JSON files or random generation."""

from __future__ import annotations

import json
import random
from dataclasses import dataclass
from pathlib import Path

from .geo import NoFlyZone, Point
from .models import Drone, Hub, Order
from .simulation import Simulation, SimulationConfig


@dataclass
class Scenario:
    hubs: list[Hub]
    drones: list[Drone]
    orders: list[Order]
    zones: list[NoFlyZone]

    def build(self, config: SimulationConfig | None = None) -> Simulation:
        return Simulation(self.hubs, self.drones, self.orders, self.zones, config)


def load_scenario(path: str | Path) -> Scenario:
    """Load a scenario file. See examples/twin_cities.json for the format."""
    data = json.loads(Path(path).read_text())
    hubs = [Hub(h["id"], Point.from_dict(h["location"]), h.get("name", "")) for h in data["hubs"]]
    hub_by_id = {h.id: h for h in hubs}
    zones = [NoFlyZone.from_dict(z) for z in data.get("zones", [])]

    drones: list[Drone] = []
    defaults = data.get("drone_defaults", {})
    for spec in data["drones"]:
        spec = {**defaults, **spec}
        count = int(spec.pop("count", 1))
        hub_id = spec.pop("hub")
        base_id = spec.pop("id", None)
        for i in range(count):
            drone_id = base_id if (base_id and count == 1) else f"{base_id or hub_id}-{i + 1}"
            drones.append(Drone(id=drone_id, hub_id=hub_id, position=hub_by_id[hub_id].location, **spec))

    orders = [
        Order(
            id=o["id"],
            pickup=Point.from_dict(o["pickup"]),
            dropoff=Point.from_dict(o["dropoff"]),
            weight_kg=float(o["weight_kg"]),
            created_at=float(o.get("created_at", 0)),
            priority=int(o.get("priority", 1)),
        )
        for o in data["orders"]
    ]
    return Scenario(hubs, drones, orders, zones)


def random_scenario(
    seed: int = 0,
    n_hubs: int = 2,
    n_drones: int = 6,
    n_orders: int = 40,
    n_zones: int = 2,
    size_km: float = 12.0,
    duration_min: float = 120.0,
) -> Scenario:
    """Generate a reproducible random city."""
    rng = random.Random(seed)

    zones: list[NoFlyZone] = []
    for i in range(n_zones):
        zones.append(
            NoFlyZone(
                Point(rng.uniform(0.25, 0.75) * size_km, rng.uniform(0.25, 0.75) * size_km),
                rng.uniform(0.6, 1.4),
                f"Zone {i + 1}",
            )
        )

    def free_point() -> Point:
        while True:
            p = Point(rng.uniform(0, size_km), rng.uniform(0, size_km))
            if not any(z.contains(p) for z in zones):
                return p

    hubs = [Hub(f"H{i + 1}", free_point(), f"Hub {i + 1}") for i in range(n_hubs)]
    drones = [
        Drone(id=f"D{i + 1}", hub_id=hubs[i % n_hubs].id, position=hubs[i % n_hubs].location)
        for i in range(n_drones)
    ]
    orders = []
    for i in range(n_orders):
        orders.append(
            Order(
                id=f"O{i + 1:03d}",
                pickup=free_point(),
                dropoff=free_point(),
                weight_kg=round(rng.uniform(0.2, 4.5), 2),
                created_at=round(rng.uniform(0, duration_min * 60)),
                priority=rng.choices([1, 2, 3], weights=[6, 3, 1])[0],
            )
        )
    return Scenario(hubs, drones, orders, zones)

import json
from pathlib import Path

from drone_delivery import (
    Drone,
    DroneStatus,
    Hub,
    NoFlyZone,
    Order,
    OrderStatus,
    Point,
    Simulation,
    SimulationConfig,
    load_scenario,
    random_scenario,
)
from drone_delivery.cli import main

ROOT = Path(__file__).resolve().parents[1]


def test_single_delivery_lifecycle():
    hub = Hub("H", Point(0, 0))
    drone = Drone("D", "H", hub.location, speed_kmh=60)
    order = Order("O", Point(1, 0), Point(2, 0), weight_kg=1)
    sim = Simulation([hub], [drone], [order], config=SimulationConfig(tick_seconds=5))
    report = sim.run()

    assert order.status == OrderStatus.DELIVERED
    assert report.delivered == 1
    # 1 km to pickup + 1 km to dropoff at 60 km/h = 2 minutes (+1 tick slack)
    assert 115 <= order.delivery_time <= 125
    assert drone.status in (DroneStatus.IDLE, DroneStatus.CHARGING)
    assert drone.position == hub.location
    assert abs(drone.odometer_km - 4) < 1e-9  # out 2 km, back 2 km


def test_battery_drains_then_recharges():
    hub = Hub("H", Point(0, 0))
    drone = Drone("D", "H", hub.location, battery_capacity_wh=500, base_wh_per_km=10, payload_wh_per_km_per_kg=0)
    order = Order("O", Point(5, 0), Point(5, 5), weight_kg=1)
    sim = Simulation([hub], [drone], [order], config=SimulationConfig(tick_seconds=5, charge_rate_w=600))
    while order.status != OrderStatus.DELIVERED:
        sim.step()
    assert drone.battery_wh < 500
    sim.run()
    # Keep stepping until fully charged
    for _ in range(10_000):
        if drone.status == DroneStatus.IDLE:
            break
        sim.step()
    assert drone.battery_wh == 500


def test_never_flies_through_zone():
    zone = NoFlyZone(Point(5, 0), 1.5)
    hub = Hub("H", Point(0, 0))
    drone = Drone("D", "H", hub.location)
    order = Order("O", Point(0.5, 0), Point(10, 0), weight_kg=1)
    sim = Simulation([hub], [drone], [order], zones=[zone], config=SimulationConfig(tick_seconds=1))
    while not sim.finished():
        sim.step()
        assert not zone.contains(drone.position)
    assert order.status == OrderStatus.DELIVERED


def test_impossible_orders_fail_with_reason():
    hub = Hub("H", Point(0, 0))
    drone = Drone("D", "H", hub.location, max_payload_kg=5)
    heavy = Order("heavy", Point(1, 0), Point(2, 0), weight_kg=20)
    sim = Simulation([hub], [drone], [heavy])
    sim.run()
    assert heavy.status == OrderStatus.FAILED
    assert heavy.failure_reason


def test_random_scenario_is_deterministic_and_completes():
    r1 = random_scenario(seed=7, n_orders=25).build().run()
    r2 = random_scenario(seed=7, n_orders=25).build().run()
    assert r1.to_dict() == r2.to_dict()
    assert r1.delivered + r1.failed == 25


def test_example_scenario_loads_and_runs():
    scenario = load_scenario(ROOT / "examples" / "twin_cities.json")
    assert len(scenario.drones) == 6
    report = scenario.build().run()
    assert report.delivered == 8
    assert report.failed == 2


def test_cli_run_and_export(tmp_path, capsys):
    out = tmp_path / "replay.json"
    assert main(["run", "--orders", "10", "--seed", "1", "--export", str(out)]) == 0
    assert "Simulation report" in capsys.readouterr().out
    data = json.loads(out.read_text())
    assert {"hubs", "zones", "orders", "frames", "report"} <= data.keys()
    assert data["frames"]

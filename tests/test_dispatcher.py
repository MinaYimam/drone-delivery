from drone_delivery import Dispatcher, Drone, DroneStatus, Hub, Order, Point, RoutePlanner


def make(hub_at=Point(0, 0)):
    hub = Hub("H", hub_at)
    return hub, Dispatcher([hub], RoutePlanner())


def test_rejects_overweight():
    hub, disp = make()
    drone = Drone("D", "H", hub.location, max_payload_kg=2)
    order = Order("O", Point(1, 0), Point(2, 0), weight_kg=3)
    assert disp.evaluate(drone, order) is None


def test_rejects_when_battery_too_low():
    hub, disp = make()
    drone = Drone("D", "H", hub.location, battery_capacity_wh=100, base_wh_per_km=10)
    order = Order("O", Point(3, 0), Point(6, 0), weight_kg=0)
    # Needs 3 + 3 + 6 = 12 km = 120 Wh > 100 Wh capacity
    assert disp.evaluate(drone, order) is None


def test_respects_reserve():
    hub, disp = make()
    drone = Drone("D", "H", hub.location, battery_capacity_wh=100, base_wh_per_km=10)
    order = Order("O", Point(2, 0), Point(4, 0), weight_kg=0)
    # 8 km = 80 Wh, leaves 20 Wh >= 15 Wh reserve -> ok
    assert disp.evaluate(drone, order) is not None
    drone.battery_wh = 90  # leaves 10 Wh < 15 Wh reserve -> rejected
    assert disp.evaluate(drone, order) is None


def test_assigns_closest_drone_and_priority_first():
    near, far = Hub("N", Point(0, 0)), Hub("F", Point(20, 0))
    disp = Dispatcher([near, far], RoutePlanner())
    d_near = Drone("near", "N", near.location)
    d_far = Drone("far", "F", far.location)
    low = Order("low", Point(1, 0), Point(2, 0), 1, created_at=0, priority=1)
    high = Order("high", Point(1, 0), Point(2, 0), 1, created_at=5, priority=3)

    plans = disp.assign([d_near, d_far], [low, high])
    by_order = {p.order.id: p.drone.id for p in plans}
    assert by_order["high"] == "near"  # urgent order gets the best drone
    assert by_order["low"] == "far"


def test_busy_drones_not_assigned():
    hub, disp = make()
    drone = Drone("D", "H", hub.location, status=DroneStatus.TO_DROPOFF)
    order = Order("O", Point(1, 0), Point(2, 0), 1)
    assert disp.assign([drone], [order]) == []


def test_is_ever_feasible_reasons():
    hub, disp = make()
    drone = Drone("D", "H", hub.location, max_payload_kg=5)
    ok, _ = disp.is_ever_feasible(Order("a", Point(1, 0), Point(2, 0), 1), [drone])
    assert ok
    ok, why = disp.is_ever_feasible(Order("b", Point(1, 0), Point(2, 0), 50), [drone])
    assert not ok and "heavy" in why
    ok, why = disp.is_ever_feasible(Order("c", Point(500, 0), Point(501, 0), 1), [drone])
    assert not ok and "range" in why

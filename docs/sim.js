// Browser port of the Python simulator (src/drone_delivery). Same algorithms:
// visibility-graph routing around no-fly zones, greedy ETA dispatch with a
// battery reserve, and a tick-based flight/charging loop.

const EPS = 1e-9;

export const dist = (a, b) => Math.hypot(a.x - b.x, a.y - b.y);

function segDist(p, a, b) {
  const dx = b.x - a.x, dy = b.y - a.y;
  const L = dx * dx + dy * dy;
  if (L < EPS) return dist(p, a);
  const t = Math.max(0, Math.min(1, ((p.x - a.x) * dx + (p.y - a.y) * dy) / L));
  return dist(p, { x: a.x + t * dx, y: a.y + t * dy });
}

export const pathLength = (pts) => pts.slice(1).reduce((s, p, i) => s + dist(pts[i], p), 0);

export class Planner {
  constructor(zones, margin = 0.15, sides = 12) {
    this.zones = zones;
    const f = (1 + margin) / Math.cos(Math.PI / sides);
    this.waypoints = [];
    for (const z of zones) {
      for (let i = 0; i < sides; i++) {
        const a = (2 * Math.PI * i) / sides;
        const p = { x: z.center.x + z.radius * f * Math.cos(a), y: z.center.y + z.radius * f * Math.sin(a) };
        if (!this.blocked(p)) this.waypoints.push(p);
      }
    }
    this.cache = new Map();
  }
  blocked(p) { return this.zones.some((z) => dist(p, z.center) < z.radius - EPS); }
  clear(a, b) { return !this.zones.some((z) => segDist(z.center, a, b) < z.radius - EPS); }

  plan(start, goal) {
    const key = `${start.x},${start.y}|${goal.x},${goal.y}`;
    if (this.cache.has(key)) return this.cache.get(key).slice();
    if (this.blocked(start) || this.blocked(goal)) return null;
    let route;
    if (this.clear(start, goal)) route = [start, goal];
    else route = this.dijkstra(start, goal);
    if (this.cache.size > 5000) this.cache.clear();
    this.cache.set(key, route);
    return route ? route.slice() : null;
  }

  dijkstra(start, goal) {
    const nodes = [start, goal, ...this.waypoints];
    const d = new Array(nodes.length).fill(Infinity);
    const prev = new Array(nodes.length).fill(-1);
    const done = new Array(nodes.length).fill(false);
    d[0] = 0;
    for (;;) {
      let u = -1;
      for (let i = 0; i < nodes.length; i++) if (!done[i] && d[i] < Infinity && (u < 0 || d[i] < d[u])) u = i;
      if (u < 0) return null;
      if (u === 1) break;
      done[u] = true;
      for (let v = 0; v < nodes.length; v++) {
        if (done[v] || v === u || !this.clear(nodes[u], nodes[v])) continue;
        const nd = d[u] + dist(nodes[u], nodes[v]);
        if (nd < d[v]) { d[v] = nd; prev[v] = u; }
      }
    }
    const path = [1];
    while (path[path.length - 1] !== 0) path.push(prev[path[path.length - 1]]);
    return path.reverse().map((i) => nodes[i]);
  }
}

export class Simulation {
  constructor({ hubs, zones, droneCount = 6, reserve = 0.15, chargeW = 1500 }) {
    this.hubs = hubs;
    this.zones = zones;
    this.reserve = reserve;
    this.chargeW = chargeW;
    this.planner = new Planner(zones);
    this.now = 0;
    this.orders = [];
    this.pending = [];
    this.events = [];
    this.drones = [];
    this.nextOrderId = 1;
    for (let i = 0; i < droneCount; i++) this.addDrone();
  }

  addDrone() {
    const hub = this.hubs[this.drones.length % this.hubs.length];
    this.drones.push({
      id: `D${this.drones.length + 1}`,
      hub: hub.id,
      pos: { ...hub.location },
      speed: 60, maxPayload: 5, capacity: 500, battery: 500,
      baseWh: 12, payloadWh: 2,
      status: "idle", route: [], order: null, km: 0, deliveries: 0, trail: [],
    });
  }

  log(msg) {
    this.events.unshift({ t: this.now, msg });
    if (this.events.length > 60) this.events.pop();
  }

  energy(d, km, kg) { return km * (d.baseWh + kg * d.payloadWh); }

  nearestHub(p) {
    let best = null;
    for (const h of this.hubs) {
      const r = this.planner.plan(p, h.location);
      if (!r) continue;
      const L = pathLength(r);
      if (!best || L < best.L) best = { hub: h, route: r, L };
    }
    return best;
  }

  evaluate(d, o, battery = d.battery, from = d.pos) {
    if (o.weight > d.maxPayload) return null;
    const l1 = this.planner.plan(from, o.pickup);
    const l2 = this.planner.plan(o.pickup, o.dropoff);
    const back = this.nearestHub(o.dropoff);
    if (!l1 || !l2 || !back) return null;
    const d1 = pathLength(l1), d2 = pathLength(l2), d3 = back.L;
    const e = this.energy(d, d1, 0) + this.energy(d, d2, o.weight) + this.energy(d, d3, 0);
    if (battery - e < this.reserve * d.capacity) return null;
    return { drone: d, order: o, l1, eta: (d1 + d2) / d.speed, e };
  }

  addOrder(pickup, dropoff, weight = 1 + Math.random() * 3.5, priority = 1) {
    const o = {
      id: `O${String(this.nextOrderId++).padStart(3, "0")}`,
      pickup, dropoff, weight: Math.round(weight * 10) / 10, priority,
      created: this.now, status: "pending", delivered: null, drone: null,
    };
    this.orders.push(o);
    const feasible = this.drones.some((d) => {
      const hub = this.hubs.find((h) => h.id === d.hub);
      return this.evaluate(d, o, d.capacity, hub.location);
    });
    if (!feasible) {
      o.status = "failed";
      this.log(`✕ ${o.id} rejected (out of range, too heavy or in a no-fly zone)`);
    } else {
      this.pending.push(o);
      this.log(`+ ${o.id} placed (${o.weight} kg${priority > 1 ? ", priority" : ""})`);
    }
    return o;
  }

  dispatch() {
    const free = this.drones.filter((d) => d.status === "idle" || d.status === "charging");
    const queue = [...this.pending].sort((a, b) => b.priority - a.priority || a.created - b.created);
    for (const o of queue) {
      if (!free.length) break;
      let best = null;
      for (const d of free) {
        const p = this.evaluate(d, o);
        if (p && (!best || p.eta < best.eta)) best = p;
      }
      if (!best) continue;
      const d = best.drone;
      free.splice(free.indexOf(d), 1);
      this.pending.splice(this.pending.indexOf(o), 1);
      o.status = "assigned"; o.drone = d.id;
      d.order = o; d.status = "to_pickup"; d.route = best.l1.slice(1);
      this.log(`→ ${d.id} assigned ${o.id}`);
    }
  }

  arrive(d) {
    const o = d.order;
    if (d.status === "to_pickup") {
      o.status = "in_flight";
      d.status = "to_dropoff";
      d.route = this.planner.plan(d.pos, o.dropoff).slice(1);
    } else if (d.status === "to_dropoff") {
      o.status = "delivered"; o.delivered = this.now;
      d.deliveries++; d.order = null;
      this.log(`✓ ${o.id} delivered by ${d.id} in ${((this.now - o.created) / 60).toFixed(1)} min`);
      const back = this.nearestHub(d.pos);
      d.hub = back.hub.id; d.status = "returning"; d.route = back.route.slice(1);
    } else if (d.status === "returning") {
      d.status = "charging";
    }
  }

  step(dt) {
    this.now += dt;
    this.dispatch();
    for (const d of this.drones) {
      if (d.status === "charging") {
        d.battery = Math.min(d.capacity, d.battery + (this.chargeW * dt) / 3600);
        if (d.battery >= d.capacity) d.status = "idle";
        continue;
      }
      if (d.status === "idle") continue;
      let budget = (d.speed * dt) / 3600;
      let guard = 0;
      while (budget > 1e-12 && guard++ < 50) {
        if (!d.route.length) {
          this.arrive(d);
          if (d.status === "charging") break;
          continue;
        }
        const t = d.route[0];
        const step = Math.min(budget, dist(d.pos, t));
        const kg = d.status === "to_dropoff" ? d.order.weight : 0;
        d.battery -= this.energy(d, step, kg);
        d.km += step;
        const L = dist(d.pos, t);
        d.pos = L <= step + EPS ? { ...t } : { x: d.pos.x + ((t.x - d.pos.x) * step) / L, y: d.pos.y + ((t.y - d.pos.y) * step) / L };
        budget -= step;
        if (dist(d.pos, t) < EPS) d.route.shift();
      }
      if (!d.route.length && d.status !== "charging" && d.status !== "idle") this.arrive(d);
      d.trail.push({ ...d.pos });
      if (d.trail.length > 40) d.trail.shift();
    }
  }

  stats() {
    const done = this.orders.filter((o) => o.status === "delivered");
    const avg = done.length ? done.reduce((s, o) => s + (o.delivered - o.created), 0) / done.length / 60 : null;
    return {
      delivered: done.length,
      pending: this.pending.length,
      active: this.orders.filter((o) => o.status === "assigned" || o.status === "in_flight").length,
      failed: this.orders.filter((o) => o.status === "failed").length,
      avgMin: avg,
      km: this.drones.reduce((s, d) => s + d.km, 0),
    };
  }
}

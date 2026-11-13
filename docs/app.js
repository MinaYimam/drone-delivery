import { Simulation } from "./sim.js";

// Same layout as examples/twin_cities.json (km grid).
const WORLD = { w: 20, h: 14 };
const HUBS = [
  { id: "MPLS", name: "Minneapolis Hub", location: { x: 4.0, y: 9.0 } },
  { id: "STP", name: "St. Paul Hub", location: { x: 15.0, y: 7.5 } },
];
const ZONES = [
  { name: "Airport", center: { x: 9.5, y: 2.0 }, radius: 2.2 },
  { name: "Stadium", center: { x: 5.5, y: 9.8 }, radius: 0.5 },
  { name: "State Capitol", center: { x: 14.5, y: 9.0 }, radius: 0.6 },
];
const STATUS_LABEL = {
  idle: "ready", charging: "charging", to_pickup: "→ pickup", to_dropoff: "→ dropoff", returning: "returning",
};
const COLORS = {
  hub: "#facc15", zone: "#f43f5e", pickup: "#a78bfa", dropoff: "#34d399", drone: "#38bdf8", grid: "#16213a",
};

const $ = (id) => document.getElementById(id);
const canvas = $("map");
const ctx = canvas.getContext("2d");

let sim, playing = true, orderClock = 0, pendingPickup = null, last = performance.now();

function reset() {
  sim = new Simulation({ hubs: HUBS, zones: ZONES, droneCount: +$("drones").value });
  orderClock = 0;
  pendingPickup = null;
  for (let i = 0; i < 4; i++) randomOrder();
}

function randomPoint() {
  for (;;) {
    const p = { x: 0.5 + Math.random() * (WORLD.w - 1), y: 0.5 + Math.random() * (WORLD.h - 1) };
    if (!sim.planner.blocked(p)) return p;
  }
}
function randomOrder() {
  sim.addOrder(randomPoint(), randomPoint(), 0.3 + Math.random() * 4.2, Math.random() < 0.15 ? 3 : 1);
}

// ---------------------------------------------------------------- drawing
let scale = 1, ox = 0, oy = 0;
function resize() {
  const r = canvas.getBoundingClientRect();
  const dpr = window.devicePixelRatio || 1;
  canvas.width = r.width * dpr;
  canvas.height = r.height * dpr;
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  scale = Math.min(r.width / WORLD.w, r.height / WORLD.h) * 0.94;
  ox = (r.width - WORLD.w * scale) / 2;
  oy = (r.height - WORLD.h * scale) / 2;
}
const sx = (p) => ox + p.x * scale;
const sy = (p) => oy + (WORLD.h - p.y) * scale; // y up
const toWorld = (px, py) => ({ x: (px - ox) / scale, y: WORLD.h - (py - oy) / scale });

function draw() {
  const r = canvas.getBoundingClientRect();
  ctx.clearRect(0, 0, r.width, r.height);

  // grid
  ctx.strokeStyle = COLORS.grid;
  ctx.lineWidth = 1;
  for (let x = 0; x <= WORLD.w; x++) line({ x, y: 0 }, { x, y: WORLD.h });
  for (let y = 0; y <= WORLD.h; y++) line({ x: 0, y }, { x: WORLD.w, y });
  ctx.fillStyle = "#3b4a6b";
  ctx.font = "10px system-ui";
  ctx.fillText("1 km grid", sx({ x: 0.1, y: 0 }), sy({ x: 0, y: 0.15 }));

  // zones
  for (const z of ZONES) {
    ctx.beginPath();
    ctx.arc(sx(z.center), sy(z.center), z.radius * scale, 0, Math.PI * 2);
    ctx.fillStyle = "rgba(244,63,94,0.12)";
    ctx.fill();
    ctx.setLineDash([5, 4]);
    ctx.strokeStyle = "rgba(244,63,94,0.7)";
    ctx.stroke();
    ctx.setLineDash([]);
    ctx.fillStyle = "rgba(244,63,94,0.9)";
    ctx.font = "11px system-ui";
    ctx.textAlign = "center";
    ctx.fillText(z.name, sx(z.center), sy(z.center) + 4);
    ctx.textAlign = "left";
  }

  // open orders
  for (const o of sim.orders) {
    if (o.status === "delivered" || o.status === "failed") continue;
    if (o.status === "pending" || o.status === "assigned") dot(o.pickup, COLORS.pickup, o.priority > 1 ? 5 : 3.5);
    dot(o.dropoff, COLORS.dropoff, 3.5);
    ctx.strokeStyle = "rgba(167,139,250,0.18)";
    line(o.pickup, o.dropoff);
  }

  // hubs
  for (const h of HUBS) {
    const x = sx(h.location), y = sy(h.location);
    ctx.fillStyle = COLORS.hub;
    ctx.fillRect(x - 7, y - 7, 14, 14);
    ctx.fillStyle = "#e6ecf7";
    ctx.font = "12px system-ui";
    ctx.fillText(h.name, x + 11, y + 4);
  }

  // drones
  for (const d of sim.drones) {
    if (d.route.length) {
      ctx.strokeStyle = d.status === "returning" ? "rgba(138,151,178,0.35)" : "rgba(56,189,248,0.45)";
      ctx.setLineDash([3, 4]);
      ctx.beginPath();
      ctx.moveTo(sx(d.pos), sy(d.pos));
      for (const p of d.route) ctx.lineTo(sx(p), sy(p));
      ctx.stroke();
      ctx.setLineDash([]);
    }
    if (d.trail.length > 1) {
      ctx.strokeStyle = "rgba(56,189,248,0.25)";
      ctx.beginPath();
      ctx.moveTo(sx(d.trail[0]), sy(d.trail[0]));
      for (const p of d.trail) ctx.lineTo(sx(p), sy(p));
      ctx.stroke();
    }
    if (d.status === "idle" || d.status === "charging") continue;
    const x = sx(d.pos), y = sy(d.pos);
    ctx.fillStyle = d.status === "to_dropoff" ? COLORS.dropoff : COLORS.drone;
    ctx.beginPath();
    ctx.moveTo(x, y - 7); ctx.lineTo(x + 6, y + 5); ctx.lineTo(x - 6, y + 5); ctx.closePath();
    ctx.fill();
    ctx.fillStyle = "#e6ecf7";
    ctx.font = "10px system-ui";
    ctx.fillText(d.id, x + 8, y - 4);
  }

  if (pendingPickup) {
    dot(pendingPickup, COLORS.pickup, 6);
    ctx.strokeStyle = COLORS.pickup;
    ctx.beginPath();
    ctx.arc(sx(pendingPickup), sy(pendingPickup), 11, 0, Math.PI * 2);
    ctx.stroke();
  }
}
function line(a, b) { ctx.beginPath(); ctx.moveTo(sx(a), sy(a)); ctx.lineTo(sx(b), sy(b)); ctx.stroke(); }
function dot(p, color, r) { ctx.fillStyle = color; ctx.beginPath(); ctx.arc(sx(p), sy(p), r, 0, Math.PI * 2); ctx.fill(); }

// ------------------------------------------------------------------ panel
const fmtClock = (s) => {
  const m = Math.floor(s / 60);
  return `${String(Math.floor(m / 60)).padStart(2, "0")}:${String(m % 60).padStart(2, "0")}`;
};
let panelTimer = 0;
function renderPanel() {
  const s = sim.stats();
  $("clock").textContent = fmtClock(sim.now);
  $("sDelivered").textContent = s.delivered;
  $("sActive").textContent = s.active;
  $("sPending").textContent = s.pending;
  $("sAvg").textContent = s.avgMin == null ? "–" : s.avgMin.toFixed(1);
  $("fleet").innerHTML = sim.drones
    .map((d) => {
      const pct = Math.max(0, (100 * d.battery) / d.capacity);
      const cls = pct < 30 ? "low" : pct < 60 ? "mid" : "";
      return `<li><b>${d.id}</b><div class="bar"><i class="${cls}" style="width:${pct}%"></i></div><span class="status">${STATUS_LABEL[d.status]}</span></li>`;
    })
    .join("");
  $("log").innerHTML = sim.events
    .slice(0, 25)
    .map((e) => `<li><time>${fmtClock(e.t)}</time>${e.msg}</li>`)
    .join("");
}

// -------------------------------------------------------------------- loop
function frame(t) {
  const realDt = Math.min(0.1, (t - last) / 1000);
  last = t;
  if (playing) {
    let simDt = realDt * +$("speed").value;
    const rate = +$("rate").value; // orders per sim-hour
    orderClock += simDt * rate / 3600;
    while (orderClock >= 1) { randomOrder(); orderClock -= 1; }
    while (simDt > 0) {
      const dt = Math.min(2, simDt);
      sim.step(dt);
      simDt -= dt;
    }
  }
  draw();
  if ((panelTimer += realDt) > 0.2) { renderPanel(); panelTimer = 0; }
  requestAnimationFrame(frame);
}

// ------------------------------------------------------------------ events
canvas.addEventListener("click", (e) => {
  const r = canvas.getBoundingClientRect();
  const p = toWorld(e.clientX - r.left, e.clientY - r.top);
  if (p.x < 0 || p.y < 0 || p.x > WORLD.w || p.y > WORLD.h) return;
  if (!pendingPickup) {
    pendingPickup = p;
    $("hint").innerHTML = "Now click the <b>dropoff</b> location.";
  } else {
    sim.addOrder(pendingPickup, p, 1.5, 2);
    pendingPickup = null;
    $("hint").innerHTML = "Order placed! Click again to add another.";
  }
});
$("play").onclick = () => {
  playing = !playing;
  $("play").textContent = playing ? "Pause" : "Play";
};
$("reset").onclick = reset;
$("drones").oninput = (e) => { $("dronesVal").textContent = e.target.value; reset(); };
$("rate").oninput = (e) => { $("rateVal").textContent = e.target.value; };
window.addEventListener("resize", resize);

reset();
resize();
requestAnimationFrame(frame);

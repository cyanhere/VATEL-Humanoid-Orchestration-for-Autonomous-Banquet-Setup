// <hall-demo>: the home page's animated floor plan. A robot shuttles from the AGV to each set
// position, one plate per trip, and every plate it lays stays on the table. It is an
// illustration of the mission, not a simulation of the R1: routes run along the aisles of a
// fixed six-table hall so the robot never cuts through a table.
//
// Motion pauses off screen, has a pause button, and is replaced by the finished hall when the
// visitor prefers reduced motion.

const NS = "http://www.w3.org/2000/svg";

// Hall geometry, in viewBox units (the SVG is 520 x 360).
const TABLE_R = 30;          // table radius
const PLATE_AT = 20;         // plate centre, from the table centre
const PLATE_R = 5;
const STAND_AT = 45;         // where the robot stands to place, from the table centre
const COLS = [190, 310, 430];
const ROWS = [110, 250];
const SET_ANGLES = [30, 90, 150, 210, 270, 330];   // degrees; 90 is the side facing down
// Aisles the robot drives along: clear of every table and every stand point.
const AISLE_Y_ABOVE = { 110: 43, 250: 180 };       // the aisle above each row
const AISLE_Y_BELOW = { 110: 180, 250: 317 };      // the aisle below each row
const AISLE_X = [110, 250, 370, 487];
const DOCK = { x: 110, y: 180 };                   // where the robot takes a plate off the AGV

const SPEED = 330;           // units per second
const LOAD_S = 0.35, PLACE_S = 0.5, REST_S = 2.4;

const CREAM = "#F4F7F5", GREEN = "#5FC2A2";

function layout() {
  const tables = [];
  ROWS.forEach((cy) => COLS.forEach((cx) => tables.push({ cx: cx, cy: cy, n: tables.length + 1 })));
  const sets = [];
  tables.forEach((t) => SET_ANGLES.forEach((deg) => {
    const a = (deg * Math.PI) / 180, cos = Math.cos(a), sin = Math.sin(a);
    sets.push({
      t: t, cos: cos, sin: sin,
      plate: { x: t.cx + PLATE_AT * cos, y: t.cy + PLATE_AT * sin },
      stand: { x: t.cx + STAND_AT * cos, y: t.cy + STAND_AT * sin },
    });
  }));
  return { tables: tables, sets: sets };
}

// Dock to stand point along the aisles. A set on a table's top or bottom is reached from the
// horizontal aisle on that side; one on its left or right from the vertical aisle there.
function route(s) {
  const { t, stand, cos, sin } = s;
  if (Math.abs(sin) > Math.abs(cos)) {
    const y = sin > 0 ? AISLE_Y_BELOW[t.cy] : AISLE_Y_ABOVE[t.cy];
    return [DOCK, { x: DOCK.x, y: y }, { x: stand.x, y: y }, stand];
  }
  const x = cos > 0 ? AISLE_X.find((ax) => ax > t.cx) : AISLE_X.filter((ax) => ax < t.cx).pop();
  return [DOCK, { x: x, y: DOCK.y }, { x: x, y: stand.y }, stand];
}

function measure(points) {
  const cum = [0];
  for (let i = 1; i < points.length; i++)
    cum.push(cum[i - 1] + Math.hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y));
  return { points: points, cum: cum, length: cum[cum.length - 1] };
}

function along(path, d) {
  const { points, cum } = path;
  for (let i = 1; i < points.length; i++) {
    if (d <= cum[i] || i === points.length - 1) {
      const seg = cum[i] - cum[i - 1] || 1, u = Math.min(1, Math.max(0, (d - cum[i - 1]) / seg));
      return { x: points[i - 1].x + (points[i].x - points[i - 1].x) * u,
               y: points[i - 1].y + (points[i].y - points[i - 1].y) * u };
    }
  }
  return points[points.length - 1];
}

const ease = (u) => (u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2);

function svg(name, attrs, parent) {
  const n = document.createElementNS(NS, name);
  for (const k in attrs) n.setAttribute(k, attrs[k]);
  if (parent) parent.appendChild(n);
  return n;
}

class HallDemo extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    this.tick = this.tick.bind(this);
  }

  connectedCallback() {
    if (!this.built) this.build();
    this.io = new IntersectionObserver((entries) => {
      this.onScreen = entries[0].isIntersecting;
      this.sync();
    });
    this.io.observe(this);
    this.motionQuery = window.matchMedia("(prefers-reduced-motion: reduce)");
    this.onChange = () => this.sync();
    this.motionQuery.addEventListener("change", this.onChange);
    // a background tab stops animation frames; pause properly so the robot does not jump
    document.addEventListener("visibilitychange", this.onChange);
    this.sync();
  }

  disconnectedCallback() {
    this.io.disconnect();
    this.motionQuery.removeEventListener("change", this.onChange);
    document.removeEventListener("visibilitychange", this.onChange);
    this.halt();
  }

  build() {
    this.built = true;
    const { tables, sets } = layout();
    this.sets = sets;
    this.trips = sets.map((s) => measure(route(s)));

    this.shadowRoot.innerHTML = `
      <div class="frame">
        <svg viewBox="0 0 520 360" role="img"
          aria-label="Animated floor plan: a robot carries plates one at a time from an AGV to the set positions on six banquet tables."></svg>
        <div class="foot">
          <span class="note">Illustration &middot; one plate per trip</span>
          <span class="count"><b id="placed">0</b> / ${sets.length} plates</span>
          <button type="button" id="toggle" aria-pressed="false">Pause</button>
        </div>
      </div>
      <style>
        :host { display: block; }
        .frame {
          padding: 14px 14px 10px;
          border: 1px solid rgba(255, 255, 255, 0.1);
          border-radius: 18px;
          background: rgba(255, 255, 255, 0.025);
        }
        svg { display: block; width: 100%; height: auto; }
        .table circle.top { fill: rgba(95, 194, 162, 0.07); stroke: rgba(95, 194, 162, 0.55);
                            stroke-width: 1.5; transition: stroke 0.2s, fill 0.2s; }
        .table:hover circle.top { fill: rgba(95, 194, 162, 0.14); stroke: ${GREEN}; }
        .table text { fill: rgba(255, 255, 255, 0.45); font: 600 10px Inter, system-ui, sans-serif;
                      letter-spacing: 0.04em; }
        .plate { fill: rgba(244, 247, 245, 0); stroke: rgba(255, 255, 255, 0.22); stroke-width: 1.2;
                 transform-box: fill-box; transform-origin: center; }
        .plate.on { fill: ${CREAM}; stroke: ${GREEN}; animation: lay 0.4s ease-out; }
        @keyframes lay { 0% { transform: scale(0.3); } 65% { transform: scale(1.3); } 100% { transform: scale(1); } }
        .plate.off { transition: fill 0.6s, stroke 0.6s; }
        .foot {
          display: flex; align-items: center; gap: 12px; margin-top: 8px;
          font-size: 0.8rem; color: #93A49D;
        }
        .note { flex: 1 1 auto; }
        .count { white-space: nowrap; }
        .count b { color: #fff; font-weight: 600; font-variant-numeric: tabular-nums; }
        button {
          padding: 4px 12px; border: 1px solid rgba(255, 255, 255, 0.22); border-radius: 999px;
          background: transparent; color: #DCE6E1; font: 600 0.75rem Inter, system-ui, sans-serif;
          cursor: pointer; transition: border-color 0.15s, background 0.15s;
        }
        button:hover, button:focus-visible { border-color: ${GREEN}; background: rgba(95, 194, 162, 0.12); outline: none; }
        button:active { background: rgba(95, 194, 162, 0.2); }
        :host([data-still]) button { display: none; }
        @media (prefers-reduced-motion: reduce) { .plate.on { animation: none; } }
      </style>`;

    const root = this.shadowRoot.querySelector("svg");
    const floor = svg("g", {}, root);
    svg("rect", { x: 12, y: 12, width: 496, height: 336, rx: 12, fill: "rgba(255,255,255,0.02)",
                  stroke: "rgba(255,255,255,0.14)", "stroke-width": 1.2 }, floor);
    for (let x = 52; x < 508; x += 40)
      svg("line", { x1: x, y1: 13, x2: x, y2: 347, stroke: "rgba(255,255,255,0.035)" }, floor);
    for (let y = 52; y < 348; y += 40)
      svg("line", { x1: 13, y1: y, x2: 507, y2: y, stroke: "rgba(255,255,255,0.035)" }, floor);

    // the AGV, with its stack of plates
    const agv = svg("g", {}, root);
    svg("rect", { x: 28, y: 146, width: 56, height: 68, rx: 9, fill: "rgba(95,194,162,0.1)",
                  stroke: GREEN, "stroke-opacity": 0.6, "stroke-dasharray": "4 3" }, agv);
    [0, 1, 2, 3].forEach((i) =>
      svg("ellipse", { cx: 56, cy: 186 - i * 6, rx: 15, ry: 4.5, fill: CREAM,
                       stroke: GREEN, "stroke-width": 1 }, agv));
    const agvLabel = svg("text", { x: 56, y: 206, "text-anchor": "middle", fill: "rgba(255,255,255,0.6)",
                                   "font-family": "Inter, system-ui, sans-serif", "font-size": 9.5,
                                   "font-weight": 600, "letter-spacing": "0.08em" }, agv);
    agvLabel.textContent = "AGV";

    // tables, and the set positions on them
    this.plates = [];
    tables.forEach((t) => {
      const g = svg("g", { class: "table" }, root);
      svg("title", {}, g).textContent = "Table " + t.n + ": " + SET_ANGLES.length + " set positions";
      svg("circle", { class: "top", cx: t.cx, cy: t.cy, r: TABLE_R }, g);
      const label = svg("text", { x: t.cx, y: t.cy + 3.5, "text-anchor": "middle" }, g);
      label.textContent = "T" + t.n;
    });
    sets.forEach((s) =>
      this.plates.push(svg("circle", { class: "plate", cx: s.plate.x, cy: s.plate.y, r: PLATE_R }, root)));

    // the robot: a short fading trail, a soft halo, the body, and the plate it carries
    this.trail = svg("polyline", { fill: "none", stroke: GREEN, "stroke-opacity": 0.35, "stroke-width": 2,
                                   "stroke-linecap": "round", "stroke-linejoin": "round" }, root);
    this.robot = svg("g", {}, root);
    svg("circle", { r: 14, fill: GREEN, "fill-opacity": 0.18 }, this.robot);
    svg("circle", { r: 8, fill: GREEN, stroke: "#0F1513", "stroke-width": 1.5 }, this.robot);
    this.carried = svg("circle", { r: 4.2, fill: CREAM, stroke: "#0F1513", "stroke-width": 1 }, this.robot);

    this.counter = this.shadowRoot.getElementById("placed");
    this.button = this.shadowRoot.getElementById("toggle");
    this.button.addEventListener("click", () => {
      this.userPaused = !this.userPaused;
      this.button.textContent = this.userPaused ? "Play" : "Pause";
      this.button.setAttribute("aria-pressed", String(this.userPaused));
      this.sync();
    });

    this.reset();
  }

  reset() {
    this.next = 0;
    this.placed = 0;
    this.history = [];
    this.plates.forEach((p) => p.setAttribute("class", "plate"));
    this.counter.textContent = "0";
    this.enter("load");
    this.put(DOCK, true);
  }

  // Play only when on screen, not paused, and motion is welcome; otherwise show a still.
  sync() {
    const still = this.motionQuery && this.motionQuery.matches;
    this.toggleAttribute("data-still", !!still);
    if (still) {
      this.halt();
      this.plates.forEach((p) => p.setAttribute("class", "plate on"));
      this.counter.textContent = String(this.plates.length);
      this.trail.setAttribute("points", "");
      this.put(DOCK, false);
      this.wasStill = true;
      return;
    }
    if (this.wasStill) { this.wasStill = false; this.reset(); }
    const run = this.onScreen && !this.userPaused && !document.hidden;
    if (run && !this.frame) {
      // shift the current phase's clock so a pause does not make the robot jump
      if (this.pausedAt) this.phaseStart += performance.now() - this.pausedAt;
      this.pausedAt = 0;
      this.frame = requestAnimationFrame(this.tick);
    } else if (!run) {
      this.halt();
    }
  }

  halt() {
    if (this.frame) cancelAnimationFrame(this.frame);
    this.frame = 0;
    if (!this.pausedAt) this.pausedAt = performance.now();
  }

  enter(phase, path) {
    this.phase = phase;
    this.phaseStart = performance.now();
    this.path = path || null;
  }

  put(p, carrying) {
    this.robot.setAttribute("transform", "translate(" + p.x.toFixed(1) + " " + p.y.toFixed(1) + ")");
    this.carried.style.display = carrying ? "" : "none";
    this.history.push(p.x.toFixed(1) + "," + p.y.toFixed(1));
    if (this.history.length > 26) this.history.shift();
    this.trail.setAttribute("points", this.history.join(" "));
  }

  tick(now) {
    this.frame = requestAnimationFrame(this.tick);
    const s = (now - this.phaseStart) / 1000;
    const trip = this.trips[this.next];

    if (this.phase === "load") {
      this.put(DOCK, true);
      if (s >= LOAD_S) this.enter("out", trip);
    } else if (this.phase === "out" || this.phase === "back") {
      const dur = this.path.length / SPEED, u = Math.min(1, s / dur);
      const d = this.phase === "out" ? ease(u) * this.path.length : (1 - ease(u)) * this.path.length;
      this.put(along(this.path, d), this.phase === "out");
      if (u >= 1) {
        if (this.phase === "out") this.enter("place", this.path);
        else if (++this.next >= this.trips.length) this.enter("rest");
        else this.enter("load");
      }
    } else if (this.phase === "place") {
      const placing = s >= PLACE_S / 2;
      if (placing && !this.plates[this.next].classList.contains("on")) {
        this.plates[this.next].setAttribute("class", "plate on");
        this.counter.textContent = String(++this.placed);
      }
      this.put(this.sets[this.next].stand, !placing);
      if (s >= PLACE_S) this.enter("back", this.path);
    } else if (this.phase === "rest") {
      this.put(DOCK, false);
      if (s >= REST_S * 0.6) this.plates.forEach((p) => p.setAttribute("class", "plate off"));
      if (s >= REST_S) this.reset();
    }
  }
}

customElements.define("hall-demo", HallDemo);

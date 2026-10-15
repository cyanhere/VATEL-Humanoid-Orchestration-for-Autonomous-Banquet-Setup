// <pipeline-scenes>: one illustration per step of "How it works", shown one at a time beside
// the steps on the home page. It pictures each step rather than restating it.
//
//   highlight(n)  show step n (1-5); null leaves the current one up
//   autoplay(on)  cycle through the steps on its own (used on narrow screens, where the panel
//                 sits above the list rather than beside it)
//   fires "step-select" (detail: n) when a dot is pressed, so the page can scroll to that step
//
// Steps 1, 2 and 5 animate in CSS. Steps 3 and 4 (the robot driving and walking) are stepped
// frame by frame in script. Motion runs only for the step on show and only while the panel is on
// screen; visitors who prefer reduced motion see each step's finished picture.

const G = "#5FC2A2", CREAM = "#F4F7F5", INK = "#0F1513", WHITE = "#E8EEEA";
const NS = "http://www.w3.org/2000/svg";
let instances = 0;

/* ======================================================================== step 1
   A ceiling CCTV camera watches the hall; tables are segmented in its feed, and each one found
   lands on the 2-D map in the planner. */
function planScene() {
  // the hall as the camera sees it: two rows of tables in perspective
  const seen = [[100, 188, 12, 4], [134, 188, 12, 4], [168, 188, 12, 4],
                [88, 236, 17, 6], [134, 236, 17, 6], [180, 236, 17, 6]];
  // the same tables on the planner's map, in the same order
  const mapped = [[306, 196], [348, 196], [390, 196], [306, 234], [348, 234], [390, 234]];
  return `
    <g opacity=".9">
      <rect x="30" y="26" width="8" height="30" rx="2" fill="#2E3633"/>
      <g transform="rotate(28 40 50)">
        <rect x="38" y="42" width="58" height="22" rx="7" fill="${WHITE}"/>
        <rect x="88" y="44" width="10" height="18" rx="4" fill="#111412"/>
        <circle cx="96" cy="53" r="5" fill="#111412" stroke="${G}" stroke-width="1.4"/>
        <circle cx="48" cy="50" r="2" fill="${G}"/>
      </g>
    </g>
    <path d="M92 86L34 120M100 92L226 120" stroke="${G}" stroke-opacity=".35" stroke-dasharray="3 4"/>
    <rect x="30" y="120" width="200" height="160" rx="10" fill="#1A2320" stroke="rgba(255,255,255,.16)"/>
    <text x="40" y="136" fill="rgba(255,255,255,.5)" font-family="Inter, system-ui, sans-serif"
      font-size="8.5" font-weight="600" letter-spacing=".08em">HALL CCTV</text>
    <circle cx="218" cy="133" r="3" fill="#E0805F" class="rec"/>
    <path d="M60 268L200 268L174 160L86 160Z" fill="#26322D" stroke="rgba(255,255,255,.1)"/>
    ${seen.map(([x, y, rx, ry], i) => `
      <ellipse cx="${x}" cy="${y + ry * 0.9}" rx="${rx * 0.8}" ry="${ry * 0.6}" fill="rgba(0,0,0,.35)"/>
      <ellipse cx="${x}" cy="${y}" rx="${rx}" ry="${ry}" fill="${WHITE}"/>
      <ellipse class="mask" style="--i:${i}" cx="${x}" cy="${y}" rx="${rx + 3}" ry="${ry + 2.5}"
        fill="${G}" fill-opacity=".38" stroke="${G}" stroke-width="1.4"/>`).join("")}
    <g clip-path="url(#feed-clip)">
      <rect class="scan" x="31" y="148" width="198" height="3" fill="${G}" opacity=".8"/>
    </g>
    <clipPath id="feed-clip"><rect x="30" y="140" width="200" height="140"/></clipPath>

    <path d="M234 200H254" stroke="${G}" stroke-width="1.6" stroke-dasharray="3 4"/>
    <path d="M250 195L256 200L250 205" fill="none" stroke="${G}" stroke-width="1.6" stroke-linecap="round" stroke-linejoin="round"/>
    <circle class="flow" cx="236" cy="200" r="3" fill="${G}"/>

    <rect x="262" y="120" width="186" height="160" rx="16" fill="#1B2420" stroke="rgba(255,255,255,.16)"/>
    <rect x="272" y="130" width="166" height="140" rx="6" fill="${CREAM}"/>
    <path d="M272 136a6 6 0 0 1 6-6h154a6 6 0 0 1 6 6v8H272z" fill="#111412"/>
    <circle cx="281" cy="137" r="2.6" fill="${G}"/>
    <rect x="288" y="135" width="30" height="4" rx="2" fill="rgba(255,255,255,.4)"/>
    <rect x="400" y="134" width="32" height="6" rx="3" fill="#1D6152"/>
    <rect x="282" y="152" width="146" height="108" fill="#fff" stroke="#111412" stroke-width="1.3"/>
    <rect x="326" y="158" width="58" height="16" fill="#E1E8E4" stroke="#86908B" stroke-width=".7"/>
    ${mapped.map(([x, y], i) => `
      <g transform="translate(${x} ${y})"><g class="pop" style="--i:${i}">
        ${Array.from({ length: 6 }, (_, k) => {
          const a = (k / 6) * Math.PI * 2;
          return `<circle cx="${(15 * Math.cos(a)).toFixed(1)}" cy="${(15 * Math.sin(a)).toFixed(1)}" r="2.3"
            fill="#EFE5CF" stroke="#BE9B57" stroke-width=".5"/>`;
        }).join("")}
        <circle r="10.5" fill="#F6FAF8" stroke="#5E8F80" stroke-width="1.2"/>
      </g></g>`).join("")}`;
}

/* ======================================================================== step 2
   The plan flying from the iPad to the robot, item by item. */
const PACKETS = [
  ["Map", `<path d="M-5.5-4.5l3.7 1.5 3.6-1.5 3.7 1.5v8.5L1.8 4.5l-3.6 1.5-3.7-1.5z" fill="none" stroke="${CREAM}" stroke-width="1.3" stroke-linejoin="round"/><path d="M-1.8-3v7.5M1.8-4.5V3" stroke="${CREAM}" stroke-width="1"/>`],
  ["Tables", `<circle r="5" fill="none" stroke="${CREAM}" stroke-width="1.5"/><circle r="1.4" fill="${CREAM}"/>`],
  ["Plates", `<circle r="5.5" fill="${CREAM}"/><circle r="3" fill="none" stroke="#1D6152" stroke-width="1"/>`],
  ["Cutlery", `<path d="M-3-6v5.5M-4.8-6v3.2a1.8 1.8 0 0 0 3.6 0V-6M-3-0.5V6M3.5-6c-1.8 1-2 4-1.6 7h1.6V6" fill="none" stroke="${CREAM}" stroke-width="1.2" stroke-linecap="round" stroke-linejoin="round"/>`],
  ["Number", `<text y="4" text-anchor="middle" fill="${CREAM}" font-family="Inter, system-ui, sans-serif" font-size="11" font-weight="700">#</text>`],
  ["Location", `<path d="M0 6C-4.5 1-5-1.8-5-2.6a5 5 0 0 1 10 0c0 .8-.5 3.6-5 8.6z" fill="${CREAM}"/><circle cy="-2.6" r="1.8" fill="#1D6152"/>`],
];

function handoffScene() {
  const mini = [[78, 150], [106, 150], [134, 150], [78, 180], [106, 180], [134, 180]]
    .map(([x, y]) => `<circle cx="${x}" cy="${y}" r="8" fill="#F6FAF8" stroke="#5E8F80" stroke-width="1.2"/>`).join("");
  // where each packet rests in the reduced-motion still: two columns above the arc
  const still = [[204, 82], [278, 82], [204, 108], [278, 108], [204, 134], [278, 134]];
  return `
    <rect x="40" y="112" width="132" height="104" rx="12" fill="#1B2420" stroke="rgba(255,255,255,.16)"/>
    <rect x="48" y="120" width="116" height="88" rx="5" fill="${CREAM}"/>
    ${mini}
    <path d="M176 160C230 80 290 80 330 140" fill="none" stroke="${G}" stroke-opacity=".45"
      stroke-width="1.6" stroke-dasharray="4 6" stroke-linecap="round"/>
    ${PACKETS.map(([label, icon], i) => {
      const w = 30 + label.length * 5.8;
      return `<g class="packet" style="--i:${i};--sx:${still[i][0]}px;--sy:${still[i][1]}px">
        <rect x="${-w / 2}" y="-11" width="${w}" height="22" rx="11" fill="#1D6152" stroke="${G}"/>
        <g transform="translate(${-w / 2 + 13} 0)">${icon}</g>
        <text x="${-w / 2 + 23}" y="3.6" fill="${CREAM}" font-family="Inter, system-ui, sans-serif"
          font-size="10" font-weight="600">${label}</text>
      </g>`;
    }).join("")}
    <g>
      <rect x="352" y="80" width="28" height="18" rx="5" fill="#111412" stroke="${G}" stroke-opacity=".5"/>
      <rect x="336" y="96" width="60" height="66" rx="28" fill="${WHITE}"/>
      <rect x="344" y="116" width="44" height="20" rx="10" fill="#111412"/>
      <rect class="visor" x="344" y="116" width="44" height="20" rx="10" fill="${G}"/>
      <rect x="356" y="160" width="20" height="12" fill="#9AA6A0"/>
      <path d="M318 178Q366 164 414 178L406 266Q366 276 326 266Z" fill="${WHITE}"/>
      <circle cx="366" cy="210" r="6" fill="${G}"/>
    </g>`;
}

/* ======================================================================== step 3
   Top view: the robot drives the aisles; its lidar beam turns the hall from unmapped to mapped,
   the planned tables move to where they really are, and each set position lights up in turn. */
const NAV_TABLES = [[150, 110, 12, -10], [270, 110, -12, 12], [390, 118, -10, -16], [212, 240, 14, -8], [336, 248, -10, 12]];
const NAV_ROUTE = [[70, 296], [70, 172], [150, 172], [150, 148], [150, 172], [212, 172], [212, 198], [212, 172],
                   [270, 172], [270, 148], [270, 172], [336, 172], [336, 206], [336, 172], [394, 172], [394, 154]];
const NAV_STOPS = [3, 6, 9, 12, 15];      // route points that are set positions

function navigateScene(id) {
  const slices = [0, 1, 2, 3].map((k) => {
    const a0 = (-k * 11 * Math.PI) / 180, a1 = (-(k + 1) * 11 * Math.PI) / 180, r = 92;
    return `<path d="M0 0L${(r * Math.cos(a0)).toFixed(1)} ${(r * Math.sin(a0)).toFixed(1)}A${r} ${r} 0 0 0
      ${(r * Math.cos(a1)).toFixed(1)} ${(r * Math.sin(a1)).toFixed(1)}Z" fill="${G}" fill-opacity="${[0.28, 0.16, 0.08, 0.04][k]}"/>`;
  }).join("");
  return `
    <defs>
      <pattern id="hatch-${id}" width="7" height="7" patternUnits="userSpaceOnUse" patternTransform="rotate(45)">
        <line x1="0" y1="0" x2="0" y2="7" stroke="rgba(255,255,255,.07)" stroke-width="2"/>
      </pattern>
      <pattern id="grid-${id}" width="24" height="24" patternUnits="userSpaceOnUse">
        <path d="M24 0H0V24" fill="none" stroke="rgba(95,194,162,.18)" stroke-width="1"/>
      </pattern>
      <mask id="seen-${id}" maskUnits="userSpaceOnUse" x="0" y="0" width="480" height="360">
        <rect width="480" height="360" fill="#000"/>
        <g class="seen" fill="#fff"></g>
      </mask>
      <clipPath id="hall-${id}"><rect x="36" y="36" width="408" height="276" rx="10"/></clipPath>
    </defs>
    <g clip-path="url(#hall-${id})">
      <rect x="36" y="36" width="408" height="276" fill="#121815"/>
      <rect x="36" y="36" width="408" height="276" fill="url(#hatch-${id})"/>
      <g mask="url(#seen-${id})">
        <rect x="36" y="36" width="408" height="276" fill="#183A30"/>
        <rect x="36" y="36" width="408" height="276" fill="url(#grid-${id})"/>
      </g>
    </g>
    <rect x="36" y="36" width="408" height="276" rx="10" fill="none" stroke="rgba(255,255,255,.18)"/>
    ${NAV_TABLES.map(([x, y, dx, dy]) => `
      <g transform="translate(${x} ${y})">
        <g class="ghost" style="--dx:${dx}px;--dy:${dy}px">
          <circle r="24" fill="none" stroke="rgba(255,255,255,.45)" stroke-width="1.3" stroke-dasharray="4 4"/>
        </g>
        <circle class="real" r="24" fill="rgba(95,194,162,.14)" stroke="${G}" stroke-width="1.6"/>
      </g>`).join("")}
    <polyline class="trail" fill="none" stroke="${G}" stroke-opacity=".7" stroke-width="2"
      stroke-dasharray="3 4" stroke-linecap="round" stroke-linejoin="round"/>
    ${NAV_STOPS.map((p) => `<g transform="translate(${NAV_ROUTE[p][0]} ${NAV_ROUTE[p][1]})"><g class="stop">
        <circle r="6.5" fill="none" stroke="rgba(255,255,255,.4)" stroke-width="1.5"/>
        <circle class="dot" r="4" fill="${G}"/></g></g>`).join("")}
    <g clip-path="url(#hall-${id})"><g class="lidar">
      <circle r="92" fill="none" stroke="${G}" stroke-opacity=".18" stroke-dasharray="2 5"/>
      <g class="beam">${slices}<line x1="0" y1="0" x2="92" y2="0" stroke="${G}" stroke-width="1.6"/></g>
    </g></g>
    <g class="bot">
      <circle r="13" fill="${G}" fill-opacity=".2"/>
      <circle r="8" fill="${G}" stroke="${INK}" stroke-width="1.5"/>
    </g>
    <g font-family="Inter, system-ui, sans-serif" font-size="9.5" font-weight="600" fill="rgba(255,255,255,.6)">
      <rect x="40" y="322" width="12" height="9" rx="2" fill="#183A30" stroke="${G}" stroke-opacity=".5"/>
      <text x="57" y="330">Mapped</text>
      <rect x="106" y="322" width="12" height="9" rx="2" fill="url(#hatch-${id})" stroke="rgba(255,255,255,.3)"/>
      <text x="123" y="330">Not yet mapped</text>
      <circle cx="214" cy="326.5" r="4" fill="${G}"/>
      <text x="223" y="330">Set position reached</text>
    </g>`;
}

class NavRun {
  constructor(root) {
    this.seen = root.querySelector(".seen");
    this.trail = root.querySelector(".trail");
    this.lidar = root.querySelector(".lidar");
    this.bot = root.querySelector(".bot");
    this.beam = root.querySelector(".beam");
    this.tables = Array.from(root.querySelectorAll(".real")).map((real, i) => ({
      real: real, ghost: real.parentNode.querySelector(".ghost"), x: NAV_TABLES[i][0], y: NAV_TABLES[i][1] }));
    this.stops = Array.from(root.querySelectorAll(".stop"));
    this.reset();
  }

  reset() {
    this.seg = 0; this.along = 0; this.pause = 0; this.hold = 0; this.angle = 0;
    this.seen.textContent = ""; this.sinceMark = 99; this.points = [];
    this.tables.forEach((t) => { t.real.classList.remove("found"); t.ghost.classList.remove("found"); });
    this.stops.forEach((s) => s.classList.remove("served"));
    this.place(NAV_ROUTE[0][0], NAV_ROUTE[0][1]);
  }

  // what the reduced-motion still shows: the whole hall mapped and every set position reached
  finish() {
    this.reset();
    const r = document.createElementNS(NS, "rect");
    r.setAttribute("width", "480"); r.setAttribute("height", "360");
    this.seen.appendChild(r);
    this.tables.forEach((t) => { t.real.classList.add("found"); t.ghost.classList.add("found"); });
    this.stops.forEach((s) => s.classList.add("served"));
    this.points = NAV_ROUTE.map((p) => p.join(","));
    this.trail.setAttribute("points", this.points.join(" "));
    const end = NAV_ROUTE[NAV_ROUTE.length - 1];
    this.angle = -40;
    this.place(end[0], end[1]);
  }

  place(x, y) {
    this.x = x; this.y = y;
    this.lidar.setAttribute("transform", `translate(${x.toFixed(1)} ${y.toFixed(1)})`);
    this.bot.setAttribute("transform", `translate(${x.toFixed(1)} ${y.toFixed(1)})`);
    this.beam.setAttribute("transform", `rotate(${this.angle.toFixed(1)})`);
    if (this.sinceMark >= 12) {       // the floor the beam has swept becomes "mapped"
      const c = document.createElementNS(NS, "circle");
      c.setAttribute("cx", x.toFixed(1)); c.setAttribute("cy", y.toFixed(1)); c.setAttribute("r", "88");
      this.seen.appendChild(c);
      this.sinceMark = 0;
      this.points.push(x.toFixed(1) + "," + y.toFixed(1));
      this.trail.setAttribute("points", this.points.join(" "));
      this.tables.forEach((t) => {
        if (Math.hypot(t.x - x, t.y - y) < 110) { t.real.classList.add("found"); t.ghost.classList.add("found"); }
      });
    }
  }

  step(dt) {
    this.angle = (this.angle + dt * 320) % 360;
    if (this.seg >= NAV_ROUTE.length - 1) {          // all set positions reached: hold, then again
      this.hold += dt;
      this.beam.setAttribute("transform", `rotate(${this.angle.toFixed(1)})`);
      if (this.hold > 2.4) this.reset();
      return;
    }
    if (this.pause > 0) {
      this.pause -= dt;
      this.beam.setAttribute("transform", `rotate(${this.angle.toFixed(1)})`);
      return;
    }
    let move = 95 * dt;
    while (move > 0 && this.seg < NAV_ROUTE.length - 1) {
      const [x0, y0] = NAV_ROUTE[this.seg], [x1, y1] = NAV_ROUTE[this.seg + 1];
      const len = Math.hypot(x1 - x0, y1 - y0), adv = Math.min(move, len - this.along);
      this.along += adv; move -= adv; this.sinceMark += adv;
      if (this.along >= len - 1e-6) {
        this.seg++; this.along = 0;
        const s = NAV_STOPS.indexOf(this.seg);
        if (s >= 0) { this.stops[s].classList.add("served"); this.pause = 0.6; this.sinceMark = 99; break; }
      }
    }
    const i = Math.min(this.seg, NAV_ROUTE.length - 2);
    const [x0, y0] = NAV_ROUTE[i], [x1, y1] = NAV_ROUTE[i + 1];
    const len = Math.hypot(x1 - x0, y1 - y0) || 1, u = this.seg >= NAV_ROUTE.length - 1 ? 1 : this.along / len;
    this.place(x0 + (x1 - x0) * u, y0 + (y1 - y0) * u);
  }
}

/* ======================================================================== step 4
   Side view of the humanoid: reach down to the AGV's stack, lift one plate, turn, walk it to the
   table on two legs, set it down, turn, walk back. Three trips, then the table is cleared. */
const FLOOR = 292;
const ARM = [36, 32];                       // upper arm, forearm: long, as on the R1
const SHOULDER = [2, -132];
const HIP = [0, -78], THIGH = 36, SHIN = 36;
const STACK_X = 44, STACK_TOP = [210, 205, 200, 195, 190, 185];
const SPOTS = [340, 368, 396], SPOT_Y = 176, PICK_X = 104, REACH = 52;
const REST = [6, 12], CARRY = [-52, -48];

const dir = (a) => { const r = (a * Math.PI) / 180; return [-Math.sin(r), Math.cos(r)]; };

// the hand, in robot coordinates, for arm angles [upper, forearm (relative)]
function handAt(p) {
  const d1 = dir(p[0]), d2 = dir(p[0] + p[1]);
  return [SHOULDER[0] + ARM[0] * d1[0] + ARM[1] * d2[0], SHOULDER[1] + ARM[0] * d1[1] + ARM[1] * d2[1]];
}

// arm angles that put the hand on a point (robot coordinates), elbow bent down and back
function reachFor(tx, ty) {
  const vx = tx - SHOULDER[0], vy = ty - SHOULDER[1];
  const d = Math.min(Math.hypot(vx, vy), ARM[0] + ARM[1] - 0.5);
  const cosK = (d * d - ARM[0] ** 2 - ARM[1] ** 2) / (2 * ARM[0] * ARM[1]);
  const k = -Math.acos(Math.max(-1, Math.min(1, cosK)));              // forearm folds forward
  const aT = (Math.atan2(-vx, vy) * 180) / Math.PI;                  // direction to the target
  const beta = (Math.atan2(ARM[1] * Math.sin(k), ARM[0] + ARM[1] * Math.cos(k)) * 180) / Math.PI;
  return [aT - beta, (k * 180) / Math.PI];
}

const lerp = (a, b, u) => a + (b - a) * u;
const ease = (u) => (u < 0.5 ? 2 * u * u : 1 - Math.pow(-2 * u + 2, 2) / 2);
const mix = (p, q, u) => [lerp(p[0], q[0], u), lerp(p[1], q[1], u)];

function limb(cls, w, len, joint) {
  return `<rect x="${-w / 2}" y="0" width="${w}" height="${len}" rx="${w / 2}" class="${cls}"/>` +
    (joint ? `<circle cy="${len}" r="${w / 2 - 0.5}" fill="#2E3633"/>` : "");
}

function robotSide() {
  const leg = (side) => `
    <g class="${side}-hip"><g class="thigh">${limb("limb-" + side, 12, THIGH, true)}
      <g class="${side}-knee" transform="translate(0 ${THIGH})">${limb("limb-" + side, 10, SHIN, false)}
        <g class="${side}-foot" transform="translate(0 ${SHIN})">
          <rect x="-7" y="-3" width="22" height="7" rx="3" class="limb-${side}"/></g>
      </g></g></g>`;
  const arm = (side) => `
    <g class="${side}-shoulder">${limb("limb-" + side, 10, ARM[0], true)}
      <g class="${side}-elbow" transform="translate(0 ${ARM[0]})">${limb("limb-" + side, 9, ARM[1] - 4, false)}
        <circle cy="${ARM[1] - 2}" r="5" class="limb-${side}"/></g></g>`;
  return `
    <ellipse class="shadow" cx="0" cy="${FLOOR}" rx="26" ry="4" fill="rgba(0,0,0,.4)"/>
    <g class="body">
      ${arm("far")}${leg("far")}
      <rect x="-12" y="-90" width="24" height="16" rx="6" fill="#D3DBD7"/>
      <rect x="-21" y="-136" width="7" height="36" rx="3" fill="#C9D3CE"/>
      <rect x="-16" y="-142" width="34" height="56" rx="13" fill="${WHITE}"/>
      <circle cx="12" cy="-121" r="3" fill="${G}"/>
      <rect x="-5" y="-150" width="10" height="10" fill="#9AA6A0"/>
      <rect x="-14" y="-180" width="30" height="33" rx="14" fill="${WHITE}"/>
      <rect x="2" y="-171" width="15" height="13" rx="6" fill="#111412"/>
      <rect x="-9" y="-191" width="18" height="11" rx="4" fill="#111412" stroke="${G}" stroke-opacity=".6"/>
      ${leg("near")}${arm("near")}
    </g>`;
}

function transferScene() {
  return `
    <line x1="0" y1="${FLOOR}" x2="480" y2="${FLOOR}" stroke="rgba(255,255,255,.18)" stroke-width="1.4"/>
    <rect x="4" y="214" width="80" height="68" rx="8" fill="#1A231F" stroke="${G}" stroke-opacity=".55"/>
    <rect x="14" y="232" width="28" height="4" rx="2" fill="${G}"/>
    <rect x="14" y="244" width="58" height="1" fill="${G}" opacity=".25"/>
    <circle cx="20" cy="286" r="7" fill="#0B100E" stroke="${G}" stroke-opacity=".35"/>
    <circle cx="68" cy="286" r="7" fill="#0B100E" stroke="${G}" stroke-opacity=".35"/>
    ${STACK_TOP.map((y) => `<ellipse class="stacked" cx="${STACK_X}" cy="${y}" rx="26" ry="5" fill="${CREAM}"
      stroke="#9DBFB3" stroke-width="1"/>`).join("")}
    <rect x="318" y="180" width="136" height="8" rx="3" fill="${WHITE}"/>
    <rect x="330" y="188" width="6" height="${FLOOR - 188}" fill="#9AA6A0"/>
    <rect x="436" y="188" width="6" height="${FLOOR - 188}" fill="#9AA6A0"/>
    ${SPOTS.map((x) => `<ellipse class="spot" cx="${x}" cy="${SPOT_Y}" rx="15" ry="4" fill="${CREAM}" stroke="#9DBFB3"/>`).join("")}
    <g class="robot">${robotSide()}</g>
    <ellipse class="carried" rx="15" ry="4" fill="${CREAM}" stroke="#9DBFB3"/>`;
}

class WalkRun {
  constructor(root) {
    const q = (s) => root.querySelector(s);
    this.robot = q(".robot"); this.body = q(".body"); this.shadow = q(".shadow");
    this.j = {};
    ["near", "far"].forEach((s) => ["hip", "knee", "foot", "shoulder", "elbow"].forEach((p) => {
      this.j[s + p] = q("." + s + "-" + p);
    }));
    this.stack = Array.from(root.querySelectorAll(".stacked"));
    this.spots = Array.from(root.querySelectorAll(".spot"));
    this.carried = q(".carried");
    this.reset();
  }

  reset() {
    this.trip = 0; this.t = 0; this.phase = 0; this.gait = 0;
    this.x = PICK_X; this.face = -1;
    this.stack.forEach((p) => (p.style.opacity = 1));
    this.spots.forEach((p) => (p.style.opacity = 0));
    this.holding = false;
    this.plan();
    this.pose(REST, REST, 0);
  }

  // one trip, as phases of [name, seconds]
  plan() {
    const placeX = SPOTS[this.trip] - REACH, walk = (placeX - PICK_X) / 72;
    this.phases = [["reach", 0.6], ["lift", 0.4], ["turn", 0.35], ["out", walk], ["place", 0.6],
                   ["let go", 0.4], ["turn back", 0.35], ["back", walk], ["pause", 0.3]];
    this.placeX = placeX;
  }

  // robot coordinates of a world point, for the arm to reach
  local(wx, wy, bob) { return [(wx - this.x) / this.face, wy - FLOOR - bob]; }

  step(dt) {
    this.t += dt;
    let [name, dur] = this.phases[this.phase];
    while (this.t >= dur) {
      this.t -= dur;
      this.enter(this.phase + 1);
      if (!this.phases[this.phase]) return;
      [name, dur] = this.phases[this.phase];
    }
    const u = ease(Math.min(1, this.t / dur));
    const stackTop = STACK_TOP[5 - this.trip];
    let arms = REST, swing = false, walking = false, flip = this.face;
    if (name === "reach") arms = mix(REST, reachFor(...this.local(STACK_X, stackTop - 2, 0)), u);
    else if (name === "lift") arms = mix(reachFor(...this.local(STACK_X, stackTop - 2, 0)), CARRY, u);
    else if (name === "turn") { arms = CARRY; flip = lerp(-1, 1, u); }
    else if (name === "out") { arms = CARRY; walking = true; this.x = lerp(PICK_X, this.placeX, Math.min(1, this.t / dur)); }
    else if (name === "place") arms = mix(CARRY, reachFor(...this.local(SPOTS[this.trip], SPOT_Y - 2, 0)), u);
    else if (name === "let go") arms = mix(reachFor(...this.local(SPOTS[this.trip], SPOT_Y - 2, 0)), REST, u);
    else if (name === "turn back") flip = lerp(1, -1, u);
    else if (name === "back") { walking = swing = true; this.x = lerp(this.placeX, PICK_X, Math.min(1, this.t / dur)); }
    else if (name === "hold") { /* the table stays set for a moment */ }
    if (walking) this.gait += (72 * dt / 40) * Math.PI;      // one step every 40 px
    else this.gait *= 0.8;                                     // legs settle when standing
    this.pose(arms, arms, walking ? this.gait : 0, swing, flip);
  }

  enter(i) {
    const was = this.phases[this.phase][0];
    if (was === "reach") { this.stack[5 - this.trip].style.opacity = 0; this.holding = true; }
    if (was === "turn") this.face = 1;
    if (was === "place") { this.holding = false; this.spots[this.trip].style.opacity = 1; }
    if (was === "turn back") this.face = -1;
    this.phase = i;
    if (!this.phases[i]) {
      if (this.trip < SPOTS.length - 1) { this.trip++; this.phase = 0; this.plan(); }
      else if (was !== "hold") { this.phases = [["hold", 1.8]]; this.phase = 0; }
      else this.reset();
    }
  }

  // set every joint: arm angles, gait phase (0 = standing), arm swing, facing (-1..1)
  pose(near, far, gait, swing, flip) {
    const face = flip === undefined ? this.face : flip;
    const s = Math.sin(gait), c = Math.cos(gait);
    const hipN = gait ? -16 * s : 0, hipF = gait ? 16 * s : 0;
    const kneeN = gait ? 4 + 24 * Math.max(0, c) : 0, kneeF = gait ? 4 + 24 * Math.max(0, -c) : 0;
    // keep the standing foot on the floor: the body drops by what the legs lose in height
    const reach = (h, k) => THIGH * Math.cos((h * Math.PI) / 180) + SHIN * Math.cos(((h + k) * Math.PI) / 180);
    const bob = THIGH + SHIN - Math.max(reach(hipN, kneeN), reach(hipF, kneeF));
    const set = (el, tr) => el.setAttribute("transform", tr);
    set(this.j.nearhip, `translate(${HIP[0]} ${HIP[1]}) rotate(${hipN.toFixed(1)})`);
    set(this.j.farhip, `translate(${HIP[0] - 3} ${HIP[1]}) rotate(${hipF.toFixed(1)})`);
    set(this.j.nearknee, `translate(0 ${THIGH}) rotate(${kneeN.toFixed(1)})`);
    set(this.j.farknee, `translate(0 ${THIGH}) rotate(${kneeF.toFixed(1)})`);
    set(this.j.nearfoot, `translate(0 ${SHIN}) rotate(${(-(hipN + kneeN)).toFixed(1)})`);
    set(this.j.farfoot, `translate(0 ${SHIN}) rotate(${(-(hipF + kneeF)).toFixed(1)})`);
    const swingN = swing ? 14 * s : 0;
    const armN = [near[0] + swingN, near[1]], armF = [far[0] - swingN, far[1]];
    set(this.j.nearshoulder, `translate(${SHOULDER[0]} ${SHOULDER[1]}) rotate(${armN[0].toFixed(1)})`);
    set(this.j.farshoulder, `translate(${SHOULDER[0] - 4} ${SHOULDER[1]}) rotate(${armF[0].toFixed(1)})`);
    set(this.j.nearelbow, `translate(0 ${ARM[0]}) rotate(${armN[1].toFixed(1)})`);
    set(this.j.farelbow, `translate(0 ${ARM[0]}) rotate(${armF[1].toFixed(1)})`);
    set(this.body, `translate(0 ${(FLOOR + bob).toFixed(1)})`);
    set(this.robot, `translate(${this.x.toFixed(1)} 0) scale(${face.toFixed(3)} 1)`);
    this.shadow.setAttribute("cy", (FLOOR - 0.5).toFixed(1));
    const h = handAt(armN);
    this.carried.style.opacity = this.holding ? 1 : 0;
    this.carried.setAttribute("cx", (this.x + face * (h[0] + 2)).toFixed(1));
    this.carried.setAttribute("cy", (FLOOR + bob + h[1] - 4).toFixed(1));
  }

  // the reduced-motion still: carrying a plate toward the table
  finish() {
    this.reset();
    this.stack[5].style.opacity = 0;
    this.holding = true; this.face = 1; this.x = 250;
    this.pose(CARRY, CARRY, 0.9, false, 1);
  }
}

/* ======================================================================== step 5
   A set table, one place allowed to miss, and nothing broken. */
function doneScene() {
  const places = Array.from({ length: 10 }, (_, i) => {
    const a = -Math.PI / 2 + (i / 10) * Math.PI * 2, x = 240 + 56 * Math.cos(a), y = 168 + 56 * Math.sin(a);
    return i === 7
      ? `<circle cx="${x.toFixed(1)}" cy="${y.toFixed(1)}" r="11" fill="none" stroke="rgba(255,255,255,.35)"
          stroke-width="1.3" stroke-dasharray="3 3"/>`
      : `<g transform="translate(${x.toFixed(1)} ${y.toFixed(1)})"><g class="pop" style="--i:${i}">
          <circle r="11" fill="${CREAM}" stroke="${G}" stroke-width="1.4"/>
          <circle r="6.5" fill="none" stroke="#CFE0D8" stroke-width="1"/></g></g>`;
  }).join("");
  const chip = (x, w, text) => `
    <rect x="${x}" y="276" width="${w}" height="28" rx="14" fill="rgba(95,194,162,.12)" stroke="${G}" stroke-opacity=".5"/>
    <text x="${x + w / 2}" y="294.5" text-anchor="middle" fill="${CREAM}" font-family="Inter, system-ui, sans-serif"
      font-size="12.5" font-weight="600">${text}</text>`;
  return `
    <circle cx="240" cy="168" r="80" fill="rgba(95,194,162,.07)" stroke="${G}" stroke-opacity=".55" stroke-width="1.6"/>
    ${places}
    <g transform="translate(240 168)"><g class="badge">
      <circle r="23" fill="${G}"/>
      <path d="M-10 0l6.5 6.5L11-8" fill="none" stroke="${INK}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>
    </g></g>
    ${chip(150, 84, "9 of 10 set")}${chip(246, 84, "0 broken")}`;
}

const SCENES = [
  ["A ceiling CCTV camera watches the banquet hall; the tables in its feed are segmented, and each appears on the 2-D map in the VATEL Planner on an iPad.", planScene, 6],
  ["The plan travels from the iPad to the robot item by item: the map, tables, plates, cutlery, numbers and locations.", handoffScene, 0],
  ["Top view of the hall: the robot drives the aisles while its spinning lidar beam turns the hall from unmapped to mapped, moves the planned tables to where they really are, and reaches five set positions in turn.", navigateScene, 0],
  ["Side view: the humanoid takes one plate off the AGV's stack, turns, walks it to the table, sets it down and walks back, three times.", transferScene, -20],
  ["A round table seen from above with nine of its ten places set, one left empty, and a check mark.", doneScene, -10],
];

class PipelineScenes extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    this.current = 1;
    this.uid = ++instances;
    this.frame = this.frame.bind(this);
  }

  connectedCallback() {
    if (this.built) return;
    this.built = true;
    this.shadowRoot.innerHTML = `
      <div class="panel">
        <span class="count" aria-hidden="true">Step <b>1</b> / ${SCENES.length}</span>
        <svg viewBox="0 0 480 360" role="img" aria-label="${SCENES[0][0]}">
          ${SCENES.map(([, draw, nudge], i) => `<g class="scene" data-n="${i + 1}">
            <g transform="translate(0 ${nudge})">${draw(this.uid)}</g></g>`).join("")}
        </svg>
        <div class="dots" role="group" aria-label="Steps">
          ${SCENES.map((_, i) => `<button type="button" data-n="${i + 1}" aria-label="Step ${i + 1}"></button>`).join("")}
        </div>
      </div>
      <style>${STYLE}</style>`;
    this.svg = this.shadowRoot.querySelector("svg");
    const scene = (n) => this.shadowRoot.querySelector('.scene[data-n="' + n + '"]');
    this.runs = { 3: new NavRun(scene(3)), 4: new WalkRun(scene(4)) };
    this.still = window.matchMedia("(prefers-reduced-motion: reduce)");
    this.shadowRoot.querySelectorAll(".dots button").forEach((b) =>
      b.addEventListener("click", () => {
        const n = Number(b.dataset.n);
        this.show(n);
        this.hold = Date.now() + 9000;          // a chosen step stays up a while, even on autoplay
        this.dispatchEvent(new CustomEvent("step-select", { detail: n, bubbles: true, composed: true }));
      }));
    this.io = new IntersectionObserver((e) => {
      this.onScreen = e[0].isIntersecting;
      this.classList.toggle("off-screen", !this.onScreen);
      this.sync();
    });
    this.io.observe(this);
    this.onVisibility = () => this.sync();
    document.addEventListener("visibilitychange", this.onVisibility);
    this.show(1);
  }

  disconnectedCallback() {
    if (this.io) this.io.disconnect();
    document.removeEventListener("visibilitychange", this.onVisibility);
    this.autoplay(false);
    this.halt();
  }

  highlight(n) {
    if (n) this.show(n);
  }

  show(n) {
    if (n === this.current && this.shadowRoot.querySelector(".scene.on")) return;
    this.current = n;
    this.shadowRoot.querySelectorAll(".scene").forEach((g) => g.classList.remove("on"));
    const scene = this.shadowRoot.querySelector('.scene[data-n="' + n + '"]');
    void scene.getBoundingClientRect();         // restart the step's animations from the top
    scene.classList.add("on");
    this.shadowRoot.querySelectorAll(".dots button").forEach((b) => {
      const on = Number(b.dataset.n) === n;
      b.classList.toggle("on", on);
      b.setAttribute("aria-current", on ? "step" : "false");
    });
    this.shadowRoot.querySelector(".count b").textContent = String(n);
    this.svg.setAttribute("aria-label", SCENES[n - 1][0]);
    const run = this.runs[n];
    if (run) this.still.matches ? run.finish() : run.reset();
    this.sync();
  }

  // the scripted steps run frame by frame, only when shown, on screen and welcome to move
  sync() {
    const run = this.runs[this.current];
    const go = run && this.onScreen && !document.hidden && !this.still.matches && !this.frozen;
    if (go && !this.raf) { this.last = 0; this.raf = requestAnimationFrame(this.frame); }
    else if (!go) this.halt();
  }

  halt() {
    if (this.raf) cancelAnimationFrame(this.raf);
    this.raf = 0;
  }

  frame(now) {
    this.raf = requestAnimationFrame(this.frame);
    const dt = this.last ? Math.min(0.05, (now - this.last) / 1000) : 0;
    this.last = now;
    const run = this.runs[this.current];
    if (run) run.step(dt);
  }

  // for checking frames: stop the clock and play step n's script forward by t seconds
  seek(n, t) {
    this.frozen = true;
    this.halt();
    this.show(n);
    const run = this.runs[n];
    if (run) { run.reset(); for (let s = 0; s < t; s += 1 / 60) run.step(1 / 60); }
  }

  autoplay(on) {
    clearInterval(this.timer);
    this.timer = 0;
    if (!on || this.still.matches) return;
    this.timer = setInterval(() => {
      if (!this.onScreen || document.hidden || Date.now() < (this.hold || 0)) return;
      this.show((this.current % SCENES.length) + 1);
    }, 7000);
  }
}

const STYLE = `
  :host { display: block; }
  .panel {
    position: relative;
    overflow: hidden;
    border: 1px solid #1E2824;
    border-radius: 18px;
    background: radial-gradient(520px 260px at 85% -10%, rgba(95,194,162,.2), transparent 70%), ${INK};
    box-shadow: 0 18px 40px rgba(15, 21, 19, .16);
  }
  svg { display: block; width: 100%; height: auto; padding: 18px 10px 44px; box-sizing: border-box; }
  .count {
    position: absolute; top: 14px; left: 18px;
    font: 600 11px Inter, system-ui, sans-serif; letter-spacing: .12em; text-transform: uppercase;
    color: rgba(255,255,255,.5);
  }
  .count b { color: ${G}; }
  .dots { position: absolute; right: 0; bottom: 12px; left: 0; display: flex; justify-content: center; gap: 2px; }
  .dots button {
    position: relative; width: 26px; height: 24px; padding: 0; border: 0;
    background: none; cursor: pointer; -webkit-tap-highlight-color: transparent;
  }
  .dots button::after {
    content: ""; position: absolute; top: 8px; left: 9px; width: 8px; height: 8px; border-radius: 99px;
    background: rgba(255,255,255,.28); transition: width .3s, left .3s, background .3s;
  }
  .dots button:hover::after { background: rgba(255,255,255,.55); }
  .dots button.on::after { left: 2px; width: 22px; background: ${G}; }
  .dots button:focus-visible { outline: 2px solid ${G}; outline-offset: 0; border-radius: 6px; }

  .scene { opacity: 0; transition: opacity .45s ease; }
  .scene.on { opacity: 1; }
  :host(.off-screen) .scene * { animation-play-state: paused !important; }

  /* shared: things that pop into place, one after another */
  .pop { transform-box: fill-box; transform-origin: center; }
  .on .pop { animation: pop .5s cubic-bezier(.2,.8,.25,1.25) both; animation-delay: calc(var(--i) * 380ms + 900ms); }
  @keyframes pop { from { transform: scale(0); opacity: 0; } }

  /* 1: the scan line sweeps the CCTV feed; each table is masked, then lands on the map */
  .scan { opacity: 0; }
  .on .scan { animation: scan 2.2s linear infinite; }
  @keyframes scan { 0% { transform: translateY(0); opacity: 0; } 10% { opacity: .8; } 90% { opacity: .8; }
                    100% { transform: translateY(124px); opacity: 0; } }
  .mask { opacity: 0; }
  .on .mask { animation: appear .4s ease-out both; animation-delay: calc(var(--i) * 380ms + 600ms); }
  @keyframes appear { from { opacity: 0; } to { opacity: 1; } }
  .flow { opacity: 0; }
  .on .flow { animation: flow .38s linear 900ms 6 both; }
  @keyframes flow { 0% { transform: translateX(0); opacity: 1; } 100% { transform: translateX(18px); opacity: 0; } }
  .on .rec { animation: rec 1.4s steps(1, end) infinite; }
  @keyframes rec { 50% { opacity: .2; } }

  /* 2: labelled packets along the arc, and the robot's visor answering */
  .packet { opacity: 0; }
  .on .packet { animation: hop 4.2s linear infinite; animation-delay: calc(var(--i) * .7s); }
  @keyframes hop {
    0%   { transform: translate(176px, 160px); opacity: 0; }
    5%   { opacity: 1; }
    12%  { transform: translate(217px, 115px); }
    25%  { transform: translate(258px, 98px); }
    37%  { transform: translate(290px, 104px); }
    42%  { opacity: 1; }
    50%  { transform: translate(308px, 120px); opacity: 0; }
    100% { transform: translate(308px, 120px); opacity: 0; }
  }
  .visor { opacity: 0; }
  .on .visor { animation: visor .6s ease-in-out infinite; }
  @keyframes visor { 0%, 100% { opacity: 0; } 50% { opacity: .55; } }

  /* 3: planned tables slide to the real ones once the beam reaches them */
  .ghost { transition: transform .9s cubic-bezier(.4,0,.2,1), opacity .9s; transform: translate(var(--dx), var(--dy)); }
  .ghost.found { transform: translate(0, 0); opacity: 0; }
  .real { opacity: 0; transition: opacity .6s .3s; }
  .real.found { opacity: 1; }
  .stop .dot { opacity: 0; transform-box: fill-box; transform-origin: center; transform: scale(.3); transition: opacity .3s, transform .35s cubic-bezier(.2,.8,.25,1.4); }
  .stop.served .dot { opacity: 1; transform: scale(1); }

  /* 4: limb colours; the near side is lit, the far side in shade */
  .limb-near { fill: #DDE4E0; }
  .limb-far { fill: #A9B5AF; }
  .stacked, .spot, .carried { transition: opacity .15s; }

  /* 5: the plates go round quickly, then the check lands */
  .badge { transform-box: fill-box; transform-origin: center; }
  .on .badge { animation: pop .55s cubic-bezier(.2,.8,.25,1.4) 1.4s both; }
  .scene[data-n="5"].on .pop { animation-delay: calc(var(--i) * 140ms + 150ms); }

  @media (prefers-reduced-motion: reduce) {
    .scene, .ghost, .real, .stop .dot, .stacked, .spot, .carried { transition: none; }
    .on .pop, .on .badge, .on .mask { animation: none; opacity: 1; }
    .on .scan, .on .flow, .on .rec, .on .visor { animation: none; }
    .on .packet { animation: none; opacity: 1; transform: translate(var(--sx), var(--sy)); }
  }
`;

customElements.define("pipeline-scenes", PipelineScenes);

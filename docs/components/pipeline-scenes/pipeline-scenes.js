// <pipeline-scenes>: one illustration per step of "How it works", shown one at a time beside
// the steps on the home page. It pictures each step rather than restating it.
//
//   highlight(n)  show step n (1-5); null leaves the current one up
//   autoplay(on)  cycle through the steps on its own (used on narrow screens, where the panel
//                 sits above the list rather than beside it)
//   fires "step-select" (detail: n) when a dot is pressed, so the page can scroll to that step
//
// Animations run only for the step on show and only while the panel is on screen; visitors who
// prefer reduced motion see each step's finished picture.

const G = "#5FC2A2", CREAM = "#F4F7F5", INK = "#0F1513", WHITE = "#E8EEEA";

// Step 1: the planner on an iPad, tables and their chairs dropping onto the plan
function planScene() {
  const tables = [[166, 150], [240, 150], [314, 150], [166, 212], [240, 212], [314, 212]];
  const chairs = (cx, cy) => Array.from({ length: 8 }, (_, k) => {
    const a = (k / 8) * Math.PI * 2;
    return `<circle cx="${(cx + 20 * Math.cos(a)).toFixed(1)}" cy="${(cy + 20 * Math.sin(a)).toFixed(1)}" r="2.8"
      fill="#EFE5CF" stroke="#BE9B57" stroke-width=".6"/>`;
  }).join("");
  return `
    <rect x="96" y="34" width="288" height="236" rx="20" fill="#1B2420" stroke="rgba(255,255,255,.16)"/>
    <rect x="110" y="48" width="260" height="208" rx="8" fill="${CREAM}"/>
    <path d="M110 56a8 8 0 0 1 8-8h244a8 8 0 0 1 8 8v12H110z" fill="#111412"/>
    <circle cx="122" cy="58" r="3.5" fill="${G}"/>
    <rect x="132" y="55.5" width="44" height="5" rx="2.5" fill="rgba(255,255,255,.4)"/>
    <rect x="304" y="53" width="56" height="10" rx="5" fill="#1D6152"/>
    <rect x="126" y="80" width="228" height="162" fill="#fff" stroke="#111412" stroke-width="1.6"/>
    <rect x="198" y="88" width="84" height="24" fill="#E1E8E4" stroke="#86908B" stroke-width=".8"/>
    ${tables.map(([cx, cy], i) => `
      <g transform="translate(${cx} ${cy})"><g class="pop" style="--i:${i}">
        ${chairs(0, 0)}
        <circle r="14" fill="#F6FAF8" stroke="#5E8F80" stroke-width="1.4"/>
      </g></g>`).join("")}
    <circle class="select" cx="240" cy="212" r="19" fill="none" stroke="#1D6152" stroke-width="2.2"/>`;
}

// Step 2: the plan flying from the iPad to the robot, packet by packet
function handoffScene() {
  const mini = [[78, 150], [106, 150], [134, 150], [78, 180], [106, 180], [134, 180]]
    .map(([x, y]) => `<circle cx="${x}" cy="${y}" r="8" fill="#F6FAF8" stroke="#5E8F80" stroke-width="1.2"/>`).join("");
  const icons = [
    `<path d="M-7-5h6v4h-6zM1-5h6v4H1zM-7 1h6v4h-6zM1 1h6v4H1z" fill="${CREAM}"/>`,        // the map
    `<circle r="5.5" fill="none" stroke="${CREAM}" stroke-width="1.8"/>`,                       // tables
    `<circle cx="-6" r="2" fill="${CREAM}"/><circle r="2" fill="${CREAM}"/><circle cx="6" r="2" fill="${CREAM}"/>`, // sets
  ];
  return `
    <rect x="40" y="112" width="132" height="104" rx="12" fill="#1B2420" stroke="rgba(255,255,255,.16)"/>
    <rect x="48" y="120" width="116" height="88" rx="5" fill="${CREAM}"/>
    ${mini}
    <path d="M176 160C230 80 290 80 330 140" fill="none" stroke="${G}" stroke-opacity=".45"
      stroke-width="1.6" stroke-dasharray="4 6" stroke-linecap="round"/>
    ${icons.map((ic, i) => `<g class="packet" style="--i:${i}">
        <rect x="-15" y="-11" width="30" height="22" rx="6" fill="#1D6152" stroke="${G}"/>${ic}</g>`).join("")}
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

// Step 3: the lidar finding where the tables really are, then a route to a set position
function navigateScene() {
  const tables = [   // real position, and how far the planned one was off
    [150, 110, 12, -10], [270, 110, -12, 12], [390, 120, -10, -16], [210, 238, 14, -8], [340, 248, -10, 12]];
  return `
    <rect x="36" y="36" width="408" height="288" rx="10" fill="rgba(255,255,255,.02)" stroke="rgba(255,255,255,.16)"/>
    ${tables.map(([x, y, dx, dy], i) => `
      <g transform="translate(${x} ${y})">
        <circle class="real" style="--i:${i}" r="24" fill="rgba(95,194,162,.1)" stroke="${G}" stroke-width="1.6"/>
        <g class="ghost" style="--i:${i};--dx:${dx}px;--dy:${dy}px">
          <circle r="24" fill="none" stroke="rgba(255,255,255,.45)" stroke-width="1.3" stroke-dasharray="4 4"/>
        </g>
      </g>`).join("")}
    <path class="route" d="M80 290V170H182V136" fill="none" stroke="${G}" stroke-width="2.2"
      stroke-dasharray="4 5" stroke-linecap="round" stroke-linejoin="round"/>
    <circle class="target" cx="182" cy="136" r="7" fill="none" stroke="${G}" stroke-width="2"/>
    <g transform="translate(80 290)">
      ${[0, 1, 2].map((i) => `<circle class="ring" style="--i:${i}" r="200" fill="none" stroke="${G}"
          stroke-width="1.4" vector-effect="non-scaling-stroke"/>`).join("")}
      <circle r="15" fill="${G}" fill-opacity=".18"/>
      <circle r="9" fill="${G}" stroke="${INK}" stroke-width="1.5"/>
    </g>`;
}

// Step 4: side view. Take the top plate off the AGV, turn, carry it to the table, set it down.
function transferScene() {
  const stack = [242, 237, 232, 227, 222, 217].map((y, i) =>
    `<ellipse class="${i === 5 ? "stack-top" : ""}" cx="100" cy="${y}" rx="34" ry="6" fill="${CREAM}"
      stroke="#9DBFB3" stroke-width="1"/>`).join("");
  return `
    <line x1="24" y1="292" x2="456" y2="292" stroke="rgba(255,255,255,.18)" stroke-width="1.4"/>
    <rect x="40" y="246" width="120" height="36" rx="6" fill="#1A231F" stroke="${G}" stroke-opacity=".55"/>
    <rect x="52" y="262" width="34" height="4" rx="2" fill="${G}"/>
    <circle cx="62" cy="286" r="7" fill="#0B100E" stroke="${G}" stroke-opacity=".35"/>
    <circle cx="138" cy="286" r="7" fill="#0B100E" stroke="${G}" stroke-opacity=".35"/>
    ${stack}
    <rect x="318" y="214" width="130" height="8" rx="3" fill="${WHITE}"/>
    <rect x="332" y="222" width="6" height="70" fill="#9AA6A0"/>
    <rect x="428" y="222" width="6" height="70" fill="#9AA6A0"/>
    <ellipse class="placed" cx="383" cy="209" rx="26" ry="5" fill="${CREAM}" stroke="#9DBFB3"/>
    <g class="walk"><g class="face">
      <rect x="-11" y="248" width="9" height="44" rx="4.5" fill="#C9D3CE"/>
      <rect x="2" y="248" width="9" height="44" rx="4.5" fill="#C9D3CE"/>
      <rect x="-20" y="178" width="40" height="72" rx="14" fill="${WHITE}"/>
      <circle cx="6" cy="200" r="4" fill="${G}"/>
      <rect x="-10" y="128" width="20" height="12" rx="4" fill="#111412" stroke="${G}" stroke-opacity=".5"/>
      <rect x="-16" y="138" width="32" height="38" rx="15" fill="${WHITE}"/>
      <rect x="-2" y="149" width="18" height="11" rx="5" fill="#111412"/>
      <rect x="6" y="196" width="46" height="9" rx="4.5" fill="#C9D3CE"/>
      <ellipse class="carried" cx="58" cy="193" rx="24" ry="5" fill="${CREAM}" stroke="#9DBFB3"/>
    </g></g>`;
}

// Step 5: a set table, one place allowed to miss, and nothing broken
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
  ["The VATEL Planner on an iPad: tables and chairs appear on the hall plan, and one table is selected.", planScene],
  ["The plan travels from the iPad to the robot as packets: the map, the tables and the set positions.", handoffScene],
  ["Top view of the hall: lidar rings spread from the robot, the planned tables move to where they really are, and a route is drawn to a set position.", navigateScene],
  ["Side view: the robot takes the top plate off the AGV's stack, carries it to the table and sets it down.", transferScene],
  ["A round table seen from above with nine of its ten places set, one left empty, and a check mark.", doneScene],
];

// vertical nudge per scene, so each drawing sits centred in the frame
const NUDGE = [26, 0, 0, -26, -10];

class PipelineScenes extends HTMLElement {
  constructor() {
    super();
    this.attachShadow({ mode: "open" });
    this.current = 1;
  }

  connectedCallback() {
    if (this.built) return;
    this.built = true;
    this.shadowRoot.innerHTML = `
      <div class="panel">
        <span class="count" aria-hidden="true">Step <b>1</b> / ${SCENES.length}</span>
        <svg viewBox="0 0 480 360" role="img" aria-label="${SCENES[0][0]}">
          ${SCENES.map(([, draw], i) => `<g class="scene" data-n="${i + 1}"><g transform="translate(0 ${NUDGE[i]})">${draw()}</g></g>`).join("")}
        </svg>
        <div class="dots" role="group" aria-label="Steps">
          ${SCENES.map((_, i) => `<button type="button" data-n="${i + 1}" aria-label="Step ${i + 1}"></button>`).join("")}
        </div>
      </div>
      <style>${STYLE}</style>`;
    this.svg = this.shadowRoot.querySelector("svg");
    this.shadowRoot.querySelectorAll(".dots button").forEach((b) =>
      b.addEventListener("click", () => {
        const n = Number(b.dataset.n);
        this.show(n);
        this.hold = Date.now() + 9000;          // a chosen step stays up a while, even on autoplay
        this.dispatchEvent(new CustomEvent("step-select", { detail: n, bubbles: true, composed: true }));
      }));
    // run animations only while the panel can be seen
    this.io = new IntersectionObserver((e) => this.classList.toggle("off-screen", !e[0].isIntersecting));
    this.io.observe(this);
    this.show(1);
  }

  disconnectedCallback() {
    if (this.io) this.io.disconnect();
    this.autoplay(false);
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
  }

  autoplay(on) {
    clearInterval(this.timer);
    this.timer = 0;
    if (!on || window.matchMedia("(prefers-reduced-motion: reduce)").matches) return;
    this.timer = setInterval(() => {
      if (this.classList.contains("off-screen") || document.hidden || Date.now() < (this.hold || 0)) return;
      this.show((this.current % SCENES.length) + 1);
    }, 5200);
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
  .on .pop { animation: pop .5s cubic-bezier(.2,.8,.25,1.25) both; animation-delay: calc(var(--i) * 120ms + 150ms); }
  @keyframes pop { from { transform: scale(0); opacity: 0; } }

  /* 1: the selected table pulses */
  .select { opacity: 0; }
  .on .select { animation: select 1.8s ease-in-out 1s infinite; }
  @keyframes select { 0%, 100% { opacity: .35; } 50% { opacity: 1; } }

  /* 2: packets along the arc, and the robot's visor answering */
  .packet { opacity: 0; }
  .on .packet { animation: hop 2.4s linear infinite; animation-delay: calc(var(--i) * .8s); }
  @keyframes hop {
    0%   { transform: translate(176px, 160px); opacity: 0; }
    8%   { opacity: 1; }
    25%  { transform: translate(217px, 115px); }
    50%  { transform: translate(258px, 98px); }
    75%  { transform: translate(297px, 107px); }
    90%  { opacity: 1; }
    100% { transform: translate(330px, 140px); opacity: 0; }
  }
  .visor { opacity: 0; }
  .on .visor { animation: visor .8s ease-in-out infinite; }
  @keyframes visor { 0%, 100% { opacity: 0; } 50% { opacity: .55; } }

  /* 3: rings sweep out; planned tables slide to the real ones; the route draws itself */
  .ring { transform-box: fill-box; transform-origin: center; opacity: 0; }
  .on .ring { animation: ring 2.4s ease-out infinite; animation-delay: calc(var(--i) * .8s); }
  @keyframes ring { from { transform: scale(.05); opacity: .7; } to { transform: scale(1); opacity: 0; } }
  .real { opacity: 0; }
  .on .real { animation: appear .5s ease-out both; animation-delay: calc(var(--i) * .25s + .9s); }
  @keyframes appear { from { opacity: 0; } to { opacity: 1; } }
  .on .ghost { animation: settle 1s cubic-bezier(.4,0,.2,1) both; animation-delay: calc(var(--i) * .25s + .5s); }
  @keyframes settle {
    from { transform: translate(var(--dx), var(--dy)); opacity: 1; }
    80%  { opacity: .8; }
    to   { transform: translate(0, 0); opacity: 0; }
  }
  .route { stroke-dashoffset: 0; opacity: 0; }
  .on .route { animation: route 1.2s ease-out 2.4s both; }
  @keyframes route { from { opacity: 0; clip-path: inset(100% 0 0 0); } to { opacity: 1; clip-path: inset(0 0 0 0); } }
  .target { opacity: 0; transform-box: fill-box; transform-origin: center; }
  .on .target { animation: target 1.6s ease-out 3.4s infinite; }
  @keyframes target { 0% { opacity: 1; transform: scale(.6); } 100% { opacity: 0; transform: scale(2.2); } }

  /* 4: pick, turn, carry, set down, turn, walk back */
  .walk { transform: translateX(178px); }
  .on .walk { animation: walk 6s ease-in-out infinite; }
  @keyframes walk {
    0%, 18%  { transform: translateX(178px); }
    52%, 64% { transform: translateX(325px); }
    96%, 100% { transform: translateX(178px); }
  }
  .face { transform: scaleX(-1); }
  .on .face { animation: face 6s steps(1, end) infinite; }
  @keyframes face { 0% { transform: scaleX(-1); } 16% { transform: scaleX(1); } 62% { transform: scaleX(-1); } }
  .carried { opacity: 0; }
  .on .carried { animation: carried 6s steps(1, end) infinite; }
  @keyframes carried { 0% { opacity: 0; } 10% { opacity: 1; } 58% { opacity: 0; } }
  .on .stack-top { animation: stacktop 6s steps(1, end) infinite; }
  @keyframes stacktop { 0% { opacity: 1; } 10% { opacity: 0; } 98% { opacity: 1; } }
  .placed { opacity: 0; }
  .on .placed { animation: placed 6s steps(1, end) infinite; }
  @keyframes placed { 0% { opacity: 0; } 58% { opacity: 1; } 96% { opacity: 0; } }

  /* 5: the check lands after the plates */
  .badge { transform-box: fill-box; transform-origin: center; }
  .on .badge { animation: pop .55s cubic-bezier(.2,.8,.25,1.4) 1.4s both; }

  @media (prefers-reduced-motion: reduce) {
    .scene { transition: none; }
    .on .pop, .on .badge, .on .real, .on .route { animation: none; opacity: 1; }
    .on .select { animation: none; opacity: 1; }
    .on .packet, .on .visor, .on .ring, .on .ghost, .on .target { animation: none; }
    .packet:nth-of-type(1) { opacity: 1; transform: translate(258px, 98px); }
    .on .route, .on .target { opacity: 1; }
    .on .ghost { opacity: 0; }
    .on .walk { animation: none; transform: translateX(325px); }
    .on .face { animation: none; transform: scaleX(1); }
    .on .carried, .on .stack-top { animation: none; }
    .on .carried { opacity: 1; }
    .on .stack-top { opacity: 0; }
    .on .placed { animation: none; opacity: 0; }
  }
`;

customElements.define("pipeline-scenes", PipelineScenes);

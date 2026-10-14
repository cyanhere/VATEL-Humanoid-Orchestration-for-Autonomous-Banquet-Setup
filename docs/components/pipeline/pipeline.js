// The VATEL pipeline figure: <pipeline-diagram tag="fig-id" subtitle="Figure N: ..."></pipeline-diagram>
// Plain HTML and CSS rather than a Mermaid chart, so it stays sharp and readable at any width or zoom.
// Colours repeat site.css's palette; the shadow DOM keeps the page's own styles out.
//
// With the `interactive` attribute, pointing at, tapping or tabbing to a step highlights it and
// fires a bubbling "step-hover" event (detail: the step number, or null when it is left), and
// highlight(n) lets the page highlight a step from outside. The report uses the plain figure.

const ICONS = {
  tablet:
    '<svg viewBox="0 0 16 16" aria-hidden="true"><rect x="3.5" y="1.5" width="9" height="13" rx="1.5"/><path d="M7 12h2"/></svg>',
  robot:
    '<svg viewBox="0 0 16 16" aria-hidden="true"><rect x="3" y="5" width="10" height="7.5" rx="2"/><path d="M8 5V2.5M6.2 8.5v.5M9.8 8.5v.5M1.5 8v2.5M14.5 8v2.5"/></svg>',
  stack:
    '<svg viewBox="0 0 16 16" aria-hidden="true"><ellipse cx="8" cy="5" rx="5.5" ry="1.8"/><path d="M2.5 5v2.2c0 1 2.5 1.8 5.5 1.8s5.5-.8 5.5-1.8V5M2.5 7.2v2.2c0 1 2.5 1.8 5.5 1.8s5.5-.8 5.5-1.8V7.2"/></svg>',
  repeat:
    '<svg viewBox="0 0 16 16" aria-hidden="true"><path d="M12.6 6.2A5 5 0 0 0 3.8 4.3M3.4 9.8a5 5 0 0 0 8.8 1.9"/><path d="M3.5 1.8v2.7h2.7M12.5 14.2v-2.7H9.8"/></svg>',
};

const step = (num, title, text, extra = "") => `
  <div class="step" data-step="${num}">
    <span class="num">${num}</span>
    <div>
      <p class="title">${title}</p>
      <p class="text">${text}</p>
      ${extra}
    </div>
  </div>`;

class PipelineDiagram extends HTMLElement {
  static get observedAttributes() {
    return ["tag", "subtitle"];
  }

  constructor() {
    super();
    this.attachShadow({ mode: "open" });
  }

  connectedCallback() {
    this.render();
  }

  attributeChangedCallback(name, _, newValue) {
    this[name] = newValue;
  }

  render() {
    // The id goes on the element itself: links like #fig-id cannot reach inside the shadow DOM.
    if (this.tag && !this.id) this.id = this.tag;
    const figure = document.createElement("figure");
    figure.innerHTML = `
    <div class="group planner">
      <span class="label">${ICONS.tablet}VATEL Planner &middot; iPad or computer</span>
      ${step(1, "Plan", "See the whole hall, segment out the tables, plan the arrangement and calculate every set position.")}
      <span class="link"></span>
      ${step(2, "Hand off", "Send the static map, tables, sets per table and set positions to the robot.")}
    </div>

    <span class="link"></span>

    <div class="group robot">
      <span class="label">${ICONS.robot}Unitree R1 humanoid</span>
      ${step(3, "Navigate", "Navigate from the static map while the JT128 lidar finds the real tables and updates the map.")}
      <span class="link gated"><span class="gate">&ge; 80 % of the workspace reached</span></span>
      ${step(
        4,
        "Transfer",
        "Take one plate off the AGV stack and place it at its designated position.",
        `<ul class="tags">
          <li class="input">${ICONS.stack}Plates stacked on an AGV</li>
          <li>${ICONS.repeat}One plate per trip</li>
        </ul>`,
      )}
    </div>

    <span class="link"></span>

    <div class="done">
      ${step(5, "Done", "&ge; 90 % of plates at their designated positions, none broken.")}
    </div>

    ${this.subtitle ? `<figcaption>${this.subtitle}</figcaption>` : ""}

    <style>
      :host {
        display: block;
        max-width: 520px;
        margin: 40px auto 32px;
        --ink: #111412;
        --text: #2E3633;
        --muted: #5E6B66;
        --accent: #1D6152;
        --line: #9DBFB3;
      }

      figure {
        margin: 0;
        text-align: left;
      }

      .group {
        position: relative;
        padding: 26px 14px 14px;
        border: 1px solid;
        border-radius: 16px;
      }

      .planner {
        border-color: #E1E7E4;
        background: #F6F8F7;
      }

      .robot {
        border-color: #C9E0D6;
        background: #ECF5F1;
      }

      /* The group's name, set into its top border like a legend. */
      .label {
        position: absolute;
        top: -12px;
        left: 14px;
        display: inline-flex;
        align-items: center;
        gap: 6px;
        padding: 3px 11px 3px 9px;
        border: 1px solid;
        border-radius: 999px;
        background: #fff;
        font-size: 11px;
        font-weight: 600;
        letter-spacing: 0.08em;
        line-height: 1.4;
        text-transform: uppercase;
        white-space: nowrap;
      }

      .planner .label {
        border-color: #D6DEDA;
        color: var(--text);
      }

      .robot .label {
        border-color: #C9E0D6;
        color: var(--accent);
      }

      svg {
        flex: none;
        width: 14px;
        height: 14px;
        fill: none;
        stroke: currentColor;
        stroke-width: 1.5;
        stroke-linecap: round;
        stroke-linejoin: round;
      }

      .step {
        display: flex;
        align-items: flex-start;
        gap: 12px;
        padding: 12px 14px;
        border: 1px solid #E1E7E4;
        border-radius: 12px;
        background: #fff;
        box-shadow: 0 1px 2px rgba(17, 20, 18, 0.05);
      }

      .num {
        flex: none;
        display: grid;
        place-items: center;
        width: 26px;
        height: 26px;
        margin-top: 1px;
        border-radius: 50%;
        background: var(--accent);
        color: #fff;
        font-size: 13px;
        font-weight: 700;
      }

      p {
        margin: 0;
      }

      .title {
        margin-bottom: 2px;
        font-size: 15px;
        font-weight: 600;
        line-height: 1.5;
        color: var(--ink);
      }

      .text {
        font-size: 13.5px;
        line-height: 1.5;
        color: var(--text);
      }

      .tags {
        display: flex;
        flex-wrap: wrap;
        gap: 6px;
        margin: 10px 0 0;
        padding: 0;
        list-style: none;
      }

      .tags li {
        display: inline-flex;
        align-items: center;
        gap: 6px;
        padding: 3px 10px 3px 8px;
        border: 1px solid #E1E7E4;
        border-radius: 999px;
        background: #F6F8F7;
        font-size: 12px;
        line-height: 1.4;
        color: var(--text);
      }

      .tags li.input {
        border: 1px dashed var(--line);
        background: #fff;
        color: var(--accent);
      }

      /* Arrows: a short line with a head, centred between two boxes. */
      .link {
        position: relative;
        display: block;
        width: 2px;
        height: 30px;
        margin: 0 auto;
        background: var(--line);
      }

      .link::after {
        content: "";
        position: absolute;
        bottom: 0;
        left: 50%;
        transform: translateX(-50%);
        border-top: 7px solid var(--line);
        border-right: 5px solid transparent;
        border-left: 5px solid transparent;
      }

      .link.gated {
        height: 52px;
      }

      .gate {
        position: absolute;
        top: 44%;
        left: 50%;
        transform: translate(-50%, -50%);
        padding: 3px 11px;
        border: 1px solid var(--line);
        border-radius: 999px;
        background: #fff;
        font-size: 12px;
        font-weight: 600;
        line-height: 1.4;
        white-space: nowrap;
        color: var(--accent);
      }

      .done .step {
        border-color: var(--accent);
        background: var(--accent);
        box-shadow: 0 6px 18px rgba(29, 97, 82, 0.22);
      }

      .done .num {
        background: #fff;
        color: var(--accent);
      }

      .done .title,
      .done .text {
        color: #fff;
      }

      /* interactive (home page only): the highlighted step lifts and takes a green edge */
      :host([interactive]) .step {
        transition: border-color 0.2s, box-shadow 0.2s, transform 0.2s;
      }

      :host([interactive]) .step:focus-visible {
        outline: 2px solid var(--accent);
        outline-offset: 2px;
      }

      .step.hl {
        border-color: var(--accent);
        box-shadow: 0 0 0 3px rgba(29, 97, 82, 0.12), 0 8px 20px rgba(17, 20, 18, 0.08);
        transform: translateY(-2px);
      }

      .done .step.hl {
        box-shadow: 0 0 0 3px rgba(29, 97, 82, 0.25), 0 10px 26px rgba(29, 97, 82, 0.32);
      }

      .step.hl .num {
        box-shadow: 0 0 0 4px rgba(29, 97, 82, 0.15);
      }

      @media (prefers-reduced-motion: reduce) {
        :host([interactive]) .step { transition: none; }
        .step.hl { transform: none; }
      }

      figcaption {
        margin-top: 14px;
        font-size: 0.9rem;
        font-style: italic;
        text-align: center;
        color: var(--muted);
      }
    </style>
  `;

    this.shadowRoot.appendChild(figure);
    if (this.hasAttribute("interactive")) this.bindSteps();
  }

  bindSteps() {
    const tell = (n) =>
      this.dispatchEvent(new CustomEvent("step-hover", { detail: n, bubbles: true, composed: true }));
    this.shadowRoot.querySelectorAll(".step").forEach((card) => {
      const n = Number(card.dataset.step);
      card.tabIndex = 0;
      card.addEventListener("pointerenter", () => { this.highlight(n); tell(n); });
      card.addEventListener("pointerleave", () => tell(null));
      card.addEventListener("focus", () => { this.highlight(n); tell(n); });
      card.addEventListener("blur", () => tell(null));
    });
  }

  highlight(n) {
    this.shadowRoot.querySelectorAll(".step").forEach((card) =>
      card.classList.toggle("hl", Number(card.dataset.step) === n));
  }
}

customElements.define("pipeline-diagram", PipelineDiagram);

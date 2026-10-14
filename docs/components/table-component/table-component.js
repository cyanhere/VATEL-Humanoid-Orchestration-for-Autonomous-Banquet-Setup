// EDIC template table component, without the template's demo Grid.js table.
// Wrap a plain <table> in <table-component subtitle="Table N: ..."> to give it a caption.

class TableComponent extends HTMLElement {
  static get observedAttributes() {
    return ["subtitle"];
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
    const div = document.createElement("div");
    div.innerHTML = `
    <slot></slot>
    <sub>${this.subtitle}</sub>
    <style>
      :host {
        display: block;
        margin: 20px 0 28px;
        text-align: center;
      }

      sub {
        display: block;
        margin-top: 12px;
        font-size: 0.9rem;
        font-style: italic;
        color: #5E6B66;
      }
    </style>
  `;

    this.shadowRoot.appendChild(div);
  }
}

customElements.define("table-component", TableComponent);

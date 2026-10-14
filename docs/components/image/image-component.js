class ImageComponent extends HTMLElement {
  static get observedAttributes() {
    return ["tag", "source", "subtitle"];
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
    <img id="${this.tag}" src="${this.source}" alt="${this.subtitle}">
    <sub>${this.subtitle}</sub>
    <style>
      :host {
        display: block;
        margin: 20px 0 28px;
        text-align: center;
      }

      img {
        display: block;
        width: 100%;
        border-radius: 12px;
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

customElements.define("image-component", ImageComponent);

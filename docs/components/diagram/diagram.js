// Mermaid set-up shared by every page that draws a diagram.
// Load after mermaid.min.js, at the end of <body>; each <pre class="mermaid"> on the page is drawn.

mermaid.initialize({
  startOnLoad: false,
  theme: "base",
  themeVariables: {
    fontFamily: '"Inter", system-ui, sans-serif',
    fontSize: "15px",
    primaryColor: "#E1EFE9",
    primaryBorderColor: "#1D6152",
    primaryTextColor: "#111412",
    lineColor: "#1D6152",
    clusterBkg: "#FFFFFF",
    clusterBorder: "#9DBFB3",
    edgeLabelBackground: "#FFFFFF",
  },
  flowchart: { htmlLabels: true, curve: "basis", wrappingWidth: 360 },
});

// Mermaid sizes each box from the text it measures, so draw once the page font is in;
// before that it measures the fallback font and the labels overflow their boxes.
Promise.all([
  document.fonts.load('400 15px "Inter"'),
  document.fonts.load('700 15px "Inter"'),
])
  .catch(() => {})
  .then(() => mermaid.run());

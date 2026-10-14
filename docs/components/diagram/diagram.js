// Mermaid set-up shared by every page that draws a diagram.
// Load after mermaid.min.js; each <pre class="mermaid"> on the page is drawn on load.

mermaid.initialize({
  startOnLoad: true,
  theme: "base",
  themeVariables: {
    primaryColor: "#E1EFE9",
    primaryBorderColor: "#1D6152",
    primaryTextColor: "#111412",
    lineColor: "#1D6152",
    clusterBkg: "#F7FAF8",
    clusterBorder: "#9DBFB3",
    edgeLabelBackground: "#FFFFFF",
    fontSize: "15px",
  },
  flowchart: { htmlLabels: true, curve: "basis", wrappingWidth: 360 },
});

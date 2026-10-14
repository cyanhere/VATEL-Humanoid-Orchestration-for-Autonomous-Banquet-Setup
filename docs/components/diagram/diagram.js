// Draws any Mermaid block on the page: <div class="diagram"><pre class="mermaid">...</pre></div>.
// Load at the end of <body>. Mermaid is about 3 MB, so it is only fetched when a page has a block.
// Keep each label line under 200px (break long ones with <br/>): Mermaid's own wrapping at
// 200px misfires under browser zoom and cuts the label off.

const MERMAID_URL = "https://cdn.jsdelivr.net/npm/mermaid@11.2.1/dist/mermaid.min.js";

// Mermaid centres a subgraph's title on its top edge, which is exactly where an arrow
// entering the subgraph from above crosses it. Move each title to the top-left corner.
function leftAlignSubgraphTitles(svg) {
  svg.querySelectorAll("g.cluster").forEach((cluster) => {
    const box = cluster.querySelector("rect");
    const title = cluster.querySelector(".cluster-label");
    const at = /translate\(\s*([-\d.]+)[\s,]+([-\d.]+)\s*\)/.exec(title?.getAttribute("transform") || "");
    if (!box || !at) return;
    const x = parseFloat(box.getAttribute("x")) + 12;
    title.setAttribute("transform", `translate(${x}, ${at[2]})`);
  });
}

function drawDiagrams() {
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
    .then(() => mermaid.run())
    .then(() => document.querySelectorAll("pre.mermaid svg").forEach(leftAlignSubgraphTitles));
}

if (document.querySelector("pre.mermaid")) {
  const script = document.createElement("script");
  script.src = MERMAID_URL;
  script.onload = drawDiagrams;
  document.head.appendChild(script);
}

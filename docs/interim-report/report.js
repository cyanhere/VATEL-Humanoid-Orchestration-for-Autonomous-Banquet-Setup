// Builds the interim report from the Markdown files in sections/, in the order listed in SECTIONS.
// How to write those files is in EDITING.md. In short:
//   - a header block at the top names the author:  ---  author: Teh Wei Sheng  ---
//   - ![fig:planner | Caption](../assets/interim/x.webp)  is a numbered figure
//   - a line "Table: tab:specs | Caption" right above a Markdown table numbers that table
//   - [[fig:planner]] or [[tab:specs]] in the text becomes "Figure 3" / "Table 2", linked
//   - > TODO: ...  is a yellow note for the team, left out of the word count
// Add ?wc to the address to see the word count against the 5,000-word limit.

import { marked } from "https://cdn.jsdelivr.net/npm/marked@12.0.2/lib/marked.esm.js";

const SECTIONS = [
  "00-summary",
  "01-introduction",
  "02-background",
  "03-requirements",
  "04-concept",
  "05-planner",
  "06-helmet",
  "07-navigation",
  "08-manipulation",
  "09-open-issues",
  "10-plan",
  "11-conclusion",
  "references",
  "appendices",
];
// The course limit excludes captions, the executive summary, references and appendices.
const UNCOUNTED = new Set(["00-summary", "references", "appendices"]);
const WORD_LIMIT = 5000;

const main = document.getElementById("report");
const toc = document.getElementById("toc");

function splitHeader(text) {
  const meta = {};
  const m = /^﻿?---\s*\r?\n([\s\S]*?)\r?\n---\s*\r?\n?/.exec(text);
  if (!m) return { meta, body: text };
  for (const line of m[1].split(/\r?\n/)) {
    const i = line.indexOf(":");
    if (i > 0) meta[line.slice(0, i).trim().toLowerCase()] = line.slice(i + 1).trim();
  }
  return { meta, body: text.slice(m[0].length) };
}

const slug = (s) =>
  s.toLowerCase().replace(/<[^>]+>/g, "").replace(/[^\w\s-]/g, "").trim().replace(/\s+/g, "-");

// "fig:planner | The planner's main page" -> { id: "fig-planner", caption: "The planner's main page" }
function labelled(text) {
  const m = /^\s*((?:fig|tab):[\w-]+)\s*\|\s*([\s\S]*)$/.exec(text);
  return m ? { id: m[1].replace(":", "-"), caption: m[2].trim() } : { id: "", caption: text.trim() };
}

function el(tag, attrs = {}, html = "") {
  const e = document.createElement(tag);
  for (const [k, v] of Object.entries(attrs)) e.setAttribute(k, v);
  if (html) e.innerHTML = html;
  return e;
}

// One Markdown file -> one <section>, with its author shown under the heading.
function renderSection(name, text) {
  const { meta, body } = splitHeader(text);
  const section = el("section", { id: meta.id || name.replace(/^\d+-/, ""), "data-file": name });
  if (meta.class) section.className = meta.class;
  section.innerHTML = marked.parse(body, { gfm: true })
    // "[TODO: ...]" inside a sentence or table cell -> a marked note, left out of the word count
    .replace(/\[TODO:\s*([^\]<]*)\]/g, '<mark class="todo-inline">TODO: $1</mark>');

  const h2 = section.querySelector("h2");
  if (meta.author && h2) {
    h2.after(el("p", { class: "lead-tag" }, "Written by " + meta.author));
  }

  // > TODO: ...  or  > TODO (name): ...  ->  a "To fill" note
  section.querySelectorAll("blockquote").forEach((q) => {
    if (!/^TODO\b[^:]*:/i.test(q.textContent.trim())) return;
    const html = q.innerHTML.trim()
      .replace(/^(<p>)?\s*TODO\s*(?:\(([^)]*)\))?\s*:\s*/i, (m, p, who) => (p || "") + (who ? `<b>${who}:</b> ` : ""))
      .replace(/<\/?p>/g, " ");
    q.replaceWith(el("p", { class: "todo" }, html.trim()));
  });

  // ```mermaid blocks -> diagrams drawn by components/diagram/diagram.js
  section.querySelectorAll("pre > code.language-mermaid").forEach((code) => {
    const wrap = el("div", { class: "diagram" });
    wrap.append(el("pre", { class: "mermaid" }));
    wrap.firstChild.textContent = code.textContent;
    code.parentElement.replaceWith(wrap);
  });

  section.querySelectorAll("a[href^='http']").forEach((a) => a.setAttribute("target", "_blank"));
  return section;
}

// Figures and tables are numbered in reading order, after every section is in place.
function numberFiguresAndTables(root) {
  const labels = {};
  let fig = 0;
  let tab = 0;

  root.querySelectorAll("img").forEach((img) => {
    const { id, caption } = labelled(img.getAttribute("alt") || "");
    fig += 1;
    const tag = id || "fig-" + fig;
    labels[tag] = "Figure " + fig;
    const figure = el("image-component", {
      id: tag,
      tag: tag + "-img",
      source: img.getAttribute("src"),
      subtitle: "Figure " + fig + ": " + caption.replace(/"/g, "&quot;"),
    });
    const p = img.parentElement;
    if (p && p.tagName === "P" && p.textContent.trim() === "" && p.children.length === 1) p.replaceWith(figure);
    else img.replaceWith(figure);
  });

  root.querySelectorAll("table").forEach((table) => {
    const prev = table.previousElementSibling;
    let id = "";
    let caption = "";
    if (prev && prev.tagName === "P" && /^Table:/.test(prev.textContent.trim())) {
      ({ id, caption } = labelled(prev.innerHTML.replace(/^\s*Table:\s*/, "")));
      prev.remove();
    }
    tab += 1;
    const tag = id || "tab-" + tab;
    labels[tag] = "Table " + tab;
    table.classList.add("report-table");
    const scroll = el("div", { class: "table-scroll" });
    const wrap = el("table-component", { id: tag, subtitle: "Table " + tab + (caption ? ": " + caption : "") });
    table.replaceWith(wrap);
    scroll.append(table);
    wrap.append(scroll);
  });

  // [[fig:planner]] -> "Figure 3", linked to the figure; "Figure [[fig:planner]]" -> "Figure 3" too
  const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT);
  const hits = [];
  while (walker.nextNode()) if (/\[\[(fig|tab):[\w-]+\]\]/.test(walker.currentNode.nodeValue)) hits.push(walker.currentNode);
  hits.forEach((node) => {
    const span = document.createElement("span");
    span.innerHTML = node.nodeValue
      .replace(/&/g, "&amp;").replace(/</g, "&lt;")
      .replace(/(\b(?:Figure|Table)\s+)?\[\[(fig|tab):([\w-]+)\]\]/g, (m, word, kind, name) => {
        const id = kind + "-" + name;
        if (!labels[id]) return `<span class="blank">${m} (no such label)</span>`;
        const text = word ? word + labels[id].split(" ")[1] : labels[id];
        return `<a href="#${id}">${text}</a>`;
      });
    node.replaceWith(...span.childNodes);
  });
}

// Headings get ids, and the contents list is built from them.
function buildContents(root) {
  const used = new Set();
  const tree = el("sl-tree");
  root.querySelectorAll(":scope > section").forEach((section) => {
    const h2 = section.querySelector("h2");
    if (!h2) return;
    if (!h2.id) h2.id = section.id;
    used.add(h2.id);
    const item = el("sl-tree-item");
    item.append(el("a", { href: "#" + h2.id }, h2.innerHTML));
    section.querySelectorAll("h3").forEach((h3) => {
      let id = h3.id || slug(h3.textContent);
      while (used.has(id)) id += "-2";
      used.add(id);
      h3.id = id;
      const sub = el("sl-tree-item");
      sub.append(el("a", { href: "#" + id }, h3.innerHTML));
      item.append(sub);
    });
    if (item.children.length > 1) item.setAttribute("expanded", "");
    tree.append(item);
  });
  toc.querySelector("sl-tree")?.remove();
  toc.append(tree);
}

function countWords(section) {
  const copy = section.cloneNode(true);
  copy.querySelectorAll(".todo, .todo-inline, .lead-tag, image-component, pre.mermaid").forEach((e) => e.remove());
  return (copy.textContent.match(/[A-Za-z0-9][\w'’.,%/-]*/g) || []).length;
}

function showWordCount(root) {
  let total = 0;
  const rows = [];
  root.querySelectorAll(":scope > section").forEach((s) => {
    const n = countWords(s);
    const counted = !UNCOUNTED.has(s.dataset.file);
    if (counted) total += n;
    rows.push(`<tr${counted ? "" : ' class="uncounted"'}><td>${s.dataset.file}</td><td>${n}</td></tr>`);
  });
  const panel = el("aside", { class: "wc-panel" },
    `<p><b>${total.toLocaleString()}</b> / ${WORD_LIMIT.toLocaleString()} words</p>
     <table>${rows.join("")}</table>
     <p class="wc-note">Captions, "To fill" notes, the executive summary, references and appendices are not counted.</p>`);
  if (total > WORD_LIMIT) panel.classList.add("over");
  document.body.append(panel);
}

async function build() {
  let texts;
  try {
    texts = await Promise.all(SECTIONS.map(async (name) => {
      const r = await fetch(`sections/${name}.md`, { cache: "no-cache" });
      if (!r.ok) throw new Error(`sections/${name}.md: ${r.status}`);
      return r.text();
    }));
  } catch (err) {
    main.innerHTML = `<p class="todo">The report could not load its sections (${err.message}). It has to be
      opened through a web server, such as the published site on GitHub Pages; a page opened straight
      from the disk is not allowed to read the section files.</p>`;
    return;
  }

  main.replaceChildren(...SECTIONS.map((name, i) => renderSection(name, texts[i])));
  numberFiguresAndTables(main);
  buildContents(main);
  if (new URLSearchParams(location.search).has("wc")) showWordCount(main);

  if (main.querySelector("pre.mermaid")) {
    document.body.append(el("script", { src: "../components/diagram/diagram.js" }));
  }
  if (location.hash) document.getElementById(location.hash.slice(1))?.scrollIntoView();
}

build();

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

// Citations "[12]" link to their reference entry, and "Appendix 5B" / "Appendices 6B and 6C" link to the
// appendix heading. Run after buildContents, so the headings already have their ids.
function linkReferencesAndAppendices(root) {
  const refs = new Set();
  root.querySelectorAll("section.references li").forEach((li) => {
    const m = /^\s*\[(\d+)\]/.exec(li.textContent);
    if (m) { li.id = "ref-" + m[1]; refs.add(m[1]); }
  });
  const apps = {};
  root.querySelectorAll("h3").forEach((h) => {
    const m = /^\s*Appendix\s+(\d?[A-Z])\b/.exec(h.textContent);
    if (m) apps[m[1]] = h.id;
  });

  const code = String.raw`\d?[A-Z](?![\w-])`;
  const appRe = new RegExp(String.raw`\b(Appendix|Appendices)\s+(${code}(?:(?:,\s*|,?\s+and\s+|\s*(?:–|-|to)\s*)${code})*)`, "g");
  const walker = document.createTreeWalker(root, NodeFilter.SHOW_TEXT, {
    acceptNode: (n) => n.parentElement.closest("a, code, pre, h1, h2, h3, h4, h5, section.references li, .wc-panel")
      ? NodeFilter.FILTER_REJECT : NodeFilter.FILTER_ACCEPT,
  });
  const hits = [];
  while (walker.nextNode()) {
    const v = walker.currentNode.nodeValue;
    if (/\[\d+\]/.test(v) || /\bAppendi(x|ces)\s+\d?[A-Z]/.test(v)) hits.push(walker.currentNode);
  }
  const esc = (s) => s.replace(/&/g, "&amp;").replace(/</g, "&lt;");
  hits.forEach((node) => {
    const html = esc(node.nodeValue)
      .replace(/\[(\d+)\]/g, (m, n) => (refs.has(n) ? `<a class="xref" href="#ref-${n}">[${n}]</a>` : m))
      // the link covers the word too: "Appendix 5B"; in a list the first code takes the word with it
      .replace(appRe, (m, word, list) => {
        let first = true;
        return list.replace(/\d?[A-Z](?![\w-])/g, (c) => {
          const text = first ? word + " " + c : c;
          first = false;
          return apps[c] ? `<a class="xref" href="#${apps[c]}">${text}</a>` : text;
        });
      });
    if (html === esc(node.nodeValue)) return;
    const span = document.createElement("span");
    span.innerHTML = html;
    node.replaceWith(...span.childNodes);
  });

  // hover preview: the reference entry, or the figure / table caption. Appendix links get a
  // floating window with the whole appendix instead (enableAppendixPreview).
  root.querySelectorAll("a[href^='#']").forEach((a) => {
    if (a.closest("#toc")) return;
    const target = document.getElementById(decodeURIComponent(a.hash.slice(1)));
    if (!target) return;
    let text = (target.getAttribute("subtitle") || target.textContent).replace(/\s+/g, " ").trim();
    if (!text) return;
    if (target.tagName === "H3" && /^Appendix\b/.test(text)) {
      a.classList.add("xref", "xref-appendix");
      return;
    }
    a.classList.add("xref");
    a.dataset.preview = text.length > 220 ? text.slice(0, 217) + "…" : text;
  });
}

// Hovering an appendix link opens a small floating window holding a copy of the whole appendix,
// which scrolls on its own. The pointer can move into the window to scroll it or follow its links.
function enableAppendixPreview(root) {
  if (!matchMedia("(hover: hover)").matches) return;   // touch screens: the link just jumps there
  const pop = el("div", { class: "appendix-pop", role: "dialog", "aria-label": "Appendix preview", hidden: "" });
  root.append(pop);                                    // inside .content, so the report's styles apply
  let showTimer = 0;
  let hideTimer = 0;

  // a copy of the appendix: its heading, then everything up to the next appendix or chapter
  function fill(h3) {
    const title = el("p", { class: "appendix-pop-title" });
    title.append(el("a", { href: "#" + h3.id }, h3.innerHTML));
    const parts = [title];
    for (let n = h3.nextElementSibling; n && !/^H[1-3]$/.test(n.tagName); n = n.nextElementSibling) {
      const copy = n.cloneNode(true);
      [copy, ...copy.querySelectorAll("[id]")].forEach((e) => e.removeAttribute("id"));
      copy.querySelectorAll("[data-preview]").forEach((e) => e.removeAttribute("data-preview"));
      parts.push(copy);
    }
    pop.replaceChildren(...parts);
    pop.scrollTop = 0;
  }

  // below the link if there is room, otherwise above it; never off the screen
  function place(a) {
    const r = a.getBoundingClientRect();
    const gap = 8;
    const w = Math.min(520, innerWidth - 24);
    const below = innerHeight - r.bottom - gap - 12;
    const above = r.top - gap - 12;
    const down = below >= 280 || below >= above;
    pop.style.width = w + "px";
    pop.style.left = Math.max(12, Math.min(r.left, innerWidth - w - 12)) + "px";
    pop.style.maxHeight = Math.min(440, down ? below : above) + "px";
    pop.style.top = down ? r.bottom + gap + "px" : "";
    pop.style.bottom = down ? "" : innerHeight - r.top + gap + "px";
  }

  function show(a) {
    clearTimeout(hideTimer);
    const h3 = document.getElementById(decodeURIComponent(a.hash.slice(1)));
    if (!h3) return;
    if (pop.sourceLink !== a || pop.hidden) fill(h3);
    pop.sourceLink = a;
    pop.hidden = false;
    place(a);
  }
  function hide() {
    clearTimeout(showTimer);
    clearTimeout(hideTimer);
    pop.hidden = true;
  }
  function hideSoon() {
    clearTimeout(showTimer);
    clearTimeout(hideTimer);
    hideTimer = setTimeout(hide, 250);
  }

  const appendixLink = (e) => {
    const a = e.target.closest?.("a.xref-appendix");
    return a && !pop.contains(a) ? a : null;
  };
  root.addEventListener("mouseover", (e) => {
    const a = appendixLink(e);
    if (!a || (a === pop.sourceLink && !pop.hidden)) { if (a) clearTimeout(hideTimer); return; }
    clearTimeout(showTimer);
    showTimer = setTimeout(() => show(a), 150);
  });
  root.addEventListener("mouseout", (e) => {
    const a = appendixLink(e);
    if (a && !a.contains(e.relatedTarget)) hideSoon();
  });
  root.addEventListener("focusin", (e) => { const a = appendixLink(e); if (a) show(a); });
  root.addEventListener("focusout", (e) => { if (appendixLink(e) && !pop.contains(e.relatedTarget)) hideSoon(); });
  root.addEventListener("click", (e) => { if (appendixLink(e)) hide(); });

  // the wheel scrolls the window only, never the page behind it, even at the window's top or end
  pop.addEventListener("wheel", (e) => {
    const atTop = pop.scrollTop <= 0;
    const atEnd = pop.scrollTop + pop.clientHeight >= pop.scrollHeight - 1;
    if ((e.deltaY < 0 && atTop) || (e.deltaY > 0 && atEnd)) e.preventDefault();
  }, { passive: false });
  pop.addEventListener("mouseenter", () => clearTimeout(hideTimer));
  pop.addEventListener("mouseleave", hideSoon);
  pop.addEventListener("focusout", (e) => { if (!pop.contains(e.relatedTarget)) hideSoon(); });
  pop.addEventListener("click", (e) => { if (e.target.closest("a[href^='#']")) hide(); });
  document.addEventListener("keydown", (e) => { if (e.key === "Escape" && !pop.hidden) hide(); });
  // the page scrolling away from the link closes it; scrolling inside the window does not
  addEventListener("scroll", () => { if (!pop.hidden && !pop.matches(":hover")) hide(); }, { passive: true });
  addEventListener("resize", hide);
}

// Following a figure, table, reference or appendix link shows a "Back" button that returns to the link.
function enableBackButton(root) {
  const btn = el("button", { class: "back-to-link", type: "button", hidden: "" }, "&#8617; Back to text");
  document.body.append(btn);
  let from = null;
  root.addEventListener("click", (e) => {
    const a = e.target.closest("a[href^='#']");
    if (!a || a.closest("#toc")) return;
    // a link inside the appendix window: come back to the link that opened the window
    from = a.closest(".appendix-pop")?.sourceLink || a;
    btn.hidden = false;
  });
  btn.addEventListener("click", () => {
    if (!from) return;
    from.scrollIntoView({ behavior: "smooth", block: "center" });
    from.classList.remove("flash");
    void from.offsetWidth;
    from.classList.add("flash");
    history.replaceState(null, "", location.pathname + location.search);
    btn.hidden = true;
    from = null;
  });
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
  linkReferencesAndAppendices(main);
  enableAppendixPreview(main);
  enableBackButton(main);
  if (new URLSearchParams(location.search).has("wc")) showWordCount(main);

  if (main.querySelector("pre.mermaid")) {
    document.body.append(el("script", { src: "../components/diagram/diagram.js" }));
  }
  if (location.hash) document.getElementById(location.hash.slice(1))?.scrollIntoView();
}

build();

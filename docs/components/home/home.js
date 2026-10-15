// Home page behaviour. Load as a plain script at the end of <body>.
//
// - the top bar sticks, takes a shadow once the page scrolls, and marks the section being read
// - sections ease in as they scroll into view
// - the "How it works" steps and the pipeline figure highlight each other: the step being read,
//   or the one pointed at, tapped or tabbed to, lights up in both
// - the success criteria count up, and their bars fill, when they come into view
//
// Visitors who prefer reduced motion get everything in its final state, with no movement.

(function () {
  const still = window.matchMedia("(prefers-reduced-motion: reduce)").matches;

  /* --------------------------------------------------------------- top bar */
  const bar = document.querySelector(".site-bar");
  const onScroll = () => bar.classList.toggle("scrolled", window.scrollY > 8);
  onScroll();
  window.addEventListener("scroll", onScroll, { passive: true });

  // Mark the nav link of the section crossing the middle of the screen; none over the hero
  // or the problem statement, which have no link.
  const links = Array.from(document.querySelectorAll('.site-nav a[href^="#"]'));
  const spy = new IntersectionObserver((entries) => {
    entries.forEach((e) => {
      if (!e.isIntersecting) return;
      links.forEach((a) => a.classList.toggle("active", a.getAttribute("href") === "#" + e.target.id));
    });
  }, { rootMargin: "-45% 0px -50% 0px" });
  document.querySelectorAll(".hero, main > section").forEach((s) => spy.observe(s));

  /* ---------------------------------------------------------------- reveal */
  if (!still) {
    const pieces = [".section .eyebrow", ".section h2", ".lead", ".steps li", ".how-visual",
                    ".stat", ".spec", ".report-card"].join(",");
    const items = Array.from(document.querySelectorAll(pieces));
    items.forEach((el) => {
      // siblings of the same kind come in one after another
      const kin = Array.from(el.parentElement.children).filter((c) => c.matches(pieces));
      el.style.setProperty("--reveal-delay", Math.min(kin.indexOf(el), 6) * 70 + "ms");
      el.classList.add("reveal");
    });
    const show = new IntersectionObserver((entries) => {
      entries.forEach((e) => {
        if (!e.isIntersecting) return;
        e.target.classList.add("in");
        show.unobserve(e.target);
      });
    }, { rootMargin: "0px 0px -8% 0px" });
    items.forEach((el) => show.observe(el));
  }

  /* ---------------------------------------------------- steps <-> figure */
  // The figure beside the steps: the step illustrations, or the interactive diagram.
  const figure = document.querySelector("pipeline-scenes, pipeline-diagram[interactive]");
  const steps = Array.from(document.querySelectorAll(".steps li"));
  let pointed = null, reading = null;

  function light() {
    const n = pointed !== null ? pointed : reading;
    steps.forEach((li, i) => li.classList.toggle("hl", i + 1 === n));
    if (figure && typeof figure.highlight === "function") figure.highlight(n);
  }

  steps.forEach((li, i) => {
    li.addEventListener("pointerenter", () => { pointed = i + 1; light(); });
    li.addEventListener("pointerleave", () => { pointed = null; light(); });
  });
  if (figure) figure.addEventListener("step-hover", (e) => { pointed = e.detail; light(); });

  const readSpy = new IntersectionObserver((entries) => {
    entries.forEach((e) => { if (e.isIntersecting) reading = steps.indexOf(e.target) + 1; });
    light();
  }, { rootMargin: "-40% 0px -55% 0px" });
  steps.forEach((li) => readSpy.observe(li));

  // Beside the steps (wide screens) the illustrations follow the reading; above them (narrow
  // screens) they play through on their own. Pressing a step's dot on a wide screen scrolls
  // to that step.
  if (figure && figure.localName === "pipeline-scenes") {
    const wide = window.matchMedia("(min-width: 1100px)");
    customElements.whenDefined("pipeline-scenes").then(() => {
      const mode = () => figure.autoplay(!wide.matches);
      mode();
      wide.addEventListener("change", mode);
    });
    figure.addEventListener("step-select", (e) => {
      if (!wide.matches) return;
      steps[e.detail - 1].scrollIntoView({ behavior: still ? "auto" : "smooth", block: "center" });
    });
  }

  /* --------------------------------------------------- success criteria */
  const stats = document.querySelector(".stats");
  const values = Array.from(document.querySelectorAll(".stat-value[data-count]"));
  const write = (v, n) => { v.textContent = (v.dataset.prefix || "") + n + (v.dataset.suffix || ""); };

  if (stats && !still) {
    stats.classList.add("counting");
    values.forEach((v) => write(v, 0));
    const watch = new IntersectionObserver((entries) => {
      if (!entries[0].isIntersecting) return;
      watch.disconnect();
      stats.classList.add("counted");
      const start = performance.now(), ms = 1100;
      const frame = (now) => {
        const u = Math.min(1, (now - start) / ms), k = 1 - Math.pow(1 - u, 3);
        values.forEach((v) => write(v, Math.round(k * Number(v.dataset.count))));
        if (u < 1) requestAnimationFrame(frame);
      };
      requestAnimationFrame(frame);
    }, { threshold: 0.35 });
    watch.observe(stats);
  }
})();

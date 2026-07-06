/* nav_study docs — theme toggle, mobile nav, on-page TOC scrollspy, client-side search. */
(function () {
  "use strict";

  // 테마: 시스템 색상 설정(prefers-color-scheme)을 그대로 따른다 (토글 없음).
  window.addEventListener("DOMContentLoaded", function () {
    // ---- mobile sidebar ----
    var menuBtn = document.getElementById("menu-btn");
    var sidebar = document.querySelector(".sidebar");
    var backdrop = document.querySelector(".backdrop");
    if (menuBtn && sidebar) {
      var toggle = function () {
        sidebar.classList.toggle("open");
        if (backdrop) backdrop.classList.toggle("open");
      };
      menuBtn.addEventListener("click", toggle);
      if (backdrop) backdrop.addEventListener("click", toggle);
    }

    // ---- sidebar disclosure open/close animation ----
    // 순수 CSS 로는 <details> 닫힘 시 grid-rows 가 전이되지 않아, [open] 제거 시점을 JS 로
    // 제어한다: 열 때는 먼저 [open] 후 0fr→1fr, 닫을 때는 1fr→0fr 를 재생하고 끝난 뒤 [open] 제거.
    document.querySelectorAll(".sidebar .nav-sub").forEach(function (d) {
      var summary = d.querySelector("summary");
      var body = d.querySelector(".nav-sub-body");
      if (!summary || !body) return;
      summary.addEventListener("click", function (e) {
        e.preventDefault();
        if (d.dataset.animating) return;
        var opening = !d.hasAttribute("open");
        d.dataset.animating = "1";
        if (opening) d.setAttribute("open", "");
        body.style.gridTemplateRows = opening ? "0fr" : "1fr";
        void body.offsetHeight; // reflow so the start value commits before the change
        body.style.gridTemplateRows = opening ? "1fr" : "0fr";
        setTimeout(function () {
          body.style.gridTemplateRows = ""; // hand resting state back to CSS
          if (!opening) d.removeAttribute("open");
          delete d.dataset.animating;
        }, 260);
      });
    });

    // ---- on-page TOC + scrollspy ----
    var tocEl = document.getElementById("toc");
    var inner = document.querySelector(".content-inner");
    if (tocEl && inner) {
      var heads = inner.querySelectorAll("h2, h3");
      if (heads.length < 2) {
        tocEl.style.display = "none";
      } else {
        var html = '<h4>On this page</h4>';
        heads.forEach(function (h) {
          if (!h.id) return;
          html += '<a href="#' + h.id + '" class="' + (h.tagName === "H3" ? "h3" : "") + '">' +
                  h.textContent.replace("¶", "").trim() + "</a>";
        });
        tocEl.innerHTML = html;
        var links = tocEl.querySelectorAll("a");
        var spy = function () {
          var y = window.scrollY + 100, cur = null;
          heads.forEach(function (h) { if (h.offsetTop <= y) cur = h.id; });
          links.forEach(function (a) {
            a.classList.toggle("active", a.getAttribute("href") === "#" + cur);
          });
        };
        window.addEventListener("scroll", spy, { passive: true });
        spy();
      }
    }

    // ---- search ----
    var input = document.getElementById("search-input");
    var box = document.getElementById("search-results");
    if (input && box) {
      var idx = [], loaded = false, sel = -1;
      var base = input.getAttribute("data-base") || "";
      var load = function () {
        if (loaded) return;
        loaded = true;
        fetch(base + "assets/search-index.json").then(function (r) { return r.json(); })
          .then(function (d) { idx = d; }).catch(function () {});
      };
      input.addEventListener("focus", load);
      var render = function (results) {
        if (!input.value.trim()) { box.classList.remove("open"); return; }
        if (!results.length) { box.innerHTML = '<div class="r-empty">결과 없음 / No results</div>'; box.classList.add("open"); return; }
        box.innerHTML = results.map(function (r) {
          return '<a href="' + base + r.url + '"><span class="r-title">' + r.title +
                 '</span><br><span class="r-crumb">' + r.crumb + "</span></a>";
        }).join("");
        box.classList.add("open"); sel = -1;
      };
      var search = function () {
        var q = input.value.trim().toLowerCase();
        if (!q) { box.classList.remove("open"); return; }
        var res = idx.map(function (it) {
          var hay = (it.title + " " + it.crumb + " " + it.text).toLowerCase();
          var score = 0, ti = it.title.toLowerCase();
          if (ti.indexOf(q) === 0) score += 100;
          else if (ti.indexOf(q) >= 0) score += 50;
          if (hay.indexOf(q) >= 0) score += 10;
          return { it: it, score: score };
        }).filter(function (x) { return x.score > 0; })
          .sort(function (a, b) { return b.score - a.score; })
          .slice(0, 8).map(function (x) { return x.it; });
        render(res);
      };
      input.addEventListener("input", search);
      input.addEventListener("keydown", function (e) {
        var items = box.querySelectorAll("a");
        if (e.key === "ArrowDown") { e.preventDefault(); sel = Math.min(sel + 1, items.length - 1); }
        else if (e.key === "ArrowUp") { e.preventDefault(); sel = Math.max(sel - 1, 0); }
        else if (e.key === "Enter" && sel >= 0 && items[sel]) { window.location = items[sel].href; return; }
        else if (e.key === "Escape") { box.classList.remove("open"); input.blur(); return; }
        else return;
        items.forEach(function (a, i) { a.classList.toggle("sel", i === sel); });
      });
      document.addEventListener("click", function (e) {
        if (!box.contains(e.target) && e.target !== input) box.classList.remove("open");
      });
    }
  });

  // ---- MathJax (arithmatex generic: \( \) inline, \[ \] display) ----
  window.MathJax = {
    tex: {
      inlineMath: [["\\(", "\\)"]],
      displayMath: [["\\[", "\\]"]],
      processEscapes: true,
      processEnvironments: true
    },
    options: { ignoreHtmlClass: "tex2jax_ignore", processHtmlClass: "arithmatex" }
  };
})();

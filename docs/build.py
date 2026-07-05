#!/usr/bin/env python3
"""navigation study 문서 정적 사이트 빌더.

docs/{ko,en}/**.md (콘텐츠 소스) → docs/**.html (커스텀 정적 사이트) 로 렌더링한다.
GitHub Pages 는 .nojekyll 로 Jekyll 을 끄고 이 HTML 을 그대로 서빙한다.
공통 크롬(topbar/sidebar/toc/footer)·CSS·JS 는 한 곳에서만 관리해 페이지 중복을 없앤다.

    python docs/build.py         # docs/ 전체 재생성

의존: markdown, pymdown-extensions.
"""
from __future__ import annotations

import hashlib
import html
import json
import re
from pathlib import Path

import markdown

DOCS = Path(__file__).resolve().parent

# --- 헤더 인라인 SVG 아이콘 (stroke = currentColor, 로고만 gradient) ---
# 로고: grid 위 start→goal 경로(A* 모티프). 브랜드 그라디언트.
LOGO_SVG = (
    '<svg class="logo" width="26" height="26" viewBox="0 0 24 24" fill="none" aria-hidden="true">'
    '<defs><linearGradient id="navlg" x1="3" y1="21" x2="21" y2="3" gradientUnits="userSpaceOnUse">'
    '<stop stop-color="#6366f1"/><stop offset="1" stop-color="#06b6d4"/></linearGradient></defs>'
    '<path d="M4 19V11h8V5h8" stroke="url(#navlg)" stroke-width="2.2" '
    'stroke-linecap="round" stroke-linejoin="round"/>'
    '<circle cx="4" cy="19" r="2.6" fill="url(#navlg)"/>'
    '<circle cx="20" cy="5" r="2.6" fill="url(#navlg)"/></svg>'
)
MENU_SVG = (
    '<svg width="22" height="22" viewBox="0 0 24 24" fill="none" stroke="currentColor" '
    'stroke-width="2" stroke-linecap="round" aria-hidden="true"><path d="M4 7h16M4 12h16M4 17h16"/></svg>'
)
GLOBE_SVG = (
    '<svg class="g-ico" width="15" height="15" viewBox="0 0 24 24" fill="none" stroke="currentColor" '
    'stroke-width="1.9" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">'
    '<circle cx="12" cy="12" r="9"/><path d="M3 12h18"/>'
    '<path d="M12 3c2.5 2.5 3.8 5.6 3.8 9s-1.3 6.5-3.8 9c-2.5-2.5-3.8-5.6-3.8-9S9.5 5.5 12 3Z"/></svg>'
)
SEARCH_SVG = (
    '<svg class="s-ico" width="16" height="16" viewBox="0 0 24 24" fill="none" stroke="currentColor" '
    'stroke-width="2" stroke-linecap="round" aria-hidden="true">'
    '<circle cx="11" cy="11" r="7"/><path d="m21 21-4.35-4.35"/></svg>'
)
# 파비콘: 로고와 동일한 route 마크 (data URI).
FAVICON = (
    "data:image/svg+xml,"
    "%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none'%3E"
    "%3Crect width='24' height='24' rx='6' fill='%230b1020'/%3E"
    "%3Cpath d='M5 18V11h7V6h7' stroke='%23818cf8' stroke-width='2.2' stroke-linecap='round' stroke-linejoin='round'/%3E"
    "%3Ccircle cx='5' cy='18' r='2.4' fill='%23818cf8'/%3E%3Ccircle cx='19' cy='6' r='2.4' fill='%2322d3ee'/%3E%3C/svg%3E"
)


def _asset_ver() -> str:
    """site.css/site.js 내용 해시 → asset URL 캐시버스팅 (?v=)."""
    h = hashlib.md5()
    for f in ("site.css", "site.js"):
        p = DOCS / "assets" / f
        if p.exists():
            h.update(p.read_bytes())
    return h.hexdigest()[:8]


ASSET_VER = _asset_ver()

# 소스 코드 링크 chip (알고리즘 페이지 H1 아래) — GitHub blob 으로 연결.
GH_BLOB = "https://github.com/robotics-study/navigation/blob/main"


def code_chips(key: str, lang: str) -> str:
    label = "소스 코드" if lang == "ko" else "Source"
    def a(href, text):
        return f'<a href="{GH_BLOB}/{href}" target="_blank" rel="noopener">{text}</a>'
    return (
        f'<div class="code-links"><span class="cl-label">{label}</span>'
        + a(f"cpp/src/global_planning/{key}.cpp", "C++")
        + a(f"cpp/include/navigation/global_planning/{key}.hpp", "C++ header")
        + a(f"python/navigation/global_planning/{key}.py", "Python")
        + a(f"python/demos/demo_{key}.py", "demo")
        + "</div>"
    )

# 언어별 사이드바 구조 + 순서 (prev/next 는 이 평탄화 순서를 따른다).
# 각 항목: (source md 경로[lang 상대], 출력 html 경로[lang 상대], 사이드바 라벨)
_GLOBAL = [
    ("algorithms/bfs.md", "algorithms/bfs.html", "BFS"),
    ("algorithms/dijkstra.md", "algorithms/dijkstra.html", "Dijkstra"),
    ("algorithms/astar.md", "algorithms/astar.html", "A*"),
    ("algorithms/rrt.md", "algorithms/rrt.html", "RRT"),
    ("algorithms/rrt_star.md", "algorithms/rrt_star.html", "RRT*"),
    ("algorithms/fast_rrt.md", "algorithms/fast_rrt.html", "Fast-RRT"),
]
# planned(미구현) 항목: src/out=None → 사이드바에 흐리게 "예정" 표기, 페이지 생성 안 함.
_LOCAL = [(None, None, n) for n in ("DWA", "Pure Pursuit", "VFH", "MPC")]
_MULTI = [(None, None, n) for n in ("Prioritized A*", "Joint-space A*", "CBS")]

NAV = {
    "ko": {
        "name": "한국어", "soon": "예정",
        "groups": [
            ("개요", [
                ("index.md", "index.html", "개요"),
                ("algorithms/index.md", "algorithms/index.html", "알고리즘 목록"),
            ]),
            ("Global planning", _GLOBAL + [("benchmarks.md", "benchmarks.html", "벤치마크")]),
            ("Local planning", _LOCAL + [(None, None, "벤치마크")]),
            ("Multi-agent", _MULTI + [(None, None, "벤치마크")]),
            ("Maps", [("maps.md", "maps.html", "맵 표현")]),
            ("레퍼런스", [
                ("architecture.md", "architecture.html", "아키텍처"),
                ("references.md", "references.html", "참고 문헌"),
            ]),
        ],
        "topnav": [("index.html", "개요"), ("algorithms/index.html", "알고리즘"),
                   ("maps.html", "맵"), ("benchmarks.html", "벤치마크")],
        "note_title": "참고", "warn_title": "주의", "onpage": "이 페이지", "prev": "이전", "next": "다음",
    },
    "en": {
        "name": "English", "soon": "planned",
        "groups": [
            ("Overview", [
                ("index.md", "index.html", "Overview"),
                ("algorithms/index.md", "algorithms/index.html", "All algorithms"),
            ]),
            ("Global planning", _GLOBAL + [("benchmarks.md", "benchmarks.html", "Benchmarks")]),
            ("Local planning", _LOCAL + [(None, None, "Benchmarks")]),
            ("Multi-agent", _MULTI + [(None, None, "Benchmarks")]),
            ("Maps", [("maps.md", "maps.html", "Map representations")]),
            ("Reference", [
                ("architecture.md", "architecture.html", "Architecture"),
                ("references.md", "references.html", "References"),
            ]),
        ],
        "topnav": [("index.html", "Overview"), ("algorithms/index.html", "Algorithms"),
                   ("maps.html", "Maps"), ("benchmarks.html", "Benchmarks")],
        "note_title": "Note", "warn_title": "Warning", "onpage": "On this page", "prev": "Prev", "next": "Next",
    },
}

IAL_LINE = re.compile(r"^\{:\s*[.#][^}]*\}\s*$")
IAL_INLINE = re.compile(r"\s*\{:\s*[.#][^}]*\}")
CALLOUT = re.compile(r"^\{:\s*\.(note|warning)\s*\}\s*$")
TOC_MARK = re.compile(r"^1\.\s+TOC\s*$")
LANG_SWITCH = re.compile(r"^\[🇰🇷")
MD_LINK = re.compile(r"\]\(([^)]+?)\.md(#[^)]*)?\)")
FRONT = re.compile(r"^---\n(.*?)\n---\n", re.DOTALL)
TITLE = re.compile(r"^title:\s*(.+?)\s*$", re.MULTILINE)


def parse_front(text: str):
    m = FRONT.match(text)
    if not m:
        return "", text
    tm = TITLE.search(m.group(1))
    return (tm.group(1).strip() if tm else ""), text[m.end():]


def preprocess(body: str, lang: str) -> str:
    lines = body.split("\n")
    out, i = [], 0
    while i < len(lines):
        line = lines[i]
        if LANG_SWITCH.match(line):          # 인라인 언어 스위처 제거 (topbar 토글로 대체)
            i += 1
            continue
        if TOC_MARK.match(line) and i + 1 < len(lines) and lines[i + 1].strip() == "{:toc}":
            i += 2                            # kramdown TOC 마커 제거 (JS 가 우측 TOC 생성)
            continue
        m = CALLOUT.match(line)
        if m:                                 # {: .note } + 블록쿼트 → admonition
            kind = m.group(1)
            title = NAV[lang]["note_title"] if kind == "note" else NAV[lang]["warn_title"]
            i += 1
            body_lines = []
            while i < len(lines) and lines[i].lstrip().startswith(">"):
                body_lines.append(re.sub(r"^\s*>\s?", "", lines[i]))
                i += 1
            out.append(f'!!! {kind} "{title}"')
            for b in body_lines:
                out.append("    " + b if b.strip() else "")
            out.append("")
            continue
        if IAL_LINE.match(line):              # 남은 standalone IAL 제거
            i += 1
            continue
        out.append(line)
        i += 1
    text = "\n".join(out)
    text = IAL_INLINE.sub("", text)           # 인라인 IAL(버튼 등) 제거
    text = MD_LINK.sub(lambda m: "](" + m.group(1) + ".html" + (m.group(2) or "") + ")", text)
    return text


def make_md():
    return markdown.Markdown(
        extensions=["tables", "fenced_code", "footnotes", "sane_lists", "admonition", "toc",
                    "pymdownx.arithmatex"],
        extension_configs={
            "pymdownx.arithmatex": {"generic": True},
            "toc": {"permalink": "¶", "permalink_class": "anchor", "toc_depth": "2-3"},
        },
    )


def search_text(body: str) -> str:
    t = re.sub(r"```.*?```", " ", body, flags=re.DOTALL)
    t = re.sub(r"\$\$.*?\$\$", " ", t, flags=re.DOTALL)
    t = re.sub(r"\$[^$\n]*\$", " ", t)
    t = re.sub(r"[#>|*_`\[\]()!-]", " ", t)
    t = re.sub(r"\s+", " ", t)
    return t.strip()[:5000]


def sidebar_html(lang: str, cur: str, base: str) -> str:
    parts = []
    for gtitle, items in NAV[lang]["groups"]:
        planned = all(src is None for src, _o, _l in items)
        soon = f' <span class="soon">{NAV[lang]["soon"]}</span>' if planned else ""
        parts.append(f'<h4>{html.escape(gtitle)}{soon}</h4>')
        for src, out_rel, label in items:
            if src is None:  # 미구현(예정) — 링크 없이 흐리게
                parts.append(f'<span class="planned">{html.escape(label)}</span>')
                continue
            href = f"{base}{lang}/{out_rel}"
            cls = " class=\"active\"" if out_rel == cur else ""
            parts.append(f'<a href="{href}"{cls}>{html.escape(label)}</a>')
    return "\n".join(parts)


def flat_pages(lang):
    """실제 페이지가 있는 항목만 (planned 제외) — prev/next·빌드 대상."""
    pages = []
    for _g, items in NAV[lang]["groups"]:
        for src, out_rel, label in items:
            if src is None:
                continue
            pages.append((src, out_rel, label))
    return pages


def page_shell(*, title, lang, base, cur_out, content, is_doc):
    cfg = NAV[lang] if lang else NAV["ko"]
    # language toggle
    if lang:
        ko_href = f"{base}ko/{cur_out}"
        en_href = f"{base}en/{cur_out}"
    else:
        ko_href, en_href = f"{base}ko/index.html", f"{base}en/index.html"
    lang_toggle = (
        f'<div class="lang-toggle" role="group" aria-label="language">{GLOBE_SVG}'
        f'<a href="{ko_href}" class="{"active" if lang=="ko" else ""}">KO</a>'
        f'<span class="l-sep">/</span>'
        f'<a href="{en_href}" class="{"active" if lang=="en" else ""}">EN</a></div>'
    )
    topnav = ""
    if lang:
        topnav = '<nav class="topnav">' + "".join(
            f'<a href="{base}{lang}/{o}">{html.escape(l)}</a>' for o, l in cfg["topnav"]
        ) + "</nav>"

    search = (
        f'<div class="search">{SEARCH_SVG}<input id="search-input" type="search" '
        f'placeholder="검색 / Search…" data-base="{base}" autocomplete="off">'
        f'<div id="search-results" class="search-results"></div></div>'
    )

    head = f"""<!doctype html>
<html lang="{lang or 'ko'}">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)} · navigation study</title>
<link rel="preconnect" href="https://fonts.googleapis.com">
<link rel="preconnect" href="https://fonts.gstatic.com" crossorigin>
<link href="https://fonts.googleapis.com/css2?family=Inter:wght@400;500;600;700&display=swap" rel="stylesheet">
<link rel="stylesheet" href="{base}assets/site.css?v={ASSET_VER}">
<link rel="icon" href="{FAVICON}">
<script src="{base}assets/site.js?v={ASSET_VER}"></script>
</head>
<body>
<header class="topbar">
  <button id="menu-btn" class="iconbtn menu-btn" aria-label="menu">{MENU_SVG}</button>
  <a class="brand" href="{base}index.html">{LOGO_SVG}<span class="wm">navigation<span class="wm-dim"> study</span></span></a>
  {topnav}
  <span class="spacer"></span>
  {search}
  {lang_toggle}
</header>
"""

    if is_doc:
        sb = sidebar_html(lang, cur_out, base)
        toc = '<aside class="toc" id="toc"></aside>'
        body = f"""<div class="layout">
<aside class="sidebar">{sb}</aside>
<main class="content"><article class="content-inner">{content}</article></main>
{toc}
</div>
<div class="backdrop"></div>
"""
    else:
        body = content

    foot = f"""<footer class="site-footer">
  <p>navigation study — robot navigation planning algorithms · C++ / Python dual implementation</p>
  <p>Built as a static site · <a href="{base}ko/index.html">한국어</a> · <a href="{base}en/index.html">English</a></p>
</footer>
<script src="https://cdn.jsdelivr.net/npm/mathjax@3/es5/tex-mml-chtml.js" id="MathJax-script" async></script>
</body></html>"""
    return head + body + foot


def build_landing(_=None):
    """미니멀·이미지 중심 랜딩. GIF 3개 + 카테고리별(global/local/multi/maps) 링크."""
    trio = (
        '<a class="gif-card" href="ko/algorithms/astar.html">'
        '<img src="assets/astar/maze01.gif" alt="A* search on maze01" loading="lazy">'
        '<div class="cap"><b>A*</b><span>informed search</span></div></a>'
        '<a class="gif-card" href="ko/algorithms/rrt_star.html">'
        '<img src="assets/rrt_star/maze01.gif" alt="RRT* on maze01" loading="lazy">'
        '<div class="cap"><b>RRT*</b><span>asymptotically optimal</span></div></a>'
        '<a class="gif-card" href="ko/algorithms/fast_rrt.html">'
        '<img src="assets/fast_rrt/maze01.gif" alt="Fast-RRT on maze01" loading="lazy">'
        '<div class="cap"><b>Fast-RRT</b><span>Fast-Sampling + shortcut</span></div></a>'
    )
    glob = [("bfs", "BFS"), ("dijkstra", "Dijkstra"), ("astar", "A*"),
            ("rrt", "RRT"), ("rrt_star", "RRT*"), ("fast_rrt", "Fast-RRT")]
    g_chips = "".join(f'<a href="ko/algorithms/{k}.html">{n}</a>' for k, n in glob)
    l_chips = "".join(f'<span class="dim">{n}</span>' for n in ("DWA", "Pure Pursuit", "VFH", "MPC"))
    m_chips = "".join(f'<span class="dim">{n}</span>' for n in ("Prioritized A*", "Joint-space A*", "CBS"))
    map_chips = ('<a href="ko/maps.html">OccupancyGrid2D</a>'
                 + "".join(f'<span class="dim">{n}</span>' for n in ("GraphMap", "TopologyMap", "ContinuousMap")))
    soon = '<span class="soon">예정</span>'
    content = f"""
<main class="lander">
  <div class="lander-top">
    {LOGO_SVG}
    <h1>navigation<span class="wm-dim"> study</span></h1>
    <p class="sub">로봇 경로 계획(navigation planning) 알고리즘 — C++ / Python 이중 구현 · 단계별 시각화 · 벤치마크</p>
    <div class="lander-btns">
      <a class="btn btn-primary" href="ko/index.html">한국어 문서</a>
      <a class="btn btn-ghost" href="en/index.html">English Docs</a>
    </div>
  </div>

  <div class="gif-trio">{trio}</div>

  <div class="lander-cats">
    <div class="lander-cat"><h3>Global planning</h3><div class="chips">{g_chips}</div></div>
    <div class="lander-cat"><h3>Local planning {soon}</h3><div class="chips">{l_chips}</div></div>
    <div class="lander-cat"><h3>Multi-agent {soon}</h3><div class="chips">{m_chips}</div></div>
    <div class="lander-cat"><h3>Map representations</h3><div class="chips">{map_chips}</div></div>
  </div>
</main>
"""
    return page_shell(title="navigation study", lang=None, base="", cur_out="index.html",
                      content=content, is_doc=False)


def main():
    md = make_md()
    search_index = []

    for lang in ("ko", "en"):
        pages = flat_pages(lang)
        for idx, (src, out_rel, label) in enumerate(pages):
            src_path = DOCS / lang / src
            raw = src_path.read_text(encoding="utf-8")
            title, body = parse_front(raw)
            title = title or label
            pre = preprocess(body, lang)
            md.reset()
            content = md.convert(pre)
            # 첫 표 = spec chip table
            content = content.replace("<table>", '<table class="spec-table">', 1)
            # 알고리즘 페이지: H1 아래에 소스 코드 링크 chip
            if lang and out_rel.startswith("algorithms/") and out_rel != "algorithms/index.html":
                content = content.replace("</h1>", "</h1>\n" + code_chips(out_rel[11:-5], lang), 1)

            # prev / next
            pager = ""
            prev_p = pages[idx - 1] if idx > 0 else None
            next_p = pages[idx + 1] if idx < len(pages) - 1 else None
            depth = out_rel.count("/") + 1  # lang/ + subdirs
            base = "../" * depth

            def rel_to(o):
                return base + lang + "/" + o

            left = (f'<a href="{rel_to(prev_p[1])}"><div class="dir">← {NAV[lang]["prev"]}</div>'
                    f'<div class="ttl">{html.escape(prev_p[2])}</div></a>' if prev_p else "<span></span>")
            right = (f'<a class="next" href="{rel_to(next_p[1])}"><div class="dir">{NAV[lang]["next"]} →</div>'
                     f'<div class="ttl">{html.escape(next_p[2])}</div></a>' if next_p else "<span></span>")
            content += f'<nav class="pager">{left}{right}</nav>'

            page = page_shell(title=title, lang=lang, base=base, cur_out=out_rel,
                              content=content, is_doc=True)
            out_path = DOCS / lang / out_rel
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(page, encoding="utf-8")

            crumb = f'{NAV[lang]["name"]} · {label}'
            search_index.append({
                "title": title,
                "crumb": crumb,
                "url": f"{lang}/{out_rel}",
                "text": search_text(pre),
            })

    # landing
    (DOCS / "index.html").write_text(build_landing(None), encoding="utf-8")

    (DOCS / "assets" / "search-index.json").write_text(
        json.dumps(search_index, ensure_ascii=False), encoding="utf-8")
    (DOCS / ".nojekyll").write_text("", encoding="utf-8")

    print(f"built {len(search_index)} doc pages + landing + search index")


if __name__ == "__main__":
    main()

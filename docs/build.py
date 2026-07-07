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
    '<defs><linearGradient id="navlg" x1="0" y1="1" x2="1" y2="0">'
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
# 외부 링크 표시 화살표 (topnav 의 GitHub 등). robotics-study 형제 사이트와 동일 모티프.
EXT_SVG = (
    '<svg width="13" height="13" viewBox="0 0 24 24" fill="none" stroke="currentColor" '
    'stroke-width="2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">'
    '<path d="M7 17 17 7M8 7h9v9"/></svg>'
)
# 상위 학습 아카이브 허브. 이 사이트는 그 하위 프로젝트라 브랜드 breadcrumb 로 되돌아갈 링크를 준다.
HUB_URL = "https://robotics-study.github.io/"
GH_REPO = "https://github.com/robotics-study/navigation"
# 파비콘: 로고와 동일한 route 마크 (data URI).
FAVICON = (
    "data:image/svg+xml,"
    "%3Csvg xmlns='http://www.w3.org/2000/svg' viewBox='0 0 24 24' fill='none'%3E"
    "%3Crect width='24' height='24' rx='6' fill='%230b1020'/%3E"
    "%3Cpath d='M4 19V11h8V5h8' stroke='%23818cf8' stroke-width='2.2' stroke-linecap='round' stroke-linejoin='round'/%3E"
    "%3Ccircle cx='4' cy='19' r='2.4' fill='%23818cf8'/%3E%3Ccircle cx='20' cy='5' r='2.4' fill='%2322d3ee'/%3E%3C/svg%3E"
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

# 배포 사이트 절대 URL — canonical / og:url / hreflang / sitemap 의 기준.
# GitHub Pages 서빙 경로(README 참조)와 일치해야 한다. 도메인 변경 시 이 한 줄만 고친다.
SITE_URL = "https://robotics-study.github.io/navigation/"
# 알고리즘이 아닌 페이지의 기본 공유 이미지 (A* 최종 경로 — 로고 모티프와 동일).
DEFAULT_OG_IMAGE = "assets/astar/maze01_final.png"


# global_planning 알고리즘의 계보 카테고리 → 소스가 놓인 하위폴더.
# C++/Python 모두 이 하위폴더 아래로 이동했으므로 chip 경로를 함께 갱신한다.
_GLOBAL_LINEAGE = {
    "bfs": "search", "dijkstra": "search", "astar": "search", "jps": "search",
    "ara_star": "search", "ad_star": "search", "anya": "search",
    "dstar_lite": "search", "theta_star": "search", "lazy_theta_star": "search",
    "visibility_astar": "search", "hybrid_astar": "search",
    "rrt": "sampling", "rrt_connect": "sampling", "rrt_star": "sampling",
    "lqr_rrt_star": "sampling",
    "kinodynamic_rrt_star": "sampling", "informed_rrt_star": "sampling", "fast_rrt": "sampling",
    "prm": "sampling", "prm_star": "sampling",
    "bit_star": "sampling", "abit_star": "sampling", "fmt_star": "sampling", "sst": "sampling",
    "ait_star": "sampling", "eit_star": "sampling", "fcit_star": "sampling",
}


def code_chips(key: str, lang: str) -> str:
    label = "소스 코드" if lang == "ko" else "Source"
    cat = _GLOBAL_LINEAGE[key]
    def a(href, text):
        return f'<a href="{GH_BLOB}/{href}" target="_blank" rel="noopener">{text}</a>'
    return (
        f'<div class="code-links"><span class="cl-label">{label}</span>'
        + a(f"cpp/src/global_planning/{cat}/{key}.cpp", "C++")
        + a(f"cpp/include/navigation/global_planning/{cat}/{key}.hpp", "C++ header")
        + a(f"python/navigation/global_planning/{cat}/{key}.py", "Python")
        + a(f"python/demos/demo_{key}.py", "demo")
        + "</div>"
    )

# 언어별 사이드바 구조 + 순서 (prev/next 는 이 평탄화 순서를 따른다).
# 각 항목: (source md 경로[lang 상대], 출력 html 경로[lang 상대], 사이드바 라벨)
# global_planning 계보 서브그룹 — 사이드바 disclosure 단위 + prev/next 순서.
# 각 그룹은 원 논문 연도 오름차순으로 정렬한다.
_GLOBAL_SEARCH = [
    ("algorithms/bfs.md", "algorithms/bfs.html", "BFS"),              # 1959
    ("algorithms/dijkstra.md", "algorithms/dijkstra.html", "Dijkstra"),  # 1959
    ("algorithms/astar.md", "algorithms/astar.html", "A*"),          # 1968
    ("algorithms/dstar_lite.md", "algorithms/dstar_lite.html", "D* Lite"),  # 2002
    ("algorithms/ara_star.md", "algorithms/ara_star.html", "ARA*"),  # 2003
    ("algorithms/ad_star.md", "algorithms/ad_star.html", "AD*"),  # 2005
    ("algorithms/theta_star.md", "algorithms/theta_star.html", "Theta*"),  # 2007
    ("algorithms/hybrid_astar.md", "algorithms/hybrid_astar.html", "Hybrid A*"),  # 2008
    ("algorithms/lazy_theta_star.md", "algorithms/lazy_theta_star.html", "Lazy Theta*"),  # 2010
    ("algorithms/jps.md", "algorithms/jps.html", "JPS"),  # 2011
    ("algorithms/visibility_astar.md", "algorithms/visibility_astar.html", "Visibility A*"),  # any-angle
    ("algorithms/anya.md", "algorithms/anya.html", "Anya"),  # 2016 · optimal any-angle
]
_GLOBAL_SAMPLING = [
    ("algorithms/prm.md", "algorithms/prm.html", "PRM"),             # 1996
    ("algorithms/rrt.md", "algorithms/rrt.html", "RRT"),             # 1998
    ("algorithms/rrt_connect.md", "algorithms/rrt_connect.html", "RRT-Connect"),  # 2000
    ("algorithms/rrt_star.md", "algorithms/rrt_star.html", "RRT*"),  # 2011
    ("algorithms/prm_star.md", "algorithms/prm_star.html", "PRM*"),  # 2011
    ("algorithms/lqr_rrt_star.md", "algorithms/lqr_rrt_star.html", "LQR-RRT*"),  # 2012
    ("algorithms/kinodynamic_rrt_star.md", "algorithms/kinodynamic_rrt_star.html", "Kinodynamic RRT*"),  # 2013
    ("algorithms/informed_rrt_star.md", "algorithms/informed_rrt_star.html", "Informed RRT*"),  # 2014
    ("algorithms/fmt_star.md", "algorithms/fmt_star.html", "FMT*"),  # 2015
    ("algorithms/bit_star.md", "algorithms/bit_star.html", "BIT*"),  # 2015
    ("algorithms/sst.md", "algorithms/sst.html", "SST"),  # 2016
    ("algorithms/abit_star.md", "algorithms/abit_star.html", "ABIT*"),  # 2020
    ("algorithms/ait_star.md", "algorithms/ait_star.html", "AIT*"),  # 2020
    ("algorithms/fast_rrt.md", "algorithms/fast_rrt.html", "Fast-RRT"),  # 2021
    ("algorithms/eit_star.md", "algorithms/eit_star.html", "EIT*"),  # 2022
    ("algorithms/fcit_star.md", "algorithms/fcit_star.html", "FCIT*"),  # 2025
]
_GLOBAL = _GLOBAL_SEARCH + _GLOBAL_SAMPLING
# 사이드바용: 계보별 disclosure 서브그룹 (dict = <details> 로 렌더).
_GLOBAL_NAV = [
    {"sub": "Search", "items": _GLOBAL_SEARCH},
    {"sub": "Sampling", "items": _GLOBAL_SAMPLING},
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
            ("Global planning", _GLOBAL_NAV + [("benchmarks.md", "benchmarks.html", "벤치마크")]),
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
            ("Global planning", _GLOBAL_NAV + [("benchmarks.md", "benchmarks.html", "Benchmarks")]),
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
DESC = re.compile(r"^description:\s*(.+?)\s*$", re.MULTILINE)


def parse_front(text: str):
    m = FRONT.match(text)
    if not m:
        return "", "", text
    fm = m.group(1)
    tm = TITLE.search(fm)
    dm = DESC.search(fm)
    return (tm.group(1).strip() if tm else ""), (dm.group(1).strip() if dm else ""), text[m.end():]


def meta_description(pre: str, limit: int = 155) -> str:
    """첫 실제 산문 문단에서 SEO description 추출 (front-matter description 없을 때 fallback).

    heading/표/blockquote/admonition/코드블록/각주정의를 건너뛰고 첫 문장을 정리·절단한다.
    """
    in_code = False
    for raw in pre.split("\n"):
        s = raw.strip()
        if s.startswith("```"):
            in_code = not in_code
            continue
        if in_code or not s:
            continue
        if s[0] in "#|>!" or s.startswith("[^") or s.startswith("{:") or s.startswith("$$"):
            continue
        text = re.sub(r"\[\^[^\]]+\]", "", s)              # 각주 참조 [^ref] 제거
        text = re.sub(r"\[([^\]]+)\]\([^)]+\)", r"\1", text)  # [txt](url) → txt
        text = re.sub(r"\\(.)", r"\1", text)               # 백슬래시 이스케이프 해제 (\* → *)
        # bold/code/strike 마커만 제거 — 알고리즘 이름의 별표(BIT*, PRM*)는 보존.
        text = text.replace("**", "").replace("__", "").replace("~~", "").replace("`", "")
        text = re.sub(r"\s+", " ", text).strip()
        if len(text) < 30:
            continue
        if len(text) > limit:
            text = text[:limit].rsplit(" ", 1)[0].rstrip(",.;:") + "…"
        return text
    return ""


def seo_meta(*, title: str, description: str, lang: str | None, cur_out: str,
             image_rel: str, is_article: bool) -> str:
    """페이지별 SEO/social/hreflang 메타 블록 (절대 URL 기준)."""
    desc = html.escape(description, quote=True)
    full_title = html.escape(f"{title} · navigation study", quote=True)
    page_url = SITE_URL + (f"{lang}/{cur_out}" if lang else "index.html")
    img_url = SITE_URL + image_rel
    og_locale = {"ko": "ko_KR", "en": "en_US"}.get(lang or "ko", "ko_KR")
    alt_locale = "en_US" if lang == "ko" else "ko_KR"

    tags = [
        f'<meta name="description" content="{desc}">',
        '<meta name="robots" content="index,follow">',
        '<meta name="theme-color" content="#ffffff" media="(prefers-color-scheme: light)">',
        '<meta name="theme-color" content="#0b1020" media="(prefers-color-scheme: dark)">',
        f'<link rel="canonical" href="{page_url}">',
    ]
    # bilingual hreflang: 같은 out_rel 이 ko/en 양쪽에 존재. x-default 는 영어로.
    if lang:
        alts = [("ko", f"{SITE_URL}ko/{cur_out}"), ("en", f"{SITE_URL}en/{cur_out}"),
                ("x-default", f"{SITE_URL}en/{cur_out}")]
    else:
        alts = [("ko", f"{SITE_URL}ko/index.html"), ("en", f"{SITE_URL}en/index.html"),
                ("x-default", SITE_URL)]
    tags += [f'<link rel="alternate" hreflang="{hl}" href="{href}">' for hl, href in alts]
    tags += [
        f'<meta property="og:type" content="{"article" if is_article else "website"}">',
        '<meta property="og:site_name" content="navigation study">',
        f'<meta property="og:title" content="{full_title}">',
        f'<meta property="og:description" content="{desc}">',
        f'<meta property="og:url" content="{page_url}">',
        f'<meta property="og:image" content="{img_url}">',
        f'<meta property="og:image:alt" content="{html.escape(title, quote=True)}">',
        f'<meta property="og:locale" content="{og_locale}">',
        f'<meta property="og:locale:alternate" content="{alt_locale}">',
        '<meta name="twitter:card" content="summary_large_image">',
        f'<meta name="twitter:title" content="{full_title}">',
        f'<meta name="twitter:description" content="{desc}">',
        f'<meta name="twitter:image" content="{img_url}">',
    ]
    if is_article:
        ld = {"@context": "https://schema.org", "@type": "TechArticle", "headline": title,
              "description": description, "image": img_url, "inLanguage": lang, "url": page_url,
              "isPartOf": {"@type": "WebSite", "name": "navigation study", "url": SITE_URL}}
    else:
        ld = {"@context": "https://schema.org", "@type": "WebSite", "name": "navigation study",
              "url": SITE_URL, "description": description, "inLanguage": ["ko", "en"]}
    tags.append('<script type="application/ld+json">' + json.dumps(ld, ensure_ascii=False) + "</script>")
    return "\n".join(tags)


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


def _nav_leaf(src, out_rel, label, lang, cur, base) -> str:
    if src is None:  # 미구현(예정) — 링크 없이 흐리게
        return f'<span class="planned">{html.escape(label)}</span>'
    cls = " class=\"active\"" if out_rel == cur else ""
    return f'<a href="{base}{lang}/{out_rel}"{cls}>{html.escape(label)}</a>'


def _all_planned(item) -> bool:
    subs = item["items"] if isinstance(item, dict) else [item]
    return all(src is None for src, _o, _l in subs)


def sidebar_html(lang: str, cur: str, base: str) -> str:
    parts = []
    for gtitle, items in NAV[lang]["groups"]:
        planned = all(_all_planned(it) for it in items)
        soon = f' <span class="soon">{NAV[lang]["soon"]}</span>' if planned else ""
        parts.append(f'<h4>{html.escape(gtitle)}{soon}</h4>')
        for it in items:
            if isinstance(it, dict):  # 계보 disclosure 서브그룹 → <details>
                links = "".join(_nav_leaf(*x, lang, cur, base) for x in it["items"])
                # 현재 페이지가 속한 그룹만 펼친 채로 렌더 (나머지는 접힘).
                open_attr = " open" if any(o == cur for _s, o, _l in it["items"]) else ""
                parts.append(
                    f'<details class="nav-sub"{open_attr}>'
                    f'<summary>{html.escape(it["sub"])}</summary>'
                    # 링크를 nav-sub-inner 로 감싸 grid-rows(0fr↔1fr) 열림/닫힘 애니메이션이
                    # 단일 클리핑 자식을 갖도록 한다 (site.css 참조).
                    f'<div class="nav-sub-body"><div class="nav-sub-inner">{links}</div></div></details>'
                )
            else:
                parts.append(_nav_leaf(*it, lang, cur, base))
    return "\n".join(parts)


def flat_pages(lang):
    """실제 페이지가 있는 항목만 (planned 제외) — prev/next·빌드 대상."""
    pages = []
    for _g, items in NAV[lang]["groups"]:
        for it in items:
            subitems = it["items"] if isinstance(it, dict) else [it]
            for src, out_rel, label in subitems:
                if src is not None:
                    pages.append((src, out_rel, label))
    return pages


def page_shell(*, title, lang, base, cur_out, content, is_doc,
               description="", image_rel=DEFAULT_OG_IMAGE, is_article=False):
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
        topnav = ('<nav class="topnav">' + "".join(
            f'<a href="{base}{lang}/{o}">{html.escape(lbl)}</a>' for o, lbl in cfg["topnav"]
        ) + f'<a href="{GH_REPO}" target="_blank" rel="noopener">GitHub{EXT_SVG}</a></nav>')

    search = (
        f'<div class="search">{SEARCH_SVG}<input id="search-input" type="search" '
        f'placeholder="검색 / Search…" data-base="{base}" autocomplete="off">'
        f'<div id="search-results" class="search-results"></div></div>'
    )

    seo = seo_meta(title=title, description=description, lang=lang, cur_out=cur_out,
                   image_rel=image_rel, is_article=is_article)
    head = f"""<!doctype html>
<html lang="{lang or 'ko'}">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>{html.escape(title)} · navigation study</title>
{seo}
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
  <div class="brandcrumb">
    <a class="brand hub" href="{HUB_URL}" aria-label="robotics-study home">{LOGO_SVG}<span class="wm">robotics<span class="wm-dim"> study</span></span></a>
    <span class="crumb-sep" aria-hidden="true">/</span>
    <a class="brand" href="{base}index.html"><span class="wm">navigation<span class="wm-dim"> study</span></span></a>
  </div>
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
    """미니멀·이미지 중심 랜딩. 구현된 전 global 알고리즘 GIF 그리드 + 카테고리 링크."""
    # 계보 순서. 모든 구현 알고리즘이 demo GIF 카드를 갖는다 (신규 포함 — 누락 방지).
    gallery = [
        ("bfs", "BFS", "uninformed search"),
        ("dijkstra", "Dijkstra", "cost-optimal"),
        ("astar", "A*", "informed search"),
        ("jps", "JPS", "grid symmetry breaking"),
        ("ara_star", "ARA*", "anytime repairing"),
        ("ad_star", "AD*", "anytime dynamic replanning"),
        ("dstar_lite", "D* Lite", "dynamic replanning"),
        ("theta_star", "Theta*", "any-angle search"),
        ("lazy_theta_star", "Lazy Theta*", "any-angle · lazy LOS"),
        ("visibility_astar", "Visibility A*", "any-angle · visibility graph"),
        ("anya", "Anya", "optimal any-angle · interval"),
        ("hybrid_astar", "Hybrid A*", "kinodynamic SE(2)"),
        ("prm", "PRM", "roadmap · multi-query"),
        ("rrt", "RRT", "feasible sampling"),
        ("rrt_connect", "RRT-Connect", "bidirectional single-query"),
        ("rrt_star", "RRT*", "asymptotically optimal"),
        ("prm_star", "PRM*", "optimal roadmap"),
        ("lqr_rrt_star", "LQR-RRT*", "LQR-derived heuristics"),
        ("kinodynamic_rrt_star", "Kinodynamic RRT*", "kinodynamic optimal"),
        ("informed_rrt_star", "Informed RRT*", "ellipsoidal informed"),
        ("fmt_star", "FMT*", "fast marching tree"),
        ("bit_star", "BIT*", "batch informed trees"),
        ("sst", "SST", "sparse kinodynamic"),
        ("abit_star", "ABIT*", "advanced batch informed"),
        ("ait_star", "AIT*", "adaptively informed trees"),
        ("fast_rrt", "Fast-RRT", "Fast-Sampling + shortcut"),
        ("eit_star", "EIT*", "effort informed trees"),
        ("fcit_star", "FCIT*", "fully connected informed"),
    ]
    cards = "".join(
        f'<a class="gif-card" href="ko/algorithms/{k}.html">'
        f'<img src="assets/{k}/maze01.gif" alt="{html.escape(n)} on maze01" loading="lazy">'
        f'<div class="cap"><b>{html.escape(n)}</b><span>{html.escape(d)}</span></div></a>'
        for k, n, d in gallery
    )
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

  <div class="gif-grid">{cards}</div>

  <div class="lander-cats">
    <div class="lander-cat"><h3>Local planning {soon}</h3><div class="chips">{l_chips}</div></div>
    <div class="lander-cat"><h3>Multi-agent {soon}</h3><div class="chips">{m_chips}</div></div>
    <div class="lander-cat"><h3>Map representations</h3><div class="chips">{map_chips}</div></div>
  </div>
</main>
"""
    desc = ("로봇 경로 계획(navigation planning) 알고리즘을 C++ 와 Python 으로 이중 구현한 "
            "학습용 스터디 — 탐색(BFS·Dijkstra·A*)과 샘플링(RRT·RRT*·PRM·FMT*·BIT*) 계열을 "
            "단계별 시각화와 벤치마크로 비교한다.")
    # 랜딩 title 은 브랜드명을 중복하지 않도록 서술형으로 (template 이 " · navigation study" 를 붙임).
    return page_shell(title="robot navigation planning algorithms", lang=None, base="",
                      cur_out="index.html", content=content, is_doc=False, description=desc,
                      image_rel=DEFAULT_OG_IMAGE, is_article=False)


def main():
    md = make_md()
    search_index = []
    sitemap_urls = [SITE_URL]  # 랜딩

    for lang in ("ko", "en"):
        pages = flat_pages(lang)
        for idx, (src, out_rel, label) in enumerate(pages):
            src_path = DOCS / lang / src
            raw = src_path.read_text(encoding="utf-8")
            title, fm_desc, body = parse_front(raw)
            title = title or label
            pre = preprocess(body, lang)
            md.reset()
            content = md.convert(pre)
            # 첫 표 = spec chip table
            content = content.replace("<table>", '<table class="spec-table">', 1)
            # 알고리즘 페이지: H1 아래에 소스 코드 링크 chip
            is_algo = out_rel.startswith("algorithms/") and out_rel != "algorithms/index.html"
            if lang and is_algo:
                content = content.replace("</h1>", "</h1>\n" + code_chips(out_rel[11:-5], lang), 1)
            # SEO: front-matter description 우선, 없으면 첫 문단에서 유도. og:image 는
            # 알고리즘이면 해당 알고리즘 최종 경로 PNG, 그 외 기본 이미지.
            key = out_rel[11:-5] if is_algo else None
            description = fm_desc or meta_description(pre) or f"{title} — navigation study."
            image_rel = f"assets/{key}/maze01_final.png" if key else DEFAULT_OG_IMAGE

            # prev / next
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
                              content=content, is_doc=True, description=description,
                              image_rel=image_rel, is_article=bool(key))
            out_path = DOCS / lang / out_rel
            out_path.parent.mkdir(parents=True, exist_ok=True)
            out_path.write_text(page, encoding="utf-8")
            sitemap_urls.append(f"{SITE_URL}{lang}/{out_rel}")

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

    # sitemap.xml + robots.txt — 크롤러가 전 페이지를 발견하고 색인하도록.
    body = "".join(f"  <url><loc>{html.escape(u)}</loc></url>\n" for u in sitemap_urls)
    (DOCS / "sitemap.xml").write_text(
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        '<urlset xmlns="http://www.sitemaps.org/schemas/sitemap/0.9">\n'
        f"{body}</urlset>\n",
        encoding="utf-8")
    (DOCS / "robots.txt").write_text(
        f"User-agent: *\nAllow: /\n\nSitemap: {SITE_URL}sitemap.xml\n", encoding="utf-8")

    print(f"built {len(search_index)} doc pages + landing + search index "
          f"+ sitemap ({len(sitemap_urls)} urls) + robots.txt")


if __name__ == "__main__":
    main()

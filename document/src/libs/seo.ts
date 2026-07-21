// SPA 라 페이지 전환은 리로드 없이 일어난다. 크롤러/링크 프리뷰가 현재 뷰를 반영하도록
// document.title, description, Open Graph, canonical, hreflang, JSON-LD 를 클라이언트에서
// 갱신한다. index.html 에 정적으로 심어 둔 태그를 찾아 값만 바꾸고, 없으면 만든다.
// 설명문은 마케팅 문구가 아니라 학습 내용(주제·개념) 중심으로 쓴다.

import {Lang, pick} from "./i18n";
import {IAlgoData} from "../../types/global";
import {ALGO_BLURBS} from "../pages/algorithms/roadmap";

const ORIGIN = "https://robotics-study.github.io";
const BASE_PATH = "/navigation/";

const SITE: Record<Lang, string> = {
    en: "Navigation · Study",
    ko: "내비게이션 · 스터디",
}

function upsertMeta(attr: "name" | "property", key: string, content: string) {
    let el = document.head.querySelector<HTMLMetaElement>(`meta[${attr}="${key}"]`)
    if (!el) {
        el = document.createElement("meta")
        el.setAttribute(attr, key)
        document.head.appendChild(el)
    }
    el.setAttribute("content", content)
}

function upsertLink(rel: string, href: string, hreflang?: string) {
    const selector = hreflang
        ? `link[rel="${rel}"][hreflang="${hreflang}"]`
        : `link[rel="${rel}"]`
    let el = document.head.querySelector<HTMLLinkElement>(selector)
    if (!el) {
        el = document.createElement("link")
        el.rel = rel
        if (hreflang) el.hreflang = hreflang
        document.head.appendChild(el)
    }
    el.href = href
}

function upsertJsonLd(id: string, data: object) {
    let el = document.head.querySelector<HTMLScriptElement>(`script#${id}`)
    if (!el) {
        el = document.createElement("script")
        el.id = id
        el.type = "application/ld+json"
        document.head.appendChild(el)
    }
    el.textContent = JSON.stringify(data)
}

function clamp(text: string, max = 155): string {
    return text.length <= max ? text : text.slice(0, max - 1).trimEnd() + "…"
}

// 해시·잡다한 파라미터를 뺀 정규화 URL. 알고리즘은 /algo/<slug>/ 경로, 언어 변형은 ?lang=ko.
export function pageUrl(lang: Lang, slug?: string): string {
    const path = slug !== undefined ? `algo/${slug}/` : ""
    const qs = lang === "ko" ? "?lang=ko" : ""
    return `${ORIGIN}${BASE_PATH}${path}${qs}`
}

declare global {
    interface Window {
        gtag?: (...args: unknown[]) => void
    }
}

// SPA 라우트 변경마다 GA4 page_view 를 직접 보낸다 (index.html 은 send_page_view: false).
// 로컬 개발 트래픽은 집계를 오염시키므로 배포 호스트에서만 보낸다.
function trackPageView(title: string) {
    if (!window.location.hostname.endsWith("github.io")) return
    window.gtag?.("event", "page_view", {
        page_title: title,
        page_location: window.location.href,
        page_path: window.location.pathname + window.location.search,
    })
}

export interface PageMeta {
    title: string
    description: string
    lang: Lang
    algo?: IAlgoData
}

export function applyPageMeta({title, description, lang, algo}: PageMeta) {
    const desc = clamp(description)
    const canonical = pageUrl(lang, algo?.slug)
    document.title = title
    document.documentElement.lang = lang
    upsertMeta("name", "description", desc)
    upsertMeta("property", "og:title", title)
    upsertMeta("property", "og:description", desc)
    upsertMeta("property", "og:url", canonical)
    upsertMeta("property", "og:locale", lang === "ko" ? "ko_KR" : "en_US")
    upsertMeta("name", "twitter:title", title)
    upsertMeta("name", "twitter:description", desc)
    upsertLink("canonical", canonical)
    trackPageView(title)
    // 언어별 대체 URL: 같은 페이지의 en/ko 쌍.
    upsertLink("alternate", pageUrl("en", algo?.slug), "en")
    upsertLink("alternate", pageUrl("ko", algo?.slug), "ko")
    upsertLink("alternate", pageUrl("en", algo?.slug), "x-default")
    // 페이지 구조화 데이터.
    if (algo) {
        upsertJsonLd("page-jsonld", {
            "@context": "https://schema.org",
            "@type": "TechArticle",
            headline: title,
            description: desc,
            inLanguage: lang,
            url: canonical,
            isPartOf: {
                "@type": "WebSite",
                name: SITE[lang],
                url: pageUrl(lang),
            },
            about: (algo.sections ?? []).map((s) => pick(lang, s)),
        })
    } else {
        upsertJsonLd("page-jsonld", {
            "@context": "https://schema.org",
            "@type": "LearningResource",
            name: SITE[lang],
            description: desc,
            url: canonical,
            inLanguage: ["en", "ko"],
            learningResourceType: "Study notes",
            about: [
                "Robotics", "Navigation", "Path Planning", "Motion Planning",
                "Graph Search", "A*", "Dijkstra", "Sampling-Based Planning", "RRT", "PRM",
            ],
        })
    }
}

const HOME_DESC: Record<Lang, string> = {
    en:
        "Study notes on robot navigation algorithms: graph search (Dijkstra, A*, D* Lite, " +
        "Theta*, JPS, Anya) and sampling-based planners (RRT, RRT*, PRM, Informed RRT*, BIT*), " +
        "with interactive step-by-step search visualizations and C++/Python implementations.",
    ko:
        "로봇 navigation 알고리즘 학습 노트: graph search (Dijkstra, A*, D* Lite, Theta*, " +
        "JPS, Anya) 와 sampling 기반 planner (RRT, RRT*, PRM, Informed RRT*, BIT*). " +
        "탐색 과정을 step-by-step 으로 재생하는 인터랙티브 시각화와 C++/Python 구현.",
}

// 알고리즘 → 페이지 메타. 설명은 한 줄 소개(내용 요약) + 주요 절 제목으로 만든다.
export function algoMeta(lang: Lang, algo?: IAlgoData): PageMeta {
    if (!algo) {
        return {title: SITE[lang], description: HOME_DESC[lang], lang}
    }
    const title = pick(lang, algo.title)
    const blurb = ALGO_BLURBS.find((b) => b.slug === algo.slug)?.blurb
    const topics = (algo.sections ?? []).map((s) => pick(lang, s)).join(", ")
    const intro = blurb ? pick(lang, blurb) : ""
    const body = lang === "ko"
        ? `${intro} ${topics ? `주요 내용: ${topics}.` : ""}`.trim()
        : `${intro} ${topics ? `Topics: ${topics}.` : ""}`.trim()
    return {
        title: `${title} · ${SITE[lang]}`,
        description: body,
        lang,
        algo,
    }
}

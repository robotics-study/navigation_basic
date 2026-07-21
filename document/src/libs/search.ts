import algorithms from "../pages/algorithms";
import {slugify} from "./slug";
import {Lang, pick} from "./i18n";

export interface SearchEntry {
    title: string
    crumb: string
    slug: string
    anchor?: string   // 섹션 앵커(slug). 없으면 페이지 최상단.
    text: string      // 매칭용 추가 텍스트
}

// 알고리즘 메타데이터(제목 + 섹션)로 정적 검색 인덱스를 구성한다.
// SPA 라 빌드 대신 런타임에 언어별로 한 번 만든다.
function buildIndex(lang: Lang): SearchEntry[] {
    const entries: SearchEntry[] = []
    for (const a of algorithms) {
        if (!a.contents) continue   // 집필된 페이지만 검색 대상
        const title = pick(lang, a.title)
        entries.push({title, crumb: "Algorithm", slug: a.slug, text: `${title} ${a.slug}`})
        for (const s of a.sections ?? []) {
            const sec = pick(lang, s)
            entries.push({title: sec, crumb: title, slug: a.slug, anchor: slugify(sec), text: sec})
        }
    }
    return entries
}

const INDEX: Record<Lang, SearchEntry[]> = {
    en: buildIndex("en"),
    ko: buildIndex("ko"),
}

export function searchDocs(query: string, lang: Lang, limit = 8): SearchEntry[] {
    const q = query.trim().toLowerCase()
    if (!q) return []
    return INDEX[lang]
        .map((it) => {
            const ti = it.title.toLowerCase()
            const hay = `${it.title} ${it.crumb} ${it.text}`.toLowerCase()
            let score = 0
            if (ti.indexOf(q) === 0) score += 100
            else if (ti.indexOf(q) >= 0) score += 50
            if (hay.indexOf(q) >= 0) score += 10
            return {it, score}
        })
        .filter((x) => x.score > 0)
        .sort((a, b) => b.score - a.score)
        .slice(0, limit)
        .map((x) => x.it)
}

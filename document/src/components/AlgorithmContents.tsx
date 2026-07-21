import {IAlgoData} from "../../types/global";
import 'katex/dist/katex.min.css';
import algorithms from "../pages/algorithms";
import {CATEGORIES, SECTIONS, sectionOf} from "../pages/algorithms/roadmap";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";

const REPO = "https://github.com/robotics-study/navigation"

// supportedExample(python/c++) → 실제 구현 파일로 가는 chip 링크.
// search/sampling 은 global_planning 하위 디렉토리 구조와 slug 가 파일명과 일치한다.
const codeLinkFor = (algo: IAlgoData, language: string): string | null => {
    if (algo.category !== "search" && algo.category !== "sampling") return null
    const sub = `global_planning/${algo.category}`
    if (language === "c++") return `${REPO}/blob/main/cpp/include/navigation/${sub}/${algo.slug}.hpp`
    if (language === "python") return `${REPO}/blob/main/python/navigation/${sub}/${algo.slug}.py`
    return null
}

const AlgorithmContents = (algo: IAlgoData) => {
    const {title, slug, contents: Contents, supportedExample, category} = algo
    const {go, goSection} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    // pager 는 집필된 페이지 사이만 오간다 (registry 배열 순서 = 학습 순서).
    const ready = algorithms.filter((a) => a.contents)
    const idx = ready.findIndex((a) => a.slug === slug)
    const prev = idx > 0 ? ready[idx - 1] : undefined
    const next = idx >= 0 && idx < ready.length - 1 ? ready[idx + 1] : undefined

    // eyebrow: "Planning · Graph Search" 처럼 대분류 · 중분류. 이름이 같으면 하나만.
    const secTitle = SECTIONS.find((s) => s.key === sectionOf(category))?.title
    const catTitle = CATEGORIES.find((c) => c.key === category)?.title

    const codeLinks = supportedExample
        ? Object.entries(supportedExample)
            .filter(([, v]) => v)
            .map(([language]) => ({language, href: codeLinkFor(algo, language)}))
            .filter((l): l is {language: string; href: string} => !!l.href)
        : []

    return (
        <main className="content">
            <article className="content-inner">
                {secTitle && catTitle && (
                    <p className="eyebrow">
                        {pick(lang, secTitle) === pick(lang, catTitle)
                            ? pick(lang, secTitle)
                            : `${pick(lang, secTitle)} · ${pick(lang, catTitle)}`}
                    </p>
                )}
                <h1>{pick(lang, title)}</h1>

                {codeLinks.length > 0 && (
                    <div className="code-links">
                        <span className="cl-label">{t("Source code", "소스 코드")}</span>
                        {codeLinks.map(({language, href}) => (
                            <a key={language} href={href} target="_blank" rel="noopener noreferrer">
                                {language}
                            </a>
                        ))}
                        <a href={`${REPO}/blob/main/python/demos/demo_${slug}.py`}
                           target="_blank" rel="noopener noreferrer">demo</a>
                    </div>
                )}

                {Contents ? <Contents/> : null}

                <nav className="pager">
                    {prev
                        ? <a onClick={() => go(prev.slug)}>
                            <div className="dir">{t("← Prev", "← 이전")}</div>
                            <div className="ttl">{pick(lang, prev.title)}</div>
                        </a>
                        : <a onClick={() => goSection(sectionOf(category))}>
                            <div className="dir">{t("← Prev", "← 이전")}</div>
                            <div className="ttl">
                                {secTitle ? pick(lang, secTitle) : t("Home", "홈")} · Introduction
                            </div>
                        </a>}
                    {next
                        ? <a className="next" onClick={() => go(next.slug)}>
                            <div className="dir">{t("Next →", "다음 →")}</div>
                            <div className="ttl">{pick(lang, next.title)}</div>
                        </a>
                        : <span/>}
                </nav>
            </article>
        </main>
    )
}

export default AlgorithmContents

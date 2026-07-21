import {useEffect, useState} from "react";
import algorithms from "../pages/algorithms";
import {CATEGORIES, SECTIONS, sectionOf} from "../pages/algorithms/roadmap";
import {AlgoSection} from "../../types/global";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";
import cn from "../libs/cn";

const Chevron = () => (
    <svg className="sb-chev" width="11" height="11" viewBox="0 0 24 24" fill="none"
         stroke="currentColor" strokeWidth="2.4" strokeLinecap="round" strokeLinejoin="round"
         aria-hidden="true">
        <path d="m9 6 6 6-6 6"/>
    </svg>
)

// 좌측 알고리즘 네비게이션 — 대분류(Planning/Control/Multi-Agent) disclosure 안에
// 중분류(Graph Search/Sampling …) sub-label 로 묶는다. 알고리즘이 수십 개라 현재 페이지가
// 속한 대분류만 기본으로 펼친다. 집필된 페이지는 링크, 미집필은 dim 처리.
const Sidebar = ({open: mobileOpen, onNavigate}: { open?: boolean; onNavigate?: () => void }) => {
    const {current, go} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    const currentAlgo = algorithms.find((a) => a.slug === current)
    const currentSection = currentAlgo ? sectionOf(currentAlgo.category) : undefined
    // 홈에서는 첫 대분류를 펼쳐 목차 역할을 하게 한다.
    const defaultOpen = currentSection ?? SECTIONS[0].key
    const [opened, setOpened] = useState<Set<AlgoSection>>(() => new Set([defaultOpen]))

    // 검색/카드 등 외부 경로로 페이지가 바뀌면 그 대분류를 펼친다 (사용자 토글은 유지).
    useEffect(() => {
        if (!currentSection) return
        setOpened((prev) => {
            if (prev.has(currentSection)) return prev
            return new Set(prev).add(currentSection)
        })
    }, [currentSection])

    const toggle = (key: AlgoSection) => setOpened((prev) => {
        const next = new Set(prev)
        if (next.has(key)) next.delete(key)
        else next.add(key)
        return next
    })

    const open = (slug: string) => {
        go(slug)
        onNavigate?.()
    }

    return (
        <aside className={cn("sidebar", mobileOpen && "open")}>
            <h4>{t("Overview", "개요")}</h4>
            <a className={cn(!current && "active")} onClick={() => {
                go(null)
                onNavigate?.()
            }}>{t("Home", "홈")}</a>

            {SECTIONS.map((sec) => {
                const items = algorithms.filter((a) => sec.categories.includes(a.category))
                const ready = items.filter((a) => a.contents).length
                const isOpen = opened.has(sec.key)
                const multiCat = sec.categories.length > 1
                return (
                    <div key={sec.key} className={cn("sb-group", isOpen && "open")}>
                        <button type="button" className="sb-head" onClick={() => toggle(sec.key)}
                                aria-expanded={isOpen}>
                            <Chevron/>
                            {pick(lang, sec.title)}
                            {ready === 0 && <span className="soon">soon</span>}
                            <span className="sb-count">{ready}/{items.length}</span>
                        </button>
                        <div className="sb-body">
                            <div>
                                {sec.categories.map((catKey) => {
                                    const cat = CATEGORIES.find((c) => c.key === catKey)!
                                    const catItems = items.filter((a) => a.category === catKey)
                                    return (
                                        <div key={catKey}>
                                            {multiCat && <div className="sb-sub">{pick(lang, cat.title)}</div>}
                                            {catItems.map((a) => a.contents
                                                ? (
                                                    <a key={a.slug}
                                                       className={cn(current === a.slug && "active")}
                                                       onClick={() => open(a.slug)}>
                                                        {pick(lang, a.title)}
                                                    </a>
                                                ) : (
                                                    <span key={a.slug} className="planned">
                                                        {pick(lang, a.title)}
                                                    </span>
                                                ))}
                                        </div>
                                    )
                                })}
                            </div>
                        </div>
                    </div>
                )
            })}
        </aside>
    )
}

export default Sidebar

import {useEffect, useState} from "react";
import algorithms from "../pages/algorithms";
import categoryIntros from "../pages/categories";
import {CATEGORIES, SECTIONS, sectionOf} from "../pages/algorithms/roadmap";
import {AlgoCategory, AlgoSection} from "../../types/global";
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

// 좌측 알고리즘 네비게이션 — 대분류 disclosure 안에 rail 계층으로 중분류를 묶는다.
// 미집필 항목은 카테고리별 "+ n more soon" 한 줄로 접어, 집필된 페이지 위주로 보여 준다.
const Sidebar = ({open: mobileOpen, onNavigate}: { open?: boolean; onNavigate?: () => void }) => {
    const {current, currentSection: introSection, currentCategory, go, goSection, goCategory} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    const currentAlgo = algorithms.find((a) => a.slug === current)
    const currentSection = currentAlgo ? sectionOf(currentAlgo.category)
        : currentCategory ? sectionOf(currentCategory)
            : introSection ?? undefined
    // 홈에서는 첫 대분류를 펼쳐 목차 역할을 하게 한다.
    const defaultOpen = currentSection ?? SECTIONS[0].key
    const [opened, setOpened] = useState<Set<AlgoSection>>(() => new Set([defaultOpen]))
    // "+ n more soon" 을 눌러 펼친 카테고리들.
    const [plannedShown, setPlannedShown] = useState<Set<AlgoCategory>>(() => new Set())

    // 검색/카드 등 외부 경로로 페이지가 바뀌면 그 대분류를 펼친다 (사용자 토글은 유지).
    useEffect(() => {
        if (!currentSection) return
        setOpened((prev) => {
            if (prev.has(currentSection)) return prev
            return new Set(prev).add(currentSection)
        })
    }, [currentSection])

    const toggleSection = (key: AlgoSection) => setOpened((prev) => {
        const next = new Set(prev)
        if (next.has(key)) next.delete(key)
        else next.add(key)
        return next
    })
    const togglePlanned = (key: AlgoCategory) => setPlannedShown((prev) => {
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
            <a className={cn(!current && !introSection && !currentCategory && "active")} onClick={() => {
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
                        <button type="button" className="sb-head" onClick={() => toggleSection(sec.key)}
                                aria-expanded={isOpen}>
                            <Chevron/>
                            {pick(lang, sec.title)}
                            <span className="sb-count">{ready}/{items.length}</span>
                        </button>
                        <div className="sb-body">
                            <div>
                                <a className={cn(introSection === sec.key && "active")}
                                   onClick={() => {
                                       goSection(sec.key)
                                       onNavigate?.()
                                   }}>
                                    Introduction
                                </a>
                                {sec.categories.map((catKey) => {
                                    const cat = CATEGORIES.find((c) => c.key === catKey)!
                                    const catItems = items.filter((a) => a.category === catKey)
                                    const written = catItems.filter((a) => a.contents)
                                    const planned = catItems.filter((a) => !a.contents)
                                    const hasIntro = categoryIntros.some((c) => c.key === catKey)
                                    const showPlanned = plannedShown.has(catKey)
                                    return (
                                        <div key={catKey}>
                                            {multiCat && (hasIntro
                                                ? <a className={cn("sb-sub", currentCategory === catKey && "active")}
                                                     onClick={() => {
                                                         goCategory(catKey)
                                                         onNavigate?.()
                                                     }}>
                                                    {pick(lang, cat.title)}
                                                </a>
                                                : <div className="sb-sub">{pick(lang, cat.title)}</div>)}
                                            <div className={cn(multiCat && "sb-cat-body")}>
                                                {/* 펼침 상태에서는 레지스트리(학습) 순서를 유지한 채 미집필을 dim 으로 끼워 넣는다 */}
                                                {(showPlanned ? catItems : written).map((a) => a.contents
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
                                                {planned.length > 0 && (
                                                    <button type="button" className="sb-more"
                                                            onClick={() => togglePlanned(catKey)}
                                                            aria-expanded={showPlanned}>
                                                        {showPlanned
                                                            ? t("show less", "show less")
                                                            : `+ ${planned.length} more soon`}
                                                    </button>
                                                )}
                                            </div>
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

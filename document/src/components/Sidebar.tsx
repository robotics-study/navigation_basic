import {useEffect, useState} from "react";
import algorithms from "../pages/algorithms";
import {CATEGORIES} from "../pages/algorithms/roadmap";
import {AlgoCategory} from "../../types/global";
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

// 좌측 알고리즘 네비게이션 — 카테고리별 disclosure. 알고리즘이 카테고리당 십수 개라
// 현재 페이지가 속한 그룹만 기본으로 펼친다. 집필된 페이지는 링크, 미집필은 dim 처리.
const Sidebar = ({open: mobileOpen, onNavigate}: { open?: boolean; onNavigate?: () => void }) => {
    const {current, go} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

    const currentCategory = algorithms.find((a) => a.slug === current)?.category
    // 홈에서는 첫 카테고리를 펼쳐 목차 역할을 하게 한다.
    const defaultOpen = currentCategory ?? CATEGORIES[0].key
    const [opened, setOpened] = useState<Set<AlgoCategory>>(() => new Set([defaultOpen]))

    // 검색/카드 등 외부 경로로 페이지가 바뀌면 그 카테고리를 펼친다 (사용자 토글은 유지).
    useEffect(() => {
        if (!currentCategory) return
        setOpened((prev) => {
            if (prev.has(currentCategory)) return prev
            return new Set(prev).add(currentCategory)
        })
    }, [currentCategory])

    const toggle = (key: AlgoCategory) => setOpened((prev) => {
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

            {CATEGORIES.map((cat) => {
                const items = algorithms.filter((a) => a.category === cat.key)
                const ready = items.filter((a) => a.contents).length
                const isOpen = opened.has(cat.key)
                return (
                    <div key={cat.key} className={cn("sb-group", isOpen && "open")}>
                        <button type="button" className="sb-head" onClick={() => toggle(cat.key)}
                                aria-expanded={isOpen}>
                            <Chevron/>
                            {pick(lang, cat.title)}
                            {ready === 0 && <span className="soon">soon</span>}
                            <span className="sb-count">{ready}/{items.length}</span>
                        </button>
                        <div className="sb-body">
                            <div>
                                {items.map((a) => a.contents
                                    ? (
                                        <a key={a.slug} className={cn(current === a.slug && "active")}
                                           onClick={() => open(a.slug)}>
                                            {pick(lang, a.title)}
                                        </a>
                                    ) : (
                                        <span key={a.slug} className="planned">{pick(lang, a.title)}</span>
                                    ))}
                            </div>
                        </div>
                    </div>
                )
            })}
        </aside>
    )
}

export default Sidebar

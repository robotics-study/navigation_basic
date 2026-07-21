import algorithms from "../pages/algorithms";
import {CATEGORIES} from "../pages/algorithms/roadmap";
import {useAlgoNav} from "../libs/nav";
import {useLang, useTr, pick} from "../libs/i18n";
import cn from "../libs/cn";

// 좌측 알고리즘 네비게이션 — 카테고리별 그룹. 집필된 페이지는 링크, 미집필은 dim 처리.
const Sidebar = ({open: mobileOpen, onNavigate}: { open?: boolean; onNavigate?: () => void }) => {
    const {current, go} = useAlgoNav()
    const {lang} = useLang()
    const t = useTr()

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
                const anyReady = items.some((a) => a.contents)
                return (
                    <div key={cat.key}>
                        <h4>
                            {pick(lang, cat.title)}
                            {!anyReady && <span className="soon">soon</span>}
                        </h4>
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
                )
            })}
        </aside>
    )
}

export default Sidebar

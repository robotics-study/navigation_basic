import algorithms from "../../../pages/algorithms";
import {useTr} from "../../../libs/i18n";

// 홈에서 "이 사이트가 무엇인가"를 사실만으로 세우는 카드 줄. 알고리즘 수는 registry에서
// 직접 세어(contents 있는 항목) 하드코딩 드리프트를 막는다.
const SiteHighlights = () => {
    const t = useTr()
    const ready = algorithms.filter((a) => a.contents)
    const search = ready.filter((a) => a.category === "search").length
    const sampling = ready.filter((a) => a.category === "sampling").length
    const total = ready.length

    const cards: Array<{kicker: string; title: string; desc: string}> = [
        {
            kicker: t(`${total} algorithms`, `${total}개 알고리즘`),
            title: t("BFS to LQR-RRT*, in lineage order", "BFS에서 LQR-RRT*까지, 계보순"),
            desc: t(
                `${search} graph-search and ${sampling} sampling planners, each page building on ` +
                "the one before it. Local planning and multi-agent are next.",
                `${search}개의 graph-search와 ${sampling}개의 sampling planner를 앞 페이지 위에 ` +
                "다음 페이지가 쌓이는 순서로 읽는다. Local planning과 multi-agent가 다음 차례다.",
            ),
        },
        {
            kicker: t("proofs", "증명"),
            title: t("Every property, proven", "모든 성질에 증명"),
            desc: t(
                "Optimality, completeness, and complexity claims come with step-by-step " +
                "proofs, not hand-waving.",
                "최적성·완전성·복잡도 주장은 말로 얼버무리지 않고 단계별 증명으로 뒷받침한다.",
            ),
        },
        {
            kicker: t("live demos", "라이브 데모"),
            title: t("Experiment on every page", "모든 페이지에서 직접 실험"),
            desc: t(
                "Draw walls, drag start and goal, and move the parameter sliders — " +
                "the planner re-solves in front of you.",
                "벽을 그리고, 시작·목표를 끌고, 파라미터 슬라이더를 움직이면 planner가 눈앞에서 " +
                "다시 푼다.",
            ),
        },
        {
            kicker: t("full source", "전체 소스"),
            title: t("Read the real implementation", "실제 구현을 그대로 읽기"),
            desc: t(
                "Each page ends with the complete C++ and Python source that the " +
                "explanations describe.",
                "각 페이지 끝에는 설명이 가리키는 C++·Python 구현 전체가 그대로 붙어 있다.",
            ),
        },
    ]

    return (
        <div className="grid gap-4 sm:grid-cols-2 mb-12">
            {cards.map((c) => (
                <div key={c.title}
                     className="flex flex-col gap-2 rounded-[var(--radius)] border border-border bg-surface p-5 shadow-card">
                    <span className="text-xs font-bold uppercase tracking-wider"
                          style={{color: "var(--accent)"}}>{c.kicker}</span>
                    <span className="font-semibold" style={{fontSize: "1.02rem"}}>{c.title}</span>
                    <p className="m-0 text-sm text-muted leading-relaxed">{c.desc}</p>
                </div>
            ))}
        </div>
    )
}

export default SiteHighlights

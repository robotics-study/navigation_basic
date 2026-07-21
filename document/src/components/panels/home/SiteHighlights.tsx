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
            title: t("One base class", "하나의 베이스 클래스"),
            desc: t(
                `${search} graph-search and ${sampling} sampling planners share a single GlobalPlanner ` +
                "abstraction. Local planning and multi-agent are next.",
                `${search}개의 graph-search와 ${sampling}개의 sampling planner가 GlobalPlanner 추상화 ` +
                "하나를 공유한다. Local planning과 multi-agent가 다음 차례다.",
            ),
        },
        {
            kicker: t("2 languages", "2개 언어"),
            title: t("C++ and Python, mirrored", "C++과 Python, 미러링"),
            desc: t(
                "Every planner is implemented twice — the same design, parameter names, " +
                "and trace events in both languages.",
                "모든 planner를 두 번 구현한다. 같은 설계, 같은 파라미터 이름, 같은 trace 이벤트를 " +
                "두 언어에서.",
            ),
        },
        {
            kicker: t("1:1 parity", "1:1 parity"),
            title: t("The demo runs the algorithm", "데모가 알고리즘을 돌린다"),
            desc: t(
                "The browser demos emit the same trace events as the repo demos. " +
                "The same seed reproduces the same samples and tree.",
                "브라우저 데모는 저장소 데모와 같은 trace 이벤트를 방출한다. 같은 seed면 표본과 " +
                "트리까지 똑같이 재현된다.",
            ),
        },
        {
            kicker: t("theory → source", "이론 → 소스"),
            title: t("Every page, end to end", "모든 페이지, 처음부터 끝까지"),
            desc: t(
                "Each page runs idea → properties → proof → a live demo → the actual " +
                "repository source, embedded.",
                "각 페이지는 아이디어 → 성질 → 증명 → 라이브 데모 → 실제 저장소 소스 순으로 " +
                "이어지고, 소스는 그대로 embed된다.",
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

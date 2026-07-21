import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runADStar} from "../../../../libs/algorithms/ad_star";
import {runDStarLite} from "../../../../libs/algorithms/dstar_lite";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";
import {
    scatterMap, SCATTER_GOAL, SCATTER_START,
    trapMap, TRAP_GOAL, TRAP_START,
} from "../dstar_lite/presets";

// 라이브 AD* sandbox. D* Lite와 같은 미지 지도 설정에서, ε 팽창 덕에 "첫 해"가 얼마나
// 빨리 나오는지가 핵심 수치다 (D* Lite는 처음부터 belief-최적을 계산한다).
const EPS_STARTS = [1.5, 2.5, 4];
type Preset = "scattered" | "trap";
const PRESETS: Record<Preset, {map: () => GridMap; start: Cell; goal: Cell}> = {
    scattered: {map: scatterMap, start: SCATTER_START, goal: SCATTER_GOAL},
    trap: {map: trapMap, start: TRAP_START, goal: TRAP_GOAL},
};

// 첫 path_found 이전의 node_expanded 수 — "첫 해까지의 지연"을 재는 단일 지표.
const expansionsToFirstSolution = (events: ReturnType<typeof runADStar>): number => {
    let n = 0
    for (const ev of events) {
        if (ev.event === "path_found") return n
        if (ev.event === "node_expanded") n++
    }
    return n
}

const AdScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [preset, setPreset] = useState<Preset>("trap")
    const [map, setMap] = useState<GridMap>(trapMap)
    const [start, setStart] = useState<Cell>(TRAP_START)
    const [goal, setGoal] = useState<Cell>(TRAP_GOAL)
    const [epsStart, setEpsStart] = useState(2.5)

    const applyPreset = (p: Preset) => {
        setPreset(p)
        setMap(PRESETS[p].map())
        setStart(PRESETS[p].start)
        setGoal(PRESETS[p].goal)
    }

    const events = useMemo(
        () => runADStar({map, start, goal, epsStart, epsFinal: 1, epsStep: 0.5, sensorRadius: 3}),
        [map, start, goal, epsStart],
    )
    const timeline = useMemo(() => buildGridTimeline(events), [events])
    const adFirst = useMemo(() => expansionsToFirstSolution(events), [events])
    const dliteFirst = useMemo(() => {
        const run = runDStarLite({map, start, goal, sensorRadius: 3})
        return expansionsToFirstSolution(run.events)
    }, [map, start, goal])

    const publications = timeline.paths.length
    const replans = timeline.metrics?.replan_count ?? 0

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const moveEndpoint = (setter: (c: Cell) => void) => (c: Cell) => {
        if (map.occupied[c[0] * map.width + c[1]]) return
        setter(c)
    }

    return (
        <TracePlayer
            map={map} timeline={timeline} start={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onReset={() => applyPreset(preset)}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
                        {(Object.keys(PRESETS) as Preset[]).map((p) => (
                            <button key={p} type="button" onClick={() => applyPreset(p)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        preset === p
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {p}
                            </button>
                        ))}
                        <span className="mx-1" aria-hidden="true">·</span>
                        {EPS_STARTS.map((e) => (
                            <button key={e} type="button" onClick={() => setEpsStart(e)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        epsStart === e
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                ε₀ = {e}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("first solution after", "첫 해까지 확장")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{adFirst}</span>
                        {" "}(AD*, ε₀) {" vs "}
                        <span className="font-semibold">{dliteFirst}</span>
                        {" "}(D* Lite)
                        {" · "}{t("publications", "발표")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{publications}</span>
                        {" · "}replan{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{replans}</span>
                    </div>
                </div>
            }
        />
    )
}

const AdStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live AD*: same unknown-map drive as D* Lite, but the first usable plan arrives after far fewer expansions thanks to ε-inflation, and each discovery re-inflates then tightens again",
            "라이브 AD*. D* Lite와 같은 미지 지도 주행이지만 ε 팽창 덕에 첫 계획이 훨씬 적은 확장으로 나오고, 발견이 있을 때마다 ε을 다시 올렸다 도로 조인다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<AdScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <AdScene panel={340}/>
    </CanvasFigure>
}

export default AdStarSandbox

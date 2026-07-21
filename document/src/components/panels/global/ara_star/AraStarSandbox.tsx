import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runARAStar} from "../../../../libs/algorithms/ara_star";
import {runAStar} from "../../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {greedyTrapMap, GREEDY_GOAL, GREEDY_START} from "./presets";


const AraScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(greedyTrapMap)
    const [start, setStart] = useState<Cell>(GREEDY_START)
    const [goal, setGoal] = useState<Cell>(GREEDY_GOAL)
    const [epsStart, setEpsStart] = useState(2.5)

    const run = useMemo(
        () => runARAStar({map, start, goal, epsStart, epsFinal: 1, epsStep: 0.5}),
        [map, start, goal, epsStart],
    )
    const timeline = useMemo(() => buildGridTimeline(run.events), [run])

    // 비교 기준: 같은 ε 스케줄로 weighted A*를 매번 처음부터 돌렸을 때의 총 확장 수.
    const naiveExpanded = useMemo(() => {
        let total = 0
        for (const it of run.iterations) {
            const events = runAStar({map, start, goal, heuristicWeight: it.eps, connectivity: 8})
            total += events[events.length - 1].metrics?.expanded_nodes ?? 0
        }
        return total
    }, [run, map, start, goal])
    const araExpanded = timeline.metrics?.expanded_nodes ?? 0

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
            onReset={() => { setMap(greedyTrapMap()); setStart(GREEDY_START); setGoal(GREEDY_GOAL) }}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
                        <ParamSlider label="ε₀" value={epsStart} min={1} max={5} step={0.25} onCommit={setEpsStart}/>
                    </div>
                    <div className="flex items-center justify-center gap-3 text-xs text-muted flex-wrap tabular-nums">
                        {run.iterations.map((it, i) => (
                            <span key={i}>
                                ε {it.eps.toFixed(1)}:{" "}
                                <span className="font-semibold" style={{color: "var(--accent)"}}>
                                    +{it.expanded}
                                </span>{" "}
                                → <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {it.cost.toFixed(2)}
                                </span>
                            </span>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("total expanded — reuse", "총 확장 — 재사용")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{araExpanded}</span>
                        {" vs "}
                        {t("weighted A* from scratch each ε", "매번 weighted A* 재실행")}{" "}
                        <span className="font-semibold">{naiveExpanded}</span>
                    </div>
                </div>
            }
        />
    )
}

const AraStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live ARA*: each ε iteration publishes a path (watch the red path shorten), and the per-iteration counters show later iterations costing almost nothing thanks to reuse",
            "라이브 ARA*. ε 반복마다 경로가 발표되고(빨간 경로가 짧아지는 것을 보라), 반복별 카운터는 재사용 덕에 뒤 반복이 거의 공짜임을 보여 준다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<AraScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <AraScene panel={340}/>
    </CanvasFigure>
}

export default AraStarSandbox

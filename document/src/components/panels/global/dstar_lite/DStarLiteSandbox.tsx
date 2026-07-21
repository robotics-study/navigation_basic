import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runDStarLite} from "../../../../libs/algorithms/dstar_lite";
import {runAStar} from "../../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";
import {
    scatterMap, SCATTER_GOAL, SCATTER_START,
    trapMap, TRAP_GOAL, TRAP_START,
} from "./presets";

type Preset = "scattered" | "trap";
const PRESETS: Record<Preset, {map: () => GridMap; start: Cell; goal: Cell}> = {
    scattered: {map: scatterMap, start: SCATTER_START, goal: SCATTER_GOAL},
    trap: {map: trapMap, start: TRAP_START, goal: TRAP_GOAL},
};

const DStarLiteScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [preset, setPreset] = useState<Preset>("scattered")
    const [map, setMap] = useState<GridMap>(scatterMap)
    const [start, setStart] = useState<Cell>(SCATTER_START)
    const [goal, setGoal] = useState<Cell>(SCATTER_GOAL)
    const [radius, setRadius] = useState(3)

    const applyPreset = (p: Preset) => {
        setPreset(p)
        setMap(PRESETS[p].map())
        setStart(PRESETS[p].start)
        setGoal(PRESETS[p].goal)
    }

    const run = useMemo(
        () => runDStarLite({map, start, goal, sensorRadius: radius}),
        [map, start, goal, radius],
    )
    const timeline = useMemo(() => buildGridTimeline(run.events), [run])

    // 비교 기준: 같은 belief 변화 시점마다 A*를 처음부터 다시 돌렸다면 확장했을 노드 수.
    const naiveExpanded = useMemo(() => {
        let total = 0
        const beliefRun = (blocked: Set<number>, from: Cell) => {
            const belief: GridMap = {
                ...map,
                occupied: map.occupied.map((_, i) => blocked.has(i)),
            }
            const events = runAStar({map: belief, start: from, goal, heuristicWeight: 1, connectivity: 8})
            const fin = events[events.length - 1]
            return fin.metrics?.expanded_nodes ?? 0
        }
        total += beliefRun(new Set(), start)             // 최초 계획
        for (const snap of run.snapshots) total += beliefRun(snap.blocked, snap.robot)
        return total
    }, [run, map, start, goal])

    const repairExpanded = timeline.metrics?.expanded_nodes ?? 0
    const replans = timeline.metrics?.replan_count ?? 0
    const sensedCells = timeline.metrics?.sensed_cells ?? 0

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
                        <ParamSlider label={t("sensor radius", "센서 반경")} value={radius} min={1} max={8} step={1} onCommit={setRadius}/>
                        <span className="tabular-nums">
                            {t("replans", "replan")}{" "}
                            <span className="font-semibold" style={{color: "var(--accent)"}}>{replans}</span>
                            {" · "}{t("walls found", "발견한 벽")}{" "}
                            <span className="font-semibold" style={{color: "var(--accent)"}}>{sensedCells}</span>
                        </span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("expanded — repair", "확장 노드 — 수리")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{repairExpanded}</span>
                        {" vs "}
                        {t("A* from scratch each replan", "매번 A* 재실행")}{" "}
                        <span className="font-semibold">{naiveExpanded}</span>
                        {" · "}
                        {t("draw true walls the robot cannot see yet",
                            "로봇이 아직 못 본 실제 벽을 그려 보라")}
                    </div>
                </div>
            }
        />
    )
}

const DStarLiteSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live D* Lite: the robot starts blind, senses walls as it drives (ghost walls are the unknown truth), and repairs only the part of the search the discovery invalidates",
            "라이브 D* Lite. 로봇은 지도를 모른 채 출발해 주행 중에 벽을 감지하고(흐린 벽이 아직 모르는 실제 지도), 발견이 무효화한 부분만 수리한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<DStarLiteScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <DStarLiteScene panel={340}/>
    </CanvasFigure>
}

export default DStarLiteSandbox

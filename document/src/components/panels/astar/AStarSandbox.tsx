import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../CanvasFigure";
import TracePlayer from "../../player/TracePlayer";
import {runAStar} from "../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../libs/trace/timeline";
import {GridMap} from "../../../libs/grid";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";
import {pocketMap, SANDBOX_GOAL, SANDBOX_START} from "./presets";

// 라이브 A* sandbox — 브라우저에서 직접 탐색을 실행한다. 벽을 그리고 시작/목표를 끌면
// 즉시 재탐색하고, heuristic 가중치를 바꾸면 Dijkstra ↔ A* ↔ weighted A* 의 탐색량
// 차이가 바로 보인다.
const WEIGHTS: Array<{w: number; label: string}> = [
    {w: 0, label: "w = 0 · Dijkstra"},
    {w: 1, label: "w = 1 · A*"},
    {w: 2, label: "w = 2"},
    {w: 3, label: "w = 3 · greedy"},
];

export const SandboxScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(pocketMap)
    const [start, setStart] = useState<Cell>(SANDBOX_START)
    const [goal, setGoal] = useState<Cell>(SANDBOX_GOAL)
    const [weight, setWeight] = useState(1)
    const [connectivity, setConnectivity] = useState<4 | 8>(8)

    const timeline = useMemo(
        () => buildGridTimeline(runAStar({map, start, goal, heuristicWeight: weight, connectivity})),
        [map, start, goal, weight, connectivity],
    )

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const moveEndpoint = (setter: (c: Cell) => void) => (c: Cell) => {
        // 벽 위로는 옮길 수 없다.
        if (map.occupied[c[0] * map.width + c[1]]) return
        setter(c)
    }

    return (
        <TracePlayer
            map={map} timeline={timeline} start={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
                        {WEIGHTS.map(({w, label}) => (
                            <button key={w} type="button" onClick={() => setWeight(w)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        weight === w
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {label}
                            </button>
                        ))}
                        <span className="mx-1" aria-hidden="true">·</span>
                        {([4, 8] as const).map((c) => (
                            <button key={c} type="button" onClick={() => setConnectivity(c)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        connectivity === c
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {c}-conn
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("drag cells to draw walls · drag the endpoints to move start/goal",
                            "셀을 드래그해 벽을 그리고, 원을 끌어 시작/목표를 옮겨 보라")}
                    </div>
                </div>
            }
        />
    )
}

const AStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live A* sandbox: f = g + w·h. Compare how many cells Dijkstra (w = 0), A* (w = 1), and greedy (w > 1) expand on the same problem",
            "라이브 A* sandbox: f = g + w·h. 같은 문제에서 Dijkstra (w = 0), A* (w = 1), greedy (w > 1) 가 몇 칸을 확장하는지 비교해 보라",
        )}
        tight
        bodyClassName="w-fit"
        className="w-full"
        modal={<SandboxScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <SandboxScene panel={340}/>
    </CanvasFigure>
}

export default AStarSandbox

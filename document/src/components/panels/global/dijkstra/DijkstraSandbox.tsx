import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {runAStar} from "../../../../libs/algorithms/astar";
import {runBFS} from "../../../../libs/algorithms/bfs";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";
import {hopcostMap, HOPCOST_GOAL, HOPCOST_START} from "../astar/presets";

// 라이브 Dijkstra sandbox — 같은 문제를 BFS(FIFO)와 Dijkstra(priority queue)로 풀어
// 비교한다. 8-connected 에서 hop-최단과 비용-최단이 갈라지는 것이 핵심 볼거리다.
type Mode = "bfs" | "dijkstra";

const DijkstraScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(hopcostMap)
    const [start, setStart] = useState<Cell>(HOPCOST_START)
    const [goal, setGoal] = useState<Cell>(HOPCOST_GOAL)
    const [mode, setMode] = useState<Mode>("dijkstra")

    // 두 엔진을 항상 같이 돌려 footer 에서 비용을 비교한다 (애니메이션은 선택 모드만).
    const timelines = useMemo(() => ({
        bfs: buildGridTimeline(runBFS({map, start, goal, connectivity: 8})),
        dijkstra: buildGridTimeline(runAStar({map, start, goal, heuristicWeight: 0, connectivity: 8})),
    }), [map, start, goal])

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

    const cost = (m: Mode) => timelines[m].metrics?.path_cost

    return (
        <TracePlayer
            map={map} timeline={timelines[mode]} start={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
                        {(["bfs", "dijkstra"] as const).map((m) => (
                            <button key={m} type="button" onClick={() => setMode(m)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        mode === m
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {m === "bfs" ? "FIFO (BFS)" : "priority (Dijkstra)"}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("path cost", "경로 비용")}:{" "}
                        BFS <span className="font-semibold" style={{color: PATH_COLOR}}>
                            {cost("bfs")?.toFixed(2) ?? "—"}
                        </span>
                        {" vs "}
                        Dijkstra <span className="font-semibold" style={{color: "var(--accent)"}}>
                            {cost("dijkstra")?.toFixed(2) ?? "—"}
                        </span>
                        {" · "}
                        {t("drag walls and endpoints", "벽과 끝점을 끌어 보라")}
                    </div>
                </div>
            }
        />
    )
}

const DijkstraSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live comparison on one problem: swap the FIFO queue for a priority queue and the fewest-edge path gives way to the cheapest path",
            "같은 문제의 라이브 비교. FIFO 큐를 priority queue 로 바꾸는 순간, edge 수 최소 경로가 비용 최소 경로로 바뀐다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<DijkstraScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <DijkstraScene panel={340}/>
    </CanvasFigure>
}

export default DijkstraSandbox

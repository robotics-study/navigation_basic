import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../CanvasFigure";
import TracePlayer from "../../player/TracePlayer";
import {runAStar} from "../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../libs/trace/timeline";
import {emptyGrid, GridMap} from "../../../libs/grid";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";

// 같은 f = g + w·h 뼈대에서 w만 0→1로 바꾸면 무엇이 달라지는지 한 컷으로 보여 준다.
// 회색 그림자는 Dijkstra(w = 0)가 확장한 전부(사방으로 퍼지는 등고선), 그 위에서
// A*(w = 1)는 목표를 향한 좁은 쐐기만 확장한다. 둘의 경로 비용은 같다 (둘 다 최적).
// 실측(22×22, 가운데 작은 벽, 대각 반대편 시작/목표): Dijkstra expanded=451,
// A* expanded=35, 두 경로 비용 모두 23.21.
const N = 22;

function openMap(): GridMap {
    const map = emptyGrid("dijkstra_vs_astar", N, N)
    for (let r = 9; r <= 12; r++) for (let c = 8; c <= 9; c++) map.occupied[r * N + c] = true
    return map
}

const START: Cell = [N - 3, 3];
const GOAL: Cell = [3, N - 3];

const Scene = ({panel = 320}: {panel?: number}) => {
    const t = useTr()
    const [showShadow, setShowShadow] = useState(true)
    const map = useMemo(() => openMap(), [])

    const astar = useMemo(
        () => buildGridTimeline(runAStar({map, start: START, goal: GOAL, heuristicWeight: 1, connectivity: 8})),
        [map],
    )
    const dijkstra = useMemo(
        () => buildGridTimeline(runAStar({map, start: START, goal: GOAL, heuristicWeight: 0, connectivity: 8})),
        [map],
    )
    const dijExpanded = dijkstra.metrics?.expanded_nodes ?? 0
    const astarExpanded = astar.metrics?.expanded_nodes ?? 0

    return (
        <TracePlayer
            map={map} timeline={astar} start={START} goal={GOAL} panel={panel}
            shadowCells={showShadow ? dijkstra.expanded.map((e) => e.cell) : undefined}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <button type="button" onClick={() => setShowShadow((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showShadow
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("Dijkstra expansions shadow", "Dijkstra 확장 영역 겹치기")}
                        </button>
                        <span>
                            {t("expanded", "확장")}: A*{" "}
                            <span className="font-semibold" style={{color: "var(--accent)"}}>{astarExpanded}</span>
                            {" vs "}Dijkstra{" "}
                            <span className="font-semibold">{dijExpanded}</span>
                        </span>
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("same graph, same optimal cost — the heuristic only changes how much gets touched",
                            "같은 그래프, 같은 최적 비용. heuristic은 얼마나 건드리는가만 바꾼다")}
                    </div>
                </div>
            }
        />
    )
}

const DijkstraVsAstarLive = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Dijkstra (w = 0) versus A* (w = 1) on one graph: the gray shadow is everything Dijkstra expands blindly; A* touches only the wedge the heuristic points at, for the same optimal path",
            "한 그래프에서 Dijkstra (w = 0) 대 A* (w = 1). 회색 그림자가 Dijkstra가 맹목적으로 확장하는 전부이고, A*는 heuristic이 가리키는 쐐기만 건드려 같은 최적 경로를 얻는다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene panel={Math.min(modalCanvasSize(1).width, 560)}/>}
    >
        <Scene panel={320}/>
    </CanvasFigure>
}

export default DijkstraVsAstarLive

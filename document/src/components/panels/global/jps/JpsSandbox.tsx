import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runJPS} from "../../../../libs/algorithms/jps";
import {runAStar} from "../../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {emptyGrid, GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// JPS 전용: 넓은 홀에 짧은 벽 토막 몇 개. 대칭 경로가 지천으로 깔린 지형이라
// A*는 수백 칸을 확장하고 JPS는 jump point 몇 개만 짚는다.
export const HALL_N = 24;
export const HALL_START: Cell = [21, 2];
export const HALL_GOAL: Cell = [2, 21];

export function hallMap(): GridMap {
    const map = emptyGrid("jps_hall", HALL_N, HALL_N)
    const set = (r: number, c: number) => { map.occupied[r * HALL_N + c] = true }
    for (let c = 6; c <= 10; c++) set(15, c)
    for (let r = 6; r <= 10; r++) set(r, 13)
    for (let c = 16; c <= 19; c++) set(9, c)
    return map
}

const JpsScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(hallMap)
    const [start, setStart] = useState<Cell>(HALL_START)
    const [goal, setGoal] = useState<Cell>(HALL_GOAL)
    const [showAstar, setShowAstar] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runJPS({map, start, goal})),
        [map, start, goal],
    )
    const astar = useMemo(() => {
        const tl = buildGridTimeline(
            runAStar({map, start, goal, heuristicWeight: 1, connectivity: 8}))
        return {expanded: tl.metrics?.expanded_nodes ?? 0, cells: tl.expanded.map((e) => e.cell)}
    }, [map, start, goal])
    const jpsExpanded = timeline.metrics?.expanded_nodes ?? 0

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
            showTree
            shadowCells={showAstar ? astar.cells : undefined}
            onPaintCell={paintCell}
            onReset={() => { setMap(hallMap()); setStart(HALL_START); setGoal(HALL_GOAL) }}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <button type="button" onClick={() => setShowAstar((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showAstar
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("A* expansions shadow", "A* 확장 영역 겹치기")}
                        </button>
                        <span>
                            {t("expanded", "확장")}: JPS{" "}
                            <span className="font-semibold" style={{color: "var(--accent)"}}>
                                {jpsExpanded}
                            </span>
                            {" vs "}A*{" "}
                            <span className="font-semibold">{astar.expanded}</span>
                        </span>
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("straight tree edges are jumps · drag walls and endpoints",
                            "곧게 뻗은 트리 선이 jump다. 벽과 끝점을 끌어 보라")}
                    </div>
                </div>
            }
        />
    )
}

const JpsSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live JPS in an open hall: the gray shadow is everything A* would expand; JPS touches only the handful of jump points its long scans return",
            "홀에서의 라이브 JPS. 회색 그림자가 A* 라면 확장했을 전부이고, JPS는 긴 스캔이 돌려주는 소수의 jump point만 짚는다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<JpsScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <JpsScene panel={340}/>
    </CanvasFigure>
}

export default JpsSandbox

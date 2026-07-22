import {ReactNode, useMemo, useState} from "react";
import TracePlayer from "../../player/TracePlayer";
import {runAStar} from "../../../libs/algorithms/astar";
import {runRRTStar} from "../../../libs/algorithms/rrt_star";
import {Point} from "../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../libs/trace/timeline";
import {GridMap} from "../../../libs/grid";
import {centerBlockMap} from "../global/rrt_star/presets";
import {useTr} from "../../../libs/i18n";
import {PATH_COLOR} from "../../2d/GridCanvas";

// 홈 히어로 — 같은 문제를 두 패러다임이 푸는 라이브 데모. 왼쪽은 격자 위 A*(정확한
// 이산 탐색), 오른쪽은 같은 맵을 sampling으로 푸는 RRT*(점근 최적). 둘 다 브라우저에서
// 실제 알고리즘을 돌리며, 저장소 demo와 같은 trace 이벤트를 방출한다. 왼쪽 격자에 벽을
// 그리면 두 planner가 같은 맵을 다시 푼다 — "라이브 엔진이 곧 이 사이트"라는 걸 첫
// 화면에서 만지게 한다.
const START_CELL: Cell = [17, 2];
const GOAL_CELL: Cell = [2, 17];

// (row, col) 셀 중심 → world 좌표. RRT*는 연속 상태이므로 A*와 같은 코너를 world로 넘긴다.
const cellToWorld = (map: GridMap, [row, col]: Cell): Point =>
    [col + 0.5, map.height - 1 - row + 0.5];

const Panel = ({tag, name, note, children}: {
    tag: string; name: string; note: string; children: ReactNode
}) => (
    <div className="flex flex-col items-center gap-2">
        {children}
        <div className="text-xs text-center leading-snug">
            <span className="font-semibold" style={{color: "var(--accent)"}}>{name}</span>
            <span className="text-muted">{" · "}{tag}</span>
            <div className="text-muted">{note}</div>
        </div>
    </div>
);

const TwinDemo = ({panel = 280}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(centerBlockMap)
    const [startCell, setStartCell] = useState<Cell>(START_CELL)
    const [goalCell, setGoalCell] = useState<Cell>(GOAL_CELL)

    const astar = useMemo(
        () => buildGridTimeline(runAStar({
            map, start: startCell, goal: goalCell, heuristicWeight: 1, connectivity: 8,
        })),
        [map, startCell, goalCell],
    )
    const rrt = useMemo(
        () => buildGridTimeline(runRRTStar({
            map, start: cellToWorld(map, startCell), goal: cellToWorld(map, goalCell),
            maxIterations: 1200, stepSize: 0.5, goalBias: 0.05, goalTolerance: 0.3,
            neighborRadius: 1.5, radiusMode: "fixed", rggGamma: 2, seed: 1,
        })),
        [map, startCell, goalCell],
    )
    const rrtStart = cellToWorld(map, startCell)
    const rrtGoal = cellToWorld(map, goalCell)

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
        <div className="flex flex-col items-center gap-4">
            <div className="flex flex-wrap justify-center gap-x-10 gap-y-6">
                <Panel name="A*" tag={t("discrete search", "이산 탐색")}
                       note={t("exact shortest path on the grid", "격자 위 정확한 최단 경로")}>
                    <TracePlayer
                        map={map} timeline={astar} start={startCell} goal={goalCell} panel={panel}
                        onPaintCell={paintCell}
                        onMoveStart={moveEndpoint(setStartCell)}
                        onMoveGoal={moveEndpoint(setGoalCell)}
                        onReset={() => {
                            setMap(centerBlockMap())
                            setStartCell(START_CELL)
                            setGoalCell(GOAL_CELL)
                        }}
                    />
                </Panel>
                <Panel name="RRT*" tag={t("sampling", "sampling")}
                       note={t("same map, grown from random samples", "같은 맵을 무작위 표본으로 키운다")}>
                    <TracePlayer
                        map={map} timeline={rrt} panel={panel} showTree
                        start={[rrtStart[0], rrtStart[1]]} goal={[rrtGoal[0], rrtGoal[1]]}
                    />
                </Panel>
            </div>
            <div className="text-xs text-muted text-center max-w-md">
                {t(
                    "Draw walls on the left grid or drag the endpoints — both planners re-solve the same problem.",
                    "왼쪽 격자에 벽을 그리거나 원을 끌면 두 planner가 같은 문제를 다시 푼다.",
                )}
                <span className="ml-1" style={{color: PATH_COLOR}}>●</span>
                {" "}{t("red is the found path", "빨간 선이 찾은 경로")}
            </div>
        </div>
    )
}

export default TwinDemo

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runLpaStar} from "../../../../libs/algorithms/lpa_star";
import {runAStar} from "../../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {emptyGrid, GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// maps/grid/lpa_replan01 미러 — col 8의 벽이 위쪽 게이트(행 2~3)와 아래쪽 게이트
// (행 8~11) 두 군데로 끊긴 17×17 지도. start/골은 행 10의 양끝이고, 아래 게이트를
// 막으면 위 게이트로 돌아가야 하고 다시 뚫으면 직선 경로가 돌아온다.
const N = 17;
export const LPA_START: Cell = [10, 2];
export const LPA_GOAL: Cell = [10, 14];

function baseMap(): GridMap {
    const map = emptyGrid("lpa_replan01", N, N);
    for (const r of [0, 1, 4, 5, 6, 7, 12, 13, 14, 15, 16]) {
        map.occupied[r * N + 8] = true;
    }
    return map;
}

const LpaStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    // LPA*는 지도를 모른 채 시작하지 않는다 — 배경은 항상 ground truth이고, 클릭은
    // planner가 "알려받은" cost change다. 반전은 timeline의 obstacle_changed로
    // 재생되고, 붓은 반전까지 반영된 모델 상태를 기준으로 삼는다.
    const map = useMemo(baseMap, [])
    const [start, setStart] = useState<Cell>(LPA_START)
    const [goal, setGoal] = useState<Cell>(LPA_GOAL)
    const [revisions, setRevisions] = useState<Array<{cells: Cell[]; blocked: boolean}>>([])

    const run = useMemo(
        () => runLpaStar({map, start, goal, revisions}),
        [map, start, goal, revisions],
    )
    const timeline = useMemo(() => buildGridTimeline(run.events), [run])

    // 모든 반전을 적용한 뒤의 모델 점유 — 붓과 endpoint 드래그 가드가 읽는다.
    const modelOccupied = useMemo(() => {
        const occ = [...map.occupied]
        for (const rev of revisions) {
            for (const c of rev.cells) occ[c[0] * map.width + c[1]] = rev.blocked
        }
        return occ
    }, [map, revisions])

    // 비교 기준: cost 변경마다 A*를 처음부터 다시 돌렸다면 확장했을 노드 수.
    // no-op 반전에도 LPA*는 plan()을 다시 부르지만 확장은 거의 0이다 — 재실행은
    // 그럴 때마다 전체 탐색 비용을 통째로 낸다.
    const naiveExpanded = useMemo(() => {
        const occ = [...map.occupied]
        let total = 0
        for (let round = 0; round <= revisions.length; round++) {
            if (round > 0) {
                for (const c of revisions[round - 1].cells) {
                    occ[c[0] * map.width + c[1]] = revisions[round - 1].blocked
                }
            }
            const events = runAStar({
                map: {...map, occupied: [...occ]}, start, goal, heuristicWeight: 1, connectivity: 8,
            })
            total += events[events.length - 1].metrics?.expanded_nodes ?? 0
        }
        return total
    }, [map, start, goal, revisions])

    const repairExpanded = timeline.metrics?.expanded_nodes ?? 0
    const replans = timeline.metrics?.replan_count ?? 0

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setRevisions((prev) => [...prev, {cells: [[row, col]], blocked: occupied}])
    }
    const moveEndpoint = (setter: (c: Cell) => void) => (c: Cell) => {
        if (modelOccupied[c[0] * map.width + c[1]]) return
        setter(c)
    }

    return (
        <TracePlayer
            map={map} timeline={timeline} start={start} goal={goal} panel={panel}
            onPaintCell={paintCell} paintOccupied={modelOccupied}
            onReset={() => { setRevisions([]); setStart(LPA_START); setGoal(LPA_GOAL) }}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5 text-xs text-muted text-center tabular-nums">
                    <div>
                        {t("expanded — repair", "확장 노드 — 수리")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{repairExpanded}</span>
                        {" vs "}
                        {t("A* from scratch each change", "바뀔 때마다 A* 재실행")}{" "}
                        <span className="font-semibold">{naiveExpanded}</span>
                    </div>
                    <div>
                        {t("replans", "replan")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{replans}</span>
                        {" · "}{t("cells flipped", "뒤집은 셀")}{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>{timeline.changes.length}</span>
                        {" · "}
                        {t("click a cell to communicate a cost change",
                            "셀을 클릭하면 cost 변경이 planner에게 전달된다")}
                    </div>
                </div>
            }
        />
    )
}

const LpaStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live LPA*: the map is known from round 0 — clicking a cell communicates a cost change and the planner repairs its search locally instead of restarting",
            "라이브 LPA*. 지도는 round 0부터 알려져 있고, 셀을 클릭하면 cost 변경이 전달된다. planner는 탐색을 다시 시작하지 않고 손상된 부분만 국소적으로 수리한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<LpaStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <LpaStarScene/>
    </CanvasFigure>
}

export default LpaStarSandbox

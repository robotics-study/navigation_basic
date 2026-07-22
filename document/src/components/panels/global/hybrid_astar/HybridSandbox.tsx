import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runHybridAStar, Pose} from "../../../../libs/algorithms/hybrid_astar";
import {buildGridTimeline} from "../../../../libs/trace/timeline";
import {cellCenterWorld, emptyGrid, GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";

// Hybrid A* 전용: 주차 베이. goal은 베이 안쪽을 향한 heading이 걸린 pose 라
// 위치만 맞추는 grid planner로는 표현 자체가 안 되는 문제다.
export const LOT_N = 18;
// 베이 안쪽, 위(+y)를 향해 진입.
export const LOT_START: Pose = [2.5, 2.5, 0];
export const LOT_GOAL: Pose = [10.5, 12.5, Math.PI / 2];

export function parkingLotMap(): GridMap {
    const map = emptyGrid("parking_lot", LOT_N, LOT_N)
    const set = (r: number, c: number) => { map.occupied[r * LOT_N + c] = true }
    for (let c = 8; c <= 12; c++) set(3, c)     // 베이 안쪽 벽
    for (let r = 3; r <= 6; r++) set(r, 8)      // 베이 왼 벽
    for (let r = 3; r <= 6; r++) set(r, 12)     // 베이 오른 벽
    for (let c = 2; c <= 5; c++) set(9, c)      // 진입로의 장애물 하나
    return map
}


const HybridScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(parkingLotMap)
    const [start, setStart] = useState<Pose>(LOT_START)
    const [goal, setGoal] = useState<Pose>(LOT_GOAL)
    const [radius, setRadius] = useState(1.6)
    const [allowReverse, setAllowReverse] = useState(true)

    const timeline = useMemo(() => buildGridTimeline(runHybridAStar({
        map, start, goal,
        minTurnRadius: radius, arcStep: 1.2, numSteering: 5, thetaBins: 72,
        xyResolution: 1.0, footprintRadius: 0.45,
        allowReverse, reversePenalty: 2, steerPenalty: 0.1,
        goalPosTolerance: 0.8, goalHeadingTolerance: 0.35,
    })), [map, start, goal, radius, allowReverse])

    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    // 위치만 옮기고 heading은 유지한다 — goal heading이 베이 진입 방향이라는 문제 설정을 지키기 위해.
    const moveEndpoint = (pose: Pose, setter: (p: Pose) => void) => (c: [number, number]) => {
        if (map.occupied[c[0] * map.width + c[1]]) return
        const [x, y] = cellCenterWorld(map, c)
        setter([x, y, pose[2]])
    }

    return (
        <TracePlayer
            map={map} timeline={timeline} panel={panel}
            start={[start[0], start[1]]}
            goal={[goal[0], goal[1]]}
            startPose={start} goalPose={goal} vehicle
            showTree
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(start, setStart)}
            onMoveGoal={moveEndpoint(goal, setGoal)}
            onReset={() => {
                setMap(parkingLotMap())
                setStart(LOT_START)
                setGoal(LOT_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="R" value={radius} min={0.8} max={2.6} step={0.1} onCommit={setRadius}/>
                        <button type="button" onClick={() => setAllowReverse((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    allowReverse
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("reverse", "후진")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {success
                            ? <>
                                {t("cost", "비용")}{" "}
                                <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {cost?.toFixed(2)}
                                </span>
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                        {" · "}{t("drag the endpoints — headings stay fixed, the goal keeps pointing into the bay",
                            "원을 끌면 시작·목표가 움직인다. heading은 고정이라 goal은 계속 베이 방향을 향한다")}
                        {" · "}{t("draw walls to reshape the lot", "벽을 그려 주차장을 바꿔 보라")}
                    </div>
                </div>
            }
        />
    )
}

const HybridSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Hybrid A* parking: the search tree is arcs, the path swings wide to enter the bay with the required heading — tighten the turn radius or forbid reverse and watch the maneuver change",
            "라이브 Hybrid A* 주차. 탐색 트리가 arc 들이고, 경로는 요구된 heading으로 베이에 들어가려고 크게 돈다. 회전 반경을 조이거나 후진을 끄면 기동이 달라진다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<HybridScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <HybridScene panel={340}/>
    </CanvasFigure>
}

export default HybridSandbox

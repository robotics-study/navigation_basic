import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runRRT} from "../../../../libs/algorithms/rrt";
import {runRRTStar} from "../../../../libs/algorithms/rrt_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {BLOCK_GOAL, BLOCK_START, centerBlockMap} from "./presets";

// 라이브 RRT* sandbox. 예산을 키우면 경로가 팽팽해지는 anytime 수렴을, 같은
// seed의 RRT 첫 경로(점선)와 비용으로 맞세워 보여 준다.
const BUDGETS = [800, 2000, 5000];

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const RrtStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(centerBlockMap)
    const [start, setStart] = useState<Point>(BLOCK_START)
    const [goal, setGoal] = useState<Point>(BLOCK_GOAL)
    const [budget, setBudget] = useState(2000)
    const [seed, setSeed] = useState(1)
    const [showRrt, setShowRrt] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runRRTStar({
            map, start, goal, maxIterations: budget, stepSize: 0.5,
            goalBias: 0.05, goalTolerance: 0.3, neighborRadius: 1.5,
            radiusMode: "fixed", rggGamma: 2, seed,
        })),
        [map, start, goal, budget, seed],
    )
    const rrt = useMemo(() => {
        const tl = buildGridTimeline(runRRT({
            map, start, goal, maxIterations: budget, stepSize: 0.5,
            goalBias: 0.05, goalTolerance: 0.3, seed,
        }))
        return {path: tl.path, cost: tl.metrics?.path_cost,
                success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, budget, seed])
    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const moveEndpoint = (setter: (p: Point) => void) => (c: Cell) => {
        if (map.occupied[c[0] * map.width + c[1]]) return
        setter(cellToWorld(map, c))
    }

    return (
        <TracePlayer
            map={map} timeline={timeline} panel={panel}
            start={[start[0], start[1]]} goal={[goal[0], goal[1]]}
            showTree
            overlayPath={showRrt && rrt.success ? rrt.path : undefined}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(centerBlockMap())
                setStart(BLOCK_START)
                setGoal(BLOCK_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {BUDGETS.map((b) => (
                            <button key={b} type="button" onClick={() => setBudget(b)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        budget === b
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {b} {t("iters", "반복")}
                            </button>
                        ))}
                        <button type="button" onClick={() => setShowRrt((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showRrt
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("RRT overlay", "RRT 겹치기")}
                        </button>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("regrow", "다시 성장")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {success
                            ? <>
                                RRT*{" "}
                                <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {cost?.toFixed(2)}
                                </span>
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                        {" vs RRT "}
                        <span className="font-semibold">
                            {rrt.success ? rrt.cost?.toFixed(2) : t("fail", "실패")}
                        </span>
                        {" · "}
                        {t("straight-line bound", "직선 하한")}{" "}
                        <span className="font-semibold">
                            {Math.hypot(goal[0] - start[0], goal[1] - start[1]).toFixed(2)}
                        </span>
                    </div>
                </div>
            }
        />
    )
}

const RrtStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live RRT* around a single block: raise the iteration budget and the red path pulls taut toward the bound while the dashed RRT path stays frozen at its first jagged answer",
            "블록 하나를 도는 라이브 RRT*. 반복 예산을 올리면 빨간 경로가 하한 쪽으로 팽팽해지는데, 점선 RRT 경로는 처음의 삐죽한 답에서 얼어붙어 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<RrtStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <RrtStarScene panel={340}/>
    </CanvasFigure>
}

export default RrtStarSandbox

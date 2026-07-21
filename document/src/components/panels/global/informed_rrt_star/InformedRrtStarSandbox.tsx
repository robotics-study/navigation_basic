import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runInformedRRTStar} from "../../../../libs/algorithms/informed_rrt_star";
import {runRRTStar} from "../../../../libs/algorithms/rrt_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {OPEN_GOAL, OPEN_START, openEllipseMap} from "./presets";

// 라이브 Informed RRT* sandbox. 첫 해가 나오면 표본이 start/goal 초점의 타원 안으로
// 몰리는 것을, 같은 seed·예산의 RRT*(점선, 균일 표본) 경로/비용과 맞세워 보여 준다.
const BUDGETS = [500, 1000, 2000];

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const InformedRrtStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(openEllipseMap)
    const [start, setStart] = useState<Point>(OPEN_START)
    const [goal, setGoal] = useState<Point>(OPEN_GOAL)
    const [budget, setBudget] = useState(1000)
    const [seed, setSeed] = useState(3)
    const [showRrtStar, setShowRrtStar] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runInformedRRTStar({
            map, start, goal, maxIterations: budget, stepSize: 0.5,
            goalBias: 0.05, goalTolerance: 0.3, neighborRadius: 1.5,
            radiusMode: "fixed", rggGamma: 2, seed,
        })),
        [map, start, goal, budget, seed],
    )
    // 같은 seed·예산의 RRT* — 균일 표본이라 첫 해 이후에도 맵 전체를 뒤진다.
    const rrtStar = useMemo(() => {
        const tl = buildGridTimeline(runRRTStar({
            map, start, goal, maxIterations: budget, stepSize: 0.5,
            goalBias: 0.05, goalTolerance: 0.3, neighborRadius: 1.5,
            radiusMode: "fixed", rggGamma: 2, seed,
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
            overlayPath={showRrtStar && rrtStar.success ? rrtStar.path : undefined}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(openEllipseMap())
                setStart(OPEN_START)
                setGoal(OPEN_GOAL)
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
                        <button type="button" onClick={() => setShowRrtStar((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showRrtStar
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("RRT* overlay", "RRT* 겹치기")}
                        </button>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("regrow", "다시 성장")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {success
                            ? <>
                                Informed{" "}
                                <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {cost?.toFixed(2)}
                                </span>
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                        {" vs RRT* "}
                        <span className="font-semibold">
                            {rrtStar.success ? rrtStar.cost?.toFixed(2) : t("fail", "실패")}
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

const InformedRrtStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Informed RRT*: once the first solution appears the samples collapse into the start–goal ellipse, so the red path converges below the dashed RRT* answer at the same seed and budget",
            "라이브 Informed RRT*. 첫 해가 나오면 표본이 start–goal 타원 안으로 접히고, 같은 seed·예산의 점선 RRT* 답보다 빨간 경로가 더 낮게 수렴한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<InformedRrtStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <InformedRrtStarScene panel={340}/>
    </CanvasFigure>
}

export default InformedRrtStarSandbox

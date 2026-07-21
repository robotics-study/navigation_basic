import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runRRT} from "../../../../libs/algorithms/rrt";
import {runFastRRT} from "../../../../libs/algorithms/fast_rrt";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {GAP_GOAL, GAP_START, narrowPassageMap} from "./presets";


const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const FastRrtScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(narrowPassageMap)
    const [start, setStart] = useState<Point>(GAP_START)
    const [goal, setGoal] = useState<Point>(GAP_GOAL)
    const [budget, setBudget] = useState(800)
    const [seed, setSeed] = useState(2)
    const [showRrt, setShowRrt] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runFastRRT({
            map, start, goal, maxIterations: budget, stepSize: 0.5,
            goalBias: 0.05, goalTolerance: 0.3, neighborRadius: 1.5,
            radiusMode: "fixed", rggGamma: 2, reachedRadius: 0.4,
            steeringAttempts: 10, seed,
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
                setMap(narrowPassageMap())
                setStart(GAP_START)
                setGoal(GAP_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label={t("iters", "반복")} value={budget} min={200} max={2000} step={50} onCommit={setBudget}/>
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
                                Fast-RRT{" "}
                                <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {cost?.toFixed(2)}
                                </span>
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                        {" vs RRT "}
                        <span className="font-semibold">
                            {rrt.success ? rrt.cost?.toFixed(2) : t("miss", "통로 놓침")}
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

const FastRrtSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Fast-RRT through a one-cell gap: fast-sampling spreads the tree and random steering threads the passage, so the red path stays smooth and reliable while the dashed RRT answer is jagged and often misses the gap — press regrow to reseed",
            "한 칸 gap을 통과하는 라이브 Fast-RRT. fast-sampling이 트리를 고루 펴고 random steering이 통로를 꿰어, 빨간 경로가 매끈하고 안정적이다. 점선 RRT 답은 삐죽하고 통로를 자주 놓친다. 다시 성장 버튼으로 seed를 바꿔 보라",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<FastRrtScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <FastRrtScene panel={340}/>
    </CanvasFigure>
}

export default FastRrtSandbox

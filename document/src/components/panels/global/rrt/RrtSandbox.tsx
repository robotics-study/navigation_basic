import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runRRT} from "../../../../libs/algorithms/rrt";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {TRAP_GOAL, TRAP_START, bugTrapMap} from "./presets";

// 라이브 RRT sandbox. bug trap에서 goal bias와 step size를 바꿔 가며 트리가
// 함정을 빠져나오는 속도(반복 수)를 비교한다.
const BIASES = [0, 0.05, 0.3];
const STEPS = [0.3, 0.5, 1.0];

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const RrtScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(bugTrapMap)
    const [start, setStart] = useState<Point>(TRAP_START)
    const [goal, setGoal] = useState<Point>(TRAP_GOAL)
    const [goalBias, setGoalBias] = useState(0.05)
    const [stepSize, setStepSize] = useState(0.5)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runRRT({
            map, start, goal, maxIterations: 4000, stepSize,
            goalBias, goalTolerance: 0.3, seed,
        })),
        [map, start, goal, goalBias, stepSize, seed],
    )
    const cost = timeline.metrics?.path_cost
    const iterations = timeline.metrics?.iterations ?? 0
    const treeSize = timeline.metrics?.tree_size ?? 0
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
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(bugTrapMap())
                setStart(TRAP_START)
                setGoal(TRAP_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {BIASES.map((b) => (
                            <button key={b} type="button" onClick={() => setGoalBias(b)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        goalBias === b
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                bias {b}
                            </button>
                        ))}
                        {STEPS.map((s) => (
                            <button key={s} type="button" onClick={() => setStepSize(s)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        stepSize === s
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                η = {s}
                            </button>
                        ))}
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("regrow", "다시 성장")}
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
                            : <span className="font-semibold">
                                {t("iteration budget exhausted", "반복 예산 소진")}
                            </span>}
                        {" · "}{t("iterations", "반복")}{" "}
                        <span className="font-semibold">{iterations}</span>
                        {" · "}{t("tree", "트리")}{" "}
                        <span className="font-semibold">{treeSize}</span>
                    </div>
                </div>
            }
        />
    )
}

const RrtSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live RRT escaping a bug trap: the tree must find the left opening before it can swing around to the goal — compare iteration counts at bias 0 (pure exploration) and 0.05",
            "bug trap을 빠져나오는 라이브 RRT. 트리가 왼쪽 입구를 찾아야 goal로 돌아갈 수 있다. bias 0(순수 탐색)과 0.05의 반복 수를 비교해 보라",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<RrtScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <RrtScene panel={340}/>
    </CanvasFigure>
}

export default RrtSandbox

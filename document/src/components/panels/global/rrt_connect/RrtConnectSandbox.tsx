import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runRRT, runRRTConnect} from "../../../../libs/algorithms/rrt";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {SCURVE_GOAL, SCURVE_START, sCurveMap} from "./presets";


const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const RrtConnectScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(sCurveMap)
    const [start, setStart] = useState<Point>(SCURVE_START)
    const [goal, setGoal] = useState<Point>(SCURVE_GOAL)
    const [stepSize, setStepSize] = useState(0.5)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runRRTConnect({
            map, start, goal, maxIterations: 4000, stepSize, goalTolerance: 0.3, seed,
        })),
        [map, start, goal, stepSize, seed],
    )
    const rrt = useMemo(() => {
        const tl = buildGridTimeline(runRRT({
            map, start, goal, maxIterations: 4000, stepSize,
            goalBias: 0.05, goalTolerance: 0.3, seed,
        }))
        return {iterations: tl.metrics?.iterations ?? 0,
                success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, stepSize, seed])
    const cost = timeline.metrics?.path_cost
    const iterations = timeline.metrics?.iterations ?? 0
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
                setMap(sCurveMap())
                setStart(SCURVE_START)
                setGoal(SCURVE_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="η" value={stepSize} min={0.2} max={1.5} step={0.05} onCommit={setStepSize}/>
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
                        {" vs "}
                        {t("single-tree RRT", "단일 트리 RRT")}{" "}
                        <span className="font-semibold">
                            {rrt.success ? rrt.iterations : t("fail", "실패")}
                        </span>
                    </div>
                </div>
            }
        />
    )
}

const RrtConnectSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live RRT-Connect in an S-corridor: two trees race from both ends and splice in the middle — the readout compares its iteration count against single-tree RRT on the same seed",
            "S자 복도의 라이브 RRT-Connect. 두 트리가 양끝에서 달려와 가운데에서 접합된다. readout이 같은 seed의 단일 트리 RRT와 반복 수를 비교한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<RrtConnectScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <RrtConnectScene panel={340}/>
    </CanvasFigure>
}

export default RrtConnectSandbox

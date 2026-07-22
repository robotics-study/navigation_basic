import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runLQRRRTStar} from "../../../../libs/algorithms/lqr_rrt_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {cellCenterWorld, GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {LQR_GOAL, LQR_START, chevronMap} from "./presets";

const BUDGET = 1200

const LqrScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(chevronMap)
    const [start, setStart] = useState<Point>(LQR_START)
    const [goal, setGoal] = useState<Point>(LQR_GOAL)
    const [rCtrl, setRCtrl] = useState(1.0)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runLQRRRTStar({
            map, start, goal, maxIterations: BUDGET, stepSize: 1.5, goalBias: 0.1,
            goalTolerance: 1.0, neighborRadius: 2.0, qPos: 1.0, qVel: 1.0, rCtrl,
            lqrDt: 0.2, controlLimit: 10.0, maxVelocity: 1.5, seed,
        })),
        [map, start, goal, rCtrl, seed],
    )

    const nodes = timeline.metrics?.tree_size
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
        setter(cellCenterWorld(map, c))
    }

    return (
        <TracePlayer
            vehicle
            map={map} timeline={timeline} panel={panel}
            start={[start[0], start[1]]} goal={[goal[0], goal[1]]}
            showTree
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(chevronMap())
                setStart(LQR_START)
                setGoal(LQR_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <span>{t("control cost r", "제어 비용 r")}</span>
                        <ParamSlider label="r" value={rCtrl} min={0.2} max={5} step={0.1} onCommit={setRCtrl}/>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("regrow", "다시 성장")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {success
                            ? <>
                                {t("tree", "트리")}{" "}
                                <span className="font-semibold" style={{color: "var(--accent)"}}>
                                    {nodes}
                                </span>{" "}
                                {t("nodes", "노드")}
                                {cost !== undefined && <>
                                    {" · "}{t("LQR cost", "LQR 비용")}{" "}
                                    <span className="font-semibold" style={{color: PATH_COLOR}}>
                                        {cost.toFixed(2)}
                                    </span>
                                </>}
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                    </div>
                </div>
            }
        />
    )
}

const LqrRrtStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live LQR-RRT* weaving a chevron gauntlet: every edge is an LQR feedback trajectory that regulates to a rest waypoint, and raising the control cost r makes the same tree pay a higher LQR cost — cheap control steers aggressively, expensive control steers gently",
            "chevron 관문을 지그재그로 감는 라이브 LQR-RRT*. 모든 간선은 rest waypoint로 조절되는 LQR 피드백 궤적이고, 제어 비용 r을 키우면 같은 트리가 더 높은 LQR 비용을 치른다. 제어가 싸면 공격적으로, 비싸면 부드럽게 steer 한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<LqrScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <LqrScene panel={340}/>
    </CanvasFigure>
}

export default LqrRrtStarSandbox

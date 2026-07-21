import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runKinodynamicRRTStar} from "../../../../libs/algorithms/kinodynamic_rrt_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {KINO_GOAL, KINO_START, slashMap} from "./presets";

const BUDGET = 2000

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const KinodynamicScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(slashMap)
    const [start, setStart] = useState<Point>(KINO_START)
    const [goal, setGoal] = useState<Point>(KINO_GOAL)
    const [weight, setWeight] = useState(1.0)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runKinodynamicRRTStar({
            map, start, goal, maxIterations: BUDGET, goalBias: 0.1, goalTolerance: 1.0,
            neighborRadius: 2.0, controlWeight: weight, maxVelocity: 1.5, seed,
        })),
        [map, start, goal, weight, seed],
    )

    const tree = timeline.metrics?.tree_size
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
            vehicle
            map={map} timeline={timeline} panel={panel}
            start={[start[0], start[1]]} goal={[goal[0], goal[1]]}
            showTree
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(slashMap())
                setStart(KINO_START)
                setGoal(KINO_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <span>{t("effort penalty r", "제어 페널티 r")}</span>
                        <ParamSlider label="r" value={weight} min={0.2} max={3} step={0.1} onCommit={setWeight}/>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("regrow", "다시 성장")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {success
                            ? <>
                                {t("control cost", "제어 비용")}{" "}
                                <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {cost?.toFixed(2)}
                                </span>
                                {" · "}{t("tree", "트리")}{" "}
                                <span className="font-semibold">{tree}</span>
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                    </div>
                </div>
            }
        />
    )
}

const KinodynamicSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Kinodynamic RRT* sweeping around a diagonal slash: the double integrator carries momentum, so the car rounds the wall's free tip in a wide arc — raise the effort penalty r and the trajectories smooth out and cost more",
            "대각 벽을 감아 도는 라이브 Kinodynamic RRT*. double integrator는 관성을 실어 차가 벽의 자유 끝을 넓은 호로 돈다. 제어 페널티 r을 키우면 궤적이 완만해지고 비용이 커진다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<KinodynamicScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <KinodynamicScene panel={340}/>
    </CanvasFigure>
}

export default KinodynamicSandbox

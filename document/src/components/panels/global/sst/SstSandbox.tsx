import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runSST} from "../../../../libs/algorithms/sst";
import {runRRTStar} from "../../../../libs/algorithms/rrt_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {SST_GOAL, SST_START, islandMap} from "./presets";

// 라이브 SST sandbox. witness 반경 δ_s를 키우면 active 트리가 희소해지는(active 노드가
// 줄어드는) sparsification을, 같은 예산에서 모든 노드를 들고 있는 RRT*의 dense 트리
// 크기와 맞세워 보여 준다.
const RADII = [0.3, 0.4, 0.5]
const BUDGET = 4000

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const SstScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(islandMap)
    const [start, setStart] = useState<Point>(SST_START)
    const [goal, setGoal] = useState<Point>(SST_GOAL)
    const [deltaS, setDeltaS] = useState(0.4)
    const [seed, setSeed] = useState(2)

    const timeline = useMemo(
        () => buildGridTimeline(runSST({
            map, start, goal, maxIterations: BUDGET, goalBias: 0.1, goalTolerance: 0.8,
            deltaBn: 1.5, deltaS, maxVelocity: 1.5, maxOmega: 1.5,
            propDurationMin: 0.2, propDurationMax: 0.8, sstStar: false, seed,
        })),
        [map, start, goal, deltaS, seed],
    )
    // 같은 예산의 RRT* 참조: 모든 표본 노드를 트리에 남기는 dense 성장. δ_s와 무관하므로
    // regrow/편집 때만 다시 돈다.
    const rrtStar = useMemo(() => {
        const tl = buildGridTimeline(runRRTStar({
            map, start, goal, maxIterations: BUDGET, stepSize: 0.5, goalBias: 0.1,
            goalTolerance: 0.8, neighborRadius: 1.5, radiusMode: "fixed", rggGamma: 2, seed,
        }))
        return {nodes: tl.metrics?.tree_size, success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, seed])

    const active = timeline.metrics?.tree_size
    const added = timeline.metrics?.expanded_nodes
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
            vehicle carLength={0.8}
            map={map} timeline={timeline} panel={panel}
            start={[start[0], start[1]]} goal={[goal[0], goal[1]]}
            showTree
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(islandMap())
                setStart(SST_START)
                setGoal(SST_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <span>{t("witness δ_s", "witness δ_s")}</span>
                        {RADII.map((r) => (
                            <button key={r} type="button" onClick={() => setDeltaS(r)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        deltaS === r
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {r.toFixed(1)}
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
                                SST{" "}
                                <span className="font-semibold" style={{color: PATH_COLOR}}>
                                    {active}
                                </span>{" "}
                                {t("active", "active")}
                                {" / "}{added} {t("added", "추가")}
                            </>
                            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
                        {" vs RRT* "}
                        <span className="font-semibold">
                            {rrtStar.success ? rrtStar.nodes : t("fail", "실패")}
                        </span>{" "}
                        {t("nodes", "노드")}
                        {success && cost !== undefined && <>
                            {" · "}{t("cost", "비용")}{" "}
                            <span className="font-semibold">{cost.toFixed(2)}</span>
                        </>}
                    </div>
                </div>
            }
        />
    )
}

const SstSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live SST looping an island: the car drives the long way around while the witness radius δ_s thins the active tree (fewer kept nodes) — the same-budget RRT* keeps every sampled node; sparsity is the whole point",
            "섬을 반 바퀴 도는 라이브 SST. witness 반경 δ_s를 키우면 active 트리가 얇아지는데(남는 노드가 줄어든다), 같은 예산의 RRT*는 표본 노드를 전부 들고 있다. 희소함이 핵심이다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<SstScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <SstScene panel={340}/>
    </CanvasFigure>
}

export default SstSandbox

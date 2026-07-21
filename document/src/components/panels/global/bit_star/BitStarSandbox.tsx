import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runBITStar} from "../../../../libs/algorithms/bit_star";
import {runInformedRRTStar} from "../../../../libs/algorithms/informed_rrt_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {BIT_BATCH_SIZE, BIT_GAMMA, BIT_GOAL, BIT_START, blockMap} from "./presets";

// 라이브 BIT* sandbox. 배치 수를 늘리면 현직 해가 어떻게 조여지는지(anytime)와,
// 같은 표본 예산의 Informed RRT*가 아직 경로를 못 찾을 때 BIT*는 이미 더 낮은
// 비용에 닿아 있음을 나란히 보여준다. 간선 큐가 목표에 닿을 수 있는 간선부터
// 꺼내므로 BIT*는 더 적은 표본으로 먼저·더 좋은 경로를 얻는다 (Gammell et al. 2015).
const BATCH_COUNTS = [1, 2, 4, 6]

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const BitStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(blockMap)
    const [start, setStart] = useState<Point>(BIT_START)
    const [goal, setGoal] = useState<Point>(BIT_GOAL)
    const [numBatches, setNumBatches] = useState(4)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runBITStar({
            map, start, goal, batchSize: BIT_BATCH_SIZE, maxBatches: numBatches,
            gamma: BIT_GAMMA, seed,
        })),
        [map, start, goal, numBatches, seed],
    )
    // 같은 표본 예산(배치 수 × 배치 크기 = 반복 횟수)의 Informed RRT* 비교.
    const informed = useMemo(() => {
        const tl = buildGridTimeline(runInformedRRTStar({
            map, start, goal, maxIterations: numBatches * BIT_BATCH_SIZE, stepSize: 1.0,
            goalBias: 0.05, goalTolerance: 0.5, neighborRadius: 3.0, radiusMode: "fixed",
            rggGamma: 2, seed,
        }))
        return {cost: tl.metrics?.path_cost, success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, numBatches, seed])

    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0
    const edges = timeline.edges.length
    const improves = timeline.candidates.length

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
                setMap(blockMap())
                setStart(BIT_START)
                setGoal(BIT_GOAL)
                setNumBatches(4)
                setSeed(1)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {BATCH_COUNTS.map((n) => (
                            <button key={n} type="button" onClick={() => setNumBatches(n)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        numBatches === n
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {n} {t("batches", "배치")}
                            </button>
                        ))}
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        <InfoRow success={success} cost={cost} edges={edges}
                                 improves={improves} informed={informed} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, cost, edges, improves, informed, t}: {
    success: boolean; cost?: number; edges: number; improves: number;
    informed: {cost?: number; success: boolean};
    t: (en: string, ko: string) => string;
}) => (
    <>
        {success
            ? <>
                BIT*{" "}
                <span className="font-semibold">{edges}</span>
                {" " + t("edges", "간선") + " · "}
                <span className="font-semibold" style={{color: PATH_COLOR}}>
                    {cost?.toFixed(2)}
                </span>
                {" · " + improves + t("× improved", "회 개선")}
            </>
            : <span className="font-semibold">{t("no path yet", "아직 경로 없음")}</span>}
        {" vs Informed RRT* "}
        <span className="font-semibold">
            {informed.success ? informed.cost?.toFixed(2) : t("no path yet", "아직 경로 없음")}
        </span>
    </>
)

const BitStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live BIT* around a central block: raise the batch count and the informed ellipse tightens the incumbent toward the taut corner path, while Informed RRT* on the same sample budget has often not found a path yet — the edge queue reaches the goal with far fewer samples",
            "가운데 블록을 도는 라이브 BIT*. 배치 수를 늘리면 informed 타원이 현직 해를 모서리에 밀착한 팽팽한 경로로 조여 가는데, 같은 표본 예산의 Informed RRT*는 아직 경로를 못 찾은 경우가 많다. 간선 큐가 훨씬 적은 표본으로 목표에 닿는다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<BitStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <BitStarScene panel={340}/>
    </CanvasFigure>
}

export default BitStarSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runABITStar} from "../../../../libs/algorithms/abit_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {
    ABIT_BATCH_COUNTS, ABIT_BATCH_SIZE, ABIT_GAMMA, ABIT_GOAL, ABIT_INFLATION,
    ABIT_INFLATION_FINAL, ABIT_START, ABIT_TRUNCATION, wedgeMap,
} from "./presets";

// 라이브 ABIT* sandbox. 같은 표본 위에서 un-inflated 기준선(inflation 1, truncation 1,
// 곧 BIT*)을 함께 돌려, 세운 간선 수(= 통과한 충돌 검사)와 최종 비용을 나란히 보여준다.
// ABIT*는 팽창된 heuristic으로 goal 쪽을 먼저 훑고 truncation으로 마지막 비싼 검사를
// 건너뛰므로, 거의 같은 비용을 훨씬 적은 간선으로 얻는다.
const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const AbitStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(wedgeMap)
    const [start, setStart] = useState<Point>(ABIT_START)
    const [goal, setGoal] = useState<Point>(ABIT_GOAL)
    const [maxBatches, setMaxBatches] = useState(6)
    const [seed, setSeed] = useState(2)

    const common = {
        map, start, goal, batchSize: ABIT_BATCH_SIZE, maxBatches, gamma: ABIT_GAMMA, seed,
    }
    const timeline = useMemo(
        () => buildGridTimeline(runABITStar({
            ...common, inflationFactor: ABIT_INFLATION,
            inflationFinal: ABIT_INFLATION_FINAL, truncationFactor: ABIT_TRUNCATION,
        })),
        [map, start, goal, maxBatches, seed],
    )
    // 같은 표본·예산의 un-inflated 기준선: inflation·truncation을 모두 1 로 두면 배치마다
    // ABIT*가 정확히 BIT*로 환원된다.
    const bit = useMemo(() => {
        const tl = buildGridTimeline(runABITStar({
            ...common, inflationFactor: 1, inflationFinal: 1, truncationFactor: 1,
        }))
        return {edges: tl.edges.length, cost: tl.metrics?.path_cost,
                success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, maxBatches, seed])
    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0
    const abitEdges = timeline.edges.length

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
                setMap(wedgeMap())
                setStart(ABIT_START)
                setGoal(ABIT_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {ABIT_BATCH_COUNTS.map((b) => (
                            <button key={b} type="button" onClick={() => setMaxBatches(b)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        maxBatches === b
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {b} {t("batches", "배치")}
                            </button>
                        ))}
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        <InfoRow success={success} cost={cost} abitEdges={abitEdges} bit={bit}
                                 start={start} goal={goal} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, cost, abitEdges, bit, start, goal, t}: {
    success: boolean; cost?: number; abitEdges: number;
    bit: {edges: number; cost?: number; success: boolean};
    start: Point; goal: Point;
    t: (en: string, ko: string) => string;
}) => (
    <>
        {success
            ? <>
                ABIT*{" "}
                <span className="font-semibold">{abitEdges}</span>
                {" " + t("edges", "간선") + " · "}
                <span className="font-semibold" style={{color: PATH_COLOR}}>
                    {cost?.toFixed(2)}
                </span>
            </>
            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
        {" vs BIT* "}
        <span className="font-semibold">{bit.edges}</span>
        {" " + t("edges", "간선") + " · "}
        <span className="font-semibold">
            {bit.success ? bit.cost?.toFixed(2) : t("no path", "경로 없음")}
        </span>
        {" · "}
        {t("straight-line bound", "직선 하한")}{" "}
        <span className="font-semibold">
            {Math.hypot(goal[0] - start[0], goal[1] - start[1]).toFixed(2)}
        </span>
    </>
)

const AbitStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live ABIT* vs an un-inflated BIT* baseline on the same samples: the inflated heuristic sweeps toward the goal and truncation skips the last barely-improving edges, so ABIT* reaches essentially the same cost while building far fewer edges — and the gap widens with more batches",
            "같은 표본 위의 라이브 ABIT* vs un-inflated BIT* 기준선. 팽창된 heuristic이 goal 쪽을 먼저 훑고 truncation이 거의 개선 없는 마지막 간선을 건너뛰어, ABIT*는 사실상 같은 비용을 훨씬 적은 간선으로 얻는다. 배치가 늘수록 격차가 벌어진다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<AbitStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <AbitStarScene panel={340}/>
    </CanvasFigure>
}

export default AbitStarSandbox

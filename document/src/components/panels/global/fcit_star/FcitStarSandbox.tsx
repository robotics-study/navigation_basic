import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runFCITStar} from "../../../../libs/algorithms/fcit_star";
import {runAITStar} from "../../../../libs/algorithms/ait_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {FCIT_BATCH_SIZE, FCIT_GOAL, FCIT_START, staggeredMap} from "./presets";

const AIT_GAMMA = 30

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const FcitStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(staggeredMap)
    const [start, setStart] = useState<Point>(FCIT_START)
    const [goal, setGoal] = useState<Point>(FCIT_GOAL)
    const [numBatches, setNumBatches] = useState(2)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runFCITStar({
            map, start, goal, batchSize: FCIT_BATCH_SIZE, maxBatches: numBatches, seed,
        })),
        [map, start, goal, numBatches, seed],
    )
    // 같은 표본 예산(배치 수 × 배치 크기)의 AIT* 비교 — 줄어드는 반경 RGG.
    const ait = useMemo(() => {
        const tl = buildGridTimeline(runAITStar({
            map, start, goal, batchSize: FCIT_BATCH_SIZE, maxBatches: numBatches,
            gamma: AIT_GAMMA, seed,
        }))
        return {cost: tl.metrics?.path_cost, success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, numBatches, seed])

    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0
    const edges = timeline.edges.length

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
                setMap(staggeredMap())
                setStart(FCIT_START)
                setGoal(FCIT_GOAL)
                setNumBatches(2)
                setSeed(1)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label={t("batches", "배치")} value={numBatches} min={1} max={6} step={1} onCommit={setNumBatches}/>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        <InfoRow success={success} cost={cost} edges={edges} ait={ait} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, cost, edges, ait, t}: {
    success: boolean; cost?: number; edges: number;
    ait: {cost?: number; success: boolean};
    t: (en: string, ko: string) => string;
}) => (
    <>
        {success
            ? <>
                FCIT*{" "}
                <span className="font-semibold">{edges}</span>
                {" " + t("edges", "간선") + " · "}
                <span className="font-semibold" style={{color: PATH_COLOR}}>
                    {cost?.toFixed(2)}
                </span>
            </>
            : <span className="font-semibold">{t("no path yet", "아직 경로 없음")}</span>}
        {" vs AIT* "}
        <span className="font-semibold">
            {ait.success ? ait.cost?.toFixed(2) : t("no path yet", "아직 경로 없음")}
        </span>
    </>
)

const FcitStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live FCIT* between staggered blocks: the fully connected graph threads the long free stretches with single direct edges and lands near the optimum on the first batch, while AIT* on the same sample budget must chain radius-bounded edges and only catches up as batches accumulate",
            "엇갈린 블록 사이의 라이브 FCIT*. 완전 연결 그래프가 긴 자유 구간을 직행 간선 하나로 꿰어 첫 배치부터 최적 근처에 닿는데, 같은 표본 예산의 AIT*는 반경에 묶인 간선을 사슬로 이어야 해 배치가 쌓여야 따라온다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<FcitStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <FcitStarScene panel={340}/>
    </CanvasFigure>
}

export default FcitStarSandbox

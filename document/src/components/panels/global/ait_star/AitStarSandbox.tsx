import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {aitStarReadout, runAITStar} from "../../../../libs/algorithms/ait_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {GATE_GOAL, GATE_START, gateMap} from "./presets";

// 라이브 AIT* sandbox. 시작점에서 본 cost-to-go를 세 값으로 나란히 보여 준다:
// 직선거리 heuristic(BIT* 방식, 벽을 못 봄) vs AIT*의 역방향 그래프 heuristic(문 위로
// 도는 우회를 반영) vs 실제 현직 해 비용. 역방향 heuristic은 직선거리를 웃돌아 참
// 비용에 붙는다. 문(벽)을 지우면 둘이 같은 값으로 만난다 — 장애물이 없으면 일치한다.
const BATCH_COUNTS = [2, 4, 6];
const BATCH_SIZE = 120;
const GAMMA = 30;

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const AitStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(gateMap)
    const [start, setStart] = useState<Point>(GATE_START)
    const [goal, setGoal] = useState<Point>(GATE_GOAL)
    const [maxBatches, setMaxBatches] = useState(4)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runAITStar(
            {map, start, goal, batchSize: BATCH_SIZE, maxBatches, gamma: GAMMA, seed})),
        [map, start, goal, maxBatches, seed],
    )
    const readout = useMemo(
        () => aitStarReadout(
            {map, start, goal, batchSize: BATCH_SIZE, maxBatches, gamma: GAMMA, seed}),
        [map, start, goal, maxBatches, seed],
    )
    const success = readout.success

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
                setMap(gateMap())
                setStart(GATE_START)
                setGoal(GATE_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {BATCH_COUNTS.map((b) => (
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
                        <InfoRow success={success} straightH={readout.straightH}
                                 reverseH={readout.reverseH} optimal={readout.optimal} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, straightH, reverseH, optimal, t}: {
    success: boolean; straightH: number; reverseH: number; optimal: number;
    t: (en: string, ko: string) => string;
}) => (
    <>
        {t("straight-line h", "직선거리 h")}{" "}
        <span className="font-semibold">{straightH.toFixed(2)}</span>
        {" vs "}
        {t("AIT* reverse h", "AIT* 역방향 h")}{" "}
        <span className="font-semibold" style={{color: PATH_COLOR}}>{reverseH.toFixed(2)}</span>
        {" · "}
        {success
            ? <>{t("optimal", "최적")}{" "}<span className="font-semibold">{optimal.toFixed(2)}</span></>
            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
    </>
)

const AitStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live AIT* on a gated wall: the straight-line heuristic points through the wall and underestimates, while AIT*'s reverse-graph heuristic routes over the gate and tracks the true cost. Erase the wall and the two estimates meet",
            "문 달린 벽 위의 라이브 AIT*. 직선거리 heuristic은 벽을 관통해 비용을 낮게 잡지만, AIT*의 역방향 그래프 heuristic은 문 위로 도는 우회를 반영해 참 비용에 붙는다. 벽을 지우면 두 추정이 같은 값으로 만난다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<AitStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <AitStarScene panel={340}/>
    </CanvasFigure>
}

export default AitStarSandbox

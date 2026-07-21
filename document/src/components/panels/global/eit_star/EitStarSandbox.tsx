import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {eitStarReadout, runEITStar} from "../../../../libs/algorithms/eit_star";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {GATE_GOAL, GATE_START, shelfMap} from "./presets";

const BATCH_SIZE = 120;
const GAMMA = 30;
const STEP_SIZE = 0.5;

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const EitStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(shelfMap)
    const [start, setStart] = useState<Point>(GATE_START)
    const [goal, setGoal] = useState<Point>(GATE_GOAL)
    const [maxBatches, setMaxBatches] = useState(4)
    const [seed, setSeed] = useState(1)

    const opts = {map, start, goal, batchSize: BATCH_SIZE, maxBatches, gamma: GAMMA,
                  stepSize: STEP_SIZE, seed}
    const timeline = useMemo(
        () => buildGridTimeline(runEITStar(opts)),
        [map, start, goal, maxBatches, seed],
    )
    const readout = useMemo(
        () => eitStarReadout(opts),
        [map, start, goal, maxBatches, seed],
    )

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
                setMap(shelfMap())
                setStart(GATE_START)
                setGoal(GATE_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label={t("batches", "배치")} value={maxBatches} min={1} max={8} step={1} onCommit={setMaxBatches}/>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        <InfoRow success={readout.success} costToGo={readout.costToGo}
                                 effortToGo={readout.effortToGo}
                                 straightEffort={readout.straightEffort} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, costToGo, effortToGo, straightEffort, t}: {
    success: boolean; costToGo: number; effortToGo: number; straightEffort: number;
    t: (en: string, ko: string) => string;
}) => {
    if (!success) return <span className="font-semibold">{t("no path", "경로 없음")}</span>
    return (
        <>
            {t("cost-to-go ĥ", "cost-to-go ĥ")}{" "}
            <span className="font-semibold">{costToGo.toFixed(1)}</span>
            {" · "}
            {t("effort-to-go ê", "effort-to-go ê")}{" "}
            <span className="font-semibold" style={{color: PATH_COLOR}}>{effortToGo}</span>
            {" "}
            {t("checks", "검사")}
            {" "}
            <span className="opacity-70">
                {t("(straight-line", "(직선")}{" "}{straightEffort}{")"}
            </span>
        </>
    )
}

const EitStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live EIT* around a shelf wall: the start reads out two reverse heuristics — cost-to-go ĥ in metres (the same estimate AIT* builds) and effort-to-go ê in collision-check segments (the second reverse search EIT* adds). Both route around the shelf's open end, so both exceed the obstacle-blind straight-line count",
            "가로 선반을 도는 라이브 EIT*. 시작점에서 두 역방향 heuristic이 읽힌다. cost-to-go ĥ(meters, AIT*가 세우는 것과 같은 추정)와 effort-to-go ê(충돌 검사 segment 수, EIT*가 더한 두 번째 역방향 탐색)다. 둘 다 선반 오른쪽 끝을 돌아, 장애물을 못 보는 직선 검사 수를 웃돈다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<EitStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <EitStarScene panel={340}/>
    </CanvasFigure>
}

export default EitStarSandbox

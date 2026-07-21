import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runPRM} from "../../../../libs/algorithms/prm";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {GATE_GOAL, GATE_START, gateMap} from "./presets";

// 라이브 PRM sandbox. 좁은 문 맵에서 표본 수와 seed를 바꿔 가며 roadmap 이
// 이어지는 순간(확률적 완전성)을 직접 본다.
const SAMPLE_COUNTS = [60, 150, 300];
const RADII = [1.8, 2.5, 3.2];

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const PrmScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(gateMap)
    const [start, setStart] = useState<Point>(GATE_START)
    const [goal, setGoal] = useState<Point>(GATE_GOAL)
    const [numSamples, setNumSamples] = useState(150)
    const [radius, setRadius] = useState(2.5)
    const [seed, setSeed] = useState(2)

    const timeline = useMemo(
        () => buildGridTimeline(runPRM({
            map, start, goal, numSamples, connectionRadius: radius, seed,
        })),
        [map, start, goal, numSamples, radius, seed],
    )
    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0
    const edgeCount = timeline.edges.length

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
                        {SAMPLE_COUNTS.map((n) => (
                            <button key={n} type="button" onClick={() => setNumSamples(n)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        numSamples === n
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {n} {t("samples", "표본")}
                            </button>
                        ))}
                        {RADII.map((r) => (
                            <button key={r} type="button" onClick={() => setRadius(r)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        radius === r
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                r = {r}
                            </button>
                        ))}
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
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
                                {t("roadmap disconnected — add samples or resample",
                                    "roadmap이 끊겼다. 표본을 늘리거나 다시 추첨해 보라")}
                            </span>}
                        {" · "}{t("edges", "간선")} <span className="font-semibold">{edgeCount}</span>
                        {" · seed "}{seed}
                    </div>
                </div>
            }
        />
    )
}

const PrmSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live PRM at a narrow gate: with few samples the two rooms rarely connect through the door — raise the sample count or reroll the seed and watch the roadmap snap together",
            "좁은 문 앞의 라이브 PRM. 표본이 적으면 두 방이 문을 통해 이어지는 일이 드물다. 표본 수를 늘리거나 seed를 다시 추첨하면 roadmap이 순간 이어진다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<PrmScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <PrmScene panel={340}/>
    </CanvasFigure>
}

export default PrmSandbox

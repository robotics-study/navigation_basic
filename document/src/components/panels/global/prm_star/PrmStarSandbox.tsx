import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {rggRadius, runPRM, runPRMStar} from "../../../../libs/algorithms/prm";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {SCATTER_GOAL, SCATTER_START, scatterMap} from "./presets";

const PRM_RADIUS = 2.5;

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const PrmStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(scatterMap)
    const [start, setStart] = useState<Point>(SCATTER_START)
    const [goal, setGoal] = useState<Point>(SCATTER_GOAL)
    const [numSamples, setNumSamples] = useState(300)
    const [seed, setSeed] = useState(2)
    const [showPrm, setShowPrm] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runPRMStar({map, start, goal, numSamples, gamma: 30, seed})),
        [map, start, goal, numSamples, seed],
    )
    const prm = useMemo(() => {
        const tl = buildGridTimeline(runPRM({
            map, start, goal, numSamples, connectionRadius: PRM_RADIUS, seed,
        }))
        return {path: tl.path, cost: tl.metrics?.path_cost,
                success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, numSamples, seed])
    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0
    const rn = rggRadius(30, numSamples + 2)

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
            overlayPath={showPrm && prm.success ? prm.path : undefined}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            onReset={() => {
                setMap(scatterMap())
                setStart(SCATTER_START)
                setGoal(SCATTER_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label={t("samples", "표본")} value={numSamples} min={50} max={1000} step={25} onCommit={setNumSamples}/>
                        <button type="button" onClick={() => setShowPrm((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showPrm
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("PRM overlay", "PRM 겹치기")}
                        </button>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        <InfoRow success={success} cost={cost} prm={prm} rn={rn} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, cost, prm, rn, t}: {
    success: boolean; cost?: number;
    prm: {cost?: number; success: boolean};
    rn: number; t: (en: string, ko: string) => string;
}) => (
    <>
        {success
            ? <>
                PRM*{" "}
                <span className="font-semibold" style={{color: PATH_COLOR}}>
                    {cost?.toFixed(2)}
                </span>
            </>
            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
        {" vs PRM(r=2.5) "}
        <span className="font-semibold">
            {prm.success ? prm.cost?.toFixed(2) : t("disconnected", "끊김")}
        </span>
        {" · "}
        <span>
            r<sub>n</sub> = <span className="font-semibold">{rn.toFixed(2)}</span>
        </span>
    </>
)

const PrmStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live PRM* vs fixed-radius PRM on the same samples: at 100 samples the wide early radius keeps PRM* connected while PRM breaks, and as n grows the shrinking radius still drives the cost toward the straight-line bound",
            "같은 표본 위의 라이브 PRM* vs 고정 반경 PRM. 표본 100개에서는 넉넉한 초기 반경 덕에 PRM*만 이어지고 PRM은 끊긴다. n이 커지면 반경이 줄면서도 비용은 직선 하한으로 수렴한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<PrmStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <PrmStarScene panel={340}/>
    </CanvasFigure>
}

export default PrmStarSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runFMTStar} from "../../../../libs/algorithms/fmt_star";
import {runPRMStar} from "../../../../libs/algorithms/prm";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {FMT_GOAL, FMT_START, zigMap} from "./presets";

const GAMMA = 30;

const cellToWorld = (map: GridMap, c: Cell): Point =>
    [map.originX + (c[1] + 0.5) * map.resolution,
     map.originY + (map.height - 1 - c[0] + 0.5) * map.resolution]

const FmtStarScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(zigMap)
    const [start, setStart] = useState<Point>(FMT_START)
    const [goal, setGoal] = useState<Point>(FMT_GOAL)
    const [numSamples, setNumSamples] = useState(300)
    const [seed, setSeed] = useState(1)

    const timeline = useMemo(
        () => buildGridTimeline(runFMTStar({map, start, goal, numSamples, gamma: GAMMA, seed})),
        [map, start, goal, numSamples, seed],
    )
    const prm = useMemo(() => {
        const tl = buildGridTimeline(runPRMStar({map, start, goal, numSamples, gamma: GAMMA, seed}))
        return {edges: tl.edges.length, cost: tl.metrics?.path_cost,
                success: tl.success !== false && tl.paths.length > 0}
    }, [map, start, goal, numSamples, seed])
    const cost = timeline.metrics?.path_cost
    const success = timeline.success !== false && timeline.paths.length > 0
    const fmtEdges = timeline.edges.length

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
                setMap(zigMap())
                setStart(FMT_START)
                setGoal(FMT_GOAL)
            }}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label={t("samples", "표본")} value={numSamples} min={50} max={800} step={25} onCommit={setNumSamples}/>
                        <button type="button" onClick={() => setSeed((s) => s + 1)}
                                className="px-2 py-0.5 rounded border border-border hover:bg-surface">
                            {t("resample", "다시 추첨")}
                        </button>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        <InfoRow success={success} cost={cost} fmtEdges={fmtEdges} prm={prm} t={t}/>
                    </div>
                </div>
            }
        />
    )
}

const InfoRow = ({success, cost, fmtEdges, prm, t}: {
    success: boolean; cost?: number; fmtEdges: number;
    prm: {edges: number; cost?: number; success: boolean};
    t: (en: string, ko: string) => string;
}) => (
    <>
        {success
            ? <>
                FMT*{" "}
                <span className="font-semibold">{fmtEdges}</span>
                {" " + t("edges", "간선") + " · "}
                <span className="font-semibold" style={{color: PATH_COLOR}}>
                    {cost?.toFixed(2)}
                </span>
            </>
            : <span className="font-semibold">{t("no path", "경로 없음")}</span>}
        {" vs PRM* "}
        <span className="font-semibold">{prm.edges}</span>
        {" " + t("edges", "간선") + " · "}
        <span className="font-semibold">
            {prm.success ? prm.cost?.toFixed(2) : t("disconnected", "끊김")}
        </span>
    </>
)

const FmtStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live FMT* vs PRM* on the same samples: both reach the same cost, but FMT*'s lazy collision check builds an order of magnitude fewer edges — one locally cheapest edge per node instead of every feasible neighbor pair",
            "같은 표본 위의 라이브 FMT* vs PRM*. 둘 다 같은 비용에 닿지만, FMT*의 lazy 충돌 검사는 노드마다 근방 최소비용 간선 하나만 세워 간선 수가 자릿수만큼 적다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<FmtStarScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <FmtStarScene panel={340}/>
    </CanvasFigure>
}

export default FmtStarSandbox

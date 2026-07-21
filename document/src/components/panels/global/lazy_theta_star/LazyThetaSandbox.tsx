import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runLazyThetaStar, runThetaStar} from "../../../../libs/algorithms/theta_star";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {rubbleMap, RUBBLE_GOAL, RUBBLE_START} from "../theta_star/presets";

// 라이브 Lazy Theta* sandbox. 잔해 지형에서 Theta* 와 Lazy 를 같이 돌려, 경로 비용은
// 사실상 같은데 LOS 검사 횟수가 크게 줄어드는 것을 수치로 비교한다.
type Mode = "theta" | "lazy";

const LazyScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(rubbleMap)
    const [start, setStart] = useState<Cell>(RUBBLE_START)
    const [goal, setGoal] = useState<Cell>(RUBBLE_GOAL)
    const [mode, setMode] = useState<Mode>("lazy")

    const runs = useMemo(() => {
        const theta = buildGridTimeline(runThetaStar({map, start, goal, heuristicWeight: 1}))
        const lazy = buildGridTimeline(runLazyThetaStar({map, start, goal, heuristicWeight: 1}))
        return {theta, lazy}
    }, [map, start, goal])
    const timeline = runs[mode]
    const losOf = (m: Mode) => runs[m].metrics?.los_checks ?? 0
    const costOf = (m: Mode) => runs[m].metrics?.path_cost ?? 0

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const moveEndpoint = (setter: (c: Cell) => void) => (c: Cell) => {
        if (map.occupied[c[0] * map.width + c[1]]) return
        setter(c)
    }

    return (
        <TracePlayer
            map={map} timeline={timeline} start={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
                        {(["theta", "lazy"] as const).map((m) => (
                            <button key={m} type="button" onClick={() => setMode(m)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        mode === m
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {m === "theta" ? "Theta*" : "Lazy Theta*"}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("LOS checks", "LOS 검사")}:{" "}
                        Theta* <span className="font-semibold">{losOf("theta")}</span>
                        {" vs "}Lazy{" "}
                        <span className="font-semibold" style={{color: "var(--accent)"}}>
                            {losOf("lazy")}
                        </span>
                        {" · "}{t("path cost", "경로 비용")}:{" "}
                        <span className="font-semibold" style={{color: PATH_COLOR}}>
                            {costOf("theta").toFixed(3)}
                        </span>
                        {" / "}
                        <span className="font-semibold" style={{color: PATH_COLOR}}>
                            {costOf("lazy").toFixed(3)}
                        </span>
                    </div>
                </div>
            }
        />
    )
}

const LazyThetaSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live comparison in rubble: nearly identical any-angle paths, but Lazy Theta* runs one line-of-sight check per expansion instead of one per edge",
            "잔해 지형의 라이브 비교. 경로는 사실상 같지만 Lazy Theta* 는 LOS 검사를 edge 마다가 아니라 확장마다 한 번만 한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<LazyScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <LazyScene panel={340}/>
    </CanvasFigure>
}

export default LazyThetaSandbox

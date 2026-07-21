import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runVisibilityAStar} from "../../../../libs/algorithms/visibility_astar";
import {runThetaStar} from "../../../../libs/algorithms/theta_star";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {SLAB_GOAL, SLAB_START, slabsMap} from "./presets";

// 라이브 Visibility A* sandbox. 같은 문제의 Theta* 경로(점선)를 겹쳐, 부모 사슬에
// 갇힌 any-angle 근사와 visibility graph 최적의 차이를 비용 숫자로 비교한다.
const VisibilityScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(slabsMap)
    const [start, setStart] = useState<Cell>(SLAB_START)
    const [goal, setGoal] = useState<Cell>(SLAB_GOAL)
    const [showTheta, setShowTheta] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runVisibilityAStar({map, start, goal, heuristicWeight: 1})),
        [map, start, goal],
    )
    const theta = useMemo(() => {
        const tl = buildGridTimeline(runThetaStar({map, start, goal, heuristicWeight: 1}))
        return {path: tl.path, cost: tl.metrics?.path_cost ?? 0}
    }, [map, start, goal])
    const visCost = timeline.metrics?.path_cost ?? 0
    const losChecks = timeline.metrics?.los_checks ?? 0

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
            overlayPath={showTheta ? theta.path : undefined}
            onPaintCell={paintCell}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <button type="button" onClick={() => setShowTheta((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showTheta
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("Theta* path overlay", "Theta* 경로 겹치기")}
                        </button>
                        <span>
                            Visibility A*{" "}
                            <span className="font-semibold" style={{color: PATH_COLOR}}>
                                {visCost.toFixed(2)}
                            </span>
                            {" vs "}Theta*{" "}
                            <span className="font-semibold">{theta.cost.toFixed(2)}</span>
                        </span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("LOS checks", "LOS 검사")}{" "}
                        <span className="font-semibold">{losChecks}</span>
                        {" · "}
                        {t("drag cells to draw walls · drag the endpoints",
                            "셀을 드래그해 벽을 그리고, 끝점을 끌어 보라")}
                    </div>
                </div>
            }
        />
    )
}

const VisibilitySandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Visibility A* on staggered slabs: a handful of expansions, and a path strictly shorter than Theta*'s — the dashed overlay misses one taut turn between the slabs",
            "엇갈린 슬래브 위의 라이브 Visibility A*. 확장 몇 번 만에, Theta*보다 엄밀히 짧은 경로가 나온다. 점선 겹치기는 슬래브 사이 taut 턴 하나를 놓친다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<VisibilityScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <VisibilityScene panel={340}/>
    </CanvasFigure>
}

export default VisibilitySandbox

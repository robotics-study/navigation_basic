import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runThetaStar} from "../../../../libs/algorithms/theta_star";
import {runAStar} from "../../../../libs/algorithms/astar";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {pillarsMap, PILLAR_GOAL, PILLAR_START} from "./presets";

// 라이브 Theta* sandbox. 같은 문제의 A* 경로(점선)를 겹쳐 그려, 45°에 갇힌 grid 경로와
// any-angle 직선 경로의 차이를 한 화면에서 비교한다.
const ThetaScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(pillarsMap)
    const [start, setStart] = useState<Cell>(PILLAR_START)
    const [goal, setGoal] = useState<Cell>(PILLAR_GOAL)
    const [showAstar, setShowAstar] = useState(true)

    const timeline = useMemo(
        () => buildGridTimeline(runThetaStar({map, start, goal, heuristicWeight: 1})),
        [map, start, goal],
    )
    const astar = useMemo(() => {
        const events = runAStar({map, start, goal, heuristicWeight: 1, connectivity: 8})
        const tl = buildGridTimeline(events)
        return {path: tl.path, cost: tl.metrics?.path_cost ?? 0}
    }, [map, start, goal])
    const thetaCost = timeline.metrics?.path_cost ?? 0

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
            overlayPath={showAstar ? astar.path : undefined}
            onPaintCell={paintCell}
            onReset={() => { setMap(pillarsMap()); setStart(PILLAR_START); setGoal(PILLAR_GOAL) }}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <button type="button" onClick={() => setShowAstar((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showAstar
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("A* path overlay", "A* 경로 겹치기")}
                        </button>
                        <span>
                            Theta*{" "}
                            <span className="font-semibold" style={{color: PATH_COLOR}}>
                                {thetaCost.toFixed(2)}
                            </span>
                            {" vs "}A*{" "}
                            <span className="font-semibold">{astar.cost.toFixed(2)}</span>
                        </span>
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("drag cells to draw walls · drag the endpoints",
                            "셀을 드래그해 벽을 그리고, 끝점을 끌어 보라")}
                    </div>
                </div>
            }
        />
    )
}

const ThetaStarSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Theta* among pillars: the red any-angle path hugs obstacle corners with long straight segments, while the dashed A* path stays locked to 45° grid moves",
            "기둥 사이의 라이브 Theta*. 빨간 any-angle 경로는 긴 직선으로 기둥 모서리를 스치고, 점선 A* 경로는 45° grid 이동에 갇혀 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<ThetaScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <ThetaScene panel={340}/>
    </CanvasFigure>
}

export default ThetaStarSandbox

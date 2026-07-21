import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runAnyaFull} from "../../../../libs/algorithms/anya";
import {runVisibilityAStar} from "../../../../libs/algorithms/visibility_astar";
import {runThetaStar} from "../../../../libs/algorithms/theta_star";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import cn from "../../../../libs/cn";
import {BLOCK_GOAL, BLOCK_START, blockMap} from "./presets";

// 라이브 Anya sandbox. 빨간 경로는 corner 기하 그대로(truePath)이고, 점선은 같은
// 문제의 Visibility A* 경로다. any-angle 위계(Anya ≤ Visibility ≤ Theta*)를 비용
// 숫자 세 개로 한 화면에서 보인다.
const AnyaScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(blockMap)
    const [start, setStart] = useState<Cell>(BLOCK_START)
    const [goal, setGoal] = useState<Cell>(BLOCK_GOAL)
    const [showVis, setShowVis] = useState(true)

    const anya = useMemo(
        () => runAnyaFull({map, start, goal, vertexEpsilon: 1e-9}),
        [map, start, goal],
    )
    const timeline = useMemo(() => buildGridTimeline(anya.events), [anya])
    const vis = useMemo(() => {
        const tl = buildGridTimeline(runVisibilityAStar({map, start, goal, heuristicWeight: 1}))
        return {path: tl.path, cost: tl.metrics?.path_cost ?? 0}
    }, [map, start, goal])
    const thetaCost = useMemo(
        () => buildGridTimeline(runThetaStar({map, start, goal, heuristicWeight: 1}))
            .metrics?.path_cost ?? 0,
        [map, start, goal],
    )
    const anyaCost = timeline.metrics?.path_cost ?? 0

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
            truePath={anya.geometry}
            overlayPath={showVis ? vis.path : undefined}
            onPaintCell={paintCell}
            onReset={() => { setMap(blockMap()); setStart(BLOCK_START); setGoal(BLOCK_GOAL) }}
            onMoveStart={moveEndpoint(setStart)}
            onMoveGoal={moveEndpoint(setGoal)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <button type="button" onClick={() => setShowVis((v) => !v)}
                                className={cn(
                                    "px-2 py-0.5 rounded border",
                                    showVis
                                        ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                        : "border-border hover:bg-surface",
                                )}>
                            {t("Visibility A* overlay", "Visibility A* 겹치기")}
                        </button>
                        <span>
                            Anya{" "}
                            <span className="font-semibold" style={{color: PATH_COLOR}}>
                                {anyaCost.toFixed(2)}
                            </span>
                            {" vs Vis A* "}
                            <span className="font-semibold">{vis.cost.toFixed(2)}</span>
                            {" vs Theta* "}
                            <span className="font-semibold">{thetaCost.toFixed(2)}</span>
                        </span>
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("the red path bends exactly on grid corners",
                            "빨간 경로는 정확히 격자 꼭짓점에서 꺾인다")}
                        {" · "}
                        {t("drag cells to draw walls · drag the endpoints",
                            "셀을 드래그해 벽을 그리고, 끝점을 끌어 보라")}
                    </div>
                </div>
            }
        />
    )
}

const AnyaSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Anya around a block: the red path hugs the block's corners — lattice points no cell centre can express — and undercuts the dashed Visibility A* path",
            "블록을 도는 라이브 Anya. 빨간 경로가 블록 모서리(어떤 셀 중심도 표현 못 하는 격자점)를 스치며, 점선 Visibility A* 경로보다 짧다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<AnyaScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <AnyaScene panel={340}/>
    </CanvasFigure>
}

export default AnyaSandbox

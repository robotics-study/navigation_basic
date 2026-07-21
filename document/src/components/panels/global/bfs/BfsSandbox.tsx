import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {runBFS} from "../../../../libs/algorithms/bfs";
import {buildGridTimeline, Cell} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";
import {pocketMap, SANDBOX_GOAL, SANDBOX_START} from "../astar/presets";

// 라이브 BFS sandbox — FIFO frontier 가 hop 동심원으로 번지는 것을 본다.
// 벽을 그리고 시작/목표를 끌면 즉시 재탐색한다.
const BfsScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(pocketMap)
    const [start, setStart] = useState<Cell>(SANDBOX_START)
    const [goal, setGoal] = useState<Cell>(SANDBOX_GOAL)
    const [connectivity, setConnectivity] = useState<4 | 8>(8)

    const timeline = useMemo(
        () => buildGridTimeline(runBFS({map, start, goal, connectivity})),
        [map, start, goal, connectivity],
    )
    const hops = timeline.path.length > 0 ? timeline.path.length - 1 : null

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
                        {([4, 8] as const).map((c) => (
                            <button key={c} type="button" onClick={() => setConnectivity(c)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border tabular-nums",
                                        connectivity === c
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {c}-conn
                            </button>
                        ))}
                        {hops !== null && (
                            <span className="tabular-nums">
                                {t("hops", "hop 수")}{" "}
                                <span className="font-semibold" style={{color: "var(--accent)"}}>{hops}</span>
                            </span>
                        )}
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("drag cells to draw walls · drag the endpoints to move start/goal",
                            "셀을 드래그해 벽을 그리고, 원을 끌어 시작/목표를 옮겨 보라")}
                    </div>
                </div>
            }
        />
    )
}

const BfsSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live BFS: the frontier expands as concentric hop-rings, blind to direction — the path it returns has the fewest edges, not the lowest cost",
            "라이브 BFS. frontier 가 방향을 모른 채 hop 동심원으로 번지고, 반환 경로는 비용이 아니라 edge 수가 최소다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<BfsScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <BfsScene panel={340}/>
    </CanvasFigure>
}

export default BfsSandbox

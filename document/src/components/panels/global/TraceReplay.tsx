import {useEffect, useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../CanvasFigure";
import TracePlayer from "../../player/TracePlayer";
import {loadGridMap, loadTrace} from "../../../libs/trace/load";
import {buildGridTimeline, Cell, GridTimeline} from "../../../libs/trace/timeline";
import {GridMap} from "../../../libs/grid";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";

// 저장소의 실제 데모가 방출한 trace를 재생하는 공용 패널. 어떤 알고리즘이든
// spec/trace_schema.json 이벤트 계약만 지키면 이 하나로 재생된다.
// C++/Python 데모는 동일한 이벤트 열을 방출하므로 웹에는 한 벌(py)만 싣는다.
interface Loaded {
    map: GridMap;
    timeline: GridTimeline;
}

interface TraceReplayProps {
    algo: string;
    maps: string[];
    // CanvasFigure 캡션 (호출부에서 번역해 넘긴다).
    label: string;
    // 참 기하 경로 재계산 (Anya처럼 turning point가 셀 중심이 아닌 planner용).
    // trace의 path는 셀로 스냅되어 스냅 꼭짓점 사이 직선이 장애물을 시각적으로
    // 가로지를 수 있다 — 동일 입력으로 라이브 엔진을 돌려 실제 기하를 그린다.
    truePathOf?: (map: GridMap, start: Cell, goal: Cell,
                  params: Record<string, unknown> | undefined) => Cell[];
    // SE(2) 차량 planner 재생: 로봇을 차로 그리고 경로를 주행한다.
    vehicle?: boolean;
}

const ReplayScene = ({algo, maps, truePathOf, vehicle, panel = 340}: {
    algo: string; maps: string[]; truePathOf?: TraceReplayProps["truePathOf"];
    vehicle?: boolean; panel?: number;
}) => {
    const t = useTr()
    const [mapName, setMapName] = useState(maps[0])
    const [loaded, setLoaded] = useState<Loaded | null>(null)
    const [error, setError] = useState<string | null>(null)

    useEffect(() => {
        let cancelled = false
        setLoaded(null)
        setError(null)
        Promise.all([
            loadGridMap(`data/maps/${mapName}.json`),
            loadTrace(`data/traces/${algo}/${mapName}.py.jsonl.gz`),
        ]).then(([map, events]) => {
            if (cancelled) return
            setLoaded({map, timeline: buildGridTimeline(events)})
        }).catch((e: unknown) => {
            if (!cancelled) setError(e instanceof Error ? e.message : String(e))
        })
        return () => { cancelled = true }
    }, [algo, mapName])

    // trace에는 시작/목표가 명시 이벤트로 없다 — 첫 확장 노드가 시작, 경로 끝이 목표다.
    // 확장 이벤트가 없는 trace(RRT 계열)는 경로 양끝, 그것도 없으면 첫 간선의
    // 부모(트리 뿌리)로 유도한다.
    const start = useMemo<Cell | undefined>(
        () => loaded?.timeline.expanded[0]?.cell
            ?? (loaded && loaded.timeline.path.length > 0
                ? loaded.timeline.path[0] : undefined)
            ?? loaded?.timeline.edges[0]?.from,
        [loaded])
    const goal = useMemo<Cell | undefined>(
        () => loaded && loaded.timeline.path.length > 0
            ? loaded.timeline.path[loaded.timeline.path.length - 1]
            : undefined,
        [loaded])
    const truePath = useMemo<Cell[] | undefined>(
        () => loaded && start && goal && truePathOf
            ? truePathOf(loaded.map, start, goal, loaded.timeline.params)
            : undefined,
        [loaded, start, goal, truePathOf])

    const selector = (
        <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
            {maps.length > 1 && maps.map((m) => (
                <button key={m} type="button" onClick={() => setMapName(m)}
                        className={cn(
                            "px-2 py-0.5 rounded border",
                            mapName === m
                                ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                : "border-border hover:bg-surface",
                        )}>
                    {m}
                </button>
            ))}
        </div>
    )

    if (error || !loaded) {
        return <div className="flex flex-col items-center gap-2">
            <div className="grid place-items-center text-sm text-muted border border-border rounded-lg"
                 style={{width: panel, height: panel}}>
                {error ? `${t("failed to load trace", "trace 로드 실패")}: ${error}` : "Loading…"}
            </div>
            {selector}
        </div>
    }
    return <TracePlayer map={loaded.map} timeline={loaded.timeline}
                        start={start} goal={goal} truePath={truePath} vehicle={vehicle}
                        panel={panel} footer={selector}/>
}

const TraceReplay = ({algo, maps, label, truePathOf, vehicle}: TraceReplayProps) => {
    return <CanvasFigure
        label={label}
        tight
        bodyClassName="w-fit"
        className="w-full"
        modal={<ReplayScene algo={algo} maps={maps} truePathOf={truePathOf} vehicle={vehicle}
                            panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <ReplayScene algo={algo} maps={maps} truePathOf={truePathOf} vehicle={vehicle}/>
    </CanvasFigure>
}

export default TraceReplay

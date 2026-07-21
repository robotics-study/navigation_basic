import {useEffect, useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import TracePlayer from "../../../player/TracePlayer";
import {loadGridMap, loadTrace} from "../../../../libs/trace/load";
import {buildGridTimeline, Cell, GridTimeline} from "../../../../libs/trace/timeline";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// 실제 C++/Python 데모가 방출한 trace 를 재생한다 — 사이트의 시각화가 저장소 구현과
// 같은 이벤트 계약(spec/trace_schema.json)을 소비한다는 것을 그대로 보여 주는 패널.
const IMPLS = ["python", "c++"] as const;
const MAPS = ["maze01", "open01"] as const;
type Impl = typeof IMPLS[number];
type MapName = typeof MAPS[number];

// c++ 산출물 파일명은 URL 친화적으로 cpp 를 쓴다.
const implFile = (impl: Impl) => impl === "c++" ? "cpp" : "py";

interface Loaded {
    map: GridMap;
    timeline: GridTimeline;
}

const ReplayScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [impl, setImpl] = useState<Impl>("python")
    const [mapName, setMapName] = useState<MapName>("maze01")
    const [loaded, setLoaded] = useState<Loaded | null>(null)
    const [error, setError] = useState<string | null>(null)

    useEffect(() => {
        let cancelled = false
        setLoaded(null)
        setError(null)
        Promise.all([
            loadGridMap(`data/maps/${mapName}.json`),
            loadTrace(`data/traces/astar/${mapName}.${implFile(impl)}.jsonl.gz`),
        ]).then(([map, events]) => {
            if (cancelled) return
            setLoaded({map, timeline: buildGridTimeline(events)})
        }).catch((e: unknown) => {
            if (!cancelled) setError(e instanceof Error ? e.message : String(e))
        })
        return () => { cancelled = true }
    }, [impl, mapName])

    // trace 에는 시작/목표가 명시 이벤트로 없다 — 첫 확장 노드가 시작, 경로 끝이 목표다.
    const start = useMemo<Cell | undefined>(
        () => loaded?.timeline.expanded[0]?.cell, [loaded])
    const goal = useMemo<Cell | undefined>(
        () => loaded && loaded.timeline.path.length > 0
            ? loaded.timeline.path[loaded.timeline.path.length - 1]
            : undefined,
        [loaded])

    const selector = (
        <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
            {IMPLS.map((i) => (
                <button key={i} type="button" onClick={() => setImpl(i)}
                        className={cn(
                            "px-2 py-0.5 rounded border",
                            impl === i
                                ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                : "border-border hover:bg-surface",
                        )}>
                    {i}
                </button>
            ))}
            <span className="mx-1" aria-hidden="true">·</span>
            {MAPS.map((m) => (
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

    if (error) {
        return <div className="flex flex-col items-center gap-2">
            <div className="grid place-items-center text-sm text-muted border border-border rounded-lg"
                 style={{width: panel, height: panel}}>
                {t("failed to load trace", "trace 로드 실패")}: {error}
            </div>
            {selector}
        </div>
    }
    if (!loaded) {
        return <div className="flex flex-col items-center gap-2">
            <div className="grid place-items-center text-sm text-muted border border-border rounded-lg"
                 style={{width: panel, height: panel}}>
                Loading…
            </div>
            {selector}
        </div>
    }
    return <TracePlayer map={loaded.map} timeline={loaded.timeline}
                        start={start} goal={goal} panel={panel} footer={selector}/>
}

const AStarReplay = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Replaying real traces emitted by the repository's C++ and Python A* demos — the same JSON event stream drives this player",
            "저장소의 C++/Python demo 가 실제로 방출한 trace 재생. 이 플레이어는 그 JSON 이벤트 스트림을 그대로 소비한다",
        )}
        tight
        bodyClassName="w-fit"
        className="w-full"
        modal={<ReplayScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <ReplayScene panel={340}/>
    </CanvasFigure>
}

export default AStarReplay

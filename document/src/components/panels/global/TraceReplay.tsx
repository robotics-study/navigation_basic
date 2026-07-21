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
}

const ReplayScene = ({algo, maps, panel = 340}: {algo: string; maps: string[]; panel?: number}) => {
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
    const start = useMemo<Cell | undefined>(
        () => loaded?.timeline.expanded[0]?.cell, [loaded])
    const goal = useMemo<Cell | undefined>(
        () => loaded && loaded.timeline.path.length > 0
            ? loaded.timeline.path[loaded.timeline.path.length - 1]
            : undefined,
        [loaded])

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
                        start={start} goal={goal} panel={panel} footer={selector}/>
}

const TraceReplay = ({algo, maps, label}: TraceReplayProps) => {
    return <CanvasFigure
        label={label}
        tight
        bodyClassName="w-fit"
        className="w-full"
        modal={<ReplayScene algo={algo} maps={maps} panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <ReplayScene algo={algo} maps={maps}/>
    </CanvasFigure>
}

export default TraceReplay

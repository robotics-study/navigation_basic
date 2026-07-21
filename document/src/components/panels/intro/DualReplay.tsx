import {ReactNode, useEffect, useMemo, useRef, useState} from "react";
import {GridMap} from "../../../libs/grid";
import {Cell, GridTimeline} from "../../../libs/trace/timeline";
import GridCanvas, {PATH_COLOR} from "../../2d/GridCanvas";
import {useTr} from "../../../libs/i18n";

// 두 timeline을 하나의 진행률로 나란히 재생한다. 카테고리 인트로에서 "같은 문제를
// 두 패러다임이 어떻게 푸는가"를 한 컷으로 대비시키는 용도 — 알고리즘 페이지의 full
// sandbox와 달리 조작은 재생/스크럽 하나로 최소화한다.
export interface ReplaySide {
    label: ReactNode;
    map: GridMap;
    timeline: GridTimeline;
    // grid planner는 (row, col) 셀, 연속 planner는 [x, y] world 좌표를 그대로 넘긴다
    // (GridCanvas가 timeline.continuous로 구분해 그린다).
    start?: Cell;
    goal?: Cell;
    showTree?: boolean;
    shadowCells?: Cell[];
}

const TICK_MS = 30;
const DURATION_MS = 3200;   // 진행률 0→1에 걸리는 시간 (timeline 길이와 무관한 고정 체감 속도)

// 현재 step 까지의 한 줄 통계 — 탐색형은 확장 수, sampling 계열은 표본 수를 보여 준다.
const SideStat = ({timeline, step}: {timeline: GridTimeline; step: number}) => {
    const t = useTr()
    const expanded = useMemo(
        () => timeline.expanded.filter((e) => e.step <= step).length, [timeline, step])
    const samples = useMemo(
        () => timeline.samples.filter((s) => s.step <= step).length, [timeline, step])
    const cost = useMemo(() => {
        let latest: number | null = null
        for (const p of timeline.paths) if (p.step <= step) latest = p.cost
        return latest
    }, [timeline, step])
    const sampling = timeline.expanded.length === 0 && timeline.samples.length > 0
    return (
        <div className="text-xs text-muted text-center tabular-nums">
            {sampling
                ? <>{t("samples", "표본")}{" "}
                    <span className="font-semibold" style={{color: "var(--accent)"}}>{samples}</span></>
                : <>{t("expanded", "확장")}{" "}
                    <span className="font-semibold" style={{color: "var(--accent)"}}>{expanded}</span></>}
            {cost != null && <>
                {" · "}{t("cost", "비용")}{" "}
                <span className="font-semibold" style={{color: PATH_COLOR}}>{cost.toFixed(2)}</span>
            </>}
        </div>
    )
}

const DualReplay = ({left, right, panel = 236, caption}: {
    left: ReplaySide; right: ReplaySide; panel?: number; caption?: ReactNode;
}) => {
    const t = useTr()
    const [frac, setFrac] = useState(0)
    const [playing, setPlaying] = useState(true)
    const timer = useRef<number>()

    // timeline이 바뀌면 처음부터 다시 재생한다.
    useEffect(() => {
        setFrac(0)
        setPlaying(true)
    }, [left.timeline, right.timeline])

    useEffect(() => {
        if (!playing) return
        const per = TICK_MS / DURATION_MS
        timer.current = window.setInterval(() => {
            setFrac((f) => {
                if (f >= 1) {
                    setPlaying(false)
                    return 1
                }
                return Math.min(1, f + per)
            })
        }, TICK_MS)
        return () => window.clearInterval(timer.current)
    }, [playing])

    const stepOf = (tl: GridTimeline) => Math.round(frac * tl.steps)
    const finished = frac >= 1
    const replay = () => { setFrac(0); setPlaying(true) }

    const side = (s: ReplaySide) => (
        <div className="flex flex-col items-center gap-1">
            <span className="text-xs font-semibold" style={{color: "var(--accent)"}}>{s.label}</span>
            <GridCanvas map={s.map} panel={panel} timeline={s.timeline} step={stepOf(s.timeline)}
                        start={s.start} goal={s.goal} showTree={s.showTree}
                        shadowCells={s.shadowCells}/>
            <SideStat timeline={s.timeline} step={stepOf(s.timeline)}/>
        </div>
    )

    return (
        <div className="flex flex-col items-center gap-2">
            <div className="flex flex-wrap items-start justify-center gap-4">
                {side(left)}
                {side(right)}
            </div>
            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: panel * 2 + 16}}>
                <button type="button" aria-label={playing ? t("pause", "일시정지") : t("play", "재생")}
                        onClick={() => finished ? replay() : setPlaying((p) => !p)}
                        className="px-1.5 py-1 rounded border border-border hover:bg-surface leading-none">
                    {playing
                        ? <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M7 5h4v14H7zM13 5h4v14h-4z"/></svg>
                        : <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M8 5v14l11-7z"/></svg>}
                </button>
                <input type="range" min={0} max={1000} value={Math.round(frac * 1000)}
                       onChange={(e) => { setPlaying(false); setFrac(parseInt(e.target.value) / 1000) }}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("search progress", "탐색 진행")}/>
            </div>
            {caption && <div className="text-xs text-muted text-center max-w-prose">{caption}</div>}
        </div>
    )
}

export default DualReplay

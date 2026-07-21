import {ReactNode, useEffect, useMemo, useRef, useState} from "react";
import {GridMap} from "../../libs/grid";
import {Cell, GridTimeline} from "../../libs/trace/timeline";
import GridCanvas, {PATH_COLOR} from "../2d/GridCanvas";
import {useTr} from "../../libs/i18n";
import cn from "../../libs/cn";

// 타임라인 전체를 도는 목표 시간. 이벤트 수와 무관하게 체감 속도를 맞춘다 (고정 배속).
const BASE_DURATION_MS = 3000;
const TICK_MS = 30;

interface TracePlayerProps {
    map: GridMap;
    timeline: GridTimeline;
    start?: Cell;
    goal?: Cell;
    panel?: number;
    showTree?: boolean;
    overlayPath?: Cell[];
    autoPlay?: boolean;
    // sandbox 상호작용 passthrough — 입력이 바뀌면 부모가 timeline을 새로 만든다.
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
    onMoveStart?: (cell: Cell) => void;
    onMoveGoal?: (cell: Cell) => void;
    // 플레이어 아래 추가 컨트롤 (heuristic 버튼 등).
    footer?: ReactNode;
}

const Btn = ({onClick, label, children, active}: {
    onClick: () => void; label: string; children: ReactNode; active?: boolean
}) => (
    <button type="button" onClick={onClick} aria-label={label}
            className={cn(
                "px-1.5 py-1 rounded border border-border hover:bg-surface leading-none",
                active && "border-[var(--accent)] text-[var(--accent)]",
            )}>
        {children}
    </button>
)

const TracePlayer = ({
                         map, timeline, start, goal, panel = 340, showTree, overlayPath,
                         autoPlay = true, onPaintCell, onMoveStart, onMoveGoal, footer,
                     }: TracePlayerProps) => {
    const t = useTr()
    const [step, setStep] = useState(autoPlay ? 0 : timeline.steps)
    const [playing, setPlaying] = useState(autoPlay)
    const timer = useRef<number>()

    // 타임라인이 바뀌면(입력 변경/트레이스 교체) 처음부터 다시 재생한다.
    useEffect(() => {
        setStep(0)
        setPlaying(autoPlay)
    }, [timeline, autoPlay])

    useEffect(() => {
        if (!playing) return
        const perTick = Math.max(1, Math.round(timeline.steps / (BASE_DURATION_MS / TICK_MS)))
        timer.current = window.setInterval(() => {
            setStep((s) => {
                if (s >= timeline.steps) {
                    setPlaying(false)
                    return s
                }
                return Math.min(timeline.steps, s + perTick)
            })
        }, TICK_MS)
        return () => window.clearInterval(timer.current)
    }, [playing, timeline])

    const expandedCount = useMemo(
        () => timeline.expanded.filter((e) => e.step <= step).length,
        [timeline, step],
    )
    const finished = step >= timeline.steps
    // anytime planner 대응: 현재 step 까지 발표된 최신 경로의 비용을 보여 준다.
    const visiblePath = useMemo(() => {
        let latest: GridTimeline["paths"][number] | null = null
        for (const p of timeline.paths) {
            if (p.step <= step) latest = p
        }
        return latest
    }, [timeline, step])
    const pathShown = visiblePath !== null

    const replay = () => {
        setStep(0)
        setPlaying(true)
    }

    return (
        <div className="flex flex-col gap-2 items-center">
            <GridCanvas map={map} panel={panel} timeline={timeline} step={step}
                        start={start} goal={goal} showTree={showTree} overlayPath={overlayPath}
                        onPaintCell={onPaintCell} onMoveStart={onMoveStart} onMoveGoal={onMoveGoal}/>

            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: panel}}>
                {playing
                    ? <Btn onClick={() => setPlaying(false)} label={t("pause", "일시정지")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M7 5h4v14H7zM13 5h4v14h-4z"/>
                        </svg>
                    </Btn>
                    : <Btn onClick={() => finished ? replay() : setPlaying(true)} label={t("play", "재생")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="currentColor" aria-hidden="true">
                            <path d="M8 5v14l11-7z"/>
                        </svg>
                    </Btn>}
                <input type="range" min={0} max={timeline.steps} value={Math.min(step, timeline.steps)}
                       onChange={(e) => {
                           setPlaying(false)
                           setStep(parseInt(e.target.value))
                       }}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("search progress", "탐색 진행")}/>
            </div>

            <div className="text-xs text-muted text-center tabular-nums">
                {t("expanded", "확장한 노드")}{" "}
                <span className="font-semibold" style={{color: "var(--accent)"}}>{expandedCount}</span>
                {pathShown && <>
                    {" · "}{t("path cost", "경로 비용")}{" "}
                    <span className="font-semibold" style={{color: PATH_COLOR}}>
                        {visiblePath!.cost.toFixed(2)}
                    </span>
                    {timeline.paths.length > 1 && <>
                        {" · "}{t("solution", "해")}{" "}
                        <span className="tabular-nums">
                            {timeline.paths.filter((p) => p.step <= step).length}/{timeline.paths.length}
                        </span>
                    </>}
                </>}
                {finished && !pathShown && <>
                    {" · "}<span className="font-semibold">{t("no path", "경로 없음")}</span>
                </>}
            </div>

            {footer}
        </div>
    )
}

export default TracePlayer

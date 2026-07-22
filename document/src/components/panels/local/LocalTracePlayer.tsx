import {ReactNode, useEffect, useMemo, useRef, useState} from "react";
import {Arrow, Circle, Layer, Line, Rect, Shape, Stage} from "react-konva";
import Konva from "konva";
import {GridMap, worldToCellUnits} from "../../../libs/grid";
import {TraceEvent} from "../../../libs/trace/types";
import {useCanvasColors} from "../../../libs/useTheme";
import {useTr} from "../../../libs/i18n";
import {PATH_COLOR} from "../../2d/GridCanvas";
import cn from "../../../libs/cn";

// local planner sandbox 3종(potential fields, VFH, pure pursuit)이 공유하는 재생기.
// TracePlayer(global)와 같은 골격(고정 배속 재생, 배속 버튼 없음, scrub, reset)이지만
// local 전용 오버레이(로봇 trail·heading, force 화살표, histogram rose, candidate,
// 참조 경로, 종료 status 배지)를 그린다. 알고리즘 상태는 전부 trace 이벤트에서만
// 읽는다 — 엔진 내부 상태 접근 금지(시각화에 필요한 정보는 trace 이벤트로 방출되어야
// 한다는 저장소 계약).
const BASE_DURATION_MS = 3000;
const TICK_MS = 30;

type Pose = [number, number, number];

interface RobotTick { step: number; pose: Pose }
interface ForceTick { step: number; pos: [number, number]; att: [number, number]; rep: [number, number] }
interface HistogramTick { step: number; pos: [number, number]; bins: number[]; threshold?: number }
interface CandidateTick { step: number; pos: [number, number]; selected: boolean }

interface LocalTimeline {
    steps: number;
    robot: RobotTick[];
    forces: ForceTick[];
    histograms: HistogramTick[];
    candidates: CandidateTick[];
    path: Pose[] | null;
    success?: boolean;
    metrics?: Record<string, number>;
}

const numField = (data: Record<string, unknown> | undefined, key: string): number => {
    const v = data?.[key]
    return typeof v === "number" ? v : 0
}

// trace 이벤트 열을 재생 가능한 타임라인으로 접는다 — global의 buildGridTimeline과 같은
// "step = 이벤트 index" 관례. local 전용 이벤트(force_computed/histogram_updated/
// candidate_evaluated/robot_moved)만 다룬다.
function buildLocalTimeline(events: TraceEvent[]): LocalTimeline {
    const timeline: LocalTimeline = {
        steps: events.length, robot: [], forces: [], histograms: [], candidates: [], path: null,
    }
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "robot_moved": {
                const s = ev.state
                if (s && s.length >= 3) timeline.robot.push({step, pose: [s[0], s[1], s[2]]})
                break
            }
            case "force_computed": {
                const s = ev.state
                if (s && s.length >= 2) {
                    timeline.forces.push({
                        step, pos: [s[0], s[1]],
                        att: [numField(ev.data, "fx_att"), numField(ev.data, "fy_att")],
                        rep: [numField(ev.data, "fx_rep"), numField(ev.data, "fy_rep")],
                    })
                }
                break
            }
            case "histogram_updated": {
                const s = ev.state
                if (s && s.length >= 2 && ev.bins) {
                    const threshold = ev.data?.threshold
                    timeline.histograms.push({
                        step, pos: [s[0], s[1]], bins: ev.bins,
                        threshold: typeof threshold === "number" ? threshold : undefined,
                    })
                }
                break
            }
            case "candidate_evaluated": {
                const s = ev.state
                if (s && s.length >= 2) {
                    timeline.candidates.push({step, pos: [s[0], s[1]], selected: ev.data?.selected === 1})
                }
                break
            }
            case "path_found":
                if (ev.path) timeline.path = ev.path.map((p) => [p[0], p[1], p[2] ?? 0])
                break
            case "planning_finished":
                timeline.success = ev.success
                timeline.metrics = ev.metrics
                break
            default:
                break
        }
    })
    return timeline
}

type SimStatus = "reached" | "collision" | "stalled" | "timeout";

// bench의 status 라벨 복원과 같은 규칙(metrics는 숫자 플래그만 나른다) — success/collided/
// stalled에서 4종 종료 상태를 되살린다.
function deriveStatus(timeline: LocalTimeline): SimStatus | undefined {
    if (!timeline.metrics) return undefined
    if (timeline.success) return "reached"
    if (timeline.metrics.collided) return "collision"
    if (timeline.metrics.stalled) return "stalled"
    return "timeout"
}

const Btn = ({onClick, label, children}: {onClick: () => void; label: string; children: ReactNode}) => (
    <button type="button" onClick={onClick} aria-label={label}
            className="px-1.5 py-1 rounded border border-border hover:bg-surface leading-none">
        {children}
    </button>
)

export interface LocalTracePlayerProps {
    map: GridMap;
    events: TraceEvent[];
    // 시작/목표는 trace에 없다(웹 엔진은 sandbox가 직접 준다) — 표시·드래그 모두 여기서 받는다.
    startPose: Pose;
    goal: [number, number];
    // Pure Pursuit 등 추종 계열의 참조 경로(회색 파선 underlay). 없으면 그리지 않는다.
    referencePath?: [number, number][];
    panel?: number;
    autoPlay?: boolean;
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
    onMoveStart?: (xy: [number, number]) => void;
    onMoveGoal?: (xy: [number, number]) => void;
    onReset?: () => void;
    footer?: ReactNode;
}

const LocalTracePlayer = ({
                              map, events, startPose, goal, referencePath, panel = 340,
                              autoPlay = true, onPaintCell, onMoveStart, onMoveGoal, onReset, footer,
                          }: LocalTracePlayerProps) => {
    const t = useTr()
    const colors = useCanvasColors()
    const timeline = useMemo(() => buildLocalTimeline(events), [events])
    const status = useMemo(() => deriveStatus(timeline), [timeline])

    const [step, setStep] = useState(autoPlay ? 0 : timeline.steps)
    const [playing, setPlaying] = useState(autoPlay)
    const timer = useRef<number>()

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

    const finished = step >= timeline.steps
    const replay = () => {
        setStep(0)
        setPlaying(true)
    }

    // --- 좌표 변환: grid.ts의 worldToCellUnits만 재사용(새 변환 없음) -----------------
    const cellPx = panel / Math.max(map.width, map.height)
    const stageW = Math.round(cellPx * map.width)
    const stageH = Math.round(cellPx * map.height)
    const toPixel = (x: number, y: number): [number, number] => {
        const [u, v] = worldToCellUnits(map, x, y)
        return [u * cellPx, v * cellPx]
    }
    // worldToCellUnits의 역변환(sandbox 드래그용) — 순수 대수적 역산이라 새 좌표계가 아니다.
    const pixelToWorld = (px: number, py: number): [number, number] => [
        map.originX + (px / cellPx) * map.resolution,
        map.originY + (map.height - py / cellPx) * map.resolution,
    ]

    // --- 벽 페인팅(GridCanvas의 붓 페인팅과 같은 관례) -----------------------------
    const paintValue = useRef<boolean | null>(null)
    const cellAt = (stage: Konva.Stage | null): [number, number] | null => {
        const pos = stage?.getPointerPosition()
        if (!pos) return null
        const col = Math.floor(pos.x / cellPx)
        const row = Math.floor(pos.y / cellPx)
        if (row < 0 || row >= map.height || col < 0 || col >= map.width) return null
        return [row, col]
    }
    const startPx = toPixel(startPose[0], startPose[1])
    const goalPx = toPixel(goal[0], goal[1])
    const nearPoint = (ax: number, ay: number, px: number, py: number): boolean =>
        Math.hypot(px - ax, py - ay) <= cellPx * 0.75
    const paint = (c: [number, number]) => {
        if (!onPaintCell || paintValue.current === null) return
        const px = (c[1] + 0.5) * cellPx
        const py = (c[0] + 0.5) * cellPx
        if (nearPoint(startPx[0], startPx[1], px, py) || nearPoint(goalPx[0], goalPx[1], px, py)) return
        onPaintCell(c[0], c[1], paintValue.current)
    }

    // --- 재생 시점(step) 이하의 최신/누적 이벤트 -----------------------------------
    const robotTrail = useMemo(
        () => timeline.robot.filter((r) => r.step <= step).map((r) => r.pose),
        [timeline, step],
    )
    const currentPose: Pose = robotTrail.length > 0 ? robotTrail[robotTrail.length - 1] : startPose
    const latestForce = useMemo(() => {
        let latest: ForceTick | undefined
        for (const f of timeline.forces) { if (f.step <= step) latest = f; else break }
        return latest
    }, [timeline, step])
    const latestHistogram = useMemo(() => {
        let latest: HistogramTick | undefined
        for (const h of timeline.histograms) { if (h.step <= step) latest = h; else break }
        return latest
    }, [timeline, step])
    const currentCandidates = useMemo(() => {
        // 가장 최근 tick 한 번 분량만 — 누적하면 궤적 전체가 후보로 뒤덮여 "이번 tick이
        // 무엇을 평가했는지"라는 교육 목적을 잃는다. 한 tick은 [candidate_evaluated* →
        // robot_moved] 순으로 방출되므로, 직전 robot_moved(있다면)와 현재 step 사이가
        // "완결됐거나 진행 중인 최신 tick"의 경계다.
        const robotSteps = timeline.robot.filter((r) => r.step <= step).map((r) => r.step)
        const lastRobotStep = robotSteps.length > 0 ? robotSteps[robotSteps.length - 1] : -1
        const prevRobotStep = robotSteps.length > 1 ? robotSteps[robotSteps.length - 2] : -1
        const boundary = step > lastRobotStep ? lastRobotStep : prevRobotStep
        return timeline.candidates.filter((c) => c.step <= step && c.step > boundary)
    }, [timeline, step])
    const visiblePath = finished && timeline.path ? timeline.path : null

    // --- 렌더 헬퍼 ------------------------------------------------------------------
    const headingArrow = (pose: Pose) => {
        const [px, py] = toPixel(pose[0], pose[1])
        const len = cellPx * 0.9
        // world θ(ccw, +x 기준) → canvas 회전은 y축 반전 때문에 부호가 반대다(GridCanvas와 동일 관례).
        const dx = Math.cos(-pose[2]) * len
        const dy = Math.sin(-pose[2]) * len
        return <Arrow key="heading" points={[px, py, px + dx, py + dy]}
                      pointerLength={cellPx * 0.32} pointerWidth={cellPx * 0.28}
                      stroke={colors.accent2} fill={colors.accent2}
                      strokeWidth={Math.max(1.5, cellPx * 0.12)}/>
    }

    const forceArrows = (f: ForceTick) => {
        const [px, py] = toPixel(f.pos[0], f.pos[1])
        // world 벡터(x,y) → canvas: x는 그대로, y는 반전.
        const scale = cellPx * 0.6
        const toArrow = (vec: [number, number], color: string, key: string) => {
            const mag = Math.hypot(vec[0], vec[1])
            if (mag < 1e-6) return null
            const ux = vec[0] / mag
            const uy = -vec[1] / mag
            const len = Math.min(cellPx * 2.2, scale * mag)
            return <Arrow key={key} points={[px, py, px + ux * len, py + uy * len]}
                          pointerLength={cellPx * 0.22} pointerWidth={cellPx * 0.2}
                          stroke={color} fill={color} strokeWidth={Math.max(1.2, cellPx * 0.08)}
                          opacity={0.85}/>
        }
        return <>
            {toArrow(f.att, "#2563eb", "att")}
            {toArrow(f.rep, "#dc2626", "rep")}
        </>
    }

    const histogramRose = (h: HistogramTick) => {
        const [cx, cy] = toPixel(h.pos[0], h.pos[1])
        const n = h.bins.length
        if (n === 0) return null
        const maxBin = Math.max(...h.bins, 1e-9)
        const baseR = cellPx * 2.4
        const sector = (2 * Math.PI) / n
        return <Shape key="hist" listening={false} sceneFunc={(ctx, shape) => {
            ctx.beginPath()
            for (let k = 0; k < n; k++) {
                const r = baseR * Math.min(1, h.bins[k] / maxBin)
                if (r <= 0) continue
                const a0 = k * sector
                const a1 = (k + 1) * sector
                // world CCW 각 → canvas 각(-각, y-flip) — 구간이 뒤집혀 start/end를 교환한다.
                const ca0 = -a1
                const ca1 = -a0
                ctx.moveTo(cx, cy)
                ctx.arc(cx, cy, r, ca0, ca1, false)
                ctx.closePath()
            }
            ctx.fillStrokeShape(shape)
        }} fill={colors.accent} opacity={0.28} stroke={colors.accent} strokeWidth={1}/>
    }

    const candidateMarkers = currentCandidates.map((c, i) => {
        const [px, py] = toPixel(c.pos[0], c.pos[1])
        return <Circle key={`cand${i}`} x={px} y={py} radius={Math.max(1.5, cellPx * 0.12)}
                       fill={c.selected ? colors.accent2 : colors.muted}
                       opacity={c.selected ? 0.95 : 0.55}/>
    })

    const statusLabel = (s: SimStatus): string => ({
        reached: t("reached", "도달"),
        collision: t("collision", "충돌"),
        stalled: t("stalled", "정체"),
        timeout: t("timeout", "시간 초과"),
    })[s]
    const statusColor = (s: SimStatus): string => ({
        reached: colors.accent2, collision: PATH_COLOR, stalled: colors.accent, timeout: colors.muted,
    })[s]

    return (
        <div className="flex flex-col gap-2 items-center">
            <Stage width={stageW} height={stageH}
                   className="bg-surface border border-border rounded-lg overflow-hidden w-fit"
                   onPointerDown={(e) => {
                       if (!onPaintCell) return
                       const c = cellAt(e.target.getStage())
                       if (!c) return
                       const pos = e.target.getStage()?.getPointerPosition()
                       if (pos && (nearPoint(startPx[0], startPx[1], pos.x, pos.y)
                           || nearPoint(goalPx[0], goalPx[1], pos.x, pos.y))) return
                       paintValue.current = !map.occupied[c[0] * map.width + c[1]]
                       paint(c)
                   }}
                   onPointerMove={(e) => {
                       if (paintValue.current === null) return
                       const c = cellAt(e.target.getStage())
                       if (c) paint(c)
                   }}
                   onPointerUp={() => { paintValue.current = null }}
                   onPointerLeave={() => { paintValue.current = null }}>
                <Layer>
                    {/* 벽 */}
                    {map.occupied.map((occ, i) => occ && (
                        <Rect key={`w${i}`} x={(i % map.width) * cellPx} y={Math.floor(i / map.width) * cellPx}
                              width={cellPx} height={cellPx} fill={colors.text} opacity={0.78}/>
                    ))}
                    {/* 참조 경로 (추종 계열) */}
                    {referencePath && referencePath.length > 1 && (
                        <Line points={referencePath.flatMap(([x, y]) => toPixel(x, y))}
                              stroke={colors.muted} strokeWidth={Math.max(1.6, cellPx * 0.14)}
                              dash={[cellPx * 0.5, cellPx * 0.4]} lineCap="round" lineJoin="round"
                              opacity={0.85}/>
                    )}
                    {/* 실행 궤적(성공 시 최종 경로 강조) */}
                    {robotTrail.length > 1 && (
                        <Line points={robotTrail.flatMap((p) => toPixel(p[0], p[1]))}
                              stroke={visiblePath ? PATH_COLOR : colors.accent2}
                              strokeWidth={Math.max(2, cellPx * (visiblePath ? 0.24 : 0.18))}
                              lineCap="round" lineJoin="round" opacity={0.9}/>
                    )}
                    {/* histogram rose (VFH, 최신 tick) */}
                    {latestHistogram && histogramRose(latestHistogram)}
                    {/* candidate 마커(직전 tick) */}
                    {candidateMarkers}
                    {/* force 화살표(PF, 최신 tick 1건만) */}
                    {latestForce && forceArrows(latestForce)}
                    {/* 로봇 heading */}
                    {headingArrow(currentPose)}
                    {/* 시작/목표 마커 */}
                    <Circle x={startPx[0]} y={startPx[1]} radius={cellPx * 0.34} fill={colors.accent}
                            stroke={colors.bg} strokeWidth={Math.max(1, cellPx * 0.06)}
                            hitStrokeWidth={cellPx * 0.9} draggable={!!onMoveStart}
                            onDragEnd={(e) => {
                                e.target.position({x: startPx[0], y: startPx[1]})
                                onMoveStart?.(pixelToWorld(e.target.x(), e.target.y()))
                            }}/>
                    <Circle x={goalPx[0]} y={goalPx[1]} radius={cellPx * 0.34} fill={PATH_COLOR}
                            stroke={colors.bg} strokeWidth={Math.max(1, cellPx * 0.06)}
                            hitStrokeWidth={cellPx * 0.9} draggable={!!onMoveGoal}
                            onDragEnd={(e) => {
                                e.target.position({x: goalPx[0], y: goalPx[1]})
                                onMoveGoal?.(pixelToWorld(e.target.x(), e.target.y()))
                            }}/>
                </Layer>
            </Stage>

            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: stageW}}>
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
                       aria-label={t("playback progress", "재생 진행")}/>
                {onReset && (
                    <Btn onClick={onReset} label={t("reset the sandbox", "sandbox 초기화")}>
                        <svg width="14" height="14" viewBox="0 0 24 24" fill="none"
                             stroke="currentColor" strokeWidth="2.4" strokeLinecap="round"
                             strokeLinejoin="round" aria-hidden="true">
                            <path d="M3 12a9 9 0 1 0 3-6.7"/>
                            <path d="M3 4v5h5"/>
                        </svg>
                    </Btn>
                )}
            </div>

            <div className={cn("text-xs text-center tabular-nums",
                finished && status ? "font-semibold" : "text-muted")}
                 style={finished && status ? {color: statusColor(status)} : undefined}>
                {finished && status
                    ? statusLabel(status)
                    : `${t("step", "step")} ${step}/${timeline.steps}`}
            </div>

            {footer}
        </div>
    )
}

export default LocalTracePlayer

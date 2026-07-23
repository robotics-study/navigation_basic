import {ReactNode, useEffect, useMemo, useRef, useState} from "react";
import {Arrow, Circle, Group, Layer, Line, Rect, Shape, Stage} from "react-konva";
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

interface RobotTick { step: number; pose: Pose; v?: number }
interface ForceTick { step: number; pos: [number, number]; att: [number, number]; rep: [number, number] }
interface HistogramTick { step: number; pos: [number, number]; bins: number[]; threshold?: number }
// Elastic Bands/TEB의 band_updated 원본을 그대로 나른다 — 항목 길이 3 = [x, y, radius]
// (bubble), 길이 4 = [x, y, theta, dt] (pose + 직전 세그먼트 ΔT)로 렌더 시점에 판별한다.
interface BandTick { step: number; band: number[][] }
interface CandidateTick {
    step: number;
    pos: [number, number];
    selected: boolean;
    // 롤아웃 채점 계열(DWA)의 예측 궤적 폴리라인·admissible 플래그. 방출하지 않는
    // 엔진(PF/VFH/추종 계열)에서는 rollout이 없어 기존 렌더가 그대로 유지된다.
    admissible: boolean;
    rollout?: [number, number][];
}

interface LocalTimeline {
    steps: number;
    robot: RobotTick[];
    forces: ForceTick[];
    histograms: HistogramTick[];
    bands: BandTick[];
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
        steps: events.length, robot: [], forces: [], histograms: [], bands: [], candidates: [], path: null,
    }
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "robot_moved": {
                const s = ev.state
                if (s && s.length >= 3) {
                    const v = ev.data?.v
                    timeline.robot.push({
                        step, pose: [s[0], s[1], s[2]],
                        v: typeof v === "number" ? Math.abs(v) : undefined,
                    })
                }
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
            case "band_updated": {
                if (ev.band && ev.band.length > 0) {
                    timeline.bands.push({step, band: ev.band})
                }
                break
            }
            case "candidate_evaluated": {
                const s = ev.state
                if (s && s.length >= 2) {
                    timeline.candidates.push({
                        step, pos: [s[0], s[1]],
                        selected: ev.data?.selected === 1,
                        // 플래그가 없는 엔진(PP의 단일 lookahead 점 등)은 admissible로
                        // 간주해 마커가 기존 불투명도를 유지한다.
                        admissible: ev.data?.admissible !== 0,
                        rollout: ev.rollout
                            ?.filter((p) => p.length >= 2)
                            .map((p): [number, number] => [p[0], p[1]]),
                    })
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
    // 로봇 충돌 원 반경(world 단위) — 차량 몸체 + footprint 원을 이 크기로 그린다.
    // local planner 는 모두 모바일 로봇용이므로 sandbox 는 항상 넘겨야 한다.
    footprintRadius?: number;
    // 추종 계열: 로봇 중심 lookahead 원(반경 = 로봇→이번 tick 후보점 거리)을 그린다.
    showLookahead?: boolean;
    // Stanley: 로봇→참조 경로 최근접점 crosstrack 선분을 그린다.
    showCrosstrack?: boolean;
    // 로봇 중심에 항상 그리는 보조 반경(world 단위, 예: RPP 의 근접 감속 반경 d_prox).
    auxCircleRadius?: number;
    // Elastic Bands/TEB: 최신 tick의 band_updated(bubble/pose 열)를 그린다. band_updated를
    // 방출하지 않는 엔진에서는 latestBand가 없어 자연히 미표시되므로, 페이지가 명시적으로
    // 끌 때만 false로 둔다.
    showBand?: boolean;
    panel?: number;
    autoPlay?: boolean;
    onPaintCell?: (row: number, col: number, occupied: boolean) => void;
    onMoveStart?: (xy: [number, number]) => void;
    onMoveGoal?: (xy: [number, number]) => void;
    onReset?: () => void;
    footer?: ReactNode;
}

const LocalTracePlayer = ({
                              map, events, startPose, goal, referencePath, footprintRadius,
                              showLookahead, showCrosstrack, showBand = true, auxCircleRadius, panel = 340,
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
    // 드롭 지점이 맵 안이고 footprint 원이 벽과 겹치지 않아야 이동을 허용한다 —
    // 충돌 지점에 놓으면 시뮬레이션이 0 tick 충돌로 끝나 조작이 고장 나 보인다.
    const droppedFree = (p: [number, number]): boolean => {
        const fr = footprintRadius ?? 0
        if (p[0] < map.originX + fr || p[1] < map.originY + fr
            || p[0] > map.originX + map.width * map.resolution - fr
            || p[1] > map.originY + map.height * map.resolution - fr) return false
        const r0 = Math.max(0, Math.floor((map.height - (p[1] - map.originY + fr) / map.resolution)))
        const r1 = Math.min(map.height - 1, Math.floor((map.height - (p[1] - map.originY - fr) / map.resolution)))
        const c0 = Math.max(0, Math.floor((p[0] - map.originX - fr) / map.resolution))
        const c1 = Math.min(map.width - 1, Math.floor((p[0] - map.originX + fr) / map.resolution))
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) {
                if (!map.occupied[r * map.width + c]) continue
                // 셀 사각형과 원의 최소 거리 검사
                const cx0 = map.originX + c * map.resolution
                const cy0 = map.originY + (map.height - 1 - r) * map.resolution
                const dx = Math.max(cx0 - p[0], 0, p[0] - (cx0 + map.resolution))
                const dy = Math.max(cy0 - p[1], 0, p[1] - (cy0 + map.resolution))
                if (Math.hypot(dx, dy) < fr) return false
            }
        }
        return true
    }
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
        () => timeline.robot.filter((r) => r.step <= step),
        [timeline, step],
    )
    const currentPose: Pose = robotTrail.length > 0 ? robotTrail[robotTrail.length - 1].pose : startPose
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
    const latestBand = useMemo(() => {
        let latest: BandTick | undefined
        for (const b of timeline.bands) { if (b.step <= step) latest = b; else break }
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
    const pxPerWorld = cellPx / map.resolution
    const vmax = useMemo(
        () => timeline.robot.reduce((m, r) => Math.max(m, r.v ?? 0), 0),
        [timeline],
    )

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

    // 모바일 로봇: footprint 원(충돌 모델 그대로) + 그 안에 내접하는 차체. GridCanvas의
    // car()와 같은 시각 어휘 — 대각선이 원 안에 들어오도록 몸체 길이를 지름의 0.86으로 잡는다.
    const vehicle = (pose: Pose, fr: number) => {
        const [px, py] = toPixel(pose[0], pose[1])
        const rPx = fr * pxPerWorld
        const len = rPx * 2 * 0.86
        const wid = len * 0.58
        const sw = Math.max(1.2, rPx * 0.14)
        return <Group key="vehicle" x={px} y={py} rotation={-pose[2] * 180 / Math.PI} listening={false}>
            <Circle radius={rPx} stroke={colors.accent2} strokeWidth={Math.max(1, rPx * 0.09)}
                    dash={[rPx * 0.35, rPx * 0.3]} opacity={0.7}/>
            <Rect x={-len / 2} y={-wid / 2} width={len} height={wid}
                  cornerRadius={wid * 0.28} fill={colors.surface}
                  stroke={colors.accent2} strokeWidth={sw}/>
            <Line points={[len * 0.14, -wid * 0.3, len * 0.14, wid * 0.3]}
                  stroke={colors.accent2} strokeWidth={sw} lineCap="round"/>
        </Group>
    }

    // 균일 굵기 trail, 감속 구간은 색이 옅어진다 — 굵기를 속도에 비례시키면 선이
    // 두꺼워졌다 얇아졌다 해 렌더링 결함처럼 읽힌다. 속도 데이터가 없으면 단색.
    const hexLerp = (a: string, b: string, f: number): string => {
        const pa = [1, 3, 5].map((i) => parseInt(a.slice(i, i + 2), 16))
        const pb = [1, 3, 5].map((i) => parseInt(b.slice(i, i + 2), 16))
        const px = pa.map((v, i) => Math.round(v + (pb[i] - v) * f))
        return `#${px.map((v) => v.toString(16).padStart(2, "0")).join("")}`
    }
    const speedTrail = (trail: RobotTick[], color: string) => {
        const wBase = Math.max(2, cellPx * 0.2)
        const canLerp = /^#[0-9a-fA-F]{6}$/.test(color) && /^#[0-9a-fA-F]{6}$/.test(colors.muted)
        return <Shape key="trail" listening={false} sceneFunc={(ctx, shape) => {
            for (let i = 1; i < trail.length; i++) {
                const [x0, y0] = toPixel(trail[i - 1].pose[0], trail[i - 1].pose[1])
                const [x1, y1] = toPixel(trail[i].pose[0], trail[i].pose[1])
                const v = trail[i].v
                const f = vmax > 0 && v !== undefined ? Math.min(1, v / vmax) : 1
                ctx.beginPath()
                ctx.moveTo(x0, y0)
                ctx.lineTo(x1, y1)
                ctx.setAttr("lineWidth", wBase)
                ctx.setAttr("lineCap", "round")
                ctx.setAttr("strokeStyle", canLerp && f < 1 ? hexLerp(colors.muted, color, 0.25 + 0.75 * f) : color)
                ctx.setAttr("globalAlpha", 0.9)
                ctx.stroke()
            }
            ctx.setAttr("globalAlpha", 1)
            ctx.fillStrokeShape(shape)
        }}/>
    }

    // 로봇→참조 경로 최근접점 (Stanley crosstrack 표시용) — 각 선분에 사영해 최소 거리 점.
    const nearestOnPath = (pose: Pose): [number, number] | null => {
        if (!referencePath || referencePath.length < 2) return null
        let best: [number, number] | null = null
        let bestD = Infinity
        for (let i = 1; i < referencePath.length; i++) {
            const [ax, ay] = referencePath[i - 1]
            const [bx, by] = referencePath[i]
            const dx = bx - ax
            const dy = by - ay
            const L2 = dx * dx + dy * dy
            const s = L2 > 0 ? Math.max(0, Math.min(1, ((pose[0] - ax) * dx + (pose[1] - ay) * dy) / L2)) : 0
            const qx = ax + s * dx
            const qy = ay + s * dy
            const d = Math.hypot(pose[0] - qx, pose[1] - qy)
            if (d < bestD) {
                bestD = d
                best = [qx, qy]
            }
        }
        return best
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

    // threshold 이상(막힘)은 경고색, 미만(열린 valley 후보)은 accent — 로즈에서 valley가
    // "색이 끊긴 틈"으로 바로 읽히게 한다.
    const histogramRose = (h: HistogramTick) => {
        const [cx, cy] = toPixel(h.pos[0], h.pos[1])
        const n = h.bins.length
        if (n === 0) return null
        const maxBin = Math.max(...h.bins, 1e-9)
        const baseR = cellPx * 2.4
        const sector = (2 * Math.PI) / n
        const wedges = (blocked: boolean) => (ctx: Konva.Context, shape: Konva.Shape) => {
            ctx.beginPath()
            for (let k = 0; k < n; k++) {
                const isBlocked = h.threshold !== undefined && h.bins[k] >= h.threshold
                if (isBlocked !== blocked) continue
                const r = baseR * Math.min(1, h.bins[k] / maxBin)
                if (r <= 0) continue
                const a0 = k * sector
                const a1 = (k + 1) * sector
                // world CCW 각 → canvas 각(-각, y-flip) — 구간이 뒤집혀 start/end를 교환한다.
                ctx.moveTo(cx, cy)
                ctx.arc(cx, cy, r, -a1, -a0, false)
                ctx.closePath()
            }
            ctx.fillStrokeShape(shape)
        }
        return <Group key="hist" listening={false}>
            <Shape sceneFunc={wedges(false)} fill={colors.accent} opacity={0.3}
                   stroke={colors.accent} strokeWidth={1}/>
            <Shape sceneFunc={wedges(true)} fill={PATH_COLOR} opacity={0.35}
                   stroke={PATH_COLOR} strokeWidth={1}/>
        </Group>
    }

    // band 오버레이 (Elastic Bands bubble 열 / TEB pose 열) — 항목 길이로 자동 판별.
    // 길이 3 = [x, y, radius] (bubble): 중심 폴리라인 + 저알파 채움/실선 테두리 원.
    // 길이 4 = [x, y, theta, dt] (pose): 세그먼트 굵기로 ΔT(시간 간격)를 인코딩 + pose 점.
    const bandOverlay = (b: BandTick) => {
        const band = b.band
        if (band.length === 0) return null
        if (band[0].length >= 4) {
            const dts = band.map((p) => p[3] ?? 0)
            const dtMax = Math.max(...dts, 0)
            const segments = []
            for (let i = 0; i < band.length - 1; i++) {
                // entry 0의 dt=0은 placeholder(이전 세그먼트 없음)이므로 세그먼트 i는
                // 그 종점(i+1)의 dt를 굵기로 쓴다.
                const dt = dts[i + 1] ?? 0
                const sw = dtMax > 0 ? 1 + 3 * (dt / dtMax) : 1
                const [x0, y0] = toPixel(band[i][0], band[i][1])
                const [x1, y1] = toPixel(band[i + 1][0], band[i + 1][1])
                segments.push(<Line key={`bseg${i}`} points={[x0, y0, x1, y1]}
                                     stroke={colors.accent} strokeWidth={sw} opacity={0.9}
                                     lineCap="round" listening={false}/>)
            }
            const poses = band.map((p, i) => {
                const [px, py] = toPixel(p[0], p[1])
                return <Circle key={`bpose${i}`} x={px} y={py} radius={Math.max(1.5, cellPx * 0.1)}
                                fill={colors.accent} opacity={0.9} listening={false}/>
            })
            return <Group key="band" listening={false}>{segments}{poses}</Group>
        }
        const centers = band.flatMap(([x, y]) => toPixel(x, y))
        const bubbles = band.flatMap(([x, y, radius], i) => {
            const [px, py] = toPixel(x, y)
            const rPx = radius * pxPerWorld
            // Konva의 opacity는 fill/stroke에 함께 적용되므로, 저알파 채움과 불투명 테두리를
            // 따로 그린다(replay.py의 facecolor/edgecolor 분리와 같은 이유).
            return [
                <Circle key={`bfill${i}`} x={px} y={py} radius={rPx} fill={colors.accent}
                        opacity={0.15} listening={false}/>,
                <Circle key={`bedge${i}`} x={px} y={py} radius={rPx} stroke={colors.accent}
                        strokeWidth={1} opacity={0.9} listening={false}/>,
            ]
        })
        return <Group key="band" listening={false}>
            <Line points={centers} stroke={colors.accent} strokeWidth={Math.max(1.2, cellPx * 0.08)}
                  opacity={0.9} lineCap="round" lineJoin="round" listening={false}/>
            {bubbles}
        </Group>
    }

    const candidateMarkers = currentCandidates.map((c, i) => {
        const [px, py] = toPixel(c.pos[0], c.pos[1])
        return <Circle key={`cand${i}`} x={px} y={py} radius={Math.max(1.5, cellPx * 0.12)}
                       fill={c.selected ? colors.accent2 : colors.muted}
                       opacity={c.selected ? 0.95 : c.admissible ? 0.55 : 0.25}/>
    })

    // 롤아웃 폴리라인(직전 tick): 비선택은 muted 얇게(기각 후보는 더 옅게), 선택 후보는
    // accent2 굵게 — 선택 arc가 항상 위에 오도록 나중에 그린다.
    const rolloutLines = currentCandidates
        .flatMap((c) => {
            // flatMap 으로 rollout 이 실재하는 후보만 남긴다 — filter 는 optional 필드를
            // 타입으로 좁혀 주지 못해 뒤에서 non-null 단언이 필요해진다.
            const rollout = c.rollout
            return rollout && rollout.length > 1 ? [{candidate: c, rollout}] : []
        })
        .sort((a, b) => Number(a.candidate.selected) - Number(b.candidate.selected))
        .map(({candidate, rollout}, i) => (
            <Line key={`roll${i}`}
                  points={rollout.flatMap(([x, y]) => toPixel(x, y))}
                  stroke={candidate.selected ? colors.accent2 : colors.muted}
                  strokeWidth={candidate.selected
                      ? Math.max(2, cellPx * 0.18)
                      : Math.max(1, cellPx * 0.07)}
                  opacity={candidate.selected ? 0.95 : candidate.admissible ? 0.4 : 0.15}
                  lineCap="round" lineJoin="round" listening={false}/>
        ))

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
                    {/* 실행 궤적 — 속도 비례 굵기 */}
                    {robotTrail.length > 1 && speedTrail(robotTrail, visiblePath ? PATH_COLOR : colors.accent2)}
                    {/* tick 오버레이는 재생/스크럽 중에만 — 종료 화면은 궤적 + 차량만 남겨
                        마지막 tick의 히스토그램/후보가 goal 위에 박제되는 것을 막는다 */}
                    {!finished && <>
                        {/* histogram rose (VFH, 최신 tick) */}
                        {latestHistogram && histogramRose(latestHistogram)}
                        {/* 롤아웃 폴리라인(DWA, 직전 tick) — candidate 마커 아래 */}
                        {rolloutLines}
                        {/* candidate 마커(직전 tick) */}
                        {candidateMarkers}
                        {/* force 화살표(PF, 최신 tick 1건만) */}
                        {latestForce && forceArrows(latestForce)}
                        {/* band 오버레이(Elastic Bands/TEB, 최신 tick 1건만) */}
                        {showBand && latestBand && bandOverlay(latestBand)}
                        {/* lookahead 원 (추종 계열): 로봇 중심, 이번 tick 후보점을 지나는 원 */}
                        {showLookahead && currentCandidates.length > 0 && (() => {
                            const cand = currentCandidates[currentCandidates.length - 1]
                            const [rx, ry] = toPixel(currentPose[0], currentPose[1])
                            const [lx, ly] = toPixel(cand.pos[0], cand.pos[1])
                            const r = Math.hypot(lx - rx, ly - ry)
                            return <Group listening={false}>
                                <Circle x={rx} y={ry} radius={r} stroke={colors.accent}
                                        strokeWidth={1.2} dash={[5, 4]} opacity={0.55}/>
                                <Line points={[rx, ry, lx, ly]} stroke={colors.accent}
                                      strokeWidth={1.2} dash={[3, 3]} opacity={0.55}/>
                                <Circle x={lx} y={ly} radius={Math.max(2.5, cellPx * 0.18)}
                                        fill={colors.accent}/>
                            </Group>
                        })()}
                        {/* crosstrack 선분 (Stanley): 로봇→참조 경로 최근접점 */}
                        {showCrosstrack && (() => {
                            const q = nearestOnPath(currentPose)
                            if (!q) return null
                            const [rx, ry] = toPixel(currentPose[0], currentPose[1])
                            const [qx, qy] = toPixel(q[0], q[1])
                            return <Group listening={false}>
                                <Line points={[rx, ry, qx, qy]} stroke={PATH_COLOR}
                                      strokeWidth={Math.max(1.5, cellPx * 0.1)} dash={[4, 3]}/>
                                <Circle x={qx} y={qy} radius={Math.max(2, cellPx * 0.14)} fill={PATH_COLOR}/>
                            </Group>
                        })()}
                    </>}
                    {/* 보조 반경 (예: RPP d_prox) — 재생 중에만 */}
                    {!finished && auxCircleRadius !== undefined && (() => {
                        const [rx, ry] = toPixel(currentPose[0], currentPose[1])
                        return <Circle x={rx} y={ry} radius={auxCircleRadius * pxPerWorld}
                                       stroke={colors.muted} strokeWidth={1} dash={[3, 4]}
                                       opacity={0.5} listening={false}/>
                    })()}
                    {/* 로봇: footprint + 차량 (없으면 heading 화살표만) */}
                    {footprintRadius ? vehicle(currentPose, footprintRadius) : headingArrow(currentPose)}
                    {/* 시작/목표 마커 */}
                    <Circle x={startPx[0]} y={startPx[1]} radius={cellPx * 0.2} fill={colors.accent}
                            stroke={colors.bg} strokeWidth={Math.max(1, cellPx * 0.05)}
                            hitStrokeWidth={cellPx * 0.9} draggable={!!onMoveStart}
                            onDragEnd={(e) => {
                                // 드롭 좌표를 먼저 읽는다 — position() 리셋 후에 읽으면 원위치가 읽힌다.
                                const dropped = pixelToWorld(e.target.x(), e.target.y())
                                e.target.position({x: startPx[0], y: startPx[1]})
                                if (droppedFree(dropped)) onMoveStart?.(dropped)
                            }}/>
                    <Circle x={goalPx[0]} y={goalPx[1]} radius={cellPx * 0.2} fill={PATH_COLOR}
                            stroke={colors.bg} strokeWidth={Math.max(1, cellPx * 0.05)}
                            hitStrokeWidth={cellPx * 0.9} draggable={!!onMoveGoal}
                            onDragEnd={(e) => {
                                const dropped = pixelToWorld(e.target.x(), e.target.y())
                                e.target.position({x: goalPx[0], y: goalPx[1]})
                                if (droppedFree(dropped)) onMoveGoal?.(dropped)
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
                    ? `${statusLabel(status)} · ${timeline.robot.length} ticks`
                    : `${t("step", "step")} ${step}/${timeline.steps}`}
                {vmax > 0 && (
                    <span className="text-muted font-normal">
                        {" · "}{t("faded trail = braking", "색이 옅은 구간 = 감속")}
                    </span>
                )}
            </div>

            {footer}
        </div>
    )
}

export default LocalTracePlayer

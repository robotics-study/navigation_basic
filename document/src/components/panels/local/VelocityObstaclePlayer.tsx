import {ReactNode, useEffect, useMemo, useRef, useState} from "react";
import {Circle, Group, Layer, Line, Rect, Shape, Stage} from "react-konva";
import {GridMap, worldToCellUnits} from "../../../libs/grid";
import {TraceEvent} from "../../../libs/trace/types";
import {useCanvasColors} from "../../../libs/useTheme";
import {useTr} from "../../../libs/i18n";
import {PATH_COLOR} from "../../2d/GridCanvas";
import cn from "../../../libs/cn";

// VO/RVO/ORCA 데모 전용 다중 로봇 재생기. LocalTracePlayer(단일 로봇: force/histogram/
// candidate 오버레이)와 같은 재생 골격(고정 배속, 배속 버튼 없음, scrub, reset)을 따르되
// N개 몸체를 robot_moved의 agent 필드로 나눠 그리고, ego(agent 0)의 이번 tick
// velocity-obstacle 결정을 속도공간 inset에 함께 보여준다. LocalTracePlayer는 단일
// 로봇 계약(force/histogram/band 오버레이)에 맞춰 설계되어 있어 N-body 렌더와 inset을
// 얹으려면 그 자체를 다중 로봇용으로 재구성해야 했으므로, 그 파일을 건드리는 대신
// 나란한 새 컴포넌트로 둔다.
const BASE_DURATION_MS = 3000;
const TICK_MS = 30;

// CVD-safe 카테고리 팔레트(Okabe-Ito 근사) -- ego(agent 0)는 항상 accent2를 쓰고
// (다른 local 데모의 "현재 로봇 = accent2" 관례와 통일), 나머지 agent가 이 순서로
// 돈다. accent2/PATH_COLOR와 시각적으로 겹치지 않는 색만 골랐다.
const OTHER_AGENT_COLORS = ["#D55E00", "#009E73", "#CC79A7", "#E69F00", "#6366f1"];

export type Pose = [number, number, number];

export interface AgentInfo {
    start: Pose;
    goal: [number, number];
    radius: number;
}

interface RobotTick { step: number; pose: Pose; v?: number }
interface ConstraintTick {
    step: number;
    state: [number, number, number];
    constraints: number[][];
    prefV: [number, number];
    newV: [number, number];
}

interface Timeline {
    steps: number;
    robot: RobotTick[][];       // agent별
    constraints: ConstraintTick[]; // ego(agent 0)만
    success?: boolean;
    metrics?: Record<string, number>;
}

const numField = (data: Record<string, unknown> | undefined, key: string): number => {
    const v = data?.[key]
    return typeof v === "number" ? v : 0
}

function buildTimeline(events: TraceEvent[], nAgents: number): Timeline {
    const timeline: Timeline = {steps: events.length, robot: Array.from({length: nAgents}, () => []), constraints: []}
    events.forEach((ev, step) => {
        switch (ev.event) {
            case "robot_moved": {
                const s = ev.state
                const agent = ev.agent ?? 0
                if (s && s.length >= 3 && timeline.robot[agent]) {
                    const v = ev.data?.v
                    timeline.robot[agent].push({
                        step, pose: [s[0], s[1], s[2]],
                        v: typeof v === "number" ? Math.abs(v) : undefined,
                    })
                }
                break
            }
            case "velocity_obstacle": {
                const s = ev.state
                if (s && s.length >= 3 && ev.constraints) {
                    timeline.constraints.push({
                        step, state: [s[0], s[1], s[2]], constraints: ev.constraints,
                        prefV: [numField(ev.data, "pref_vx"), numField(ev.data, "pref_vy")],
                        newV: [numField(ev.data, "new_vx"), numField(ev.data, "new_vy")],
                    })
                }
                break
            }
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

// bench의 status 라벨 복원과 같은 규칙 -- metrics는 숫자 플래그만 나른다.
function deriveStatus(timeline: Timeline): SimStatus | undefined {
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

export interface VelocityObstaclePlayerProps {
    map: GridMap;
    events: TraceEvent[];
    agents: AgentInfo[];
    // ego 속도공간 inset의 축 스케일(대략 max_speed) -- 원뿔/half-plane을 그릴 만큼
    // 충분히 넓게 잡는 기준값.
    maxSpeedScale: number;
    panel?: number;
    autoPlay?: boolean;
    onReset?: () => void;
    footer?: ReactNode;
}

const agentColor = (i: number, accent2: string): string => (i === 0 ? accent2 : OTHER_AGENT_COLORS[(i - 1) % OTHER_AGENT_COLORS.length])

const VelocityObstaclePlayer = ({
                                    map, events, agents, maxSpeedScale, panel = 340,
                                    autoPlay = true, onReset, footer,
                                }: VelocityObstaclePlayerProps) => {
    const t = useTr()
    const colors = useCanvasColors()
    const timeline = useMemo(() => buildTimeline(events, agents.length), [events, agents.length])
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

    // --- 좌표 변환: grid.ts의 worldToCellUnits만 재사용 --------------------------------
    const cellPx = panel / Math.max(map.width, map.height)
    const stageW = Math.round(cellPx * map.width)
    const stageH = Math.round(cellPx * map.height)
    const toPixel = (x: number, y: number): [number, number] => {
        const [u, v] = worldToCellUnits(map, x, y)
        return [u * cellPx, v * cellPx]
    }
    const pxPerWorld = cellPx / map.resolution

    // --- 재생 시점 이하로 잘라낸 agent별 trail / 최신 ego 제약 -----------------------
    const trails = useMemo(
        () => timeline.robot.map((ticks) => ticks.filter((r) => r.step <= step)),
        [timeline, step],
    )
    const currentPoses = useMemo(
        () => trails.map((trail, i) => (trail.length > 0 ? trail[trail.length - 1].pose : agents[i].start)),
        [trails, agents],
    )
    const latestConstraint = useMemo(() => {
        let latest: ConstraintTick | undefined
        for (const c of timeline.constraints) { if (c.step <= step) latest = c; else break }
        return latest
    }, [timeline, step])
    const vmax = useMemo(
        () => trails.reduce((m, trail) => trail.reduce((mm, r) => Math.max(mm, r.v ?? 0), m), 0),
        [trails],
    )

    // --- 렌더 헬퍼: 몸체(footprint + 내접 차체) + 속도 비례 옅어지는 trail -----------
    const hexLerp = (a: string, b: string, f: number): string => {
        const pa = [1, 3, 5].map((i) => parseInt(a.slice(i, i + 2), 16))
        const pb = [1, 3, 5].map((i) => parseInt(b.slice(i, i + 2), 16))
        const px = pa.map((v, i) => Math.round(v + (pb[i] - v) * f))
        return `#${px.map((v) => v.toString(16).padStart(2, "0")).join("")}`
    }
    const speedTrail = (trail: RobotTick[], color: string) => {
        const wBase = Math.max(1.6, cellPx * 0.16)
        const canLerp = /^#[0-9a-fA-F]{6}$/.test(color) && /^#[0-9a-fA-F]{6}$/.test(colors.muted)
        return <Shape listening={false} sceneFunc={(ctx, shape) => {
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
                ctx.setAttr("globalAlpha", 0.85)
                ctx.stroke()
            }
            ctx.setAttr("globalAlpha", 1)
            ctx.fillStrokeShape(shape)
        }}/>
    }
    const vehicle = (pose: Pose, radius: number, color: string) => {
        const [px, py] = toPixel(pose[0], pose[1])
        const rPx = radius * pxPerWorld
        const len = rPx * 2 * 0.86
        const wid = len * 0.58
        const sw = Math.max(1.1, rPx * 0.14)
        return <Group x={px} y={py} rotation={-pose[2] * 180 / Math.PI} listening={false}>
            <Circle radius={rPx} stroke={color} strokeWidth={Math.max(1, rPx * 0.09)}
                    dash={[rPx * 0.35, rPx * 0.3]} opacity={0.7}/>
            <Rect x={-len / 2} y={-wid / 2} width={len} height={wid}
                  cornerRadius={wid * 0.28} fill={colors.surface} stroke={color} strokeWidth={sw}/>
            <Line points={[len * 0.14, -wid * 0.3, len * 0.14, wid * 0.3]}
                  stroke={color} strokeWidth={sw} lineCap="round"/>
        </Group>
    }
    const goalMarker = (goal: [number, number], color: string) => {
        const [gx, gy] = toPixel(goal[0], goal[1])
        return <Circle x={gx} y={gy} radius={cellPx * 0.18} stroke={color}
                       strokeWidth={Math.max(1.2, cellPx * 0.07)} dash={[cellPx * 0.16, cellPx * 0.12]}
                       fill={colors.bg} opacity={0.9}/>
    }

    // --- ego 속도공간 inset --------------------------------------------------------
    const insetSize = Math.min(160, Math.round(panel * 0.42))
    const insetScale = (insetSize / 2) / (maxSpeedScale * 1.25)
    const insetToPx = (v: [number, number]): [number, number] => [
        insetSize / 2 + v[0] * insetScale, insetSize / 2 - v[1] * insetScale,
    ]
    const insetContent = () => {
        const c = latestConstraint
        const center = insetSize / 2
        const maxSpeedPx = maxSpeedScale * insetScale
        const elems: ReactNode[] = [
            <Circle key="axis" x={center} y={center} radius={maxSpeedPx} stroke={colors.border}
                    strokeWidth={1} dash={[4, 3]} listening={false}/>,
            <Line key="ax" points={[0, center, insetSize, center]} stroke={colors.border} strokeWidth={1} opacity={0.6} listening={false}/>,
            <Line key="ay" points={[center, 0, center, insetSize]} stroke={colors.border} strokeWidth={1} opacity={0.6} listening={false}/>,
        ]
        if (c) {
            // 큰 변으로 뻗어 inset 전체를 덮는 사각형/삼각형을 그려, 화면 안에서
            // "이 경계의 어느 쪽이 금지/허용인가"만 잘려 보이게 한다(캔버스 경계 밖은
            // 자동 클리핑되므로 far point가 화면보다 훨씬 크기만 하면 된다).
            const big = maxSpeedScale * 8
            c.constraints.forEach((con, i) => {
                if (con.length >= 6) {
                    // VO/RVO truncated cone: apex + left/right 단위 방향으로 뻗은 쐐기.
                    const apex: [number, number] = [con[0], con[1]]
                    const left: [number, number] = [con[2], con[3]]
                    const right: [number, number] = [con[4], con[5]]
                    const L = maxSpeedScale * 2.4
                    const pts = [apex, [apex[0] + left[0] * L, apex[1] + left[1] * L], [apex[0] + right[0] * L, apex[1] + right[1] * L]]
                        .flatMap((p) => insetToPx(p as [number, number]))
                    elems.push(<Line key={`cone${i}`} points={pts} closed fill={PATH_COLOR}
                                     stroke={PATH_COLOR} strokeWidth={1} opacity={0.16} listening={false}/>)
                } else if (con.length >= 4) {
                    // ORCA half-plane: point/normal. infeasible 쪽(=normal 반대편)을 채운다.
                    const point: [number, number] = [con[0], con[1]]
                    const normal: [number, number] = [con[2], con[3]]
                    const dir: [number, number] = [normal[1], -normal[0]]
                    const a: [number, number] = [point[0] - dir[0] * big, point[1] - dir[1] * big]
                    const b: [number, number] = [point[0] + dir[0] * big, point[1] + dir[1] * big]
                    const a2: [number, number] = [a[0] - normal[0] * big, a[1] - normal[1] * big]
                    const b2: [number, number] = [b[0] - normal[0] * big, b[1] - normal[1] * big]
                    const pts = [a, b, b2, a2].flatMap((p) => insetToPx(p))
                    elems.push(<Line key={`hp${i}`} points={pts} closed fill={PATH_COLOR} opacity={0.14} listening={false}/>)
                    const bLine = [insetToPx([point[0] - dir[0] * maxSpeedScale * 1.3, point[1] - dir[1] * maxSpeedScale * 1.3]),
                        insetToPx([point[0] + dir[0] * maxSpeedScale * 1.3, point[1] + dir[1] * maxSpeedScale * 1.3])].flat()
                    elems.push(<Line key={`hpl${i}`} points={bLine} stroke={PATH_COLOR} strokeWidth={1.2} opacity={0.55} listening={false}/>)
                }
            })
            const [px0, py0] = insetToPx([0, 0])
            const [prx, pry] = insetToPx(c.prefV)
            const [nrx, nry] = insetToPx(c.newV)
            elems.push(<Line key="pref" points={[px0, py0, prx, pry]} stroke={colors.muted} strokeWidth={1.4}
                             dash={[4, 3]} opacity={0.8} listening={false}/>)
            elems.push(<Circle key="prefDot" x={prx} y={pry} radius={2.4} fill={colors.muted} listening={false}/>)
            elems.push(<Line key="new" points={[px0, py0, nrx, nry]} stroke={colors.accent2} strokeWidth={2}
                             opacity={0.95} listening={false}/>)
            elems.push(<Circle key="newDot" x={nrx} y={nry} radius={3} fill={colors.accent2} listening={false}/>)
        }
        return elems
    }

    return (
        <div className="flex flex-col gap-2 items-center">
            <div className="flex items-start gap-3 flex-wrap justify-center">
                <Stage width={stageW} height={stageH}
                       className="bg-surface border border-border rounded-lg overflow-hidden w-fit">
                    <Layer>
                        {map.occupied.map((occ, i) => occ && (
                            <Rect key={`w${i}`} x={(i % map.width) * cellPx} y={Math.floor(i / map.width) * cellPx}
                                  width={cellPx} height={cellPx} fill={colors.text} opacity={0.78}/>
                        ))}
                        {agents.map((a, i) => (
                            <Group key={`goal${i}`}>{goalMarker(a.goal, agentColor(i, colors.accent2))}</Group>
                        ))}
                        {trails.map((trail, i) => trail.length > 1 && (
                            <Group key={`trail${i}`}>{speedTrail(trail, agentColor(i, colors.accent2))}</Group>
                        ))}
                        {agents.map((a, i) => (
                            <Group key={`veh${i}`}>{vehicle(currentPoses[i], a.radius, agentColor(i, colors.accent2))}</Group>
                        ))}
                    </Layer>
                </Stage>

                <div className="flex flex-col items-center gap-1">
                    <Stage width={insetSize} height={insetSize}
                           className="bg-surface border border-border rounded-lg overflow-hidden">
                        <Layer>{insetContent()}</Layer>
                    </Stage>
                    <span className="text-[10px] text-muted text-center max-w-[9rem] leading-tight">
                        {t("ego velocity space (dashed = preferred, solid = chosen)",
                            "ego 속도공간 (파선=선호, 실선=선택)")}
                    </span>
                </div>
            </div>

            <div className="flex items-center gap-1.5 text-xs text-muted w-full" style={{maxWidth: stageW + insetSize + 12}}>
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
                 style={finished && status ? {color: status === "reached" ? colors.accent2
                         : status === "collision" ? PATH_COLOR
                             : status === "stalled" ? colors.accent : colors.muted} : undefined}>
                {finished && status
                    ? `${{
                        reached: t("reached", "도달"), collision: t("collision", "충돌"),
                        stalled: t("stalled", "정체"), timeout: t("timeout", "시간 초과"),
                    }[status]} · ${timeline.steps} ticks`
                    : `${t("step", "step")} ${step}/${timeline.steps}`}
            </div>

            {footer}
        </div>
    )
}

export default VelocityObstaclePlayer

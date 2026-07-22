import {useMemo, useState} from "react";
import {Circle, Group, Layer, Line, Rect, Shape, Stage, Text} from "react-konva";
import Konva from "konva";
import CanvasFigure, {modalScale} from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {PATH_COLOR} from "../../2d/GridCanvas";
import {useTr} from "../../../libs/i18n";

// DWA의 핵심 그림 (Fox, Burgard & Thrun 1997). 왼쪽은 실제 장면(로봇·장애물·goal·
// 명령이 그리는 원호), 오른쪽은 같은 장면의 (v, ω) 속도 공간이다. 오른쪽의 충돌 위험
// 영역은 지어낸 모양이 아니라 논문의 admissible velocity 조건
//   v ≤ sqrt(2 · dist(v, ω) · v̇_b)
// 을 원호를 따라가는 충돌 거리 dist로 그대로 계산해 칠한다. 선택 명령도 논문의
// 목적함수(heading + clearance + velocity 가중합)를 window ∩ admissible 위에서 최대화한다.
const V_MAX = 1.0;                 // m/s
const OM_MAX = 1.4;                // rad/s
const A_V = 0.35;                  // 가속 한계 × dt (창 반높이, m/s)
const A_OM = 0.6;                  // 각가속 한계 × dt (창 반너비, rad/s)
const A_BRAKE = 0.22;              // 제동 감속 v̇_b (m/s²) — 경계가 v 축 범위 안에 들어오는 값
const ROBOT_R = 0.12;              // m
const OBST: [number, number] = [0.22, 1.0];   // 로봇 기준 (x: 좌+, y: 전방) — 전방 약간 오른쪽
const OBST_R = 0.28;
const GOAL: [number, number] = [-0.35, 2.3];   // 장애물 왼쪽 뒤편
const S_MAX = 4;                   // 충돌 거리 탐색 상한 (m)

// 곡률 κ 원호를 따라 장애물(로봇 반경으로 팽창)까지의 거리. 로봇은 원점, 전방 +y.
function arcDistToObstacle(kappa: number): number {
    const rHit = OBST_R + ROBOT_R
    let x = 0
    let y = 0
    let th = 0                     // 전방(+y) 기준 편각. 왼쪽 회전이 +.
    const ds = 0.02
    for (let s = 0; s < S_MAX; s += ds) {
        x += -Math.sin(th) * ds
        y += Math.cos(th) * ds
        th += kappa * ds
        if (Math.hypot(x - OBST[0], y - OBST[1]) <= rHit) return s
    }
    return Infinity
}

const admissibleV = (v: number, om: number): boolean => {
    if (v <= 0) return true
    const dist = arcDistToObstacle(om / v)
    return v <= Math.sqrt(2 * A_BRAKE * dist)
}

// 원호 폴리라인 (좌표는 로봇 기준 m). length만큼 전진했을 때의 자취.
function arcPoints(kappa: number, length: number): [number, number][] {
    const pts: [number, number][] = [[0, 0]]
    let x = 0
    let y = 0
    let th = 0
    const ds = 0.05
    for (let s = 0; s < length; s += ds) {
        x += -Math.sin(th) * ds
        y += Math.cos(th) * ds
        th += kappa * ds
        pts.push([x, y])
    }
    return pts
}

// --- 오른쪽 (v, ω) 평면 레이아웃 -----------------------------------------------
const PAD_L = 40;
const PAD_B = 30;
const PAD_T = 12;
const PLOT = 220;
const VW = PAD_L + PLOT + 12;
const VH = PAD_T + PLOT + PAD_B;
const vx = (om: number) => PAD_L + ((om + OM_MAX) / (2 * OM_MAX)) * PLOT;
const vy = (v: number) => PAD_T + (1 - v / V_MAX) * PLOT;
const clamp = (x: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, x));

// --- 왼쪽 장면 레이아웃 (m → px) ------------------------------------------------
const SW = 210;
const SH = VH;
const SCALE = 78;                  // px per m
const RX = SW / 2 - 12;            // 로봇 화면 x (약간 왼쪽)
const RY = SH - 46;                // 로봇 화면 y (아래쪽)
const sx = (mx: number) => RX + mx * SCALE;
const sy = (my: number) => RY - my * SCALE;

const Scene = ({scale = 1}: {scale?: number}) => {
    const colors = useCanvasColors()
    const t = useTr()
    const [vc, setVc] = useState(0.55)
    const [oc, setOc] = useState(0.0)

    const wLo = clamp(vc - A_V, 0.02, V_MAX)
    const wHi = clamp(vc + A_V, 0.02, V_MAX)
    const wLeft = clamp(oc - A_OM, -OM_MAX, OM_MAX)
    const wRight = clamp(oc + A_OM, -OM_MAX, OM_MAX)

    // admissible 격자 (오른쪽 평면 채색용) — 60×40 셀.
    const NC = 60
    const NR = 40
    const grid = useMemo(() => {
        const g: boolean[][] = []
        for (let ir = 0; ir < NR; ir++) {
            const row: boolean[] = []
            const v = V_MAX * (1 - (ir + 0.5) / NR)
            for (let ic = 0; ic < NC; ic++) {
                const om = -OM_MAX + ((ic + 0.5) / NC) * 2 * OM_MAX
                row.push(admissibleV(v, om))
            }
            g.push(row)
        }
        return g
    }, [])

    // 논문 목적함수: heading(예측 pose에서 goal을 향한 정렬) + clearance + velocity.
    const best = useMemo(() => {
        let bestCmd: {v: number, om: number, J: number} | null = null
        const T = 1.4
        const steps = 16
        for (let iv = 0; iv <= steps; iv++) {
            const v = wLo + (iv / steps) * (wHi - wLo)
            for (let io = 0; io <= steps; io++) {
                const om = wLeft + (io / steps) * (wRight - wLeft)
                if (!admissibleV(v, om)) continue
                const kappa = v > 0 ? om / v : 0
                const s = v * T
                const thEnd = kappa * s
                // 원호 끝점 (근사 적분과 동일한 기하)
                const pts = arcPoints(kappa, Math.max(s, 0.01))
                const [ex, ey] = pts[pts.length - 1]
                const angToGoal = Math.atan2(-(GOAL[0] - ex), GOAL[1] - ey)
                const heading = 1 - Math.abs(angToGoal - thEnd) / Math.PI
                const dist = arcDistToObstacle(kappa)
                const clear = Math.min(dist, 2) / 2
                const J = 2.0 * heading + 0.3 * clear + 0.6 * (v / V_MAX)
                if (!bestCmd || J > bestCmd.J) bestCmd = {v, om, J}
            }
        }
        return bestCmd
    }, [wLo, wHi, wLeft, wRight])

    // 왼쪽 장면에 그릴 후보 원호들 (window 모서리 + 중앙 열) — admissible 여부로 색 구분.
    const sampleArcs = useMemo(() => {
        const arcs: {pts: [number, number][], ok: boolean}[] = []
        const T = 1.4
        for (let io = 0; io <= 6; io++) {
            const om = wLeft + (io / 6) * (wRight - wLeft)
            const v = wHi
            const ok = admissibleV(v, om)
            const kappa = v > 0 ? om / v : 0
            const dist = arcDistToObstacle(kappa)
            const len = ok ? v * T : Math.min(dist, v * T)
            arcs.push({pts: arcPoints(kappa, Math.max(len, 0.05)), ok})
        }
        return arcs
    }, [wLeft, wRight, wHi])

    return (
        <div className="flex flex-col items-center gap-2">
            <div className="flex gap-2 flex-wrap justify-center">
                {/* 왼쪽: 실제 장면 */}
                <Stage width={SW * scale} height={SH * scale}
                       className="bg-surface border border-border rounded-lg overflow-hidden">
                    <Layer scaleX={scale} scaleY={scale}>
                        {/* goal */}
                        <Circle x={sx(GOAL[0])} y={sy(GOAL[1])} radius={6} fill={PATH_COLOR}
                                stroke={colors.bg} strokeWidth={1.5}/>
                        <Text x={sx(GOAL[0]) + 8} y={sy(GOAL[1]) - 6} text={t("goal", "목표")}
                              fontSize={11} fill={PATH_COLOR} fontStyle="bold"/>
                        {/* 장애물 */}
                        <Circle x={sx(OBST[0])} y={sy(OBST[1])} radius={OBST_R * SCALE}
                                fill={colors.text} opacity={0.75}/>
                        {/* 후보 원호: admissible = 회색, 위험(제때 못 멈춤) = 경고색 */}
                        {sampleArcs.map((a, i) => (
                            <Line key={`arc${i}`}
                                  points={a.pts.flatMap(([mx, my]) => [sx(mx), sy(my)])}
                                  stroke={a.ok ? colors.muted : PATH_COLOR}
                                  strokeWidth={a.ok ? 1.3 : 1.6} opacity={a.ok ? 0.5 : 0.7}
                                  dash={a.ok ? undefined : [4, 3]}/>
                        ))}
                        {/* 위험 원호가 장애물에 닿는 지점 — "제때 못 멈춰 들이받는" 명령임을 명시 */}
                        {sampleArcs.filter((a) => !a.ok).map((a, i) => {
                            const [mx, my] = a.pts[a.pts.length - 1]
                            return <Circle key={`hit${i}`} x={sx(mx)} y={sy(my)} radius={3}
                                           fill={PATH_COLOR} stroke={colors.bg} strokeWidth={1}/>
                        })}
                        {/* 선택 명령의 원호 */}
                        {best && (
                            <Line points={arcPoints(best.v > 0 ? best.om / best.v : 0, best.v * 1.4)
                                .flatMap(([mx, my]) => [sx(mx), sy(my)])}
                                  stroke={colors.accent2} strokeWidth={3} lineCap="round"/>
                        )}
                        {/* 로봇 (전방 = 위) */}
                        <Group x={RX} y={RY}>
                            <Circle radius={ROBOT_R * SCALE} stroke={colors.accent2}
                                    strokeWidth={1.2} dash={[3, 3]} opacity={0.7}/>
                            <Rect x={-7} y={-10} width={14} height={20} cornerRadius={4}
                                  fill={colors.surface} stroke={colors.accent2} strokeWidth={2}/>
                            <Line points={[-4, -4, 4, -4]} stroke={colors.accent2} strokeWidth={2}
                                  lineCap="round"/>
                        </Group>
                    </Layer>
                </Stage>
                {/* 오른쪽: (v, ω) 평면 */}
                <Stage width={VW * scale} height={VH * scale}
                       className="bg-surface border border-border rounded-lg overflow-hidden">
                    <Layer scaleX={scale} scaleY={scale}>
                        {/* admissible 격자: 위험 셀만 칠한다 */}
                        <Shape listening={false} sceneFunc={(ctx: Konva.Context, shape: Konva.Shape) => {
                            const cw = PLOT / NC
                            const ch = PLOT / NR
                            ctx.beginPath()
                            for (let ir = 0; ir < NR; ir++) {
                                for (let ic = 0; ic < NC; ic++) {
                                    if (grid[ir][ic]) continue
                                    ctx.rect(PAD_L + ic * cw, PAD_T + ir * ch, cw + 0.5, ch + 0.5)
                                }
                            }
                            ctx.fillStrokeShape(shape)
                        }} fill={PATH_COLOR} opacity={0.18}/>
                        <Rect x={PAD_L} y={PAD_T} width={PLOT} height={PLOT}
                              stroke={colors.border} strokeWidth={1}/>
                        {/* dynamic window */}
                        <Rect x={vx(wLeft)} y={vy(wHi)}
                              width={vx(wRight) - vx(wLeft)} height={vy(wLo) - vy(wHi)}
                              fill={colors.accent} opacity={0.14}
                              stroke={colors.accent} strokeWidth={2}/>
                        {/* 현재 속도 */}
                        <Circle x={vx(oc)} y={vy(vc)} radius={4} fill={colors.text}/>
                        {/* 선택된 명령 */}
                        {best && (
                            <Circle x={vx(best.om)} y={vy(best.v)} radius={6}
                                    fill={colors.accent2} stroke={colors.bg} strokeWidth={1.5}/>
                        )}
                        {/* 축 */}
                        <Line points={[PAD_L, PAD_T, PAD_L, PAD_T + PLOT]}
                              stroke={colors.muted} strokeWidth={1}/>
                        <Line points={[PAD_L, PAD_T + PLOT, PAD_L + PLOT, PAD_T + PLOT]}
                              stroke={colors.muted} strokeWidth={1}/>
                        <Line points={[vx(0), PAD_T, vx(0), PAD_T + PLOT]}
                              stroke={colors.border} strokeWidth={0.5} dash={[4, 4]}/>
                        <Text x={0} y={PAD_T - 2} width={PAD_L - 6} align="right"
                              text="v" fontSize={13} fill={colors.muted} fontStyle="italic"/>
                        <Text x={PAD_L + PLOT - 14} y={PAD_T + PLOT + 8}
                              text="ω" fontSize={13} fill={colors.muted} fontStyle="italic"/>
                    </Layer>
                </Stage>
            </div>
            <div className="flex flex-col gap-1 text-xs text-muted w-full" style={{maxWidth: (SW + VW + 8) * scale}}>
                <label className="flex items-center gap-2">
                    <span className="w-14 shrink-0">v = {vc.toFixed(2)}</span>
                    <input type="range" min={0.02} max={V_MAX} step={0.01} value={vc}
                           onChange={(e) => setVc(parseFloat(e.target.value))}
                           className="flex-1 accent-[var(--accent)]"
                           aria-label={t("current translational velocity", "현재 병진 속도")}/>
                </label>
                <label className="flex items-center gap-2">
                    <span className="w-14 shrink-0">ω = {oc.toFixed(2)}</span>
                    <input type="range" min={-OM_MAX} max={OM_MAX} step={0.01} value={oc}
                           onChange={(e) => setOc(parseFloat(e.target.value))}
                           className="flex-1 accent-[var(--accent)]"
                           aria-label={t("current angular velocity", "현재 각속도")}/>
                </label>
            </div>
            <div className="text-xs text-muted text-center">
                <span style={{color: "var(--accent)"}} className="font-semibold">dynamic window</span>
                {" · "}
                <span style={{color: "var(--accent-2)"}} className="font-semibold">
                    {t("chosen command and its arc", "선택된 명령과 그 원호")}
                </span>
                {" · "}
                <span style={{color: PATH_COLOR}} className="font-semibold">
                    {t("cannot stop before the obstacle", "장애물 앞에서 못 멈추는 명령")}
                </span>
            </div>
        </div>
    )
}

const LocalVelocityWindow = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Left: the robot, an obstacle on the way, and the circular arcs its commands trace. Right: the same scene in velocity space — a command (v, ω) is inadmissible (red) when v exceeds √(2·dist·brake), the speed that can still stop along its arc before the obstacle. DWA scores only the dynamic window (the box of velocities reachable within one cycle's acceleration limits) intersected with the admissible region, and picks the command that best balances heading to the goal, clearance, and speed.",
            "왼쪽은 로봇과 길목의 장애물, 그리고 각 명령이 그리는 원호다. 오른쪽은 같은 장면의 속도 공간이다. 명령 (v, ω)는 자신의 원호를 따라 장애물 앞에서 멈출 수 있는 한계 √(2·dist·brake)를 넘으면 위험(빨강)으로 배제된다. DWA는 한 주기의 가감속으로 도달 가능한 상자(dynamic window)와 이 admissible 영역의 교집합만 채점해, goal 방향·여유 거리·속도를 저울질한 최선의 명령을 고른다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(SW + VW + 8, VH)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default LocalVelocityWindow

import {useState} from "react";
import {Layer, Line, Rect, Stage, Text, Circle} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {PATH_COLOR} from "../../2d/GridCanvas";
import {useTr} from "../../../libs/i18n";

// DWA의 핵심 그림: 로봇이 고를 수 있는 명령은 (v, ω) 평면 전체가 아니라
// 현재 속도에서 한 제어 주기의 가감속으로 도달 가능한 작은 사각형(dynamic window)뿐이다.
// 그 창을 장애물이 허용하는 영역과 교차한 부분에서 최선을 고른다 (Fox et al., 1997).
const PAD_L = 44;
const PAD_B = 34;
const PAD_T = 14;
const PAD_R = 14;
const PLOT = 240;
const W = PAD_L + PLOT + PAD_R;
const H = PAD_T + PLOT + PAD_B;

const V_MAX = 1.0;                 // m/s
const OM_MAX = 1.4;                // rad/s
const A_V = 0.35;                  // 가속 한계 × dt (창 반높이, m/s)
const A_OM = 0.6;                  // 각가속 한계 × dt (창 반너비, rad/s)

// 값 → 픽셀. v는 아래가 0, 위가 V_MAX. ω는 가운데가 0.
const vx = (om: number) => PAD_L + ((om + OM_MAX) / (2 * OM_MAX)) * PLOT;
const vy = (v: number) => PAD_T + (1 - v / V_MAX) * PLOT;
const clamp = (x: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, x));

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [vc, setVc] = useState(0.55)      // 현재 병진 속도
    const [oc, setOc] = useState(-0.2)      // 현재 각속도

    // dynamic window: 현재 속도를 중심으로 한 주기의 가감속 사각형, 허용 공간으로 clip.
    const wLo = clamp(vc - A_V, 0, V_MAX)
    const wHi = clamp(vc + A_V, 0, V_MAX)
    const wLeft = clamp(oc - A_OM, -OM_MAX, OM_MAX)
    const wRight = clamp(oc + A_OM, -OM_MAX, OM_MAX)

    // 정면 장애물이 만드는 금지 영역: 빠른 v + 작은 |ω|는 제때 못 멈춰 충돌한다.
    // v ≤ V_MAX·(|ω|/OM_MAX)^0.5 위쪽이 금지 (직진일수록 낮은 속도만 안전).
    const safeV = (om: number) => V_MAX * Math.sqrt(Math.abs(om) / OM_MAX)
    const forbidPts: number[] = []
    const stepsF = 40
    for (let i = 0; i <= stepsF; i++) {
        const om = -OM_MAX + (i / stepsF) * 2 * OM_MAX
        forbidPts.push(vx(om), vy(Math.min(V_MAX, safeV(om))))
    }
    // 위 경계 두 꼭짓점으로 닫아 상단 금지 폴리곤을 만든다.
    forbidPts.push(vx(OM_MAX), vy(V_MAX), vx(-OM_MAX), vy(V_MAX))

    // 창 안에서 안전하고 병진 속도가 가장 큰 명령을 최선으로 고른다 (진행 선호).
    let best: {v: number, om: number} | null = null
    const stepsW = 12
    for (let iv = 0; iv <= stepsW; iv++) {
        const v = wLo + (iv / stepsW) * (wHi - wLo)
        for (let io = 0; io <= stepsW; io++) {
            const om = wLeft + (io / stepsW) * (wRight - wLeft)
            if (v > safeV(om)) continue
            if (!best || v - Math.abs(om) * 0.15 > best.v - Math.abs(best.om) * 0.15) best = {v, om}
        }
    }

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {/* 허용 속도 공간 (전체 사각형) */}
                    <Rect x={PAD_L} y={PAD_T} width={PLOT} height={PLOT}
                          stroke={colors.border} strokeWidth={1}/>
                    {/* 장애물 금지 영역 (제때 못 멈추는 고속) */}
                    <Line points={forbidPts} closed fill={PATH_COLOR} opacity={0.16}/>
                    <Line points={forbidPts.slice(0, (stepsF + 1) * 2)}
                          stroke={PATH_COLOR} strokeWidth={1.5} opacity={0.6}/>
                    {/* dynamic window */}
                    <Rect x={vx(wLeft)} y={vy(wHi)}
                          width={vx(wRight) - vx(wLeft)} height={vy(wLo) - vy(wHi)}
                          fill={colors.accent} opacity={0.16}
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
                    <Text x={PAD_L} y={PAD_T + PLOT + 8} text="−" fontSize={12} fill={colors.muted}/>
                </Layer>
            </Stage>
            <div className="flex flex-col gap-1 text-xs text-muted w-full" style={{maxWidth: W}}>
                <label className="flex items-center gap-2">
                    <span className="w-14 shrink-0">v = {vc.toFixed(2)}</span>
                    <input type="range" min={0} max={V_MAX} step={0.01} value={vc}
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
                    {t("chosen command", "선택된 명령")}
                </span>
                {" · "}
                <span style={{color: PATH_COLOR}} className="font-semibold">
                    {t("unsafe", "충돌 위험")}
                </span>
            </div>
        </div>
    )
}

const LocalVelocityWindow = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "DWA searches only the dynamic window — the (v, ω) box reachable within one cycle's acceleration limits — intersected with velocities that can still stop before an obstacle. Drag the sliders to move the current velocity and watch the window follow.",
            "DWA는 (v, ω) 평면 전체가 아니라 dynamic window, 곧 한 주기의 가감속으로 도달 가능한 상자만, 그것도 장애물 앞에서 멈출 수 있는 속도와 교차한 부분만 탐색한다. 슬라이더로 현재 속도를 옮기면 창이 따라 움직인다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default LocalVelocityWindow

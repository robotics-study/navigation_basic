import {useState} from "react";
import {Arrow, Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";

// Khatib(1986)의 힘 합산을 그대로 그린다: 인력은 목표까지 거리에 선형 비례해 늘 같은
// 방향을 향하고, 반발력은 FIRAS로 장애물에 가까워질수록 발산한다. 슬라이더로 장애물
// 거리 d를 줄이면 반발 화살표가 자라 합력(굵은 화살표)이 인력과 반대로 꺾이는 것이
// 보인다 — replay.py의 색 관례(인력 #2563eb / 반발 #dc2626)를 그대로 쓴다.
const W = 340;
const H = 260;
const ROBOT: [number, number] = [110, H - 60];
const GOAL: [number, number] = [300, 60];
const K_ATT = 1;
const K_REP = 6;
const RHO0 = 130;      // px, 반발 영향 반경
const D_MIN = 22;      // px, 접촉 클램프

const ForceVectors = () => {
    const t = useTr()
    const colors = useCanvasColors()
    const [d, setD] = useState(70)   // 로봇-장애물 거리 (px)

    // 장애물은 로봇 아래쪽(-y, 화면 기준 +y)에 둔다 — 반발이 인력과 부분적으로 반대 방향.
    const obstacle: [number, number] = [ROBOT[0], ROBOT[1] + d]

    const gx = GOAL[0] - ROBOT[0]
    const gy = GOAL[1] - ROBOT[1]
    const fAttX = K_ATT * gx
    const fAttY = K_ATT * gy

    const dClamped = Math.max(d, D_MIN)
    const ox = ROBOT[0] - obstacle[0]
    const oy = ROBOT[1] - obstacle[1]
    const inRange = dClamped < RHO0
    const mag = inRange ? K_REP * (1 / dClamped - 1 / RHO0) * (1 / (dClamped * dClamped)) : 0
    const fRepX = inRange ? (mag * ox) / dClamped * 4000 : 0
    const fRepY = inRange ? (mag * oy) / dClamped * 4000 : 0

    // 표시 스케일: 인력은 길이 고정 비례, 반발은 위 식의 크기를 그대로 쓰되 화면 밖으로
    // 나가지 않도록 클램프한다.
    const scale = 0.6
    const clamp = (vx: number, vy: number, maxLen: number): [number, number] => {
        const len = Math.hypot(vx, vy)
        if (len <= maxLen || len === 0) return [vx, vy]
        return [(vx / len) * maxLen, (vy / len) * maxLen]
    }
    const [attX, attY] = clamp(fAttX * scale * 0.35, fAttY * scale * 0.35, 90)
    const [repX, repY] = clamp(fRepX * scale, fRepY * scale, 110)
    const resX = attX + repX
    const resY = attY + repY

    const arrow = (dx: number, dy: number, color: string, width: number, key: string) => (
        Math.hypot(dx, dy) < 1 ? null : (
            <Arrow key={key} points={[ROBOT[0], ROBOT[1], ROBOT[0] + dx, ROBOT[1] + dy]}
                   pointerLength={9} pointerWidth={8} stroke={color} fill={color} strokeWidth={width}/>
        )
    )

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {inRange && (
                        <Circle x={obstacle[0]} y={obstacle[1]} radius={RHO0}
                                stroke={colors.muted} strokeWidth={1} dash={[4, 4]} opacity={0.5}/>
                    )}
                    <Line points={[ROBOT[0], ROBOT[1], GOAL[0], GOAL[1]]}
                          stroke={colors.muted} strokeWidth={1.5} dash={[5, 4]} opacity={0.6}/>
                    {arrow(attX, attY, "#2563eb", 2.2, "att")}
                    {arrow(repX, repY, "#dc2626", 2.2, "rep")}
                    {arrow(resX, resY, colors.text, 3.2, "res")}
                    <Rect x={obstacle[0] - 12} y={obstacle[1] - 12} width={24} height={24}
                          cornerRadius={3} fill={colors.text} opacity={0.75}/>
                    <Circle x={ROBOT[0]} y={ROBOT[1]} radius={8} fill={colors.accent2}/>
                    <Circle x={GOAL[0]} y={GOAL[1]} radius={7} fill={PATH_COLOR}
                            stroke={colors.bg} strokeWidth={1.5}/>
                    <Text x={GOAL[0] + 8} y={GOAL[1] - 6} text={t("goal", "목표")}
                          fontSize={11} fill={PATH_COLOR} fontStyle="bold"/>
                    <Text x={12} y={12} text="F_att" fontSize={11} fill="#2563eb" fontStyle="bold"/>
                    <Text x={12} y={28} text="F_rep" fontSize={11} fill="#dc2626" fontStyle="bold"/>
                    <Text x={12} y={44} text="F = F_att + F_rep" fontSize={11} fill={colors.text} fontStyle="bold"/>
                </Layer>
            </Stage>
            <div className="flex items-center gap-2 text-xs text-muted w-full" style={{maxWidth: W}}>
                <span className="w-28 shrink-0">{t("obstacle distance", "장애물 거리")} d</span>
                <input type="range" min={D_MIN} max={RHO0 + 20} step={1} value={d}
                       onChange={(e) => setD(parseInt(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("obstacle distance", "장애물 거리")}/>
            </div>
            <div className="text-xs text-muted text-center">
                {t(
                    "shrink d and the repulsive arrow grows without bound as it nears the contact clamp, bending the resultant away from the goal",
                    "d를 줄이면 반발 화살표가 접촉 클램프에 가까워질수록 한없이 자라, 합력이 목표 방향에서 꺾여 나간다",
                )}
            </div>
        </div>
    )
}

const ForceVectorsFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Attractive pull toward the goal (blue) plus FIRAS repulsion from the nearest obstacle (red) sum to the resultant (black) that the robot steers toward — shrink the obstacle distance to watch repulsion take over",
            "목표로의 인력(파랑)과 가장 가까운 장애물의 FIRAS 반발(빨강)을 더한 합력(검정) 쪽으로 로봇이 조향한다. 장애물 거리를 줄여 반발이 우세해지는 것을 보라",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<ForceVectors/>}
    >
        <ForceVectors/>
    </CanvasFigure>
}

export default ForceVectorsFigure

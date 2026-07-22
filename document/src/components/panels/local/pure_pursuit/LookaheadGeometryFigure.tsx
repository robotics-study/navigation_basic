import {Arrow, Circle, Layer, Line, Shape, Stage, Text} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";

// Coulter (1992) 접선-현 각(tangent-chord angle) 구성을 고정 예시(α=35°)로 그린 정적
// figure -- The Lookahead Circle 절의 유도(현·호 기하 → 원 유일성 → 곡률)를 눈으로
// 따라가게 한다. sandbox와 달리 라이브 데이터가 아니라 기하 관계 자체를 보여주는
// 고정 다이어그램이라 world 좌표 변환 없이 canvas 픽셀 좌표로 직접 그린다.
const PANEL = 320;
const ALPHA_DEG = 35;
const LD_PX = 150;

// a1에서 a2까지 |π| 이하의 최단 방향으로 스윕하는 원호를 그린다 (ctx.arc 표준 각도:
// 0=+x, 시계방향 증가). 두 원호(α 표시 호, 곡률 호) 모두 이 한 헬퍼를 공유한다.
const drawMinorArc = (
    ctx: {arc: (x: number, y: number, r: number, a1: number, a2: number, ccw: boolean) => void},
    cx: number, cy: number, r: number, a1: number, a2: number,
) => {
    const diff = Math.atan2(Math.sin(a2 - a1), Math.cos(a2 - a1))
    ctx.arc(cx, cy, r, a1, a1 + diff, diff < 0)
}

const Scene = () => {
    const colors = useCanvasColors()

    const alpha = (ALPHA_DEG * Math.PI) / 180
    const robot: [number, number] = [70, 190]
    const headingAngle = 0                       // 화면 +x 방향을 향한다고 둔 기준 heading
    const targetAngle = -alpha                    // 왼쪽으로 도는 예시라 화면 위쪽(-y)
    const target: [number, number] = [
        robot[0] + LD_PX * Math.cos(targetAngle),
        robot[1] + LD_PX * Math.sin(targetAngle),
    ]
    // R = L_d / (2 sin α) -- 아래 The Lookahead Circle 유도와 같은 식. heading에
    // 접하는 원의 중심은 robot에서 heading의 수직 방향(회전 쪽)으로 R만큼 떨어진다.
    const radius = LD_PX / (2 * Math.sin(alpha))
    const center: [number, number] = [robot[0], robot[1] - radius]
    const angleToRobot = Math.atan2(robot[1] - center[1], robot[0] - center[0])
    const angleToTarget = Math.atan2(target[1] - center[1], target[0] - center[0])

    // 참조 경로: target을 지나는, chord/곡률호와는 다른 방향의 완만한 곡선.
    const pathPts = [
        target[0] - 170, target[1] + 46,
        target[0] - 90, target[1] + 14,
        target[0], target[1],
        target[0] + 80, target[1] - 26,
        target[0] + 160, target[1] - 46,
    ]

    return (
        <Stage width={PANEL} height={PANEL}
               className="bg-surface border border-border rounded-lg overflow-hidden">
            <Layer>
                {/* 참조 경로 (파선) */}
                <Line points={pathPts} stroke={colors.muted} strokeWidth={2} dash={[7, 5]}
                      lineCap="round" lineJoin="round" tension={0.35}/>
                {/* lookahead 원 (로봇 중심, 반지름 L_d) */}
                <Circle x={robot[0]} y={robot[1]} radius={LD_PX} stroke={colors.accent}
                        strokeWidth={1.5} dash={[4, 4]} opacity={0.55}/>
                {/* 곡률 원호 (로봇→target, heading에 접하는 원) */}
                <Shape listening={false} sceneFunc={(ctx, shape) => {
                    ctx.beginPath()
                    drawMinorArc(ctx, center[0], center[1], radius, angleToRobot, angleToTarget)
                    ctx.fillStrokeShape(shape)
                }} stroke={colors.accent2} strokeWidth={2.5}/>
                {/* 곡률 중심 반지름 보조선 */}
                <Line points={[center[0], center[1], robot[0], robot[1]]}
                      stroke={colors.muted} strokeWidth={1} dash={[3, 3]} opacity={0.7}/>
                <Line points={[center[0], center[1], target[0], target[1]]}
                      stroke={colors.muted} strokeWidth={1} dash={[3, 3]} opacity={0.7}/>
                <Circle x={center[0]} y={center[1]} radius={2.5} fill={colors.muted}/>
                {/* 현 L_d (로봇→target) */}
                <Line points={[robot[0], robot[1], target[0], target[1]]}
                      stroke={colors.text} strokeWidth={1.5} opacity={0.8}/>
                {/* α 표시 호 (heading ↔ chord) */}
                <Shape listening={false} sceneFunc={(ctx, shape) => {
                    ctx.beginPath()
                    drawMinorArc(ctx, robot[0], robot[1], 26, headingAngle, targetAngle)
                    ctx.fillStrokeShape(shape)
                }} stroke={colors.accent} strokeWidth={1.5}/>
                {/* heading 화살표 */}
                <Arrow points={[robot[0], robot[1], robot[0] + 55, robot[1]]}
                       pointerLength={9} pointerWidth={8}
                       stroke={colors.accent2} fill={colors.accent2} strokeWidth={2.5}/>
                {/* target / robot 점 */}
                <Circle x={target[0]} y={target[1]} radius={4.5} fill={colors.accent2}/>
                <Circle x={robot[0]} y={robot[1]} radius={5.5} fill={colors.accent2}
                        stroke={colors.bg} strokeWidth={1.5}/>
                {/* label */}
                <Text x={robot[0] + 30} y={robot[1] - 24} text="α" fontSize={15}
                      fontStyle="bold" fill={colors.accent}/>
                <Text x={(robot[0] + target[0]) / 2 + 6} y={(robot[1] + target[1]) / 2 - 4}
                      text="L_d" fontSize={13} fill={colors.text}/>
                <Text x={(center[0] + target[0]) / 2 + 6} y={(center[1] + target[1]) / 2 - 2}
                      text="R" fontSize={13} fill={colors.muted}/>
            </Layer>
        </Stage>
    )
}

const LookaheadGeometryFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The tangent-chord construction: the single circle tangent to the robot's heading that also passes through the lookahead point — its radius is R = L_d / (2 sin α)",
            "접선-현 구성. 로봇의 heading에 접하면서 lookahead 점도 지나는 유일한 원 -- 그 반지름이 R = L_d / (2 sin α)다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default LookaheadGeometryFigure

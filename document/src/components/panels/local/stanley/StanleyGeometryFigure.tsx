import {Arrow, Circle, Layer, Line, Shape, Stage, Text} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";

// Stanley의 두 오차(psi, e)와 front-axle→unicycle 변환을 고정 예시 각도로 그린 정적
// figure -- "From Bicycle to Unicycle" 절의 서술(가상 축간거리 L, 전륜축 오차 기준,
// delta 조향)을 눈으로 따라가게 한다. sandbox와 달리 라이브 데이터가 아니라 기하 관계
// 자체를 보여주는 고정 다이어그램이라 world 좌표 변환 없이 canvas 픽셀 좌표로 그린다.
const PANEL = 320;
const HEADING_DEG = -8;     // 로봇 heading (canvas 기준, 살짝 위쪽을 향함)
const PSI_DEG = -16;        // psi = theta_path - theta: 경로 접선이 heading보다 더 위로 꺾여 있다
const DELTA_DEG = -24;      // 조향각 delta: 전륜이 heading보다 더 꺾여 경로 쪽으로 향한다
const L_PX = 118;           // 가상 축간거리 L의 시각적 길이
const E_PX = 42;            // crosstrack error e의 시각적 길이

// psi/delta 각 표시 호와 heading↔tangent 사이 짧은 원호 모두 이 헬퍼 하나를 공유한다.
// 프로젝트에 공용 arc 헬퍼가 없고(LookaheadGeometryFigure에도 같은 이름의 파일 전용
// 사본이 있다) 각 figure가 고정 다이어그램마다 조금씩 다른 각도 조합을 쓰므로, 이
// 10줄 남짓한 헬퍼는 캔버스마다 자체 보유하는 기존 관례를 따른다.
const drawMinorArc = (
    ctx: {arc: (x: number, y: number, r: number, a1: number, a2: number, ccw: boolean) => void},
    cx: number, cy: number, r: number, a1: number, a2: number,
) => {
    const diff = Math.atan2(Math.sin(a2 - a1), Math.cos(a2 - a1))
    ctx.arc(cx, cy, r, a1, a1 + diff, diff < 0)
}

const deg = (d: number) => (d * Math.PI) / 180

const Scene = () => {
    const colors = useCanvasColors()

    const heading = deg(HEADING_DEG)
    const thetaPath = heading + deg(PSI_DEG)
    const deltaAngle = heading + deg(DELTA_DEG)

    const rear: [number, number] = [55, 235]
    const front: [number, number] = [
        rear[0] + L_PX * Math.cos(heading),
        rear[1] + L_PX * Math.sin(heading),
    ]
    // foot point: front axle에서 경로 접선에 수직 방향(perp)으로 E_PX만큼 물러난 점 --
    // "front axle을 경로 위로 정사영한 점"을 고정 다이어그램에서 그대로 배치한 것이다.
    const perp = thetaPath - Math.PI / 2
    const foot: [number, number] = [
        front[0] + E_PX * Math.cos(perp),
        front[1] + E_PX * Math.sin(perp),
    ]
    const pathBack: [number, number] = [foot[0] - 95 * Math.cos(thetaPath), foot[1] - 95 * Math.sin(thetaPath)]
    const pathBack2: [number, number] = [
        pathBack[0] - 65 * Math.cos(thetaPath + 0.18), pathBack[1] - 65 * Math.sin(thetaPath + 0.18),
    ]
    const pathFwd: [number, number] = [foot[0] + 95 * Math.cos(thetaPath), foot[1] + 95 * Math.sin(thetaPath)]
    const pathFwd2: [number, number] = [
        pathFwd[0] + 60 * Math.cos(thetaPath - 0.12), pathFwd[1] + 60 * Math.sin(thetaPath - 0.12),
    ]
    const headTip: [number, number] = [
        rear[0] + (L_PX + 26) * Math.cos(heading), rear[1] + (L_PX + 26) * Math.sin(heading),
    ]
    const deltaTip: [number, number] = [
        front[0] + 58 * Math.cos(deltaAngle), front[1] + 58 * Math.sin(deltaAngle),
    ]
    const tangentTip: [number, number] = [
        foot[0] + 46 * Math.cos(thetaPath), foot[1] + 46 * Math.sin(thetaPath),
    ]

    return (
        <Stage width={PANEL} height={PANEL}
               className="bg-surface border border-border rounded-lg overflow-hidden">
            <Layer>
                {/* 참조 경로 (파선) */}
                <Line points={[...pathBack2, ...pathBack, ...foot, ...pathFwd, ...pathFwd2]}
                      stroke={colors.muted} strokeWidth={2} dash={[7, 5]}
                      lineCap="round" lineJoin="round" tension={0.4}/>
                {/* 경로 접선(psi 기준선) */}
                <Arrow points={[...foot, ...tangentTip]} pointerLength={7} pointerWidth={6}
                       stroke={colors.accent} fill={colors.accent} strokeWidth={1.6} opacity={0.85}/>
                {/* 축간거리 L (후륜축 → 전륜축) */}
                <Line points={[...rear, ...front]} stroke={colors.text} strokeWidth={1.6} opacity={0.85}/>
                {/* heading 방향(전륜축 너머로 살짝 연장) */}
                <Arrow points={[...front, ...headTip]} pointerLength={8} pointerWidth={7}
                       stroke={colors.accent2} fill={colors.accent2} strokeWidth={2.2} opacity={0.9}/>
                {/* delta: 조향된 전륜 방향 */}
                <Arrow points={[...front, ...deltaTip]} pointerLength={8} pointerWidth={7}
                       stroke={colors.accent} fill={colors.accent} strokeWidth={2.2}/>
                {/* e: 전륜축 -> foot point (crosstrack error) */}
                <Line points={[...front, ...foot]} stroke={colors.text} strokeWidth={1.4}
                      dash={[3, 3]} opacity={0.75}/>
                {/* psi 각 표시 호 (heading ↔ 경로 접선) */}
                <Shape listening={false} sceneFunc={(ctx, shape) => {
                    ctx.beginPath()
                    drawMinorArc(ctx, front[0], front[1], 22, heading, thetaPath)
                    ctx.fillStrokeShape(shape)
                }} stroke={colors.accent} strokeWidth={1.4}/>
                {/* delta 각 표시 호 (heading ↔ 조향 방향) */}
                <Shape listening={false} sceneFunc={(ctx, shape) => {
                    ctx.beginPath()
                    drawMinorArc(ctx, front[0], front[1], 34, heading, deltaAngle)
                    ctx.fillStrokeShape(shape)
                }} stroke={colors.accent2} strokeWidth={1.4} opacity={0.8}/>
                {/* 후륜축(로봇 중심) / 전륜축 / foot point */}
                <Circle x={rear[0]} y={rear[1]} radius={7} fill={colors.accent2}
                        stroke={colors.bg} strokeWidth={1.5}/>
                <Circle x={front[0]} y={front[1]} radius={5} fill={colors.accent2}
                        stroke={colors.bg} strokeWidth={1.2}/>
                <Circle x={foot[0]} y={foot[1]} radius={4.5} fill={colors.accent}/>
                {/* label (최소한) */}
                <Text x={(rear[0] + front[0]) / 2 - 4} y={(rear[1] + front[1]) / 2 + 8}
                      text="L" fontSize={13} fill={colors.text}/>
                <Text x={(front[0] + foot[0]) / 2 + 6} y={(front[1] + foot[1]) / 2 - 4}
                      text="e" fontSize={13} fill={colors.text}/>
                <Text x={front[0] - 30} y={front[1] - 30} text="ψ" fontSize={15}
                      fontStyle="bold" fill={colors.accent}/>
                <Text x={front[0] + 40} y={front[1] - 42} text="δ" fontSize={15}
                      fontStyle="bold" fill={colors.accent2}/>
            </Layer>
        </Stage>
    )
}

const StanleyGeometryFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Both errors measured at the front axle: heading error ψ against the path tangent, crosstrack error e as lateral offset from the foot point — steering angle δ corrects both, then ω = v·tanδ/L converts to the unicycle command",
            "두 오차 모두 전륜축에서 잰다. 경로 접선 대비 heading 오차 ψ, foot point로부터의 측방 오프셋 crosstrack 오차 e. 조향각 δ가 둘을 함께 보정하고, ω = v·tanδ/L로 unicycle 명령으로 바뀐다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default StanleyGeometryFigure

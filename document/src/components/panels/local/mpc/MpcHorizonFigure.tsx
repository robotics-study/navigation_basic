import {Arrow, Circle, Layer, Line, Stage, Text} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";

const W = 320
const H = 210

// 개념 삽화(실행 엔진 아님): receding horizon의 핵심 -- 매 tick H 스텝을 예측하고 첫
// 제어 u_0만 실행한 뒤, 다음 tick에 한 칸 앞으로 밀린 새 horizon을 다시 최적화한다.
// tick t의 예측 사슬(진한 accent, 첫 구간만 실행)과 tick t+1의 사슬(옅게, 한 칸 전진)을
// 겹쳐 그 슬라이딩을 보인다. 좌표는 임의의 도식 좌표다.
const CHAIN_T = [
    [26, 150], [58, 138], [92, 124], [128, 112], [166, 104], [204, 100], [242, 100], [280, 104],
]
const CHAIN_T1 = [
    [58, 138], [92, 124], [128, 112], [166, 104], [204, 100], [242, 100], [280, 104], [300, 112],
]
const GOAL_PT = [300, 96]

const Scene = ({scale = 1}: {scale?: number}) => {
    const t = useTr()
    const colors = useCanvasColors()
    const seg = (pts: number[][], from: number, to: number): number[] =>
        pts.slice(from, to + 1).flatMap((p) => p)

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W * scale} height={H * scale}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer scaleX={scale} scaleY={scale}>
                    {/* tick t+1: 한 칸 앞으로 밀린 다음 horizon (옅게) */}
                    <Line points={CHAIN_T1.flatMap((p) => p)} stroke={colors.muted}
                          strokeWidth={2} opacity={0.5} lineCap="round" lineJoin="round" dash={[5, 4]}/>
                    {CHAIN_T1.map((p, i) => (
                        <Circle key={`b${i}`} x={p[0]} y={p[1]} radius={2.6} fill={colors.muted} opacity={0.5}/>
                    ))}

                    {/* tick t: 예측된 horizon 사슬 (예측 구간, muted) */}
                    <Line points={seg(CHAIN_T, 1, CHAIN_T.length - 1)} stroke={colors.accent}
                          strokeWidth={2.4} opacity={0.55} lineCap="round" lineJoin="round"/>
                    {CHAIN_T.slice(1).map((p, i) => (
                        <Circle key={`a${i}`} x={p[0]} y={p[1]} radius={3} fill={colors.accent} opacity={0.7}/>
                    ))}

                    {/* 실행되는 첫 제어 u_0 = 첫 구간만 (accent2, 굵게 + 화살표) */}
                    <Arrow points={[...CHAIN_T[0], ...CHAIN_T[1]]} stroke={colors.accent2}
                           fill={colors.accent2} strokeWidth={4}
                           pointerLength={8} pointerWidth={8}/>

                    {/* 로봇 (사슬 시작 = s_0) */}
                    <Circle x={CHAIN_T[0][0]} y={CHAIN_T[0][1]} radius={6}
                            fill={colors.surface} stroke={colors.accent2} strokeWidth={2.4}/>

                    {/* goal */}
                    <Circle x={GOAL_PT[0]} y={GOAL_PT[1]} radius={5} fill="#dc2626"/>

                    <Text x={CHAIN_T[0][0] - 6} y={CHAIN_T[0][1] + 12} text="s₀"
                          fontSize={12} fill={colors.text}/>
                    <Text x={CHAIN_T[1][0] - 30} y={CHAIN_T[1][1] - 22} text={t("execute u₀", "u₀ 실행")}
                          fontSize={11} fill={colors.accent2}/>
                    <Text x={150} y={70} text={t("predicted, then discarded", "예측 후 폐기")}
                          fontSize={11} fill={colors.accent}/>
                    <Text x={GOAL_PT[0] - 26} y={GOAL_PT[1] - 18} text={t("goal", "goal")}
                          fontSize={11} fill="#dc2626"/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center max-w-[22rem]">
                {t(
                    "each tick optimizes the whole H-step control sequence (the chain), executes only its first control u₀, then slides the horizon one step forward and re-optimizes — the receding horizon",
                    "매 tick H-스텝 제어열 전체(사슬)를 최적화하지만 첫 제어 u₀만 실행하고, horizon을 한 칸 앞으로 밀어 다시 최적화한다. 이것이 receding horizon이다",
                )}
            </div>
        </div>
    )
}

const MpcHorizonFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Receding horizon: the controller plans an H-step chain every tick but commits only the first control, then re-plans a chain shifted one step forward",
            "Receding horizon. 컨트롤러는 매 tick H-스텝 사슬을 계획하지만 첫 제어만 실행하고, 한 칸 앞으로 밀린 사슬을 다시 계획한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(W, H)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default MpcHorizonFigure

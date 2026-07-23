import {Circle, Layer, Line, Stage, Text} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {T, useTr} from "../../../../libs/i18n";

const W = 320
const H = 210

// 개념 삽화(실행 엔진 아님): MPPI 한 tick의 그림 -- 공칭 제어열에 Gauss 노이즈를 K개
// 뿌려 나온 표본 rollout 다발(옅게)과, 그것을 softmax 가중 평균한 실행 궤적(진하게)을
// 겹쳐 "표본을 뿌리고 비용으로 가중 평균한다"는 축을 보인다. 좌표는 도식 좌표이고 arc는
// 서로 다른 곡률의 손그림 폴리라인이다.
const ROBOT: [number, number] = [30, 120]
const GOAL: [number, number] = [292, 78]
const BASE_ANGLE = Math.atan2(GOAL[1] - ROBOT[1], GOAL[0] - ROBOT[0])
const STEPS = 14
const STEP_LEN = 15

// 곡률(회전) 스프레드로 K개 표본 arc를 만든다. 목표 방향에 가까운 arc일수록 비용이 낮아
// 진하게, 가중 평균은 그 저비용 arc들 쪽으로 당겨진다.
const sampleArc = (curvature: number): number[] => {
    let [x, y] = ROBOT
    let th = BASE_ANGLE
    const pts = [x, y]
    for (let i = 0; i < STEPS; i++) {
        th += curvature
        x += STEP_LEN * Math.cos(th)
        y += STEP_LEN * Math.sin(th)
        pts.push(x, y)
    }
    return pts
}
const CURVATURES = [-0.14, -0.10, -0.06, -0.03, 0.0, 0.03, 0.06, 0.10, 0.14]
// 가중 평균 궤적: 저비용(목표 방향, curvature≈0 부근) arc들로 당겨진 완만한 곡선.
const MEAN_ARC = sampleArc(-0.012)

const Scene = ({scale = 1}: {scale?: number}) => {
    const t = useTr()
    const colors = useCanvasColors()
    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W * scale} height={H * scale}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer scaleX={scale} scaleY={scale}>
                    {/* K개 표본 rollout 다발 (옅게) */}
                    {CURVATURES.map((c, i) => (
                        <Line key={`s${i}`} points={sampleArc(c)} stroke={colors.muted}
                              strokeWidth={1.6} opacity={0.4} lineCap="round" lineJoin="round"/>
                    ))}
                    {/* softmax 가중 평균 = 실행 궤적 (진하게) */}
                    <Line points={MEAN_ARC} stroke={colors.accent2} strokeWidth={3.4}
                          opacity={0.95} lineCap="round" lineJoin="round"/>

                    {/* 로봇 */}
                    <Circle x={ROBOT[0]} y={ROBOT[1]} radius={6}
                            fill={colors.surface} stroke={colors.accent2} strokeWidth={2.4}/>
                    {/* goal */}
                    <Circle x={GOAL[0]} y={GOAL[1]} radius={5} fill="#dc2626"/>

                    <Text x={ROBOT[0] - 6} y={ROBOT[1] + 12} text="s₀" fontSize={12} fill={colors.text}/>
                    <Text x={120} y={150} text={t("K sampled rollouts", "K개 표본 rollout")}
                          fontSize={11} fill={colors.muted}/>
                    <Text x={150} y={40} text={t("weighted average", "가중 평균")}
                          fontSize={11} fill={colors.accent2}/>
                    <Text x={GOAL[0] - 24} y={GOAL[1] - 18} text={t("goal", "goal")}
                          fontSize={11} fill="#dc2626"/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center max-w-[22rem]">
                <T
                    en={<>each tick perturbs the control sequence into K rollouts, scores each with the same
                        cost J(U), and moves the nominal sequence by their softmax weighted average — the
                        cheaper a rollout, the more it pulls</>}
                    ko={<>매 tick 제어열을 K개 rollout으로 섭동해 각각 같은 비용 J(U)로 채점하고, 공칭
                        제어열을 그 softmax 가중 평균만큼 옮긴다. rollout이 쌀수록 더 세게 당긴다</>}
                />
            </div>
        </div>
    )
}

const MppiRolloutFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "One MPPI tick: K sampled rollouts, each scored by the shared cost, combined into one executed sequence by a softmax weighted average that leans toward the cheaper rollouts",
            "MPPI 한 tick. K개 표본 rollout을 각각 공유 비용으로 채점하고, 더 싼 rollout 쪽으로 기우는 softmax 가중 평균으로 하나의 실행 제어열로 합친다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(W, H)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default MppiRolloutFigure

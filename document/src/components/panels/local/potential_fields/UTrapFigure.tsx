import {Arrow, Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";

// 증명에서 다루는 대칭 배치를 그대로 그린다: 목표 축 위에 로봇이 있고, 축에 대해
// 대칭인 두 장애물이 옆에서 밀며, 정면(목표 쪽)의 벽이 반대로 민다. 옆 성분은
// 상쇄되고(점선 회색), 정면 성분만 인력과 맞서 합력이 0이 되는 지점이 균형점이다.
const W = 340;
const H = 260;
const AXIS_X = 170;
const GOAL: [number, number] = [AXIS_X, 30];
const ROBOT: [number, number] = [AXIS_X, 168];
const START: [number, number] = [AXIS_X, 232];
const WALL_Y = 110;
const SIDE_Y0 = 118;
const SIDE_Y1 = 220;

const UTrapFigure = () => {
    const t = useTr()
    const colors = useCanvasColors()

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {/* 대칭축 */}
                    <Line points={[AXIS_X, 20, AXIS_X, H - 20]} stroke={colors.muted}
                          strokeWidth={1} dash={[3, 4]} opacity={0.5}/>
                    {/* ㄷ자 벽: 정면 + 좌우 대칭 */}
                    <Rect x={AXIS_X - 70} y={WALL_Y - 10} width={140} height={12}
                          fill={colors.text} opacity={0.78}/>
                    <Rect x={AXIS_X - 70} y={SIDE_Y0} width={12} height={SIDE_Y1 - SIDE_Y0}
                          fill={colors.text} opacity={0.78}/>
                    <Rect x={AXIS_X + 58} y={SIDE_Y0} width={12} height={SIDE_Y1 - SIDE_Y0}
                          fill={colors.text} opacity={0.78}/>

                    {/* 인력: 로봇 -> 목표 (벽을 관통하는 방향) */}
                    <Arrow points={[ROBOT[0], ROBOT[1], ROBOT[0], ROBOT[1] - 78]}
                           pointerLength={9} pointerWidth={8} stroke="#2563eb" fill="#2563eb"
                           strokeWidth={2.4}/>
                    {/* 반발: 정면 벽 -> 로봇 (인력과 반대) */}
                    <Arrow points={[ROBOT[0], ROBOT[1], ROBOT[0], ROBOT[1] + 60]}
                           pointerLength={9} pointerWidth={8} stroke="#dc2626" fill="#dc2626"
                           strokeWidth={2.4}/>
                    {/* 반발: 좌/우 벽 -> 로봇, 대칭이라 상쇄(점선 회색으로 표시) */}
                    <Arrow points={[ROBOT[0], ROBOT[1], ROBOT[0] - 46, ROBOT[1] - 6]}
                           pointerLength={7} pointerWidth={6} stroke={colors.muted} fill={colors.muted}
                           strokeWidth={1.6} opacity={0.75} dash={[5, 3]}/>
                    <Arrow points={[ROBOT[0], ROBOT[1], ROBOT[0] + 46, ROBOT[1] - 6]}
                           pointerLength={7} pointerWidth={6} stroke={colors.muted} fill={colors.muted}
                           strokeWidth={1.6} opacity={0.75} dash={[5, 3]}/>

                    <Circle x={ROBOT[0]} y={ROBOT[1]} radius={8} fill={colors.accent2}
                            stroke={colors.bg} strokeWidth={1.5}/>
                    <Circle x={GOAL[0]} y={GOAL[1]} radius={7} fill={PATH_COLOR}
                            stroke={colors.bg} strokeWidth={1.5}/>
                    <Circle x={START[0]} y={START[1]} radius={5} fill={colors.muted}/>

                    <Text x={GOAL[0] + 10} y={GOAL[1] - 6} text={t("goal", "목표")}
                          fontSize={11} fill={PATH_COLOR} fontStyle="bold"/>
                    <Text x={START[0] + 10} y={START[1] - 6} text={t("start", "start")}
                          fontSize={11} fill={colors.muted}/>
                    <Text x={ROBOT[0] + 10} y={ROBOT[1] + 4} text="F = 0"
                          fontSize={13} fill={colors.text} fontStyle="italic bold"/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center" style={{maxWidth: W}}>
                {t(
                    "front wall repulsion cancels the goal's pull while the side walls' lateral push cancels itself by symmetry — the robot stops on the axis, short of the goal",
                    "정면 벽의 반발이 목표로의 인력과 상쇄되고, 좌우 벽의 옆 성분은 대칭이라 스스로 상쇄된다. 로봇은 축 위, 목표에 못 미친 지점에서 멈춘다",
                )}
            </div>
        </div>
    )
}

const UTrapEquilibrium = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The U-trap on the algorithm's own axis of symmetry: lateral repulsion cancels by mirror symmetry, and where the remaining front-wall repulsion exactly balances the goal's pull, the resultant force is zero",
            "알고리즘 자신의 대칭축 위 U-trap. 좌우 반발은 거울 대칭으로 상쇄되고, 남은 정면 벽 반발이 목표 인력과 정확히 맞서는 지점에서 합력이 0이 된다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<UTrapFigure/>}
    >
        <UTrapFigure/>
    </CanvasFigure>
}

export default UTrapEquilibrium

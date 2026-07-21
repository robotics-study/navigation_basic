import {Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {useTr} from "../../../../libs/i18n";

// 확대해 본 turning point 하나: 참 최단 경로는 블록 모서리(격자 꼭짓점)를 정확히
// 스치고, 셀 중심에 묶인 planner는 반 셀 비껴간 중심에서 꺾어 그만큼 손해 본다.
const CELL = 62;
const N = 5;
const PANEL = CELL * N;

// 셀 단위 좌표 (x 오른쪽, y 아래). 블록은 x∈[2,4], y∈[0,3].
const S: [number, number] = [0.5, 4.5];
const G: [number, number] = [4.5, 0.5];
const CORNER: [number, number] = [4, 3];
const CENTRE: [number, number] = [4.5, 3.5];

const px = (p: [number, number]): [number, number] => [p[0] * CELL, p[1] * CELL]

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    <Rect x={2 * CELL} y={0} width={2 * CELL} height={3 * CELL}
                          fill={colors.text} opacity={0.78}/>
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`v${k}`} points={[k * CELL, 0, k * CELL, PANEL]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                    ))}
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`h${k}`} points={[0, k * CELL, PANEL, k * CELL]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                    ))}
                    {/* 셀 중심에 묶인 경로: 모서리에서 반 셀 비껴 꺾는다 */}
                    <Line points={[...px(S), ...px(CENTRE), ...px(G)]}
                          stroke={colors.muted} strokeWidth={2.5}
                          dash={[9, 7]} lineCap="round" lineJoin="round" opacity={0.9}/>
                    {/* 참 최단 경로: 모서리를 정확히 스친다 */}
                    <Line points={[...px(S), ...px(CORNER), ...px(G)]}
                          stroke={PATH_COLOR} strokeWidth={3.5}
                          lineCap="round" lineJoin="round"/>
                    <Circle x={px(CENTRE)[0]} y={px(CENTRE)[1]} radius={6}
                            fill={colors.muted} stroke={colors.bg} strokeWidth={1.5}/>
                    <Circle x={px(CORNER)[0]} y={px(CORNER)[1]} radius={7}
                            fill={colors.accent2} stroke={colors.bg} strokeWidth={1.5}/>
                    <Circle x={px(S)[0]} y={px(S)[1]} radius={7}
                            fill={colors.accent} stroke={colors.bg} strokeWidth={1.5}/>
                    <Circle x={px(G)[0]} y={px(G)[1]} radius={7}
                            fill={PATH_COLOR} stroke={colors.bg} strokeWidth={1.5}/>
                    <Text x={px(CORNER)[0] - 150} y={px(CORNER)[1] + 2}
                          width={140} align="right"
                          text={t("grid corner", "격자 꼭짓점")}
                          fontSize={13} fill={colors.accent2} fontStyle="bold"/>
                    <Text x={px(CENTRE)[0] + 12} y={px(CENTRE)[1] + 8}
                          text={t("cell centre", "셀 중심")}
                          fontSize={13} fill={colors.muted}/>
                </Layer>
            </Stage>
        </div>
    )
}

const CornerTurn = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "One turning point, zoomed in: the true shortest path (red) grazes the block's corner — a lattice point — while a cell-centre path (dashed) must bend half a cell away and pays for it",
            "turning point 하나를 확대한 그림. 참 최단 경로(빨강)는 블록 모서리, 곧 격자 꼭짓점을 정확히 스치고, 셀 중심 경로(점선)는 반 셀 비껴서 꺾어 그만큼 손해 본다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default CornerTurn

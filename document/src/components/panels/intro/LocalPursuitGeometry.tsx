import {useState} from "react";
import {Arc, Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {PATH_COLOR} from "../../2d/GridCanvas";
import {useTr} from "../../../libs/i18n";

// Pure Pursuit의 핵심 그림: 로봇은 참조 경로 위에서 lookahead 거리 L_d 앞의 점 하나를 골라,
// 그 점을 지나는 원호를 따라간다. 조향 곡률은 κ = 2·sin(α)/L_d 하나로 정해진다 (Coulter, 1992).
// L_d가 짧으면 급하게 꺾어 진동하고, 길면 부드럽지만 모서리를 크게 자른다.
const W = 340;
const H = 300;
const ROBOT = {x: 70, y: H - 70};        // 로봇 위치(뒷축), 위쪽(-y)이 진행 방향

// 참조 경로: 오른쪽으로 완만히 휘는 곡선. t∈[0,1] 매개변수.
const pathPoint = (u: number): [number, number] => {
    const x = ROBOT.x + u * 250
    const y = ROBOT.y - 30 - Math.sin(u * 1.5) * 150
    return [x, y]
}

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [ld, setLd] = useState(120)        // lookahead 거리(px)

    const pathPts: number[] = []
    for (let i = 0; i <= 60; i++) {
        const [x, y] = pathPoint(i / 60)
        pathPts.push(x, y)
    }

    // lookahead 점: 경로 위에서 로봇으로부터 거리가 L_d에 가장 가까운 점.
    let goal = pathPoint(1)
    let bestErr = Infinity
    for (let i = 0; i <= 120; i++) {
        const p = pathPoint(i / 120)
        const d = Math.hypot(p[0] - ROBOT.x, p[1] - ROBOT.y)
        if (Math.abs(d - ld) < bestErr) {
            bestErr = Math.abs(d - ld)
            goal = p
        }
    }

    // 로봇 기준 goal 방향. 진행축은 위쪽(-y). α는 진행축과 goal 사이 각.
    const dx = goal[0] - ROBOT.x
    const dy = goal[1] - ROBOT.y
    const alpha = Math.atan2(dx, -dy)        // 오른쪽(+x)이 양의 조향

    // 곡률 κ = 2 sin(α) / L_d → 원호 반경 R = 1/κ. 원 중심은 로봇에서 진행축에 수직(오른/왼).
    const kappa = (2 * Math.sin(alpha)) / ld
    const R = Math.abs(kappa) > 1e-4 ? 1 / kappa : 1e6
    // 진행축(-y)에 수직 = ±x. α>0(goal이 오른쪽)이면 중심은 오른쪽.
    const cx = ROBOT.x + R
    const cy = ROBOT.y

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {/* 참조 경로 */}
                    <Line points={pathPts} stroke={colors.muted} strokeWidth={2.5}
                          dash={[8, 6]} lineCap="round"/>
                    {/* 조향 원호: 로봇에서 goal까지 따라갈 원 */}
                    {Math.abs(R) < 5000 && (
                        <Circle x={cx} y={cy} radius={Math.abs(R)}
                                stroke={colors.accent2} strokeWidth={1.5} opacity={0.55}
                                dash={[5, 5]}/>
                    )}
                    {/* lookahead 원: 로봇 중심 반경 L_d */}
                    <Circle x={ROBOT.x} y={ROBOT.y} radius={ld}
                            stroke={colors.accent} strokeWidth={1.5} opacity={0.7}/>
                    {/* 로봇→goal 시선 (L_d) */}
                    <Line points={[ROBOT.x, ROBOT.y, goal[0], goal[1]]}
                          stroke={colors.accent} strokeWidth={1.5}/>
                    {/* 로봇 진행축 */}
                    <Line points={[ROBOT.x, ROBOT.y, ROBOT.x, ROBOT.y - 70]}
                          stroke={colors.text} strokeWidth={1.5} dash={[4, 4]}/>
                    {/* goal 점 */}
                    <Circle x={goal[0]} y={goal[1]} radius={7}
                            fill={PATH_COLOR} stroke={colors.bg} strokeWidth={1.5}/>
                    {/* 로봇 */}
                    <Rect x={ROBOT.x - 9} y={ROBOT.y - 9} width={18} height={18}
                          cornerRadius={4} fill={colors.text}/>
                    <Text x={goal[0] + 10} y={goal[1] - 6}
                          text={t("lookahead point", "lookahead 점")}
                          fontSize={12} fill={PATH_COLOR} fontStyle="bold"/>
                    <Text x={(ROBOT.x + goal[0]) / 2 - 26} y={(ROBOT.y + goal[1]) / 2 - 16}
                          text="Lₐ" fontSize={13} fill={colors.accent} fontStyle="italic bold"/>
                    <Text x={ROBOT.x + 6} y={ROBOT.y - 44}
                          text="α" fontSize={13} fill={colors.text} fontStyle="italic"/>
                    {/* α 호 표시 */}
                    <Arc x={ROBOT.x} y={ROBOT.y} innerRadius={26} outerRadius={26}
                         angle={(alpha * 180) / Math.PI}
                         rotation={-90} stroke={colors.text} strokeWidth={1.5}/>
                </Layer>
            </Stage>
            <div className="flex items-center gap-2 text-xs text-muted w-full" style={{maxWidth: W}}>
                <span className="w-24 shrink-0">Lₐ = {(ld / 40).toFixed(2)} m</span>
                <input type="range" min={55} max={230} step={1} value={ld}
                       onChange={(e) => setLd(parseInt(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("lookahead distance", "lookahead 거리")}/>
            </div>
            <div className="text-xs text-muted text-center">
                {t(
                    "short lookahead cuts corners tightly and can oscillate; long lookahead smooths the turn but drifts wide",
                    "짧은 lookahead는 모서리를 급히 잘라 진동할 수 있고, 긴 lookahead는 부드럽지만 크게 벌어진다",
                )}
            </div>
        </div>
    )
}

const LocalPursuitGeometry = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Pure Pursuit picks the point where the lookahead circle (radius Lₐ) meets the path, then follows the unique arc through it — steering curvature is κ = 2·sin(α)/Lₐ. Slide Lₐ to trade sharpness for smoothness.",
            "Pure Pursuit은 lookahead 원(반경 Lₐ)이 경로와 만나는 점을 고르고, 그 점을 지나는 유일한 원호를 따라간다. 조향 곡률은 κ = 2·sin(α)/Lₐ 하나다. Lₐ를 움직여 민첩함과 부드러움을 맞바꿔 보라.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default LocalPursuitGeometry

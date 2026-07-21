import {useState} from "react";
import {Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {useTr} from "../../../../libs/i18n";

// "왜 goal 에서 뒤로 탐색하는가"를 보여 주는 figure. g 는 goal 까지의 비용장이라
// 로봇이 어디로 움직여도 그대로 유효하다. 로봇(청록 원)을 끌어 보면 비용장은 변하지
// 않고, 로봇은 어느 위치에서든 내리막(gradient)을 따라가면 된다.
const N = 13;
const PANEL = 300;
const GOAL: [number, number] = [2, 10];

const octile = (r1: number, c1: number, r2: number, c2: number) => {
    const dr = Math.abs(r1 - r2)
    const dc = Math.abs(c1 - c2)
    return Math.max(dr, dc) + (Math.SQRT2 - 1) * Math.min(dr, dc)
}

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [robot, setRobot] = useState<[number, number]>([10, 2])
    const cell = PANEL / N
    const maxD = octile(N - 1, 0, GOAL[0], GOAL[1])

    // 로봇 위치에서 goal 까지 내리막을 따라가는 경로 (비용장 위의 greedy descent).
    const descent: Array<[number, number]> = []
    let cur = robot
    let guard = 0
    while ((cur[0] !== GOAL[0] || cur[1] !== GOAL[1]) && guard++ < N * N) {
        descent.push(cur)
        let best: [number, number] = cur
        let bestD = octile(cur[0], cur[1], GOAL[0], GOAL[1])
        for (const [dr, dc] of [[-1, 0], [1, 0], [0, -1], [0, 1], [-1, -1], [-1, 1], [1, -1], [1, 1]]) {
            const nr = cur[0] + dr
            const nc = cur[1] + dc
            if (nr < 0 || nr >= N || nc < 0 || nc >= N) continue
            const d = octile(nr, nc, GOAL[0], GOAL[1])
            if (d < bestD) {
                bestD = d
                best = [nr, nc]
            }
        }
        cur = best
    }
    descent.push(GOAL)

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {/* goal 기준 비용장 g — 어두울수록 goal 에서 멀다 */}
                    {Array.from({length: N * N}, (_, i) => {
                        const r = Math.floor(i / N)
                        const c = i % N
                        const d = octile(r, c, GOAL[0], GOAL[1]) / maxD
                        return <Rect key={i} x={c * cell} y={r * cell} width={cell} height={cell}
                                     fill={colors.accent} opacity={0.06 + d * 0.34}/>
                    })}
                    <Line points={descent.flatMap(([r, c]) => [(c + 0.5) * cell, (r + 0.5) * cell])}
                          stroke={colors.accent2} strokeWidth={3} lineCap="round" lineJoin="round"/>
                    <Circle x={(GOAL[1] + 0.5) * cell} y={(GOAL[0] + 0.5) * cell}
                            radius={cell * 0.34} fill={PATH_COLOR}/>
                    <Text x={(GOAL[1] - 1.6) * cell} y={(GOAL[0] - 1.1) * cell} text="goal"
                          fontSize={11} fill={colors.muted}/>
                    <Circle x={(robot[1] + 0.5) * cell} y={(robot[0] + 0.5) * cell}
                            radius={cell * 0.36} fill={colors.accent2}
                            stroke={colors.bg} strokeWidth={1.5}
                            draggable
                            onDragEnd={(e) => {
                                const c = Math.max(0, Math.min(N - 1, Math.floor(e.target.x() / cell)))
                                const r = Math.max(0, Math.min(N - 1, Math.floor(e.target.y() / cell)))
                                e.target.position({x: (robot[1] + 0.5) * cell, y: (robot[0] + 0.5) * cell})
                                setRobot([r, c])
                            }}/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center">
                {t("drag the robot — the cost-to-goal field g stays valid wherever it moves",
                    "로봇을 끌어 보라. goal 기준 비용장 g 는 로봇이 어디로 가든 그대로 유효하다")}
            </div>
        </div>
    )
}

const BackwardField = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Why D* Lite searches backward: g measures cost-to-goal, so the moving robot never invalidates it — only newly discovered walls do",
            "D* Lite 가 backward 로 탐색하는 이유. g 는 goal 까지의 비용이라 로봇의 이동은 그것을 무효화하지 않는다. 무효화하는 것은 새로 발견된 벽뿐이다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default BackwardField

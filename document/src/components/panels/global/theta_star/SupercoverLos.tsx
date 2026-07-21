import {useState} from "react";
import {Circle, Layer, Line, Rect, Stage} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {PATH_COLOR} from "../../../2d/GridCanvas";
import {emptyGrid} from "../../../../libs/grid";
import {lineOfSight} from "../../../../libs/algorithms/theta_star";
import {Cell} from "../../../../libs/trace/timeline";
import {useTr} from "../../../../libs/i18n";

// line-of-sight 검사 자체를 보여 주는 figure. 두 끝점을 끌면 supercover가 지나는
// 셀들이 칠해지고, 벽(클릭 토글)에 막히면 선이 빨갛게 바뀐다.
const N = 11;
const PANEL = 300;

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [a, setA] = useState<Cell>([8, 1])
    const [b, setB] = useState<Cell>([2, 9])
    const [walls, setWalls] = useState<Set<number>>(() => new Set([5 * N + 5, 4 * N + 5]))
    const cell = PANEL / N

    const map = emptyGrid("los", N, N)
    walls.forEach((i) => { map.occupied[i] = true })
    const visible = lineOfSight(map, a, b)

    // supercover가 지나는 셀: 전 셀에 대해 "그 셀 하나만 막았을 때 LOS가 깨지는가"로
    // 계산하면 구현 중복 없이 정확히 같은 규칙을 따른다 (교육용 규모라 비용 무시 가능).
    const crossed: number[] = []
    for (let i = 0; i < N * N; i++) {
        if (map.occupied[i]) continue
        const probe = emptyGrid("p", N, N)
        walls.forEach((w) => { probe.occupied[w] = true })
        probe.occupied[i] = true
        if (!lineOfSight(probe, a, b) && visible) crossed.push(i)
    }

    const center = (c: Cell): [number, number] => [(c[1] + 0.5) * cell, (c[0] + 0.5) * cell]
    const endpoint = (c: Cell, setter: (n: Cell) => void, fill: string) => (
        <Circle x={center(c)[0]} y={center(c)[1]} radius={cell * 0.33} fill={fill}
                stroke={colors.bg} strokeWidth={1.5} draggable
                onDragEnd={(e) => {
                    const col = Math.max(0, Math.min(N - 1, Math.floor(e.target.x() / cell)))
                    const row = Math.max(0, Math.min(N - 1, Math.floor(e.target.y() / cell)))
                    e.target.position({x: center(c)[0], y: center(c)[1]})
                    if (!walls.has(row * N + col)) setter([row, col])
                }}/>
    )

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden"
                   onClick={(e) => {
                       const pos = e.target.getStage()?.getPointerPosition()
                       if (!pos) return
                       const col = Math.floor(pos.x / cell)
                       const row = Math.floor(pos.y / cell)
                       const i = row * N + col
                       if ((row === a[0] && col === a[1]) || (row === b[0] && col === b[1])) return
                       setWalls((prev) => {
                           const next = new Set(prev)
                           if (next.has(i)) next.delete(i)
                           else next.add(i)
                           return next
                       })
                   }}>
                <Layer>
                    {visible && crossed.map((i) => (
                        <Rect key={i} x={(i % N) * cell} y={Math.floor(i / N) * cell}
                              width={cell} height={cell} fill={colors.accent} opacity={0.18}/>
                    ))}
                    {Array.from(walls).map((i) => (
                        <Rect key={`w${i}`} x={(i % N) * cell} y={Math.floor(i / N) * cell}
                              width={cell} height={cell} fill={colors.text} opacity={0.75}/>
                    ))}
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`v${k}`} points={[k * cell, 0, k * cell, PANEL]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.7}/>
                    ))}
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`h${k}`} points={[0, k * cell, PANEL, k * cell]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.7}/>
                    ))}
                    <Line points={[...center(a), ...center(b)]}
                          stroke={visible ? colors.accent : PATH_COLOR}
                          strokeWidth={3} lineCap="round"/>
                    {endpoint(a, setA, colors.accent)}
                    {endpoint(b, setB, PATH_COLOR)}
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center">
                {visible
                    ? t("line of sight: clear — highlighted cells are what the supercover checks",
                        "line of sight: 통과. 칠해진 셀들이 supercover가 검사하는 셀이다")
                    : t("line of sight: blocked", "line of sight: 차단됨")}
                {" · "}
                {t("drag the endpoints, click cells to toggle walls",
                    "끝점을 끌고, 셀을 클릭해 벽을 토글해 보라")}
            </div>
        </div>
    )
}

const SupercoverLos = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The line-of-sight primitive Theta* is built on: a supercover walk visits every cell the segment crosses, and an exact corner needs both orthogonal cells free",
            "Theta*의 토대인 line-of-sight primitive. supercover 순회가 선분이 지나는 모든 셀을 방문하고, 정확한 corner 통과에는 양쪽 직교 셀이 모두 비어 있어야 한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default SupercoverLos

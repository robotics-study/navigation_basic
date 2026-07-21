import {useState} from "react";
import {Circle, Layer, Line, Rect, Stage} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";

// "격자는 그래프다"를 그대로 그린다: 자유 셀 중심 = vertex, 이웃 관계 = edge.
// 4/8-connected 토글로 간선 집합이 어떻게 달라지는지 보여 준다.
const N = 7;
const PANEL = 300;
const WALLS = new Set([10, 17, 24, 31, 16, 37, 38])   // row*N+col

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [conn, setConn] = useState<4 | 8>(4)
    const cell = PANEL / N

    const free = (r: number, c: number) =>
        r >= 0 && r < N && c >= 0 && c < N && !WALLS.has(r * N + c)

    const deltas: Array<[number, number]> = conn === 4
        ? [[0, 1], [1, 0]]
        : [[0, 1], [1, 0], [1, 1], [1, -1]]

    const edges: Array<[number, number, number, number]> = []
    for (let r = 0; r < N; r++) {
        for (let c = 0; c < N; c++) {
            if (!free(r, c)) continue
            for (const [dr, dc] of deltas) {
                if (free(r + dr, c + dc)) edges.push([r, c, r + dr, c + dc])
            }
        }
    }

    const cx = (c: number) => (c + 0.5) * cell
    const cy = (r: number) => (r + 0.5) * cell

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {Array.from(WALLS).map((i) => (
                        <Rect key={i} x={(i % N) * cell} y={Math.floor(i / N) * cell}
                              width={cell} height={cell} fill={colors.text} opacity={0.72}/>
                    ))}
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`v${k}`} points={[k * cell, 0, k * cell, PANEL]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.7}/>
                    ))}
                    {Array.from({length: N + 1}, (_, k) => (
                        <Line key={`h${k}`} points={[0, k * cell, PANEL, k * cell]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.7}/>
                    ))}
                    {edges.map(([r1, c1, r2, c2], i) => (
                        <Line key={i} points={[cx(c1), cy(r1), cx(c2), cy(r2)]}
                              stroke={colors.accent} strokeWidth={1.3} opacity={0.55}/>
                    ))}
                    {Array.from({length: N * N}, (_, i) => {
                        const r = Math.floor(i / N)
                        const c = i % N
                        return free(r, c) && (
                            <Circle key={i} x={cx(c)} y={cy(r)} radius={3.2} fill={colors.accent}/>
                        )
                    })}
                </Layer>
            </Stage>
            <div className="flex items-center gap-1.5 text-xs text-muted">
                {([4, 8] as const).map((k) => (
                    <button key={k} type="button" onClick={() => setConn(k)}
                            className={cn(
                                "px-2 py-0.5 rounded border tabular-nums",
                                conn === k
                                    ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                    : "border-border hover:bg-surface",
                            )}>
                        {k}-connected
                    </button>
                ))}
                <span className="tabular-nums">
                    {t("edges", "edge")} <span className="font-semibold"
                                              style={{color: "var(--accent)"}}>{edges.length}</span>
                </span>
            </div>
        </div>
    )
}

const GridAsGraph = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "A grid is a graph: free cells are vertices, neighbor relations are edges — switching 4→8 changes the edge set, not the algorithm",
            "grid는 그래프다. 자유 셀이 vertex, 이웃 관계가 edge이고, 4→8 전환은 알고리즘이 아니라 edge 집합을 바꾼다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default GridAsGraph

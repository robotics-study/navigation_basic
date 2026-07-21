import {useMemo, useState} from "react";
import {Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {useTr} from "../../../libs/i18n";
import {mulberry32} from "../../../libs/rng";

// 같은 장애물 세계를 두 계열이 각각 어떻게 "보는지" 나란히 그린다:
// 왼쪽은 격자 이산화(occupied/free 셀), 오른쪽은 무작위 샘플 + 최근접 연결 트리.
const W = 240;
const N = 12;                       // 격자 해상도
const OBSTACLES = [                 // 정규화 좌표의 원형 장애물 (cx, cy, r)
    {x: 0.32, y: 0.35, r: 0.16},
    {x: 0.72, y: 0.68, r: 0.19},
    {x: 0.68, y: 0.18, r: 0.10},
];

const inObstacle = (x: number, y: number) =>
    OBSTACLES.some((o) => (x - o.x) ** 2 + (y - o.y) ** 2 <= o.r ** 2)

const Obstacles = ({fill}: {fill: string}) => (
    <>
        {OBSTACLES.map((o, i) => (
            <Circle key={i} x={o.x * W} y={o.y * W} radius={o.r * W} fill={fill} opacity={0.75}/>
        ))}
    </>
)

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [seed, setSeed] = useState(7)

    // 오른쪽 패널: 샘플을 뽑아 가장 가까운 기존 노드에 잇는다 (RRT 풍의 트리).
    const tree = useMemo(() => {
        const rand = mulberry32(seed)
        const nodes: Array<[number, number]> = [[0.08, 0.9]]
        const edges: Array<[number, number, number, number]> = []
        let guard = 0
        while (nodes.length < 46 && guard++ < 600) {
            const x = rand()
            const y = rand()
            if (inObstacle(x, y)) continue
            let best = 0
            let bestD = Infinity
            nodes.forEach(([nx, ny], i) => {
                const d = (nx - x) ** 2 + (ny - y) ** 2
                if (d < bestD) {
                    bestD = d
                    best = i
                }
            })
            const [px, py] = nodes[best]
            // 이동 경로가 장애물을 지나면 버린다 (간단한 중점 검사).
            if (inObstacle((px + x) / 2, (py + y) / 2)) continue
            nodes.push([x, y])
            edges.push([px, py, x, y])
        }
        return {nodes, edges}
    }, [seed])

    const cell = W / N

    return (
        <div className="flex flex-col items-center gap-2">
            <div className="flex flex-wrap justify-center gap-4">
                <div className="flex flex-col items-center gap-1">
                    <Stage width={W} height={W}
                           className="bg-surface border border-border rounded-lg overflow-hidden">
                        <Layer>
                            {/* 셀 중심이 장애물 안이면 occupied */}
                            {Array.from({length: N * N}, (_, i) => {
                                const cx = ((i % N) + 0.5) / N
                                const cy = (Math.floor(i / N) + 0.5) / N
                                return inObstacle(cx, cy) && (
                                    <Rect key={i} x={(i % N) * cell} y={Math.floor(i / N) * cell}
                                          width={cell} height={cell} fill={colors.text} opacity={0.7}/>
                                )
                            })}
                            {Array.from({length: N + 1}, (_, k) => (
                                <Line key={`v${k}`} points={[k * cell, 0, k * cell, W]}
                                      stroke={colors.border} strokeWidth={0.5}/>
                            ))}
                            {Array.from({length: N + 1}, (_, k) => (
                                <Line key={`h${k}`} points={[0, k * cell, W, k * cell]}
                                      stroke={colors.border} strokeWidth={0.5}/>
                            ))}
                            <Obstacles fill={colors.muted}/>
                        </Layer>
                    </Stage>
                    <span className="text-xs text-muted">{t("discretize: grid", "이산화: grid")}</span>
                </div>
                <div className="flex flex-col items-center gap-1">
                    <Stage width={W} height={W}
                           className="bg-surface border border-border rounded-lg overflow-hidden">
                        <Layer>
                            <Obstacles fill={colors.muted}/>
                            {tree.edges.map(([x1, y1, x2, y2], i) => (
                                <Line key={i} points={[x1 * W, y1 * W, x2 * W, y2 * W]}
                                      stroke={colors.accent} strokeWidth={1.2} opacity={0.6}/>
                            ))}
                            {tree.nodes.map(([x, y], i) => (
                                <Circle key={i} x={x * W} y={y * W} radius={2.4}
                                        fill={colors.accent}/>
                            ))}
                            <Circle x={tree.nodes[0][0] * W} y={tree.nodes[0][1] * W} radius={5}
                                    fill={colors.accent}/>
                            <Text x={tree.nodes[0][0] * W + 8} y={tree.nodes[0][1] * W - 4}
                                  text="start" fontSize={11} fill={colors.muted}/>
                        </Layer>
                    </Stage>
                    <span className="text-xs text-muted">{t("sample: tree", "sampling: tree")}</span>
                </div>
            </div>
            <button type="button"
                    className="px-2 py-0.5 rounded border border-border text-xs text-muted hover:bg-surface"
                    onClick={() => setSeed((s) => s + 1)}>
                {t("resample", "resample")}
            </button>
        </div>
    )
}

const DiscretizeVsSample = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The same world seen by the two families: a grid overlays everything, a sampler only touches what it draws",
            "같은 세계를 보는 두 계열의 방식. grid 는 전부를 덮고, sampler 는 뽑은 것만 만진다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default DiscretizeVsSample

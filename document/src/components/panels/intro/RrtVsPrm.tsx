import {useEffect, useMemo, useRef, useState} from "react";
import {Circle, Layer, Line, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {useTr} from "../../../libs/i18n";
import cn from "../../../libs/cn";
import {mulberry32} from "../../../libs/rng";

// Tree(RRT)와 Roadmap(PRM)의 구조 차이를 성장 애니메이션으로 보여 준다.
// 둘 다 같은 장애물·같은 시드에서 자란다.
const W = 300;
const OBSTACLES = [
    {x: 0.5, y: 0.42, r: 0.15},
    {x: 0.2, y: 0.72, r: 0.12},
    {x: 0.82, y: 0.25, r: 0.11},
];
const START: [number, number] = [0.08, 0.92];

const inObstacle = (x: number, y: number) =>
    OBSTACLES.some((o) => (x - o.x) ** 2 + (y - o.y) ** 2 <= o.r ** 2)

const motionFree = (x1: number, y1: number, x2: number, y2: number) => {
    // 짧은 선분이라 중간점 몇 개 검사로 충분하다 (교육용 근사).
    for (let s = 0.2; s < 1; s += 0.2) {
        if (inObstacle(x1 + (x2 - x1) * s, y1 + (y2 - y1) * s)) return false
    }
    return true
}

type Mode = "tree" | "roadmap";

interface Built {
    nodes: Array<[number, number]>;
    edges: Array<[number, number, number, number]>;
}

const build = (mode: Mode, seed: number): Built => {
    const rand = mulberry32(seed)
    if (mode === "tree") {
        const nodes: Array<[number, number]> = [START]
        const edges: Built["edges"] = []
        const ETA = 0.13
        let guard = 0
        while (nodes.length < 70 && guard++ < 1200) {
            const sx = rand()
            const sy = rand()
            let best = 0
            let bestD = Infinity
            nodes.forEach(([nx, ny], i) => {
                const d = (nx - sx) ** 2 + (ny - sy) ** 2
                if (d < bestD) {
                    bestD = d
                    best = i
                }
            })
            const [px, py] = nodes[best]
            const d = Math.sqrt(bestD)
            const t = Math.min(1, ETA / d)
            const x = px + (sx - px) * t
            const y = py + (sy - py) * t
            if (inObstacle(x, y) || !motionFree(px, py, x, y)) continue
            nodes.push([x, y])
            edges.push([px, py, x, y])
        }
        return {nodes, edges}
    }
    // roadmap: 유효 샘플을 먼저 다 뽑고, 반경 이웃을 잇는다.
    const nodes: Array<[number, number]> = []
    let guard = 0
    while (nodes.length < 46 && guard++ < 800) {
        const x = rand()
        const y = rand()
        if (!inObstacle(x, y)) nodes.push([x, y])
    }
    const edges: Built["edges"] = []
    const R = 0.2
    for (let i = 0; i < nodes.length; i++) {
        for (let j = i + 1; j < nodes.length; j++) {
            const [x1, y1] = nodes[i]
            const [x2, y2] = nodes[j]
            const d = Math.hypot(x1 - x2, y1 - y2)
            if (d < R && motionFree(x1, y1, x2, y2)) edges.push([x1, y1, x2, y2])
        }
    }
    return {nodes, edges}
}

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [mode, setMode] = useState<Mode>("tree")
    const [seed, setSeed] = useState(11)
    const [frame, setFrame] = useState(0)
    const timer = useRef<number>()

    const built = useMemo(() => build(mode, seed), [mode, seed])

    // 성장 애니메이션: edge를 순서대로 늘린다.
    useEffect(() => {
        setFrame(0)
        timer.current = window.setInterval(() => {
            setFrame((f) => {
                if (f >= built.edges.length) {
                    window.clearInterval(timer.current)
                    return f
                }
                return f + 1
            })
        }, 45)
        return () => window.clearInterval(timer.current)
    }, [built])

    const visible = built.edges.slice(0, frame)

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={W}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {OBSTACLES.map((o, i) => (
                        <Circle key={i} x={o.x * W} y={o.y * W} radius={o.r * W}
                                fill={colors.muted} opacity={0.75}/>
                    ))}
                    {visible.map(([x1, y1, x2, y2], i) => (
                        <Line key={i} points={[x1 * W, y1 * W, x2 * W, y2 * W]}
                              stroke={colors.accent} strokeWidth={1.2} opacity={0.6}/>
                    ))}
                    {mode === "roadmap" && built.nodes.map(([x, y], i) => (
                        <Circle key={i} x={x * W} y={y * W} radius={2.6} fill={colors.accent}/>
                    ))}
                    {mode === "tree" && (
                        <>
                            <Circle x={START[0] * W} y={START[1] * W} radius={5} fill={colors.accent}/>
                            <Text x={START[0] * W + 8} y={START[1] * W - 4} text="start"
                                  fontSize={11} fill={colors.muted}/>
                        </>
                    )}
                </Layer>
            </Stage>
            <div className="flex items-center gap-1.5 text-xs text-muted">
                {(["tree", "roadmap"] as const).map((m) => (
                    <button key={m} type="button" onClick={() => setMode(m)}
                            className={cn(
                                "px-2 py-0.5 rounded border",
                                mode === m
                                    ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                    : "border-border hover:bg-surface",
                            )}>
                        {m === "tree" ? "tree (RRT)" : "roadmap (PRM)"}
                    </button>
                ))}
                <button type="button"
                        className="px-2 py-0.5 rounded border border-border hover:bg-surface"
                        onClick={() => setSeed((s) => s + 1)}>
                    {t("resample", "resample")}
                </button>
            </div>
        </div>
    )
}

const RrtVsPrm = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Two ways to wire random samples: a tree grows outward from the start (single query), a roadmap connects everything to its neighbors (many queries)",
            "무작위 샘플을 잇는 두 방식. tree는 시작점에서 바깥으로 자라고(단일 질의), roadmap은 모든 샘플을 이웃과 잇는다(다중 질의)",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default RrtVsPrm

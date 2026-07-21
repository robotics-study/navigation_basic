import {useState} from "react";
import {Circle, Layer, Line, Stage} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {integrate, Pose} from "../../../../libs/algorithms/hybrid_astar";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// Hybrid A*의 successor를 그대로 보여 주는 figure: 한 pose에서 뻗는 일정 곡률
// motion primitive 부채꼴. 조향 수·회전 반경·후진을 바꿔 가며 탐색 가지가 어떻게
// 생기는지 본다.
const PANEL = 320;
const SCALE = 46;                          // world 1.0 → 픽셀
const ORIGIN: Pose = [0, 0, Math.PI / 2];  // 화면 아래 중앙에서 위를 보는 pose

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [numSteering, setNumSteering] = useState(7)
    const [radius, setRadius] = useState(1.4)
    const [reverse, setReverse] = useState(false)

    const kappaMax = 1 / radius
    const span = (2 * kappaMax) / (numSteering - 1)
    const arcLen = 2.4
    // 후진 arc는 origin 아래로 뻗으므로, 후진이 켜지면 origin을 올려 잘림을 막는다.
    const originY = PANEL - (reverse ? 34 + arcLen * 0.6 * SCALE : 34)
    const toPx = (p: Pose): [number, number] => [
        PANEL / 2 + p[0] * SCALE,
        originY - p[1] * SCALE,
    ]

    const arcs: Array<{pts: number[]; rev: boolean}> = []
    const lengths = reverse ? [arcLen, -arcLen * 0.6] : [arcLen]
    for (const len of lengths) {
        for (let i = 0; i < numSteering; i++) {
            const kappa = -kappaMax + i * span
            const pts: number[] = [...toPx(ORIGIN)]
            for (let j = 1; j <= 16; j++) {
                pts.push(...toPx(integrate(ORIGIN, kappa, len * (j / 16))))
            }
            arcs.push({pts, rev: len < 0})
        }
    }

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {arcs.map((a, i) => (
                        <Line key={i} points={a.pts}
                              stroke={a.rev ? colors.muted : colors.accent}
                              strokeWidth={2} opacity={a.rev ? 0.7 : 0.85}
                              dash={a.rev ? [6, 5] : undefined}
                              lineCap="round"/>
                    ))}
                    <Circle x={toPx(ORIGIN)[0]} y={toPx(ORIGIN)[1]} radius={7}
                            fill={colors.accent2} stroke={colors.bg} strokeWidth={1.5}/>
                </Layer>
            </Stage>
            <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap">
                {[3, 5, 7, 9].map((n) => (
                    <button key={n} type="button" onClick={() => setNumSteering(n)}
                            className={cn(
                                "px-2 py-0.5 rounded border tabular-nums",
                                numSteering === n
                                    ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                    : "border-border hover:bg-surface",
                            )}>
                        {n} {t("steer", "조향")}
                    </button>
                ))}
                <button type="button" onClick={() => setReverse((v) => !v)}
                        className={cn(
                            "px-2 py-0.5 rounded border",
                            reverse
                                ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                : "border-border hover:bg-surface",
                        )}>
                    {t("reverse", "후진")}
                </button>
            </div>
            <div className="flex items-center gap-2 text-xs text-muted w-full max-w-[280px]">
                <span className="whitespace-nowrap">{t("turn radius", "회전 반경")}</span>
                <input type="range" min={0.7} max={3} step={0.1} value={radius}
                       onChange={(e) => setRadius(parseFloat(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("minimum turn radius", "최소 회전 반경")}/>
                <span className="tabular-nums">{radius.toFixed(1)}</span>
            </div>
        </div>
    )
}

const PrimitiveFan = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The successor set of a single pose: constant-curvature arcs bounded by the minimum turn radius — dashes are reverse primitives",
            "pose 하나의 successor 집합. 최소 회전 반경이 한계 짓는 일정 곡률 arc 들이고, 점선은 후진 primitive 다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default PrimitiveFan

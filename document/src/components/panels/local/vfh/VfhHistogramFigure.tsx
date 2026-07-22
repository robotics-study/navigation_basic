import {useMemo, useState} from "react";
import {Arrow, Circle, Layer, Shape, Stage} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";

// "polar histogram + valley 선택" 개념 그림 — 실제 엔진(runVfh)이 아니라, 두 장애물
// 뭉치가 만드는 합성 밀도 곡선을 직접 그려 threshold 슬라이더 하나로 valley가
// 열리고 닫히는 것을 보여준다(로봇의 실제 tick과 무관한 정적 예시라 vfh.ts를
// 재사용하지 않는다 — LocalVelocityWindow가 DWA 엔진과 분리된 예시를 쓰는 것과 같은 관례).
const N = 36;
const GOAL_SECTOR = 27;   // 히스토그램 위쪽 근처 — 로봇이 도달하려는 방향
const SIZE = 260;
const CENTER = SIZE / 2;
const BASE_R = SIZE * 0.16;
const MAX_R = SIZE * 0.42;

// 두 장애물 뭉치가 만드는 합성 밀도(폭 넓은 뭉치 하나 + 좁은 뭉치 하나) — 실제
// occupied_within 누적을 흉내 낸 정적 예시일 뿐, 물리적 배치를 표현하지 않는다.
function densityAt(k: number): number {
    const bump = (center: number, width: number, height: number) => {
        let d = Math.abs(k - center)
        d = Math.min(d, N - d)
        return height * Math.exp(-(d * d) / (2 * width * width))
    }
    return bump(4, 3.2, 0.85) + bump(18, 5.5, 0.55) + 0.03
}

interface Valley { start: number; end: number; width: number }

function findValleys(below: boolean[]): Valley[] {
    if (below.every((b) => b)) return [{start: 0, end: N - 1, width: N}]
    if (!below.some((b) => b)) return []
    const cut = below.findIndex((b) => !b)
    const order = Array.from({length: N}, (_, i) => (cut + 1 + i) % N)
    const valleys: Valley[] = []
    let start: number | null = null
    let end = -1
    for (const idx of order) {
        if (below[idx]) {
            if (start === null) start = idx
            end = idx
        } else if (start !== null) {
            valleys.push({start, end, width: ((end - start + N) % N) + 1})
            start = null
        }
    }
    return valleys
}

const circularDist = (a: number, b: number) => Math.min(((a - b) % N + N) % N, ((b - a) % N + N) % N)
const contains = (v: Valley, k: number) => (((k - v.start) % N + N) % N) < v.width

const Scene = () => {
    const t = useTr()
    const colors = useCanvasColors()
    const [threshold, setThreshold] = useState(0.4)

    const bins = useMemo(() => Array.from({length: N}, (_, k) => densityAt(k)), [])
    const maxBin = Math.max(...bins)
    const below = bins.map((v) => v < threshold)
    const valleys = findValleys(below)
    const selected = useMemo(() => {
        let best: Valley | null = null
        let bestGap = Infinity
        for (const v of valleys) {
            const gap = contains(v, GOAL_SECTOR) ? 0
                : Math.min(circularDist(v.start, GOAL_SECTOR), circularDist(v.end, GOAL_SECTOR))
            if (gap < bestGap || (gap === bestGap && v.width > (best?.width ?? -1))) { best = v; bestGap = gap }
        }
        return best
    }, [valleys])

    const sector = (2 * Math.PI) / N
    // world CCW 각(sector 0 = +x) → canvas 각: y축 반전 때문에 부호가 반대.
    const angleOf = (k: number) => -(k + 0.5) * sector

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={SIZE} height={SIZE} className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {/* 히스토그램 wedge */}
                    <Shape listening={false} sceneFunc={(ctx, shape) => {
                        ctx.beginPath()
                        for (let k = 0; k < N; k++) {
                            const r = BASE_R + (MAX_R - BASE_R) * (bins[k] / maxBin)
                            const a0 = -(k + 1) * sector
                            const a1 = -k * sector
                            ctx.moveTo(CENTER, CENTER)
                            ctx.arc(CENTER, CENTER, r, a0, a1, false)
                            ctx.closePath()
                        }
                        ctx.fillStrokeShape(shape)
                    }} fill={colors.muted} opacity={0.4} stroke={colors.border} strokeWidth={0.5}/>
                    {/* 선택된 valley 강조(부채꼴) */}
                    {selected && (
                        <Shape listening={false} sceneFunc={(ctx, shape) => {
                            ctx.beginPath()
                            ctx.moveTo(CENTER, CENTER)
                            const a0 = -(selected.start + selected.width) * sector
                            const a1 = -selected.start * sector
                            ctx.arc(CENTER, CENTER, MAX_R * 1.02, a0, a1, false)
                            ctx.closePath()
                            ctx.fillStrokeShape(shape)
                        }} fill={colors.accent2} opacity={0.16}/>
                    )}
                    {/* threshold 원 */}
                    <Circle x={CENTER} y={CENTER} radius={BASE_R + (MAX_R - BASE_R) * (threshold / maxBin)}
                            stroke={PATH_COLOR} strokeWidth={1.5} dash={[5, 4]} opacity={0.85}/>
                    {/* 로봇(중심) */}
                    <Circle x={CENTER} y={CENTER} radius={4} fill={colors.text}/>
                    {/* goal 방향 화살표 */}
                    <Arrow points={[
                        CENTER, CENTER,
                        CENTER + Math.cos(angleOf(GOAL_SECTOR)) * MAX_R * 1.18,
                        CENTER + Math.sin(angleOf(GOAL_SECTOR)) * MAX_R * 1.18,
                    ]} stroke={colors.accent} fill={colors.accent} strokeWidth={2}
                           pointerLength={8} pointerWidth={7} dash={[1, 3]} opacity={0.9}/>
                    {/* 선택 방향 화살표 */}
                    {selected && (() => {
                        const idx = (selected.start + (selected.width - 1) / 2 + N) % N
                        const a = angleOf(idx)
                        return <Arrow points={[
                            CENTER, CENTER, CENTER + Math.cos(a) * MAX_R * 1.32, CENTER + Math.sin(a) * MAX_R * 1.32,
                        ]} stroke={colors.accent2} fill={colors.accent2} strokeWidth={2.5}
                                      pointerLength={9} pointerWidth={8}/>
                    })()}
                </Layer>
            </Stage>
            <label className="flex items-center gap-2 text-xs text-muted w-full" style={{maxWidth: SIZE}}>
                <span className="whitespace-nowrap">{t("threshold", "threshold")}</span>
                <input type="range" min={0.05} max={maxBin * 0.95} step={0.01} value={threshold}
                       onChange={(e) => setThreshold(parseFloat(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("valley threshold", "valley threshold")}/>
            </label>
            <div className="text-xs text-muted text-center" style={{maxWidth: SIZE}}>
                <span style={{color: "var(--accent)"}} className="font-semibold">{t("goal", "goal")}</span>
                {" · "}
                <span style={{color: "var(--accent-2)"}} className="font-semibold">
                    {t("selected valley", "선택된 valley")}
                </span>
                {" · "}
                {selected
                    ? t("raise threshold to shrink or split the open valley", "threshold를 올리면 valley가 좁아지거나 갈라진다")
                    : t("no valley is open — every sector is at or above threshold", "열린 valley가 없다 — 모든 sector가 threshold 이상")}
            </div>
        </div>
    )
}

const VfhHistogramFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Two obstacle clusters carve two dips into the polar histogram. Raise the threshold (dashed ring) and the dips shrink into valleys, then close outright — VFH steers for whichever open valley sits closest to the goal bearing.",
            "장애물 뭉치 둘이 폴라 히스토그램에 골 두 개를 낸다. threshold(점선 원)를 올리면 그 골이 valley로 좁아지다가 결국 닫힌다. VFH는 goal 방향에 가장 가까운 열린 valley로 조향한다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default VfhHistogramFigure

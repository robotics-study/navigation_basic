import {useMemo, useState} from "react";
import {Arrow, Circle, Layer, Shape, Stage} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
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

const Scene = ({scale = 1}: {scale?: number}) => {
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
    // 조향 방향 — VFH 규칙: goal 방향 sector가 열린 valley 안이면 goal 로 직행,
    // 밖이면 valley 의 goal 쪽 경계에서 안쪽으로 살짝 들어간 방향.
    const steerIdx = useMemo(() => {
        if (!selected) return null
        if (contains(selected, GOAL_SECTOR)) return GOAL_SECTOR
        const inset = Math.min(2, (selected.width - 1) / 2)
        const dStart = circularDist(selected.start, GOAL_SECTOR)
        const dEnd = circularDist(selected.end, GOAL_SECTOR)
        return dStart <= dEnd
            ? (selected.start + inset + N) % N
            : (selected.end - inset + N) % N
    }, [selected])

    const sector = (2 * Math.PI) / N
    // world CCW 각(sector 0 = +x) → canvas 각: y축 반전 때문에 부호가 반대.
    const angleOf = (k: number) => -(k + 0.5) * sector

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={SIZE * scale} height={SIZE * scale} className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer scaleX={scale} scaleY={scale}>
                    {/* 히스토그램 wedge — threshold 이상(막힘)은 경고색, 미만(valley)은 회색 */}
                    {[false, true].map((blocked) => (
                        <Shape key={`w${blocked}`} listening={false} sceneFunc={(ctx, shape) => {
                            ctx.beginPath()
                            for (let k = 0; k < N; k++) {
                                if ((bins[k] >= threshold) !== blocked) continue
                                const r = BASE_R + (MAX_R - BASE_R) * (bins[k] / maxBin)
                                const a0 = -(k + 1) * sector
                                const a1 = -k * sector
                                ctx.moveTo(CENTER, CENTER)
                                ctx.arc(CENTER, CENTER, r, a0, a1, false)
                                ctx.closePath()
                            }
                            ctx.fillStrokeShape(shape)
                        }} fill={blocked ? PATH_COLOR : colors.muted} opacity={blocked ? 0.35 : 0.4}
                               stroke={colors.border} strokeWidth={0.5}/>
                    ))}
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
                    {/* 조향 방향 화살표 — goal 이 valley 안이면 goal 과 겹친다 */}
                    {steerIdx !== null && (() => {
                        const a = angleOf(steerIdx)
                        return <Arrow points={[
                            CENTER, CENTER, CENTER + Math.cos(a) * MAX_R * 1.32, CENTER + Math.sin(a) * MAX_R * 1.32,
                        ]} stroke={colors.accent2} fill={colors.accent2} strokeWidth={2.5}
                                      pointerLength={9} pointerWidth={8}/>
                    })()}
                </Layer>
            </Stage>
            <label className="flex items-center gap-2 text-xs text-muted w-full" style={{maxWidth: SIZE * scale}}>
                <span className="whitespace-nowrap">{t("threshold", "threshold")}</span>
                <input type="range" min={0.05} max={maxBin * 0.95} step={0.01} value={threshold}
                       onChange={(e) => setThreshold(parseFloat(e.target.value))}
                       className="flex-1 accent-[var(--accent)]"
                       aria-label={t("valley threshold", "valley threshold")}/>
            </label>
            {/* 색 범례 ("X = ..." 형식, 상태 문구와 분리해 오독 방지) + 상태 한 줄 */}
            <div className="text-xs text-muted text-center flex items-center justify-center gap-3 flex-wrap"
                 style={{maxWidth: SIZE * scale}}>
                {([
                    ["var(--accent)", t("blue arrow = goal bearing", "파란 화살표 = goal 방향")],
                    ["var(--accent-2)", t("teal = selected valley and steering", "청록 = 선택된 valley와 조향")],
                ] as const).map(([c, label]) => (
                    <span key={label} className="inline-flex items-center gap-1.5">
                        <span aria-hidden="true" className="inline-block w-2.5 h-2.5 rounded-sm"
                              style={{background: c}}/>
                        <span className="font-semibold" style={{color: c}}>{label}</span>
                    </span>
                ))}
            </div>
            <div className="text-xs text-muted text-center" style={{maxWidth: SIZE * scale}}>
                {selected
                    ? t("lower the threshold and the open valleys narrow, split, then close",
                        "threshold를 내리면 열린 valley가 좁아지고 갈라지다 결국 닫힌다")
                    : t("no valley is open: every sector's density is at or above the threshold",
                        "지금은 열린 valley가 없다. 모든 sector의 밀도가 threshold 이상이다")}
            </div>
        </div>
    )
}

const VfhHistogramFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Two obstacle clusters raise two humps in the polar histogram; sectors whose density stays below the threshold (dashed ring) form open valleys. VFH steers into the valley nearest the goal bearing — straight at the goal when its sector is open, otherwise just inside the valley's near border. Lower the threshold and the valleys narrow and close.",
            "장애물 뭉치 둘이 폴라 히스토그램에 봉우리 두 개를 세운다. 밀도가 threshold(점선 원) 미만인 sector들이 열린 valley다. VFH는 goal 방향에 가장 가까운 valley로 조향한다. goal sector가 열려 있으면 goal로 직행하고, 아니면 valley의 goal 쪽 경계 안쪽을 겨눈다. threshold를 내리면 valley가 좁아지다 닫힌다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(SIZE, SIZE)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default VfhHistogramFigure

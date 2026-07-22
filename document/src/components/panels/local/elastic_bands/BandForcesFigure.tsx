import {Fragment} from "react";
import {Arrow, Circle, Layer, Line, Rect, Stage} from "react-konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";

// 본문 "Forces on the Band"의 세 식(내부 수축력 f_c, 외부 반발력 f_r, 접선 제거 후
// 합력 f̃)을 고정 toy 밴드 위에 그대로 계산해 화살표로 그린다 -- elastic_bands.py의
// k_contraction=1.0, k_repulsion=2.0, rho_influence=1.0, rho_max=1.5 기본값을 그대로
// 쓴다(다른 해석식으로 대체하지 않는다). c1은 장애물에서 멀어 f_r이 0에 가깝고, c3은
// 장애물 뭉치에 바짝 붙어 f_r이 f_c를 압도한다 -- 두 경우를 나란히 보여준다.
const K_CONTRACTION = 1.0
const K_REPULSION = 2.0
const RHO_INFLUENCE = 1.0
const RHO_MAX = 1.5
const EPS_SQ = 1e-9

type Vec = [number, number]

// world(m) 밴드 -- 대각으로 나아가다 c2/c3 부근에서 장애물 뭉치를 스치는 7개 bubble.
const CENTERS: Vec[] = [
    [0.5, 1.0], [1.8, 1.6], [3.0, 2.4], [4.1, 2.9], [5.3, 3.3], [6.6, 3.7], [7.6, 4.2],
]
// occupied cell 중심 4개 -- c2/c3 진행 방향 위쪽에 뭉쳐 있어 반발이 아래로 민다.
const OBSTACLES: Vec[] = [[3.35, 2.85], [3.85, 2.85], [3.35, 3.35], [3.85, 3.35]]

const sub = (a: Vec, b: Vec): Vec => [a[0] - b[0], a[1] - b[1]]
const norm = (v: Vec): number => Math.sqrt(v[0] * v[0] + v[1] * v[1])
const unit = (v: Vec): Vec => {
    const n = norm(v)
    return n * n < EPS_SQ ? [0, 0] : [v[0] / n, v[1] / n]
}
const add = (a: Vec, b: Vec): Vec => [a[0] + b[0], a[1] + b[1]]
const scale = (v: Vec, s: number): Vec => [v[0] * s, v[1] * s]

// clearance rho(c) = min(최근접 occupied 점까지 거리, rho_max) -- _band.py의
// occupied_within 대신 이 toy에서는 명시적 점 4개까지의 최단 거리로 근사한다.
function clearance(c: Vec): number {
    let best = Infinity
    for (const o of OBSTACLES) best = Math.min(best, norm(sub(c, o)))
    return Math.min(best, RHO_MAX)
}

// elastic_bands.py의 힘 계산식 그대로: 내부 수축력(양 이웃 방향 단위벡터 합) + 외부
// 반발력(rho_influence 내 occupied 점 합산) + 접선 제거는 반발력에만.
function forcesAt(i: number): {fc: Vec; frRaw: Vec; frDetangented: Vec; resultant: Vec} {
    const c = CENTERS[i]
    let fc: Vec = [0, 0]
    for (const j of [i - 1, i + 1]) fc = add(fc, unit(sub(CENTERS[j], c)))
    fc = scale(fc, K_CONTRACTION)

    let fr: Vec = [0, 0]
    for (const o of OBSTACLES) {
        const d = norm(sub(c, o))
        if (d >= RHO_INFLUENCE || d * d < EPS_SQ) continue
        fr = add(fr, scale(unit(sub(c, o)), RHO_INFLUENCE - d))
    }
    fr = scale(fr, K_REPULSION)

    const t = unit(sub(CENTERS[i + 1], CENTERS[i - 1]))
    const proj = fr[0] * t[0] + fr[1] * t[1]
    const frDetangented: Vec = norm(t) < 1e-6 ? fr : [fr[0] - proj * t[0], fr[1] - proj * t[1]]

    return {fc, frRaw: fr, frDetangented, resultant: add(fc, frDetangented)}
}

const W = 340
const H = 240
const SCALE = 32
const MARGIN_X = 24
const MARGIN_Y = 30
const toPx = (v: Vec): Vec => [MARGIN_X + v[0] * SCALE, H - MARGIN_Y - v[1] * SCALE]

const Scene = () => {
    const colors = useCanvasColors()
    const interior = [1, 2, 3, 4, 5]
    const forces = interior.map((i) => ({i, ...forcesAt(i)}))

    const arrow = (from: Vec, vec: Vec, color: string, width: number, key: string, arrowScale: number) => {
        const mag = norm(vec)
        if (mag < 1e-6) return null
        const [x0, y0] = toPx(from)
        const len = Math.min(60, arrowScale * mag)
        const [ux, uy] = unit(vec)
        // world 벡터(x,y) -> canvas: x는 그대로, y는 반전.
        return <Arrow key={key} points={[x0, y0, x0 + ux * len, y0 - uy * len]}
                      pointerLength={7} pointerWidth={6} stroke={color} fill={color}
                      strokeWidth={width} opacity={0.9}/>
    }

    return (
        <Stage width={W} height={H} className="bg-surface border border-border rounded-lg overflow-hidden">
            <Layer>
                {OBSTACLES.map(([x, y], i) => {
                    const [px, py] = toPx([x, y])
                    return <Rect key={`o${i}`} x={px - 8} y={py - 8} width={16} height={16}
                                 cornerRadius={2} fill={colors.text} opacity={0.75}/>
                })}
                <Line points={CENTERS.flatMap((c) => toPx(c))} stroke={colors.accent}
                      strokeWidth={2} opacity={0.35} lineCap="round" lineJoin="round"/>
                {CENTERS.map((c, i) => {
                    const [px, py] = toPx(c)
                    const rPx = clearance(c) * SCALE
                    // Konva Circle은 fill/stroke가 opacity 하나를 공유하므로, 저알파 채움과
                    // 불투명 테두리를 따로 그린다(LocalTracePlayer의 bandOverlay와 같은 이유).
                    return <Fragment key={`b${i}`}>
                        <Circle x={px} y={py} radius={rPx} fill={colors.accent} opacity={0.08}/>
                        <Circle x={px} y={py} radius={rPx} stroke={colors.accent} strokeWidth={1} opacity={0.5}/>
                    </Fragment>
                })}
                {forces.map(({i, fc, frDetangented, resultant}) => (
                    <Fragment key={`f${i}`}>
                        {arrow(CENTERS[i], fc, "#2563eb", 2, `fc${i}`, 30)}
                        {arrow(CENTERS[i], frDetangented, "#dc2626", 2, `fr${i}`, 30)}
                        {arrow(CENTERS[i], resultant, colors.text, 2.6, `res${i}`, 30)}
                    </Fragment>
                ))}
                {CENTERS.map((c, i) => {
                    const [px, py] = toPx(c)
                    return <Circle key={`c${i}`} x={px} y={py} radius={3.5} fill={colors.accent}
                                   stroke={colors.bg} strokeWidth={1}/>
                })}
            </Layer>
        </Stage>
    )
}

const BandForcesFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Contraction (blue), repulsion after tangent removal (red), and their sum f̃ (black) computed with the force equations above on a fixed toy band — bubbles far from the obstacle cluster (left) feel almost pure contraction, the one grazing it (right of center) gets pushed clear by repulsion",
            "위 힘 수식 그대로 고정 toy 밴드 위에서 계산한 수축력(파랑), 접선 제거 후 반발력(빨강), 그 합 f̃(검정). 장애물 뭉치에서 먼 bubble(왼쪽)은 거의 순수한 수축력만 느끼고, 뭉치를 스치는 bubble(가운데 오른쪽)은 반발력에 밀려난다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default BandForcesFigure

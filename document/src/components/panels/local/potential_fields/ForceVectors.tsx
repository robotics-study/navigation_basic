import {useMemo, useState} from "react";
import {Arrow, Circle, Layer, Line, Stage, Text} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";
import {PATH_COLOR} from "../../../2d/GridCanvas";

// Khatib(1986)의 힘 합산을 맵 전체 위에 그린 벡터장 그림. 배경 quiver는 격자마다
// F = F_att + F_rep 를 본문과 똑같은 식으로 계산해 방향 화살표로 깔고(밀도는 |F|에
// 비례한 진하기), 그 위에 자유롭게 끌 수 있는 로봇 한 점의 인력·반발·합력 분해를
// 겹친다 — 지어낸 모양이 아니라 같은 식의 표본이라, 장애물을 끌면 장 전체가 그것을
// 따라 휘고, 로봇을 끌면 분해가 그 자리의 힘으로 바뀐다. 색 관례는 replay.py 그대로
// (인력 #2563eb / 반발 #dc2626), 배경 장은 중립 회색이라 분해 화살표가 도드라진다.
type Vec = [number, number];

const K_ATT = 0.32;                    // 인력 게인 (거리 선형)
const K_REP = 0.9;                     // 반발 게인 (FIRAS)
const RHO0 = 1.7;                      // m, 반발 영향 반경
const D_MIN = 0.22;                    // m, 접촉 클램프 (footprint)

const WORLD_W = 9;                     // m
const WORLD_H = 7;                     // m
const OBST_INIT: Vec = [4.4, 3.9];     // 장애물 초기 중심 (m)
const OBST_R = 0.5;                    // m, 장애물 반경 (표면까지 거리로 반발 계산)
const GOAL: Vec = [7.8, 5.7];          // m
const ROBOT_INIT: Vec = [3.5, 2.8];    // m, 로봇 초기 위치

// 본문 두 식 그대로: 인력은 목표까지 선형, 반발은 최근접 장애물 표면까지 거리 d의
// FIRAS. 반발은 원형 장애물의 표면까지 거리(|p-c| - R)로 계산해 셀 합산을 한 점으로
// 요약한다 — 방향·발산 형태는 본문의 셀 합산과 동일하다.
function fieldAt(p: Vec, obst: Vec): {att: Vec; rep: Vec; res: Vec} {
    const att: Vec = [K_ATT * (GOAL[0] - p[0]), K_ATT * (GOAL[1] - p[1])]
    const cx = p[0] - obst[0]
    const cy = p[1] - obst[1]
    const dc = Math.hypot(cx, cy)
    const d = Math.max(dc - OBST_R, D_MIN)
    let rep: Vec = [0, 0]
    if (d < RHO0 && dc > 1e-6) {
        const mag = K_REP * (1 / d - 1 / RHO0) * (1 / (d * d))
        rep = [(mag * cx) / dc, (mag * cy) / dc]
    }
    return {att, rep, res: [att[0] + rep[0], att[1] + rep[1]]}
}

const norm = (v: Vec) => Math.hypot(v[0], v[1])

const NX = 17
const NY = 13

const BASE_W = 360
const BASE_H = BASE_W * (WORLD_H / WORLD_W)
const SCALE = BASE_W / WORLD_W        // px per m

const Scene = ({scale = 1}: {scale?: number}) => {
    const t = useTr()
    const colors = useCanvasColors()
    const [obst, setObst] = useState<Vec>(OBST_INIT)
    const [robot, setRobot] = useState<Vec>(ROBOT_INIT)

    // 배경 quiver 격자 — 셀 중심에서 F를 표본화한다. 장애물 안쪽 셀은 건너뛴다.
    // 장애물이 드래그로 움직이므로 장 전체를 그 위치에 대해 다시 계산한다.
    const quiver = useMemo(() => {
        const cw = WORLD_W / NX
        const ch = WORLD_H / NY
        const cells: {p: Vec; res: Vec; mag: number}[] = []
        let maxMag = 1e-6
        for (let iy = 0; iy < NY; iy++) {
            for (let ix = 0; ix < NX; ix++) {
                const p: Vec = [(ix + 0.5) * cw, (iy + 0.5) * ch]
                if (Math.hypot(p[0] - obst[0], p[1] - obst[1]) < OBST_R + 0.06) continue
                const {res} = fieldAt(p, obst)
                const mag = norm(res)
                maxMag = Math.max(maxMag, mag)
                cells.push({p, res, mag})
            }
        }
        return {cells, maxMag}
    }, [obst])

    const {att, rep, res} = fieldAt(robot, obst)

    const toPx = (p: Vec): Vec => [p[0] * SCALE, BASE_H - p[1] * SCALE]
    const fromPx = (px: number, py: number): Vec => [px / SCALE, (BASE_H - py) / SCALE]
    const clampWorld = (p: Vec, margin: number): Vec => [
        Math.min(WORLD_W - margin, Math.max(margin, p[0])),
        Math.min(WORLD_H - margin, Math.max(margin, p[1])),
    ]

    // 분해 화살표 — 인력은 길이 고정 비례, 반발/합력은 크기에 비례하되 클램프.
    const arrowScale = 26
    const clampLen = (v: Vec, maxLen: number): Vec => {
        const len = norm(v) * arrowScale
        const l = Math.min(len, maxLen)
        const n = norm(v)
        return n < 1e-6 ? [0, 0] : [(v[0] / n) * l, (v[1] / n) * l]
    }
    const decomp = (v: Vec, color: string, width: number, key: string, maxLen: number) => {
        const [dx, dy] = clampLen(v, maxLen)
        if (Math.hypot(dx, dy) < 1.5) return null
        const [x0, y0] = toPx(robot)
        // world y up -> canvas y down: y 성분 반전.
        return <Arrow key={key} points={[x0, y0, x0 + dx, y0 - dy]}
                      pointerLength={9} pointerWidth={8} stroke={color} fill={color} strokeWidth={width}/>
    }

    const [rx, ry] = toPx(robot)
    const [gx, gy] = toPx(GOAL)
    const [ox, oy] = toPx(obst)

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={BASE_W * scale} height={BASE_H * scale}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer scaleX={scale} scaleY={scale}>
                    {/* 반발 영향 반경 (장애물 표면 + ρ₀) */}
                    <Circle x={ox} y={oy} radius={(OBST_R + RHO0) * SCALE}
                            stroke={colors.muted} strokeWidth={1} dash={[4, 4]} opacity={0.4}
                            listening={false}/>
                    {/* 배경 벡터장 (quiver): F 방향, 진하기 ∝ |F| */}
                    {quiver.cells.map(({p, res: r, mag}, i) => {
                        const [px, py] = toPx(p)
                        const n = norm(r)
                        if (n < 1e-6) return null
                        const len = 12
                        const ux = (r[0] / n) * len
                        const uy = (r[1] / n) * len
                        const op = 0.16 + 0.5 * (mag / quiver.maxMag)
                        return <Arrow key={`q${i}`}
                                      points={[px - ux / 2, py + uy / 2, px + ux / 2, py - uy / 2]}
                                      pointerLength={4} pointerWidth={3.5}
                                      stroke={colors.muted} fill={colors.muted} strokeWidth={1.1}
                                      opacity={op} listening={false}/>
                    })}
                    {/* 로봇 -> 목표 직선(참조) */}
                    <Line points={[rx, ry, gx, gy]} stroke={colors.muted} strokeWidth={1.2}
                          dash={[5, 4]} opacity={0.5} listening={false}/>
                    {/* 로봇 한 점의 분해: 인력(파랑) / 반발(빨강) / 합력(검정) */}
                    {decomp(att, "#2563eb", 2.4, "att", 78)}
                    {decomp(rep, "#dc2626", 2.4, "rep", 92)}
                    {decomp(res, colors.text, 3.4, "res", 96)}
                    {/* goal */}
                    <Circle x={gx} y={gy} radius={7} fill={PATH_COLOR} stroke={colors.bg} strokeWidth={1.5}/>
                    <Text x={gx + 9} y={gy - 7} text={t("goal", "목표")} fontSize={12}
                          fill={PATH_COLOR} fontStyle="bold"/>
                    {/* 장애물 — 드래그하면 장 전체가 따라 휜다. 드롭 좌표를 world로 되돌려
                        상태에 반영하고 노드 위치는 리셋한다(다음 렌더가 상태 기준으로 그린다). */}
                    <Circle x={ox} y={oy} radius={OBST_R * SCALE} fill={colors.text} opacity={0.78}
                            draggable hitStrokeWidth={14}
                            onDragMove={(e) => {
                                setObst(clampWorld(fromPx(e.target.x(), e.target.y()), OBST_R + 0.15))
                            }}
                            onMouseEnter={(e) => {
                                const stage = e.target.getStage()
                                if (stage) stage.container().style.cursor = "grab"
                            }}
                            onMouseLeave={(e) => {
                                const stage = e.target.getStage()
                                if (stage) stage.container().style.cursor = ""
                            }}/>
                    {/* 로봇 — 자유 드래그. 어느 자리에 놓아도 그 자리의 힘 분해가 그려진다. */}
                    <Circle x={rx} y={ry} radius={7} fill={colors.accent2} stroke={colors.bg} strokeWidth={1.5}
                            draggable hitStrokeWidth={14}
                            onDragMove={(e) => {
                                setRobot(clampWorld(fromPx(e.target.x(), e.target.y()), 0.25))
                            }}
                            onMouseEnter={(e) => {
                                const stage = e.target.getStage()
                                if (stage) stage.container().style.cursor = "grab"
                            }}
                            onMouseLeave={(e) => {
                                const stage = e.target.getStage()
                                if (stage) stage.container().style.cursor = ""
                            }}/>
                    {/* 범례 */}
                    <Text x={12} y={12} text="F_att" fontSize={11} fill="#2563eb" fontStyle="bold"/>
                    <Text x={12} y={28} text="F_rep" fontSize={11} fill="#dc2626" fontStyle="bold"/>
                    <Text x={12} y={44} text="F = F_att + F_rep" fontSize={11} fill={colors.text} fontStyle="bold"/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center" style={{maxWidth: BASE_W * scale}}>
                {t(
                    "drag the dark obstacle and the whole field bends around wherever you put it; drag the teal robot and the arrows decompose the force at that exact spot — push the robot near the obstacle to watch repulsion (red) overwhelm the pull (blue)",
                    "검은 장애물을 끌면 장 전체가 그 위치를 따라 휜다. 청록 로봇을 끌면 바로 그 자리의 힘이 화살표로 분해된다. 로봇을 장애물 가까이 밀어 넣으면 반발(빨강)이 인력(파랑)을 압도하는 것이 보인다",
                )}
            </div>
        </div>
    )
}

const ForceVectorsFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The full artificial force field: at every grid point the attractive pull toward the goal plus FIRAS repulsion from the obstacle sum to the resultant the robot would follow. Both the obstacle and the robot are draggable — the field and the F_att (blue) / F_rep (red) / F (black) decomposition update live.",
            "인공 힘의 장 전체. 격자마다 목표로의 인력과 장애물의 FIRAS 반발을 더한 합력이 로봇이 따라갈 방향이다. 장애물과 로봇 모두 끌 수 있고, 장과 F_att(파랑)/F_rep(빨강)/F(검정) 분해가 실시간으로 갱신된다.",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(BASE_W, BASE_H)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default ForceVectorsFigure

import {useState} from "react";
import {Circle, Layer, Line, Rect, Stage, Text} from "react-konva";
import CanvasFigure from "../../CanvasFigure";
import {useCanvasColors} from "../../../libs/useTheme";
import {PATH_COLOR} from "../../2d/GridCanvas";
import {useTr} from "../../../libs/i18n";

// local planning 의 핵심 그림: 로봇이 (v, ω) 후보 궤적(호)들을 전방 시뮬레이션하고
// 충돌·진행도로 채점한다. 장애물을 끌면 채점이 즉시 바뀐다 (DWA 풍).
const W = 320;
const H = 300;
const ROBOT = {x: W / 2, y: H - 34};
const HORIZON = 150;                 // 호 길이(px) — 전방 시뮬레이션 지평
const CURVATURES = [-3.2, -2.2, -1.4, -0.7, 0, 0.7, 1.4, 2.2, 3.2];  // 1/반경 스케일

// 곡률 k 의 호를 점열로 편다 (로봇 기준 위쪽 진행).
const arcPoints = (k: number): number[] => {
    const pts: number[] = []
    const steps = 24
    for (let i = 0; i <= steps; i++) {
        const s = (i / steps) * HORIZON
        if (Math.abs(k) < 1e-6) {
            pts.push(ROBOT.x, ROBOT.y - s)
        } else {
            const r = HORIZON / k
            const a = s / Math.abs(r)
            const sign = Math.sign(k)
            pts.push(
                ROBOT.x + sign * (Math.abs(r) - Math.abs(r) * Math.cos(a)),
                ROBOT.y - Math.abs(r) * Math.sin(a),
            )
        }
    }
    return pts
}

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const [obs, setObs] = useState({x: W / 2 + 30, y: H / 2 - 20})
    const OBS_R = 30

    const arcs = CURVATURES.map((k) => {
        const pts = arcPoints(k)
        // 충돌: 호 위의 점이 장애물 반경 안이면 그 지점부터 잘라 낸다.
        let cut = pts.length
        for (let i = 0; i < pts.length; i += 2) {
            if (Math.hypot(pts[i] - obs.x, pts[i + 1] - obs.y) < OBS_R + 7) {
                cut = i
                break
            }
        }
        const collided = cut < pts.length
        // 점수: 지평 도달 거리(클수록 좋음) + 직진 선호. 충돌 호는 탈락.
        const progress = (cut / pts.length) * (1 - Math.abs(k) * 0.06)
        return {k, pts: pts.slice(0, Math.max(cut, 4)), collided, progress}
    })
    const best = arcs.reduce((a, b) =>
        (!b.collided && b.progress > (a.collided ? -1 : a.progress)) ? b : a)

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W} height={H}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer>
                    {/* global path (참조 경로) */}
                    <Line points={[ROBOT.x, ROBOT.y, ROBOT.x, -10]} stroke={colors.border}
                          strokeWidth={2} dash={[6, 6]}/>
                    {arcs.map((a) => (
                        <Line key={a.k} points={a.pts}
                              stroke={a.collided ? PATH_COLOR : colors.accent}
                              strokeWidth={a === best ? 3.4 : 1.4}
                              opacity={a.collided ? 0.4 : a === best ? 1 : 0.55}
                              lineCap="round"/>
                    ))}
                    <Circle x={obs.x} y={obs.y} radius={OBS_R} fill={colors.muted} opacity={0.8}
                            draggable
                            onDragMove={(e) => setObs({
                                x: Math.max(OBS_R, Math.min(W - OBS_R, e.target.x())),
                                y: Math.max(OBS_R, Math.min(H - OBS_R - 50, e.target.y())),
                            })}/>
                    <Rect x={ROBOT.x - 9} y={ROBOT.y - 9} width={18} height={18} cornerRadius={4}
                          fill={colors.text}/>
                    <Text x={8} y={H - 20} text={t("drag the obstacle", "장애물을 끌어 보라")}
                          fontSize={11} fill={colors.muted}/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center tabular-nums">
                {t("candidates", "후보")} {arcs.length}
                {" · "}
                <span style={{color: "var(--accent)"}} className="font-semibold">
                    {t("best arc", "최선 호")}
                </span>
                {" · "}
                <span style={{color: PATH_COLOR}} className="font-semibold">
                    {t("collision", "충돌")}
                </span>
            </div>
        </div>
    )
}

const ArcCandidates = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The local planner's move: forward-simulate a fan of (v, ω) candidates, discard colliding ones, pick the best-scoring arc",
            "local planner 의 한 수. (v, ω) 후보 부채꼴을 전방 시뮬레이션하고, 충돌 호를 버리고, 최고 점수 호를 고른다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default ArcCandidates

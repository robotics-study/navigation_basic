import {useMemo, useState} from "react";
import {Circle, Layer, Line, Rect, Stage} from "react-konva";
import Konva from "konva";
import CanvasFigure from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {lineOfSight} from "../../../../libs/algorithms/theta_star";
import {Cell} from "../../../../libs/trace/timeline";
import {useTr} from "../../../../libs/i18n";
import {FAN_N, FAN_SOURCE, fanMap} from "./presets";

// Visibility A*의 successor 관계를 그대로 보여 주는 figure: 한 셀에서 supercover
// LOS로 보이는 모든 셀이 그 셀의 이웃이다. 소스를 옮기면 가시 영역(과 벽 뒤
// 그림자)이 통째로 바뀐다.
const PANEL = 320;

const Scene = () => {
    const colors = useCanvasColors()
    const t = useTr()
    const map = useMemo(fanMap, [])
    const [source, setSource] = useState<Cell>(FAN_SOURCE)
    const cell = PANEL / FAN_N

    const visible = useMemo(() => {
        const out: Cell[] = []
        for (let r = 0; r < FAN_N; r++) {
            for (let c = 0; c < FAN_N; c++) {
                if (map.occupied[r * FAN_N + c]) continue
                if (r === source[0] && c === source[1]) continue
                if (lineOfSight(map, source, [r, c])) out.push([r, c])
            }
        }
        return out
    }, [map, source])

    const pick = (stage: Konva.Stage | null) => {
        const pos = stage?.getPointerPosition()
        if (!pos) return
        const col = Math.floor(pos.x / cell)
        const row = Math.floor(pos.y / cell)
        if (row < 0 || row >= FAN_N || col < 0 || col >= FAN_N) return
        if (map.occupied[row * FAN_N + col]) return
        setSource([row, col])
    }

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={PANEL} height={PANEL}
                   className="bg-surface border border-border rounded-lg overflow-hidden"
                   onPointerDown={(e) => pick(e.target.getStage())}
                   onPointerMove={(e) => {
                       if (e.evt.buttons === 1) pick(e.target.getStage())
                   }}>
                <Layer>
                    {visible.map((c, i) => (
                        <Rect key={i} x={c[1] * cell} y={c[0] * cell}
                              width={cell} height={cell} fill={colors.accent} opacity={0.26}/>
                    ))}
                    {map.occupied.map((occ, i) => occ && (
                        <Rect key={`w${i}`} x={(i % FAN_N) * cell} y={Math.floor(i / FAN_N) * cell}
                              width={cell} height={cell} fill={colors.text} opacity={0.78}/>
                    ))}
                    {Array.from({length: FAN_N + 1}, (_, k) => (
                        <Line key={`gv${k}`} points={[k * cell, 0, k * cell, PANEL]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                    ))}
                    {Array.from({length: FAN_N + 1}, (_, k) => (
                        <Line key={`gh${k}`} points={[0, k * cell, PANEL, k * cell]}
                              stroke={colors.border} strokeWidth={0.5} opacity={0.6}/>
                    ))}
                    <Circle x={(source[1] + 0.5) * cell} y={(source[0] + 0.5) * cell}
                            radius={cell * 0.36} fill={colors.accent2}
                            stroke={colors.bg} strokeWidth={1.5}/>
                </Layer>
            </Stage>
            <div className="text-xs text-muted text-center tabular-nums">
                {t("visible cells", "보이는 셀")}{" "}
                <span className="font-semibold" style={{color: "var(--accent)"}}>{visible.length}</span>
                {" · "}{t("click or drag to move the source", "클릭/드래그로 소스를 옮겨 보라")}
            </div>
        </div>
    )
}

const VisibilityFan = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The successor set of one expansion: every cell with line of sight becomes a neighbor at straight-line cost — walls cast shadows the search cannot cross in one step",
            "확장 한 번의 successor 집합. line of sight가 닿는 모든 셀이 직선거리 비용의 이웃이 되고, 벽은 한 스텝에 넘을 수 없는 그림자를 드리운다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default VisibilityFan

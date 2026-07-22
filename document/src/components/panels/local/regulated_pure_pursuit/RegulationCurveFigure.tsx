import {Layer, Line, Stage, Text} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";

// 두 규제(곡률/근접)를 하나의 v축 위에 겹쳐 그린 정적 램프 도해 —
// configs/local_planning/regulated_pure_pursuit.yaml 기본값(r_min=0.9, d_prox=0.6,
// v_max=0.8)을 그대로 쓴다. 둘 다 "0에서 선형으로 올라가 임계값에서 v_max에
// 닿고 그 뒤로는 평평"이라는 같은 모양이지만, 서로 다른 임계값에서 v_max에
// 닿아 두 규제가 독립적으로 작동함을 한눈에 보여준다.
const PANEL_W = 380;
const PANEL_H = 260;
const MARGIN = {left: 42, right: 16, top: 18, bottom: 36};
const PLOT_W = PANEL_W - MARGIN.left - MARGIN.right;
const PLOT_H = PANEL_H - MARGIN.top - MARGIN.bottom;

const MAX_SPEED = 0.8;
const R_MIN = 0.9;
const D_PROX = 0.6;
const X_MAX = 1.3;
const Y_AXIS_MAX = MAX_SPEED * 1.2;

const mapX = (v: number) => MARGIN.left + (v / X_MAX) * PLOT_W;
const mapY = (v: number) => MARGIN.top + PLOT_H - (v / Y_AXIS_MAX) * PLOT_H;

// (0,0) -> (threshold, v_max) 선형 램프 후 x_max까지 평평.
const rampPoints = (threshold: number): number[] => [
    mapX(0), mapY(0),
    mapX(threshold), mapY(MAX_SPEED),
    mapX(X_MAX), mapY(MAX_SPEED),
];

const Scene = ({scale = 1}: {scale?: number}) => {
    const t = useTr()
    const colors = useCanvasColors()
    const axisY0 = mapY(0)
    const axisX0 = mapX(0)

    return (
        <Stage width={PANEL_W * scale} height={PANEL_H * scale}
               className="bg-surface border border-border rounded-lg overflow-hidden">
            <Layer scaleX={scale} scaleY={scale}>
                {/* 축 */}
                <Line points={[axisX0, MARGIN.top, axisX0, MARGIN.top + PLOT_H]}
                      stroke={colors.muted} strokeWidth={1.3}/>
                <Line points={[axisX0, axisY0, MARGIN.left + PLOT_W, axisY0]}
                      stroke={colors.muted} strokeWidth={1.3}/>

                {/* v_max 보조선 */}
                <Line points={[axisX0, mapY(MAX_SPEED), MARGIN.left + PLOT_W, mapY(MAX_SPEED)]}
                      stroke={colors.muted} strokeWidth={1} dash={[3, 4]} opacity={0.55}/>
                <Text x={4} y={mapY(MAX_SPEED) - 6} text="v_max" fontSize={11} fill={colors.muted}/>

                {/* 임계값 세로 보조선 */}
                <Line points={[mapX(R_MIN), axisY0, mapX(R_MIN), mapY(MAX_SPEED)]}
                      stroke={colors.accent} strokeWidth={1} dash={[3, 4]} opacity={0.55}/>
                <Line points={[mapX(D_PROX), axisY0, mapX(D_PROX), mapY(MAX_SPEED)]}
                      stroke={colors.accent2} strokeWidth={1} dash={[3, 4]} opacity={0.55}/>

                {/* 두 규제 램프 */}
                <Line points={rampPoints(R_MIN)} stroke={colors.accent} strokeWidth={2.6}
                      lineCap="round" lineJoin="round"/>
                <Line points={rampPoints(D_PROX)} stroke={colors.accent2} strokeWidth={2.6}
                      lineCap="round" lineJoin="round"/>

                {/* 축 라벨 (최소) */}
                <Text x={mapX(R_MIN) - 14} y={axisY0 + 8} text="r_min" fontSize={11} fill={colors.accent}/>
                <Text x={mapX(D_PROX) - 16} y={axisY0 + 22} text="d_prox" fontSize={11} fill={colors.accent2}/>
                <Text x={MARGIN.left + PLOT_W - 60} y={axisY0 + 8} text={t("r, d (m)", "r, d (m)")}
                      fontSize={11} fill={colors.muted}/>
                <Text x={axisX0 - 34} y={MARGIN.top - 4} text="v (m/s)" fontSize={11} fill={colors.muted}/>

                {/* 범례 */}
                <Line points={[MARGIN.left + PLOT_W - 96, MARGIN.top + 6, MARGIN.left + PLOT_W - 76, MARGIN.top + 6]}
                      stroke={colors.accent} strokeWidth={2.6}/>
                <Text x={MARGIN.left + PLOT_W - 72} y={MARGIN.top} text={t("curvature: r -> v", "곡률: r -> v")}
                      fontSize={10} fill={colors.text}/>
                <Line points={[MARGIN.left + PLOT_W - 96, MARGIN.top + 20, MARGIN.left + PLOT_W - 76, MARGIN.top + 20]}
                      stroke={colors.accent2} strokeWidth={2.6}/>
                <Text x={MARGIN.left + PLOT_W - 72} y={MARGIN.top + 14} text={t("proximity: d -> v", "근접: d -> v")}
                      fontSize={10} fill={colors.text}/>
            </Layer>
        </Stage>
    )
}

const RegulationCurveFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Both regulations are the same linear ramp shape — speed falls off to zero as radius or clearance shrinks toward zero, and reaches v_max once the radius clears r_min (curvature) or the clearance clears d_prox (proximity)",
            "두 규제 모두 같은 선형 램프 모양이다. 회전 반경이나 장애물 거리가 0에 가까워질수록 속도도 0에 가까워지고, 반경이 r_min(곡률)을 넘거나 거리가 d_prox(근접)를 넘으면 v_max에 도달한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(PANEL_W, PANEL_H)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default RegulationCurveFigure

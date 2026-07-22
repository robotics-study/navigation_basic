import {useMemo} from "react";
import {Circle, Layer, Line, Stage} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
import {runTeb} from "../../../../libs/algorithms/teb";
import {GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {useCanvasColors} from "../../../../libs/useTheme";
import {useTr} from "../../../../libs/i18n";

// 실제 teb.ts의 runTeb를 그대로 호출한다(내부 gradientStep을 따로 export하지 않으므로
// 반복 횟수만 다르게 준 세 번의 온전한 실행에서 첫 tick의 band_updated를 뽑는다) --
// 지어낸 폴리라인이 아니라 진짜 최적화 결과다. 90도 코너를 도는 toy 참조 경로에
// iterations=0(재초기화 직후, 미최적화)/10(부분 수렴)/40(기본값, 완전 수렴)을 주면
// 코너가 안쪽으로 당겨지고 ΔT가 줄어드는 과정이 그대로 드러난다.
const TOY_MAP: GridMap = {
    name: "teb_toy", width: 12, height: 12, resolution: 0.5, originX: 0, originY: 0,
    occupied: (() => {
        const occ = new Array(12 * 12).fill(false)
        for (let row = 0; row < 12; row++) {
            for (let col = 0; col < 12; col++) {
                if (row === 0 || row === 11 || col === 0 || col === 11) occ[row * 12 + col] = true
            }
        }
        return occ
    })(),
}
const TOY_PATH: Point[] = [[0.75, 0.75], [0.75, 3.5], [3.5, 3.5]]
const TOY_START: Pose = [0.75, 0.75, Math.PI / 2]
const TOY_GOAL: [number, number, number] = [3.5, 3.5, 0]

const TOY_FIXED = {
    map: TOY_MAP, startPose: TOY_START, goal: TOY_GOAL, referencePath: TOY_PATH,
    wPath: 1.0, wObstacle: 15.0, wVelocity: 10.0, wAcceleration: 5.0, wKinematics: 50.0,
    maxSpeed: 0.8, maxOmega: 1.5, aMax: 1.5, minObstacleDist: 0.8, dtRef: 0.85, dtMin: 0.05,
    horizon: 4.0, stepAlpha: 0.02, maxStepXy: 0.05, maxStepTheta: 0.1, maxStepDt: 0.02,
    maxPoses: 40, reinitDistance: 1.0, controlDt: 0.1, maxSteps: 1, goalTolerance: 0.3,
    footprintRadius: 0.2, stallWindow: 20, stallDistance: 0.05,
}

// 첫 tick(재초기화 직후 이번 tick 최적화까지 마친 상태)의 band_updated 하나만 필요하므로
// maxSteps=1로 폐루프를 1 tick만 돌리고 그 이벤트를 뽑는다.
function firstBand(iterations: number): number[][] {
    const events = runTeb({...TOY_FIXED, wTime: 1.0, iterations})
    const ev = events.find((e) => e.event === "band_updated")
    return ev?.band ?? []
}

const W = 300
const H = 300
// 콘텐츠(0.75~3.5 코너 경로 + 밴드 흔들림 여유)를 캔버스에 꽉 차게 맞춘 world 창.
const WORLD_MIN = 0.35
const WORLD_MAX = 3.9
const MARGIN = 16
const SCALE = (W - 2 * MARGIN) / (WORLD_MAX - WORLD_MIN)
const toPx = (x: number, y: number): [number, number] =>
    [MARGIN + (x - WORLD_MIN) * SCALE, H - MARGIN - (y - WORLD_MIN) * SCALE]

// 세 반복 횟수의 band는 테마와 무관한 순수 계산이므로, 컴포넌트 밖에서 한 번만 구한다
// (테마 토글마다 runTeb를 다시 돌릴 이유가 없다).
const BANDS_BY_ITERATIONS = [0, 10, 40].map((iterations) => ({iterations, band: firstBand(iterations)}))

const Scene = ({scale = 1}: {scale?: number}) => {
    const colors = useCanvasColors()
    const bands = useMemo(() => ([
        {...BANDS_BY_ITERATIONS[0], color: colors.muted},
        {...BANDS_BY_ITERATIONS[1], color: colors.accent},
        {...BANDS_BY_ITERATIONS[2], color: colors.accent2},
    ]), [colors])

    return (
        <Stage width={W * scale} height={H * scale} className="bg-surface border border-border rounded-lg overflow-hidden">
            <Layer scaleX={scale} scaleY={scale}>
                <Line points={TOY_PATH.flatMap(([x, y]) => toPx(x, y))} stroke={colors.muted}
                      strokeWidth={1.4} dash={[5, 4]} opacity={0.6}/>
                {bands.map(({iterations, band, color}) => {
                    if (band.length === 0) return null
                    const dts = band.map((p) => p[3] ?? 0)
                    const dtMax = Math.max(...dts, 1e-9)
                    return band.slice(0, -1).map((p, i) => {
                        const dt = dts[i + 1] ?? 0
                        const sw = 1 + 3.2 * (dt / dtMax)
                        const [x0, y0] = toPx(p[0], p[1])
                        const [x1, y1] = toPx(band[i + 1][0], band[i + 1][1])
                        return <Line key={`b${iterations}-${i}`} points={[x0, y0, x1, y1]}
                                     stroke={color} strokeWidth={sw} opacity={0.85} lineCap="round"/>
                    })
                })}
                {bands.map(({iterations, band, color}) => band.map((p, i) => {
                    const [px, py] = toPx(p[0], p[1])
                    return <Circle key={`p${iterations}-${i}`} x={px} y={py} radius={2.4} fill={color}/>
                }))}
            </Layer>
        </Stage>
    )
}

const TebOptimizationFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "The same 90° toy corner optimized by the real teb.ts solver at 0 (gray), 10 (teal), and 40 (bright, default) gradient-descent iterations — the pose chain pulls tighter into the corner and segment thickness (∝ ΔT) shrinks as it converges",
            "실제 teb.ts 솔버로 같은 90도 toy 코너를 gradient descent 0회(회색)/10회(청록)/40회(밝음, 기본값) 반복 최적화한 결과. 수렴할수록 pose 열이 코너 안쪽으로 당겨지고 세그먼트 굵기(∝ΔT)가 줄어든다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(W, H)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default TebOptimizationFigure

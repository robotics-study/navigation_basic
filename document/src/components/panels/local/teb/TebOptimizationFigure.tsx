import {useMemo, useState} from "react";
import {Circle, Layer, Line, Stage} from "react-konva";
import CanvasFigure, {modalScale} from "../../../CanvasFigure";
import ParamSlider from "../../../player/ParamSlider";
import {runTeb} from "../../../../libs/algorithms/teb";
import {GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {useCanvasColors} from "../../../../libs/useTheme";
import {T, useTr} from "../../../../libs/i18n";

// 실제 teb.ts의 runTeb를 그대로 호출한다(내부 gradientStep을 따로 export하지 않으므로
// 반복 횟수만 다르게 준 온전한 실행에서 첫 tick의 band_updated를 뽑는다) --
// 지어낸 폴리라인이 아니라 진짜 최적화 결과다. 90도 코너를 도는 toy 참조 경로에
// 반복 횟수를 슬라이더로 바꾸면, 코너가 안쪽으로 당겨지고 계획 시간 Σ ΔT가 줄어드는
// 과정이 그대로 드러난다.
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

// 슬라이더가 밟는 반복 횟수들. 밴드는 테마와 무관한 순수 계산이므로 컴포넌트 밖에서
// 한 번만 전부 구해 둔다(슬라이더 이동마다 runTeb를 다시 돌릴 이유가 없다).
const ITER_STEP = 5
const ITER_MAX = 40
const BANDS_BY_ITERATIONS = new Map(
    Array.from({length: ITER_MAX / ITER_STEP + 1}, (_, i) => i * ITER_STEP)
        .map((iterations) => [iterations, firstBand(iterations)] as const),
)
// entry 0의 dt는 placeholder(이전 세그먼트 없음)라 0이고, 합에는 자연히 기여하지 않는다.
const totalTime = (band: number[][]): number => band.reduce((s, p) => s + (p[3] ?? 0), 0)
// 세그먼트 굵기는 모든 반복 횟수에 대해 같은 기준(전역 최대 ΔT)으로 정규화한다.
// 밴드별 정규화는 ΔT가 전부 함께 줄어들 때 굵기 변화를 지워 버려, "수렴하며 얇아진다"가
// 보이지 않게 된다.
const DT_MAX_GLOBAL = Math.max(
    ...[...BANDS_BY_ITERATIONS.values()].flatMap((band) => band.map((p) => p[3] ?? 0)), 1e-9,
)

const Scene = ({scale = 1}: {scale?: number}) => {
    const t = useTr()
    const colors = useCanvasColors()
    const [iterations, setIterations] = useState(ITER_MAX)
    const band = BANDS_BY_ITERATIONS.get(iterations) ?? []
    const band0 = BANDS_BY_ITERATIONS.get(0) ?? []
    const time = useMemo(() => totalTime(band), [band])
    const time0 = useMemo(() => totalTime(band0), [band0])

    const bandSegments = (b: number[][], color: string, thick: boolean, opacity: number) =>
        b.slice(0, -1).map((p, i) => {
            const dt = b[i + 1][3] ?? 0
            const sw = thick ? 1 + 3.2 * (dt / DT_MAX_GLOBAL) : 1.2
            const [x0, y0] = toPx(p[0], p[1])
            const [x1, y1] = toPx(b[i + 1][0], b[i + 1][1])
            return <Line key={`s${i}`} points={[x0, y0, x1, y1]} stroke={color}
                         strokeWidth={sw} opacity={opacity} lineCap="round"/>
        })

    return (
        <div className="flex flex-col items-center gap-2">
            <Stage width={W * scale} height={H * scale}
                   className="bg-surface border border-border rounded-lg overflow-hidden">
                <Layer scaleX={scale} scaleY={scale}>
                    <Line points={TOY_PATH.flatMap(([x, y]) => toPx(x, y))} stroke={colors.muted}
                          strokeWidth={1.4} dash={[5, 4]} opacity={0.6}/>
                    {/* 반복 0회(재초기화 직후) 밴드: 항상 깔아 두는 비교 기준 */}
                    {bandSegments(band0, colors.muted, false, 0.55)}
                    {/* 현재 반복 횟수의 밴드 */}
                    {bandSegments(band, colors.accent2, true, 0.9)}
                    {band.map((p, i) => {
                        const [px, py] = toPx(p[0], p[1])
                        return <Circle key={`p${i}`} x={px} y={py} radius={2.4} fill={colors.accent2}/>
                    })}
                </Layer>
            </Stage>
            <ParamSlider label="iterations" value={iterations}
                         min={0} max={ITER_MAX} step={ITER_STEP} onCommit={setIterations}
                         format={(v) => String(Math.round(v))}/>
            <div className="text-xs text-muted text-center tabular-nums">
                <T
                    en={<>plan time <span className="font-semibold">Σ ΔT = {time.toFixed(2)} s</span>
                        {" "}(before optimization: {time0.toFixed(2)} s)</>}
                    ko={<>계획 시간 <span className="font-semibold">Σ ΔT = {time.toFixed(2)} s</span>
                        {" "}(최적화 전: {time0.toFixed(2)} s)</>}
                />
            </div>
            <div className="text-xs text-muted text-center max-w-[21rem]">
                {t(
                    "gray thin chain = band before optimization · teal chain = band after the chosen iterations · segment thickness = its ΔT (thicker = that stretch is crossed more slowly)",
                    "회색 가는 사슬 = 최적화 전 밴드 · 청록 사슬 = 선택한 반복 횟수만큼 최적화된 밴드 · 세그먼트 굵기 = 그 구간의 ΔT (굵을수록 느리게 지나간다)",
                )}
            </div>
        </div>
    )
}

const TebOptimizationFigure = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Drag the iteration slider and watch the real teb.ts solver optimize a 90° toy corner: the pose chain pulls into the corner and the plan time Σ ΔT drops",
            "iteration 슬라이더를 끌면 실제 teb.ts 솔버가 90도 toy 코너를 최적화하는 과정이 보인다. pose 사슬이 코너 안쪽으로 당겨지고 계획 시간 Σ ΔT가 줄어든다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene scale={modalScale(W, H)}/>}
    >
        <Scene/>
    </CanvasFigure>
}

export default TebOptimizationFigure

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runMppi} from "../../../../libs/algorithms/mppi";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, parseGridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// 저장소 maps/grid/open01.yaml + open01_s4(start (1.5,1.5), goal (8,8)) reach 시나리오.
// K는 라이브 렌더 부담을 줄이려 sandbox에서 낮춘다(yaml 기본 200 대신 60) -- DWA sandbox가
// omega_samples를 고정하는 선례. seed는 mppi.yaml 기본(1).
const OPEN01_ROWS = [
    "####################", "#..................#", "#..................#", "#..................#",
    "#....####..........#", "#....####..........#", "#....####....###...#", "#....####....###...#",
    "#....####....###...#", "#............###...#", "#..................#", "#..........#####...#",
    "#..........#####...#", "#..........#####...#", "#..........#####...#", "#..........#####...#",
    "#..................#", "#..................#", "#..................#", "####################",
]
const buildOpen01 = (): GridMap => parseGridMap({
    name: "open01", width: 20, height: 20, resolution: 0.5, origin: [0, 0], rows: OPEN01_ROWS,
})
const START: Pose = [1.5, 1.5, Math.PI / 4]
const GOAL: [number, number] = [8.0, 8.0]

// configs/local_planning/mppi.yaml 기본값 -- 슬라이더가 없는 나머지 파라미터와 sim 6종.
const SIGMA_OMEGA = 0.5
const W_GOAL = 2.0
const W_OBSTACLE = 25.0
const W_CONTROL = 0.05
const MIN_OBSTACLE_DIST = 0.8
const V_MAX = 0.8
const OMEGA_MAX = 1.5
const A_MAX = 1.5
const SEED = 1
const HORIZON = 20
const FOOTPRINT_RADIUS = 0.2
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const MppiScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(buildOpen01)
    const [start, setStart] = useState<Pose>(START)
    const [numSamples, setNumSamples] = useState(60)
    const [temperature, setTemperature] = useState(1.0)
    const [sigmaV, setSigmaV] = useState(0.3)

    const events = useMemo(() => runMppi({
        map, startPose: start, goal: GOAL,
        horizon: HORIZON, numSamples, temperature, sigmaV, sigmaOmega: SIGMA_OMEGA,
        wGoal: W_GOAL, wObstacle: W_OBSTACLE, wControl: W_CONTROL, minObstacleDist: MIN_OBSTACLE_DIST,
        vMax: V_MAX, omegaMax: OMEGA_MAX, aMax: A_MAX, seed: SEED,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, numSamples, temperature, sigmaV])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const reset = () => {
        setMap(buildOpen01())
        setStart(START)
    }

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS} durationMs={8000}
            map={map} events={events} startPose={start} goal={GOAL} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={reset}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="K" value={numSamples}
                                     min={20} max={100} step={10} onCommit={setNumSamples}
                                     format={(v) => String(Math.round(v))}/>
                        <ParamSlider label="λ" value={temperature}
                                     min={0.1} max={5.0} step={0.1} onCommit={setTemperature}/>
                        <ParamSlider label="σ_v" value={sigmaV}
                                     min={0.1} max={0.7} step={0.05} onCommit={setSigmaV}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "K — how many control sequences are sampled each tick; more samples cover the space better but cost more",
                            "K: 매 tick 뽑는 제어열 표본 수. 많을수록 공간을 잘 덮지만 계산이 늘어난다",
                        )}</span>
                        <span>{t(
                            "λ — softmax temperature; small λ concentrates weight on the cheapest sample, large λ averages them uniformly",
                            "λ: softmax 온도. 작으면 최소비용 표본에 가중치가 몰리고, 크면 표본을 균일하게 평균한다",
                        )}</span>
                        <span>{t(
                            "σ_v — spread of the velocity noise; wider explores more but the average gets noisier",
                            "σ_v: 속도 노이즈의 퍼짐. 넓으면 더 탐색하지만 평균이 거칠어진다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center max-w-[24rem]">
                        {t(
                            "faint arcs = the K sampled rollouts this tick · blue chain = the softmax weighted-average sequence that is actually executed (first control only) · gray trail = where the robot went",
                            "옅은 원호 = 이번 tick의 K개 표본 rollout · 파란 사슬 = 실제로 실행되는 softmax 가중 평균 제어열(첫 제어만) · 회색 trail = 로봇이 지나간 자취",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const MppiSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live MPPI: the faint fan is the K rollouts sampled this tick, the blue chain is their softmax weighted average — the sequence actually executed. Raise λ toward uniform weighting and watch it drift, or drop it and watch it chase one sample",
            "라이브 MPPI. 옅은 부채꼴은 이번 tick 뽑은 K개 rollout이고, 파란 사슬은 그것의 softmax 가중 평균, 즉 실제로 실행되는 제어열이다. λ를 올려 균일 가중으로 가면 표류하고, 내리면 한 표본을 쫓는 모습을 볼 수 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<MppiScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <MppiScene panel={340}/>
    </CanvasFigure>
}

export default MppiSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runMppi} from "../../../../libs/algorithms/mppi";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, parseGridMap} from "../../../../libs/grid";
import {TraceEvent} from "../../../../libs/trace/types";
import {T, useTr} from "../../../../libs/i18n";

// 온도 knife-edge 전용 데모: open01 reach 위에서 λ만 움직여 유효 표본수 N_eff =
// (Σw)²/Σw²의 붕괴를 보인다. 실행 검증: λ=0.1 → 평균 N_eff ≈ 1.3(한 표본에 붕괴), λ=1.0
// → ≈ 5.9(도달), λ=5.0 → ≈ 18.5(가중치가 균일해 비용을 무시, 표류하다 미도달). 가중치는
// 정규화돼 있어 N_eff = 1/Σw²로 계산한다.
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

const K = 60
const SIGMA_V = 0.3
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

// 각 tick의 candidate_evaluated weight 묶음(다음 band_updated 직전까지)에서 N_eff를 구해
// 전 tick 평균을 낸다. 가중치가 이미 정규화(Σw=1)라 N_eff = 1/Σw².
const meanNeff = (events: TraceEvent[]): number => {
    const neffs: number[] = []
    let sumSq = 0
    let count = 0
    for (const ev of events) {
        if (ev.event === "candidate_evaluated") {
            const w = typeof ev.data?.weight === "number" ? ev.data.weight : 0
            sumSq += w * w
            count += 1
        } else if (ev.event === "band_updated" && count > 0) {
            neffs.push(1 / sumSq)
            sumSq = 0
            count = 0
        }
    }
    if (neffs.length === 0) return 0
    return neffs.reduce((a, b) => a + b, 0) / neffs.length
}

const TemperatureScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(buildOpen01)
    const [start, setStart] = useState<Pose>(START)
    const [temperature, setTemperature] = useState(1.0)

    const events = useMemo(() => runMppi({
        map, startPose: start, goal: GOAL,
        horizon: HORIZON, numSamples: K, temperature, sigmaV: SIGMA_V, sigmaOmega: SIGMA_OMEGA,
        wGoal: W_GOAL, wObstacle: W_OBSTACLE, wControl: W_CONTROL, minObstacleDist: MIN_OBSTACLE_DIST,
        vMax: V_MAX, omegaMax: OMEGA_MAX, aMax: A_MAX, seed: SEED,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, temperature])

    const neff = useMemo(() => meanNeff(events), [events])

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
                    <ParamSlider label="λ" value={temperature}
                                 min={0.05} max={5.0} step={0.05} onCommit={setTemperature}/>
                    <div className="text-sm tabular-nums">
                        <T
                            en={<>mean effective samples{" "}
                                <span className="font-semibold text-[var(--accent2)]">
                                    N_eff = {neff.toFixed(1)}</span> of {K}</>}
                            ko={<>평균 유효 표본수{" "}
                                <span className="font-semibold text-[var(--accent2)]">
                                    N_eff = {neff.toFixed(1)}</span> / {K}</>}
                        />
                    </div>
                    <div className="text-xs text-muted text-center max-w-[24rem]">
                        {t(
                            "N_eff = (Σw)² / Σw² counts how many samples actually carry weight. Drop λ toward zero and it collapses to about 1 — the update becomes a single high-variance sample. Raise λ and it climbs toward K, but the weights stop caring about cost and the robot drifts off and never reaches",
                            "N_eff = (Σw)² / Σw²는 실제로 가중치를 지는 표본이 몇 개인지 센다. λ를 0으로 낮추면 약 1로 붕괴한다. 갱신이 고분산 표본 하나가 된다. λ를 올리면 K로 오르지만 가중치가 비용을 무시하게 되어 로봇이 표류해 끝내 도달하지 못한다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const MppiTemperatureDemo = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "MPPI's temperature knife-edge: N_eff collapses to ~1 as λ→0 (the update follows one noisy sample) and climbs to K as λ grows (the weights ignore cost and the robot drifts). A good λ lives on the narrow ridge between",
            "MPPI 온도의 외줄타기. λ→0이면 N_eff가 ~1로 붕괴하고(갱신이 잡음 표본 하나를 따른다), λ가 커지면 K로 올라 가중치가 비용을 무시해 로봇이 표류한다. 좋은 λ는 그 사이 좁은 능선에 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<TemperatureScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <TemperatureScene panel={340}/>
    </CanvasFigure>
}

export default MppiTemperatureDemo

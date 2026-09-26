import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runMpc} from "../../../../libs/algorithms/mpc";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, parseGridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// 저장소 maps/grid/open01.yaml을 그대로 인라인한 프리셋 -- open01_s4(start (1.5,1.5), goal
// (8,8))는 벽 penalty zone 밖 내부 goal이라 goal-seeking MPC가 폐루프로 안정 도달하는
// reach 시나리오다(parity fixture와 같은 맵/시작/목표).
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

// configs/local_planning/mpc.yaml 기본값 -- 슬라이더가 없는 나머지 파라미터와 sim 6종은
// 이 값으로 고정한다. max_steps는 yaml 기본(1000)보다 낮춰 sandbox 재생 이벤트 수를 억제한다.
const STEP_ALPHA = 0.3
const GRAD_EPS = 0.02
const MAX_STEP_V = 0.2
const MAX_STEP_OMEGA = 0.4
const W_GOAL = 2.0
const W_CONTROL = 0.05
const MIN_OBSTACLE_DIST = 0.8
const V_MAX = 0.8
const OMEGA_MAX = 1.5
const A_MAX = 1.5
const FOOTPRINT_RADIUS = 0.2
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const MpcScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(buildOpen01)
    const [start, setStart] = useState<Pose>(START)
    const [horizon, setHorizon] = useState(20)
    const [wObstacle, setWObstacle] = useState(25)
    const [iterations, setIterations] = useState(10)

    const events = useMemo(() => runMpc({
        map, startPose: start, goal: GOAL,
        horizon, iterations, stepAlpha: STEP_ALPHA, gradEps: GRAD_EPS,
        maxStepV: MAX_STEP_V, maxStepOmega: MAX_STEP_OMEGA,
        wGoal: W_GOAL, wObstacle, wControl: W_CONTROL, minObstacleDist: MIN_OBSTACLE_DIST,
        vMax: V_MAX, omegaMax: OMEGA_MAX, aMax: A_MAX,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, horizon, wObstacle, iterations])

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
                        <ParamSlider label="horizon" value={horizon}
                                     min={5} max={40} step={1} onCommit={setHorizon}
                                     format={(v) => String(Math.round(v))}/>
                        <ParamSlider label="w_obstacle" value={wObstacle}
                                     min={0} max={80} step={5} onCommit={setWObstacle}/>
                        <ParamSlider label="iterations" value={iterations}
                                     min={2} max={20} step={1} onCommit={setIterations}
                                     format={(v) => String(Math.round(v))}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "horizon — how many steps H the controller predicts each tick; longer looks further but costs more per tick",
                            "horizon: 매 tick 예측하는 스텝 수 H. 길수록 멀리 내다보지만 tick당 계산이 늘어난다",
                        )}</span>
                        <span>{t(
                            "w_obstacle — weight on staying clear; raise it and the predicted chain bows further from the blocks",
                            "w_obstacle: 장애물에서 멀어지려는 가중치. 올리면 예측 사슬이 블록에서 더 크게 휜다",
                        )}</span>
                        <span>{t(
                            "iterations — fixed gradient-descent steps per tick; more polishing lowers the cost the chain settles at",
                            "iterations: tick당 고정 경사하강 반복 수. 많을수록 사슬이 안착하는 비용이 낮아진다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center max-w-[24rem]">
                        {t(
                            "blue chain = the predicted horizon re-optimized this tick, running ahead of the robot · only its first control is executed, then it re-optimizes · gray trail = where the robot actually went",
                            "파란 사슬 = 이번 tick 다시 최적화된 예측 horizon으로, 로봇보다 앞서 달린다 · 첫 제어만 실행하고 다음 tick에 재최적화한다 · 회색 trail = 로봇이 실제로 지나간 자취",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const MpcSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live MPC: the blue chain is the H-step horizon re-optimized this tick by projected gradient descent — only its first control is executed before the whole thing re-optimizes next tick. Raise w_obstacle and watch the chain bow away from the blocks",
            "라이브 MPC. 파란 사슬은 이번 tick 투영 경사하강으로 다시 최적화된 H-스텝 horizon이다. 첫 제어만 실행하고 다음 tick에 전체를 재최적화한다. w_obstacle을 올리면 사슬이 블록에서 멀어지는 모습을 볼 수 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<MpcScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <MpcScene panel={340}/>
    </CanvasFigure>
}

export default MpcSandbox

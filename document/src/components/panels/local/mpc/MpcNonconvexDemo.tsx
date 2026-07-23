import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import {runMpc} from "../../../../libs/algorithms/mpc";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, parseGridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// 비볼록 함정 전용 데모: 저장소 maps/grid/clutter01.yaml + clutter01_s1의 코너 goal
// (9.25,9.25). 이 goal은 경계 벽에 붙어 clearance(~0.5 − footprint 0.2 = 0.3)가
// min_obstacle_dist(0.8)보다 작아 장애물 페널티 등고선 안에 있다 -- soft penalty를 쓰는
// 예측 planner는 페널티가 밀어내는 코너를 물리적으로 밟을 수 없어, 유한차분 gradient가
// goal 인력과 장애물 반발이 상쇄되는 국소 최적에서 얼어붙는다. 실행 검증: 약 181 tick
// 뒤 goal 코앞(~(8.98,8.95))에서 STALLED. MPPI가 stochastic 표본으로 이 정체를 흔드는
// 대비의 출발점이다.
const CLUTTER01_ROWS = [
    "####################", "#..................#", "#..................#", "#..............##..#",
    "#..............##..#", "#....##............#", "#....##......##....#", "#............##....#",
    "#..................#", "#.........##.......#", "#.........##.......#", "#..................#",
    "#......##..........#", "#......##..........#", "#..................#", "#...##....##.......#",
    "#...##....##.......#", "#..................#", "#..................#", "####################",
]
const buildClutter01 = (): GridMap => parseGridMap({
    name: "clutter01", width: 20, height: 20, resolution: 0.5, origin: [0, 0], rows: CLUTTER01_ROWS,
})
const START: Pose = [0.75, 0.75, Math.PI / 4]
const GOAL: [number, number] = [9.25, 9.25]

// configs/local_planning/mpc.yaml 기본값 그대로 -- 약점이 파라미터 조작이 아니라 비용판
// 자체에서 나옴을 보이려 슬라이더 없이 고정한다.
const HORIZON = 20
const ITERATIONS = 10
const STEP_ALPHA = 0.3
const GRAD_EPS = 0.02
const MAX_STEP_V = 0.2
const MAX_STEP_OMEGA = 0.4
const W_GOAL = 2.0
const W_OBSTACLE = 25.0
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

const NonconvexScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(buildClutter01)
    const [start, setStart] = useState<Pose>(START)

    const events = useMemo(() => runMpc({
        map, startPose: start, goal: GOAL,
        horizon: HORIZON, iterations: ITERATIONS, stepAlpha: STEP_ALPHA, gradEps: GRAD_EPS,
        maxStepV: MAX_STEP_V, maxStepOmega: MAX_STEP_OMEGA, wGoal: W_GOAL, wObstacle: W_OBSTACLE,
        wControl: W_CONTROL, minObstacleDist: MIN_OBSTACLE_DIST, vMax: V_MAX, omegaMax: OMEGA_MAX,
        aMax: A_MAX, controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const reset = () => {
        setMap(buildClutter01())
        setStart(START)
    }

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS} durationMs={8000}
            map={map} events={events} startPose={start} goal={GOAL} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={reset}
            footer={
                <div className="flex flex-col items-center gap-1.5 text-xs text-muted text-center">
                    <div className="max-w-[24rem]">
                        {t(
                            "the goal sits in the top-right corner, closer to the boundary walls than min_obstacle_dist — so the obstacle penalty pushes outward exactly where the goal pulls inward, and the finite-difference gradient freezes in the local optimum where they cancel. The robot stalls a step short of a goal it can never touch",
                            "goal은 우상단 코너, 경계 벽까지의 거리가 min_obstacle_dist보다 가깝다. 장애물 페널티가 바깥으로 미는 바로 그 자리에서 goal이 안으로 당겨, 유한차분 gradient가 둘이 상쇄되는 국소 최적에서 얼어붙는다. 로봇은 결코 밟을 수 없는 goal 한 발짝 앞에서 정체한다",
                        )}
                    </div>
                    <div className="max-w-[24rem]">
                        {t(
                            "this is the honest cost of a soft-penalty gradient planner: it optimizes one control sequence downhill and cannot see the discrete choice of going around. MPPI, up next, shakes the same trap with random samples",
                            "이것이 soft-penalty gradient planner의 정직한 대가다. 하나의 제어열을 내리막으로 최적화할 뿐, 돌아가는 이산적 선택을 보지 못한다. 다음 장의 MPPI는 무작위 표본으로 같은 함정을 흔든다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const MpcNonconvexDemo = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "MPC's honest weakness: a corner goal buried inside the obstacle-penalty zone. The goal's pull and the wall's push cancel, the gradient settles in a local optimum, and the robot stalls a step short of a goal it physically cannot reach",
            "MPC의 정직한 약점. 장애물 페널티 구역 안에 묻힌 코너 goal이다. goal의 인력과 벽의 반발이 상쇄되어 gradient가 국소 최적에 안착하고, 로봇은 물리적으로 도달할 수 없는 goal 한 발짝 앞에서 정체한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<NonconvexScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <NonconvexScene panel={340}/>
    </CanvasFigure>
}

export default MpcNonconvexDemo

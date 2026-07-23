import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import {runTeb} from "../../../../libs/algorithms/teb";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// homotopy 고착 약점 전용 데모: 중앙 블록(x=[4,6), y=[2.5,6.5))의 아래로 폭 2m 통로가
// 곧게 열려 있는데 reference path는 위로 크게 돈다. 밴드 변형은 연속이라 장애물을
// "건너뛰어" 반대편 homotopy로 넘어갈 수 없고, 최적화는 reference가 정한 위쪽 부류
// 안의 국소 최적만 다듬는다 — 실행 검증: 아래 직선 약 7.5m가 열려 있어도 위로 약 19m를
// 돌아 REACHED. 후속 연구가 서로 다른 homotopy의 TEB 여러 개를 병렬 최적화해
// 고친다(Rösmann 2017).
const RES = 0.5
const homotopyMap = (): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            if (row === 0 || row === height - 1 || col === 0 || col === width - 1) {
                occupied[row * width + col] = true
                continue
            }
            const x = (col + 0.5) * RES
            const y = (height - 1 - row + 0.5) * RES
            occupied[row * width + col] = x >= 4.0 && x < 6.0 && y >= 2.5 && y < 6.5
        }
    }
    return {name: "teb_homotopy", width, height, occupied, resolution: RES, originX: 0, originY: 0}
}
const REF_PATH: Point[] = [[1.25, 1.5], [1.25, 7.5], [8.75, 7.5], [8.75, 1.5]]
const START: Pose = [1.25, 1.5, Math.PI / 2]
const GOAL: [number, number] = [8.75, 1.5]

// configs/local_planning/teb.yaml 기본값 — 메인 sandbox와 동일한 고정 파라미터.
// 이 데모는 시나리오만 다르고 파라미터는 동일해야 "reference 선택"만 변수가 된다.
const W_PATH = 1.0
const W_OBSTACLE = 15.0
const W_VELOCITY = 10.0
const W_ACCELERATION = 5.0
const W_KINEMATICS = 50.0
const W_TIME = 1.0
const MAX_SPEED = 0.8
const OMEGA_MAX = 0.5
const A_MAX = 1.5
const MIN_OBSTACLE_DIST = 0.8
const DT_REF = 0.85
const DT_MIN = 0.05
const HORIZON = 4.0
const ITERATIONS = 40
const STEP_ALPHA = 0.02
const MAX_STEP_XY = 0.05
const MAX_STEP_THETA = 0.1
const MAX_STEP_DT = 0.02
const MAX_POSES = 40
const REINIT_DISTANCE = 1.0
const FOOTPRINT_RADIUS = 0.3
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const HomotopyScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(homotopyMap)
    const [start, setStart] = useState<Pose>(START)

    const events = useMemo(() => runTeb({
        map, startPose: start, goal: [GOAL[0], GOAL[1], 0], referencePath: REF_PATH,
        wPath: W_PATH, wObstacle: W_OBSTACLE, wVelocity: W_VELOCITY, wAcceleration: W_ACCELERATION,
        wTime: W_TIME, wKinematics: W_KINEMATICS, maxSpeed: MAX_SPEED, maxOmega: OMEGA_MAX, aMax: A_MAX,
        minObstacleDist: MIN_OBSTACLE_DIST, dtRef: DT_REF, dtMin: DT_MIN, horizon: HORIZON,
        iterations: ITERATIONS, stepAlpha: STEP_ALPHA, maxStepXy: MAX_STEP_XY,
        maxStepTheta: MAX_STEP_THETA, maxStepDt: MAX_STEP_DT, maxPoses: MAX_POSES,
        reinitDistance: REINIT_DISTANCE, controlDt: CONTROL_DT, maxSteps: MAX_STEPS,
        goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const reset = () => {
        setMap(homotopyMap())
        setStart(START)
    }

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS} durationMs={8000}
            map={map} events={events} startPose={start} goal={GOAL}
            referencePath={REF_PATH} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={reset}
            footer={
                <div className="flex flex-col items-center gap-1.5 text-xs text-muted text-center">
                    <div className="max-w-[24rem]">
                        {t(
                            "a straight 2 m corridor is open below the block, but the band only ever deforms continuously — it polishes the detour the reference path chose and never jumps to the other side, so the robot travels about 19 m where 7.5 m would do",
                            "블록 아래로 폭 2m 직선 통로가 열려 있지만, 밴드는 연속으로만 변형되어 reference path가 고른 우회로를 다듬을 뿐 반대편으로 건너뛰지 못한다. 7.5m면 될 길을 로봇이 약 19m 돌아간다",
                        )}
                    </div>
                    <div className="max-w-[24rem]">
                        {t(
                            "the gray dashed reference path is the sole culprit: hand the same optimizer a reference through the lower corridor and it takes the 7.5 m line — the weakness lives in the initialization, not the terrain",
                            "유일한 원인은 회색 점선 reference path다. 같은 최적화에 아래 통로를 지나는 reference를 주면 7.5m 직선으로 간다. 약점은 지형이 아니라 초기화에 있다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const TebHomotopyDemo = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "TEB's honest weakness: the band stays in the homotopy class of its reference path, detouring far around the block even though a much shorter corridor is open on the other side",
            "TEB의 정직한 약점. 밴드는 reference path의 homotopy 부류에 갇혀, 반대편에 훨씬 짧은 통로가 열려 있어도 블록을 크게 돌아간다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<HomotopyScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <HomotopyScene panel={340}/>
    </CanvasFigure>
}

export default TebHomotopyDemo

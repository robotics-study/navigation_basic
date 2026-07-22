import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {discCollides} from "./sampling_space";
import {distanceToNearest} from "./obstacle_grid";

// 브라우저 라이브 local planner 데모의 폐루프 실행 하니스. 저장소
// local_planning/simulation.py의 tick 루프를 그대로 미러한다 — 3개 엔진(potential
// fields, VFH, pure pursuit)이 이 한 곳을 공유해, 적분·종료 판정·trace 방출 순서가
// python demo와 정확히 일치한다 (parity 전제).

export type Pose = [number, number, number];   // world (x, y, theta)

export interface VelocityCommand {
    v: number;      // m/s, 전진(+)
    omega: number;  // rad/s, 반시계(+)
}

export interface RobotState3 {
    pose: Pose;
    v: number;
    omega: number;
}

export type SimStatus = "reached" | "collision" | "stalled" | "timeout";

export interface ClosedLoopResult {
    status: SimStatus;
    success: boolean;
    steps: number;
    timeToGoal: number;
    distanceTraveled: number;
    minClearance: number;
    trajectory: Pose[];   // 시작 pose 포함 실행 궤적
}

export interface ClosedLoopOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

export type EmitFn = (ev: Omit<TraceEvent, "seq">) => void;

// 매 tick 로봇 상태로부터 명령을 계산한다. 알고리즘 고유 trace 이벤트(force_computed,
// histogram_updated, candidate_evaluated)는 tick 내부에서 emit을 호출해 robot_moved보다
// 먼저 방출한다 — python도 compute_command 안에서 알고리즘이 방출한 뒤 시뮬레이터가
// robot_moved를 방출하는 순서라, 같은 seq 순서를 지켜야 parity가 성립한다.
export type ClosedLoopTick = (state: RobotState3, dt: number, emit: EmitFn) => VelocityCommand;

// 각속도가 이 이하면 원호 닫힌형의 division-by-omega가 수치적으로 불안정해 직선 극한으로
// 대체한다 — 튜닝 대상 tolerance가 아니라서 옵션으로 노출하지 않는다(python 미러).
const OMEGA_EPS = 1e-9;

export function wrapToPi(angle: number): number {
    let wrapped = (angle + Math.PI) % (2 * Math.PI)
    if (wrapped <= 0.0) wrapped += 2 * Math.PI
    return wrapped - Math.PI
}

// 조향 법칙(heading error -> 속도 명령), reactive/_steering.py의 heading_command 미러.
// PF와 VFH가 공유한다. omega는 게인 클램프, v는 cos(theta_err) 게이트로 목표가 뒤에
// 있으면 제자리 회전한다. 호출자가 자기 유효 속도를 maxSpeed로 넘기므로 이 함수는
// 조향 법칙만 담당한다.
export function headingCommand(
    thetaErr: number, gain: number, maxSpeed: number, maxOmega: number,
): VelocityCommand {
    const omega = Math.max(-maxOmega, Math.min(maxOmega, gain * thetaErr))
    const v = maxSpeed * Math.max(0, Math.cos(thetaErr))
    return {v, omega}
}

// 상수 (v, omega) 구간의 정확한 원호 적분(closed form) — dt 크기와 무관하게 정확해
// Euler 이산화 오차가 알고리즘 특성(PF 진동, PP 추종오차)과 섞이지 않는다.
export function integrateUnicycle(pose: Pose, cmd: VelocityCommand, dt: number): Pose {
    const [x, y, theta] = pose
    const {v, omega} = cmd
    if (Math.abs(omega) < OMEGA_EPS) {
        return [x + v * dt * Math.cos(theta), y + v * dt * Math.sin(theta), theta]
    }
    const newTheta = theta + omega * dt
    const nx = x + (v / omega) * (Math.sin(newTheta) - Math.sin(theta))
    const ny = y - (v / omega) * (Math.cos(newTheta) - Math.cos(theta))
    return [nx, ny, wrapToPi(newTheta)]
}

const xyDist = (a: [number, number] | Pose, b: [number, number] | Pose): number =>
    Math.hypot(a[0] - b[0], a[1] - b[1])

// 시작 pose부터 planner를 폐루프로 돌려 REACHED/COLLISION/STALLED/TIMEOUT까지 진행한다.
// robot_moved(매 tick) / path_found(성공 시) / planning_finished 방출은 여기 한 곳뿐이다
// (엔진은 자신의 planning_started + 알고리즘 고유 이벤트만 emit한다).
export function runClosedLoop(
    opts: ClosedLoopOptions, emit: EmitFn, tick: ClosedLoopTick,
): ClosedLoopResult {
    const {map, startPose, goal, controlDt, maxSteps, goalTolerance,
           footprintRadius, stallWindow, stallDistance} = opts

    let minClearance = distanceToNearest(map, [startPose[0], startPose[1]])
    const trajectory: Pose[] = [startPose]

    const finish = (status: SimStatus, steps: number, distanceTraveled: number): ClosedLoopResult => {
        const success = status === "reached"
        const timeToGoal = steps * controlDt
        if (success) emit({event: "path_found", path: trajectory})
        emit({
            event: "planning_finished",
            success,
            metrics: {
                time_to_goal: timeToGoal,
                distance_traveled: distanceTraveled,
                min_clearance: minClearance,
                steps,
                collided: status === "collision" ? 1 : 0,
                stalled: status === "stalled" ? 1 : 0,
            },
        })
        return {status, success, steps, timeToGoal, distanceTraveled, minClearance, trajectory}
    }

    if (discCollides(map, footprintRadius, startPose[0], startPose[1])) {
        return finish("collision", 0, 0)
    }

    let state: RobotState3 = {pose: startPose, v: 0, omega: 0}
    let distanceTraveled = 0
    for (let step = 1; step <= maxSteps; step++) {
        const cmd = tick(state, controlDt, emit)
        const newPose = integrateUnicycle(state.pose, cmd, controlDt)
        emit({event: "robot_moved", state: newPose, data: {v: cmd.v, omega: cmd.omega}})
        if (discCollides(map, footprintRadius, newPose[0], newPose[1])) {
            return finish("collision", step, distanceTraveled)
        }
        minClearance = Math.min(minClearance, distanceToNearest(map, [newPose[0], newPose[1]]))
        distanceTraveled += xyDist(newPose, state.pose)
        trajectory.push(newPose)
        if (xyDist(newPose, goal) <= goalTolerance) {
            return finish("reached", step, distanceTraveled)
        }
        if (step >= stallWindow) {
            const ref = trajectory[step - stallWindow]
            if (xyDist(newPose, ref) < stallDistance) {
                return finish("stalled", step, distanceTraveled)
            }
        }
        state = {pose: newPose, v: cmd.v, omega: cmd.omega}
    }
    return finish("timeout", maxSteps, distanceTraveled)
}

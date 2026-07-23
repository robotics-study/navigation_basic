import {GridMap} from "../grid";
import {discCollides} from "./sampling_space";
import {EmitFn, integrateUnicycle, Pose, RobotState3, VelocityCommand} from "./local_sim";
import {DynamicObstacle, Point} from "./velocity_obstacle";

// VO/RVO/ORCA 계열의 다중 로봇 폐루프 하니스 — 저장소 velocity/agent_sim.py의
// simulate_agents를 그대로 미러한다.
//
// N개 몸체가 맵 하나를 공유하고, 각각은 자신의 velocity-obstacle 전략으로
// 구동되거나(commandFn) 고정 속도로 움직인다(scriptedVelocity, 비협조적
// mover). 매 tick은 read-all-then-write다: 모든 몸체의 명령이 이번 tick
// 시작 시점의 같은 스냅샷을 보고 계산된 뒤에야 전체가 함께 적분된다 —
// 순서 무관하고 재현 가능하며, 상호 회피(van den Berg et al. 2008)가
// 전제하는 성질이다(순차 갱신이면 agent 0은 agent 1의 *이전* 위치를 보고
// 반응하는데 agent 1은 agent 0의 *새* 위치를 보고 반응하게 되어, 알고리즘이
// 가정하는 상호성이 깨진다).

export interface AgentSpec {
    start: RobotState3;
    goal: Point;
    radius: number;
    // undefined -> 자신의 commandFn으로 구동. 값이 있으면 비협조적 등속
    // mover(VO/RVO/ORCA 데모의 스크립트된 교차 트래픽).
    scriptedVelocity?: Point;
}

export type SimStatus = "reached" | "collision" | "stalled" | "timeout";

export interface AgentResult {
    status: SimStatus;
    steps: number;
    trajectory: Pose[];
    minPairClearance: number;
}

// 한 agent의 velocity-obstacle 전략(vo/rvo/orca.ts가 구현) — emit은 ego(index 0)에만
// 전달된다(agent_sim.py에서 recorder가 k==0에만 전달되는 것과 동일).
export type AgentCommandFn = (
    state: RobotState3, neighbors: DynamicObstacle[], goal: Point, dt: number, emit?: EmitFn,
) => VelocityCommand;

export interface AgentSimConfig {
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

const xyDist = (a: [number, number] | Pose, b: [number, number] | Pose): number =>
    Math.hypot(a[0] - b[0], a[1] - b[1])

function pairwiseMinClearance(states: RobotState3[], specs: AgentSpec[]): number {
    let best = Infinity
    const n = states.length
    for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
            const gap = xyDist(states[i].pose, states[j].pose) - specs[i].radius - specs[j].radius
            if (gap < best) best = gap
        }
    }
    return best
}

// planner-driven agent들의 명령을 이번 tick 시작 스냅샷(모든 몸체의 이전
// pose/속도)에 대해 계산한다 -- read-all-then-write의 "read" 절반.
function computeCommands(
    commandFns: (AgentCommandFn | null)[], plannerIndices: number[], states: RobotState3[],
    specs: AgentSpec[], snapshot: DynamicObstacle[], dt: number, emit: EmitFn,
): (VelocityCommand | null)[] {
    const commands: (VelocityCommand | null)[] = new Array(states.length).fill(null)
    for (const k of plannerIndices) {
        const neighbors = snapshot.filter((_, j) => j !== k)
        const fn = commandFns[k]
        if (!fn) throw new Error(`agent ${k} has no planner but is not scripted`)
        commands[k] = fn(states[k], neighbors, specs[k].goal, dt, k === 0 ? emit : undefined)
    }
    return commands
}

// 모든 몸체를 함께 적분한다(read-all-then-write의 "write" 절반) -- scripted
// mover는 고정 속도 직선 운동, planner-driven agent는 unicycle 적분.
function integrateAll(
    specs: AgentSpec[], states: RobotState3[], commands: (VelocityCommand | null)[], dt: number,
): {states: RobotState3[]; worldVel: Point[]} {
    const n = specs.length
    const newStates: RobotState3[] = new Array(n)
    const newWorldVel: Point[] = new Array(n)
    for (let k = 0; k < n; k++) {
        const scripted = specs[k].scriptedVelocity
        if (scripted) {
            const [vx, vy] = scripted
            const [x, y] = states[k].pose
            const newPose: Pose = [x + vx * dt, y + vy * dt, Math.atan2(vy, vx)]
            newWorldVel[k] = [vx, vy]
            newStates[k] = {pose: newPose, v: Math.hypot(vx, vy), omega: 0}
        } else {
            const cmd = commands[k]
            if (!cmd) throw new Error(`agent ${k} planner produced no command`)
            const newPose = integrateUnicycle(states[k].pose, cmd, dt)
            newWorldVel[k] = [cmd.v * Math.cos(newPose[2]), cmd.v * Math.sin(newPose[2])]
            newStates[k] = {pose: newPose, v: cmd.v, omega: cmd.omega}
        }
    }
    return {states: newStates, worldVel: newWorldVel}
}

// planner가 하나라도 딸려 있고(scripted 전용 agent는 REACHED에 관여하지
// 않는다), agent_sim.py와 동일한 종료 우선순위(충돌 > 전원 도달 > 정체 >
// 시간초과)로 REACHED/COLLISION/STALLED/TIMEOUT까지 N-body를 폐루프로 돌린다.
export function simulateAgents(
    commandFns: (AgentCommandFn | null)[], specs: AgentSpec[], map: GridMap,
    config: AgentSimConfig, emit: EmitFn,
): AgentResult[] {
    const n = specs.length
    let states: RobotState3[] = specs.map((s) => s.start)
    let worldVel: Point[] = specs.map(() => [0, 0] as Point)
    const trajectories: Pose[][] = specs.map((s) => [s.start.pose])
    const reached: boolean[] = new Array(n).fill(false)
    const plannerIndices = specs.map((_, k) => k).filter((k) => specs[k].scriptedVelocity === undefined)

    let minPairClearance = pairwiseMinClearance(states, specs)
    let terminal: SimStatus = "timeout"
    let steps = config.maxSteps

    for (let step = 1; step <= config.maxSteps; step++) {
        // 1) 이번 tick 이전, 모든 몸체의 스냅샷.
        const snapshot: DynamicObstacle[] = states.map((s, k) => ({
            position: [s.pose[0], s.pose[1]], velocity: worldVel[k], radius: specs[k].radius,
        }))
        // 2) 그 고정 스냅샷에 대해 모든 명령을 계산한다(index 순).
        const commands = computeCommands(commandFns, plannerIndices, states, specs, snapshot, config.controlDt, emit)
        // 3) 모든 명령이 확정된 뒤에만 전체를 적분한다.
        const stepped = integrateAll(specs, states, commands, config.controlDt)
        states = stepped.states
        worldVel = stepped.worldVel
        // 4) 모든 몸체의 trace + trajectory 기록.
        for (let k = 0; k < n; k++) {
            trajectories[k].push(states[k].pose)
            emit({
                event: "robot_moved", state: states[k].pose,
                data: {v: states[k].v, omega: states[k].omega}, agent: k,
            })
        }
        // 5) 종료 판정.
        const tickClearance = pairwiseMinClearance(states, specs)
        minPairClearance = Math.min(minPairClearance, tickClearance)
        const collided = tickClearance < 0
            || states.some((s) => discCollides(map, config.footprintRadius, s.pose[0], s.pose[1]))
        if (collided) {
            terminal = "collision"
            steps = step
            break
        }
        for (const k of plannerIndices) {
            if (!reached[k] && xyDist(states[k].pose, specs[k].goal) <= config.goalTolerance) reached[k] = true
        }
        if (plannerIndices.every((k) => reached[k])) {
            terminal = "reached"
            steps = step
            break
        }
        const stillActive = plannerIndices.filter((k) => !reached[k])
        if (
            stillActive.length > 0
            && step >= config.stallWindow
            && stillActive.every(
                (k) => xyDist(states[k].pose, trajectories[k][step - config.stallWindow]) < config.stallDistance,
            )
        ) {
            terminal = "stalled"
            steps = step
            break
        }
        steps = step
    }

    return specs.map((_, k) => ({
        status: reached[k] ? "reached" : terminal,
        steps,
        trajectory: trajectories[k],
        minPairClearance,
    }))
}

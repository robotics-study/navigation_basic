import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {discCollides, Point} from "./sampling_space";
import {distanceToNearest} from "./obstacle_grid";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// Dynamic Window Approach (Fox, Burgard & Thrun 1997, DOI 10.1109/100.580977) 브라우저
// 라이브 엔진. 저장소 reactive/dwa.py를 그대로 미러한다 — 매 tick 현재 (v, omega)를
// 중심으로 한 가감속 한계 상자를 goal 근접 감속 상한과 교차해 dynamic window를 만들고,
// 그 안을 결정적 격자로 채점해 admissible 후보 중 최선을 고른다. 폐루프 적분·종료
// 판정은 local_sim.ts의 runClosedLoop 한 곳이 맡는다(3개 local 엔진 공유 — DRY).
export interface DwaOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    maxSpeed: number;
    minSpeed: number;
    maxOmega: number;
    accel: number;
    accelOmega: number;
    vSamples: number;
    omegaSamples: number;
    simTime: number;
    simSteps: number;
    headingWeight: number;
    clearanceWeight: number;
    velocityWeight: number;
    clearanceLimit: number;
    slowRadius: number;
    footprintRadius: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    stallWindow: number;
    stallDistance: number;
}

// 이 아래로 |omega|가 작으면 원호 닫힌형(division by omega)이 수치적으로 불안정해
// 직선 극한으로 대체한다 — local_sim.ts의 OMEGA_EPS와 같은 값이지만, 알고리즘 모듈은
// 시뮬레이터를 import하지 않는다는 저장소 원칙에 따라 별도로 둔다(python/C++ 미러).
const OMEGA_EPS = 1e-9;

interface Candidate {
    v: number;
    omega: number;
    rollout: Pose[];
}

interface Score {
    cost: number;
    heading: number;
    clearance: number;
    velocity: number;
    admissible: boolean;
}

function rolloutArc(pose: Pose, v: number, omega: number, simTime: number, simSteps: number): Pose[] {
    // 후보 (v, omega)를 simTime 동안 상수로 유지한다고 보고, simSteps개 등간격 시각에서
    // closed-form 원호 적분으로 pose를 계산한다(Fox 1997의 원호 예측). 각 점은 시작
    // pose에서 직접 계산되며(step-to-step 연쇄가 아님) 상수 명령에 대해 정확하다.
    const [x, y, theta] = pose
    const poses: Pose[] = []
    for (let k = 1; k <= simSteps; k++) {
        const t = simTime * k / simSteps
        if (Math.abs(omega) < OMEGA_EPS) {
            poses.push([x + v * t * Math.cos(theta), y + v * t * Math.sin(theta), theta])
            continue
        }
        const newTheta = theta + omega * t
        const px = x + (v / omega) * (Math.sin(newTheta) - Math.sin(theta))
        const py = y - (v / omega) * (Math.cos(newTheta) - Math.cos(theta))
        poses.push([px, py, wrapToPi(newTheta)])
    }
    return poses
}

function scoreCandidate(
    map: GridMap, footprintRadius: number, rollout: Pose[], v: number, omega: number,
    goalX: number, goalY: number, opts: DwaOptions,
): Score {
    // 충돌하는 롤아웃은 채점하지 않고 즉시 기각한다(Fox 1997의 hard obstacle constraint).
    if (rollout.some(([px, py]) => discCollides(map, footprintRadius, px, py))) {
        return {cost: 0, heading: 0, clearance: 0, velocity: 0, admissible: false}
    }

    // Fox 1997 eq. 14는 곡률 상 최근접 장애물까지의 참거리를 쓰지만, 여기서는 유한
    // 롤아웃을 따라 표본화한 최소 clearance로 보수적으로 근사한다.
    let nearest = Infinity
    for (const [px, py] of rollout) nearest = Math.min(nearest, distanceToNearest(map, [px, py] as Point))
    const clearance = Math.max(0, nearest - footprintRadius)
    const [xEnd, yEnd, thetaEnd] = rollout[rollout.length - 1]
    // Fox 1997 eq. 15의 target-heading 항. 원 논문은 "명령 후 최대 감속 정지 위치"에서
    // 평가하지만, 이 구현은 롤아웃 종점에서 평가한다(단순화).
    const goalBearing = Math.atan2(goalY - yEnd, goalX - xEnd)
    const heading = 1 - Math.abs(wrapToPi(goalBearing - thetaEnd)) / Math.PI
    const velocity = v / opts.maxSpeed
    const clearanceTerm = Math.min(clearance, opts.clearanceLimit) / opts.clearanceLimit

    // 후보 집합 전체에 대한 배치 정규화 대신 고정 정규화를 쓴다 — 한 후보의 채점이
    // 나머지 후보 집합에 의존하지 않아야 py/cpp/TS 채점이 결정적으로 일치한다.
    const cost = opts.headingWeight * heading + opts.clearanceWeight * clearanceTerm + opts.velocityWeight * velocity
    // Fox 1997 eq. 14: 정지 가능 거리 부등식의 유한-롤아웃 근사.
    const admissible = v <= Math.sqrt(2 * clearance * opts.accel)
        && Math.abs(omega) <= Math.sqrt(2 * clearance * opts.accelOmega)
    return {cost, heading, clearance, velocity, admissible}
}

function decelerate(vA: number, omegaA: number, accel: number, accelOmega: number, dt: number): VelocityCommand {
    // 이번 tick에 admissible 후보가 하나도 살아남지 못했다(예: 사방이 막힘) — 채점되지
    // 않은 명령을 실행하는 대신 가감속 한계로 감속한다. 계속 발동되면 local minima이며,
    // 시뮬레이터의 stall 검출이 STALLED로 정직하게 보고한다.
    const v = Math.max(0, vA - accel * dt)
    const sign = omegaA > 0 ? 1 : (omegaA < 0 ? -1 : 0)
    const omega = omegaA - sign * Math.min(Math.abs(omegaA), accelOmega * dt)
    return {v, omega}
}

export function runDwa(opts: DwaOptions): TraceEvent[] {
    const {map, startPose, goal, maxSpeed, minSpeed, maxOmega, accel, accelOmega,
           vSamples, omegaSamples, simTime, simSteps, slowRadius, footprintRadius,
           controlDt, maxSteps, goalTolerance, stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "dwa",
        params: {
            max_speed: maxSpeed, min_speed: minSpeed, max_omega: maxOmega, accel, accel_omega: accelOmega,
            v_samples: vSamples, omega_samples: omegaSamples, sim_time: simTime, sim_steps: simSteps,
            heading_weight: opts.headingWeight, clearance_weight: opts.clearanceWeight,
            velocity_weight: opts.velocityWeight, clearance_limit: opts.clearanceLimit, slow_radius: slowRadius,
            control_dt: controlDt, max_steps: maxSteps, goal_tolerance: goalTolerance,
            footprint_radius: footprintRadius, stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    const tick = (state: RobotState3, dt: number, tickEmit: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose
        const vA = state.v, omegaA = state.omega
        const [goalX, goalY] = goal

        const remaining = Math.hypot(goalX - x, goalY - y)
        // goal 근접 감속 상한: Fox 1997에는 없는 실용적 확장으로, 에피소드가 goal 주변
        // 진동/오버슈트 대신 REACHED로 마감되게 한다.
        const vMaxEff = maxSpeed * Math.min(1, remaining / slowRadius)

        const vLo = Math.max(minSpeed, vA - accel * dt)
        const vHi = Math.min(vMaxEff, vA + accel * dt)
        const omegaLo = Math.max(-maxOmega, omegaA - accelOmega * dt)
        const omegaHi = Math.min(maxOmega, omegaA + accelOmega * dt)

        let bestIndex: number | null = null
        let bestCost = 0
        let bestV = 0
        let bestOmega = 0
        const buffered: Array<{candidate: Candidate; cost: number; data: Record<string, number>}> = []

        if (vLo <= vHi && omegaLo <= omegaHi) {
            const vStep = vSamples > 1 ? (vHi - vLo) / (vSamples - 1) : 0
            const omegaStep = omegaSamples > 1 ? (omegaHi - omegaLo) / (omegaSamples - 1) : 0
            let candidateIndex = 0
            // 결정적 균등 격자(난수 아님): v 외측 → omega 내측 순회 고정이 py/cpp/TS
            // 채점·동률 처리를 bit-identical하게 유지한다.
            for (let i = 0; i < vSamples; i++) {
                const v = vLo + vStep * i
                for (let j = 0; j < omegaSamples; j++) {
                    const omega = omegaLo + omegaStep * j
                    const rollout = rolloutArc([x, y, theta], v, omega, simTime, simSteps)
                    const s = scoreCandidate(map, footprintRadius, rollout, v, omega, goalX, goalY, opts)
                    if (s.admissible && (bestIndex === null || s.cost > bestCost)) {
                        bestIndex = candidateIndex
                        bestCost = s.cost
                        bestV = v
                        bestOmega = omega
                    }
                    buffered.push({
                        candidate: {v, omega, rollout},
                        cost: s.cost,
                        data: {
                            v, omega, heading: s.heading, clearance: s.clearance, velocity: s.velocity,
                            admissible: s.admissible ? 1 : 0, selected: 0, // 아래에서 선택 확정 후 갱신
                        },
                    })
                    candidateIndex++
                }
            }
        }

        const cmd = bestIndex !== null
            ? {v: bestV, omega: bestOmega}
            : decelerate(vA, omegaA, accel, accelOmega, dt)

        // selected는 전 후보 평가가 끝나야 확정되므로 tick 안에서 버퍼링했다가 일괄
        // 방출한다([candidate_evaluated* → robot_moved] 순서는 유지된다).
        buffered.forEach((b, idx) => {
            b.data.selected = idx === bestIndex ? 1 : 0
            const [xEnd, yEnd] = b.candidate.rollout[b.candidate.rollout.length - 1]
            tickEmit({
                event: "candidate_evaluated",
                state: [xEnd, yEnd],
                cost: b.cost,
                data: b.data,
                rollout: b.candidate.rollout.map(([px, py]) => [px, py]),
            })
        })

        return cmd
    }

    runClosedLoop({
        map, startPose, goal, controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

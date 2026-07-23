import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {clamp, rolloutControls, sequenceCost} from "./predictive";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop} from "./local_sim";

// Model Predictive Control (Klančar & Škrjanc, Robotics and Autonomous Systems
// 55(6):460-469, 2007, DOI 10.1016/j.robot.2007.01.002) 브라우저 라이브 엔진. 저장소
// local_planning/predictive/mpc.py를 연산 순서까지 그대로 미러한다 -- 매 tick 다음 H
// 스텝을 unicycle 모델로 예측하고 제어열 U를 공유 비용 J(U)에 대해 고정 반복 투영
// 경사하강(central finite-difference)으로 최적화한 뒤 첫 제어 u_0만 실행하고 다음 tick에
// 재최적화한다(receding horizon). 폐루프 적분·종료 판정은 local_sim.ts의 runClosedLoop이
// 맡는다(모든 local 엔진 공유).
export interface MpcOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    horizon: number;
    iterations: number;
    stepAlpha: number;
    gradEps: number;
    maxStepV: number;
    maxStepOmega: number;
    wGoal: number;
    wObstacle: number;
    wControl: number;
    minObstacleDist: number;
    vMax: number;
    omegaMax: number;
    aMax: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

type Controls = Array<[number, number]>;

export function runMpc(opts: MpcOptions): TraceEvent[] {
    const {map, startPose, goal, horizon, iterations, stepAlpha, gradEps, maxStepV,
           maxStepOmega, wGoal, wObstacle, wControl, minObstacleDist, vMax, omegaMax, aMax,
           controlDt, maxSteps, goalTolerance, footprintRadius, stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "mpc",
        params: {
            horizon, iterations, step_alpha: stepAlpha, grad_eps: gradEps,
            max_step_v: maxStepV, max_step_omega: maxStepOmega,
            w_goal: wGoal, w_obstacle: wObstacle, w_control: wControl,
            min_obstacle_dist: minObstacleDist, v_max: vMax, omega_max: omegaMax, a_max: aMax,
            control_dt: controlDt, max_steps: maxSteps, goal_tolerance: goalTolerance,
            footprint_radius: footprintRadius, stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // tick 간 warm-start를 위해 이어지는 제어열 U. 비어 있으면 cold start(첫 tick / reset
    // 직후)로 zero 제어 H개를 시드한다 (py MpcPlanner._controls 미러).
    let controls: Controls = []

    const warmStart = (): Controls => {
        if (controls.length === 0) return Array.from({length: horizon}, () => [0.0, 0.0] as [number, number])
        // 왼쪽 시프트 + 마지막 제어 복제(실행된 u_0는 버린다).
        return [...controls.slice(1), controls[controls.length - 1]]
    }

    const cost = (s0: Pose, u: Controls, h: number): number => {
        const traj = rolloutControls(s0, u, h)
        return sequenceCost(map, traj, u, goal, footprintRadius, wGoal, wObstacle, minObstacleDist, wControl)
    }

    // 투영 경사하강 1회: 현재 U에서 2H 성분 전체의 central finite-difference gradient(성분
    // 순서 k 오름차순, v 먼저 ω -- 언어 간 결정론 계약)를 구한 뒤, 성분별 step clamp와 box
    // 투영(v∈[0,v_max], |ω|≤ω_max)으로 U ← U − step_alpha·grad를 한 번 적용한다 (py
    // MpcPlanner._descent_step 미러).
    const descentStep = (s0: Pose, u: Controls, h: number): void => {
        const n = u.length
        const gradV = new Array<number>(n).fill(0.0)
        const gradOmega = new Array<number>(n).fill(0.0)
        for (let k = 0; k < n; k++) {
            const [v, omega] = u[k]
            u[k] = [v + gradEps, omega]
            let jPlus = cost(s0, u, h)
            u[k] = [v - gradEps, omega]
            let jMinus = cost(s0, u, h)
            gradV[k] = (jPlus - jMinus) / (2.0 * gradEps)
            u[k] = [v, omega + gradEps]
            jPlus = cost(s0, u, h)
            u[k] = [v, omega - gradEps]
            jMinus = cost(s0, u, h)
            gradOmega[k] = (jPlus - jMinus) / (2.0 * gradEps)
            u[k] = [v, omega]
        }
        for (let k = 0; k < n; k++) {
            let [v, omega] = u[k]
            v -= clamp(stepAlpha * gradV[k], -maxStepV, maxStepV)
            omega -= clamp(stepAlpha * gradOmega[k], -maxStepOmega, maxStepOmega)
            u[k] = [clamp(v, 0.0, vMax), clamp(omega, -omegaMax, omegaMax)]
        }
    }

    const emitBand = (tickEmit: EmitFn, s0: Pose, traj: Pose[], h: number, totalCost: number): void => {
        const band = [[s0[0], s0[1], s0[2], 0.0]]
        for (const p of traj) band.push([p[0], p[1], p[2], h])
        tickEmit({
            event: "band_updated",
            band,
            data: {iterations, horizon, total_cost: totalCost},
        })
    }

    const tick = (state: RobotState3, dt: number, tickEmit: EmitFn): VelocityCommand => {
        const s0 = state.pose
        // 예측 스텝은 제어 주기(dt)와 같다 -- 같은 이산화로 예측·실행하면 실행된 u_0가
        // 그것이 최적화된 horizon과 일관된다.
        const h = dt

        const u = warmStart()
        for (let i = 0; i < iterations; i++) descentStep(s0, u, h)
        controls = u

        const traj = rolloutControls(s0, u, h)
        const totalCost = sequenceCost(
            map, traj, u, goal, footprintRadius, wGoal, wObstacle, minObstacleDist, wControl)
        emitBand(tickEmit, s0, traj, h, totalCost)

        // 실행 명령: 시뮬레이터가 넘기는 직전 실행 속도(RobotState.v, DWA처럼 별도 _v_prev
        // 상태를 두지 않는다)에 대해 병진 가속을 clamp한 뒤 box-clamp해 항상 한계 이내로.
        let [v0, omega0] = u[0]
        v0 = clamp(v0, state.v - aMax * h, state.v + aMax * h)
        v0 = clamp(v0, 0.0, vMax)
        omega0 = clamp(omega0, -omegaMax, omegaMax)
        return {v: v0, omega: omega0}
    }

    runClosedLoop({
        map, startPose, goal, controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

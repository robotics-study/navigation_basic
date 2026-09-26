import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {clamp, gaussian, rolloutControls, sequenceCost} from "./predictive";
import {NumpyRandom} from "./numpy_rng";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop} from "./local_sim";

// Model Predictive Path Integral control (Williams, Aldrich & Theodorou, ICRA 2016,
// DOI 10.1109/ICRA.2016.7487277; Williams et al., IEEE T-RO 34(6):1603-1622, 2018,
// DOI 10.1109/TRO.2018.2865891) 브라우저 라이브 엔진. 저장소
// local_planning/predictive/mppi.py를 미러한다 -- 매 tick 제어열에 Gauss 노이즈를 K개
// 뿌려 rollout하고 각 비용을 MPC와 완전히 같은 J(U)로 채점한 뒤, softmax importance
// weight로 공칭 제어열을 가중 평균 갱신한다(MPC의 유한차분 gradient 대신 derivative-free
// 표본). 첫 제어 u_0만 실행하고 다음 tick에 재최적화한다(receding horizon). 노이즈는
// numpy_rng.ts의 NumpyRandom(PCG64 uniform bit-mirror) + predictive.ts의 Box-Muller로
// 뽑아 py default_rng와 균등 스트림이 bit-identical이다.
export interface MppiOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    horizon: number;
    numSamples: number;
    temperature: number;
    sigmaV: number;
    sigmaOmega: number;
    wGoal: number;
    wObstacle: number;
    wControl: number;
    minObstacleDist: number;
    vMax: number;
    omegaMax: number;
    aMax: number;
    seed: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

type Controls = Array<[number, number]>;

export function runMppi(opts: MppiOptions): TraceEvent[] {
    const {map, startPose, goal, horizon, numSamples, temperature, sigmaV, sigmaOmega,
           wGoal, wObstacle, wControl, minObstacleDist, vMax, omegaMax, aMax, seed,
           controlDt, maxSteps, goalTolerance, footprintRadius, stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "mppi",
        params: {
            horizon, num_samples: numSamples, temperature, sigma_v: sigmaV, sigma_omega: sigmaOmega,
            w_goal: wGoal, w_obstacle: wObstacle, w_control: wControl,
            min_obstacle_dist: minObstacleDist, v_max: vMax, omega_max: omegaMax, a_max: aMax, seed,
            control_dt: controlDt, max_steps: maxSteps, goal_tolerance: goalTolerance,
            footprint_radius: footprintRadius, stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // 표본이 궤적을 결정하므로 RNG는 실행마다 seed로 새로 만든다(에피소드 재현성). tick 간
    // warm-start를 위해 이어지는 공칭 제어열 Û (py MppiPlanner._controls/_rng 미러).
    const rng = new NumpyRandom(seed)
    let controls: Controls = []

    const warmStart = (): Controls => {
        if (controls.length === 0) return Array.from({length: horizon}, () => [0.0, 0.0] as [number, number])
        return [...controls.slice(1), controls[controls.length - 1]]
    }

    const emitBand = (tickEmit: EmitFn, s0: Pose, traj: Pose[], h: number, minCost: number): void => {
        const band = [[s0[0], s0[1], s0[2], 0.0]]
        for (const p of traj) band.push([p[0], p[1], p[2], h])
        tickEmit({
            event: "band_updated",
            band,
            data: {min_cost: minCost, num_samples: numSamples, temperature},
        })
    }

    const tick = (state: RobotState3, dt: number, tickEmit: EmitFn): VelocityCommand => {
        const s0 = state.pose
        const h = dt

        const u = warmStart()
        const hLen = u.length

        // K개 섭동 제어열을 뽑아 각각 채점한다. 모든 RNG draw는 무조건 실행된다(표본이
        // 궤적을 결정하므로 trace on/off가 로봇 동작을 바꾸면 안 된다).
        const epsSamples: Controls[] = []
        const costs: number[] = []
        const terminals: Array<[number, number]> = []
        const rolloutsXy: Array<Array<[number, number]>> = []
        for (let k = 0; k < numSamples; k++) {
            const epsSeq: Controls = []
            const perturbed: Controls = []
            for (let j = 0; j < hLen; j++) {
                const epsV = gaussian(rng) * sigmaV
                const epsOmega = gaussian(rng) * sigmaOmega
                epsSeq.push([epsV, epsOmega])
                const [baseV, baseOmega] = u[j]
                perturbed.push([
                    clamp(baseV + epsV, 0.0, vMax),
                    clamp(baseOmega + epsOmega, -omegaMax, omegaMax),
                ])
            }
            const traj = rolloutControls(s0, perturbed, h)
            const cost = sequenceCost(
                map, traj, perturbed, goal, footprintRadius, wGoal, wObstacle, minObstacleDist, wControl)
            epsSamples.push(epsSeq)
            costs.push(cost)
            const [tx, ty] = traj[traj.length - 1]
            terminals.push([tx, ty])
            rolloutsXy.push(traj.map((p): [number, number] => [p[0], p[1]]))
        }

        // Softmax importance weight. exp 전에 beta = min_k S_k를 빼 지수를 범위 안에 둔다
        // (최소비용 표본이 exp(0)=1을 기여해 정규화항이 >=1, underflow 없음). 이 shift는
        // 정규화에서 상쇄된다 (Williams et al. 2018).
        let beta = costs[0]
        for (const c of costs) if (c < beta) beta = c
        const weights: number[] = []
        let total = 0.0
        for (const c of costs) {
            const w = Math.exp(-(c - beta) / temperature)
            weights.push(w)
            total += w
        }
        const invTotal = 1.0 / total
        for (let k = 0; k < weights.length; k++) weights[k] *= invTotal

        // 공칭 제어열을 raw 노이즈의 가중 평균으로 갱신한 뒤 box 투영. 누적 순서(j 외측,
        // k 내측)는 언어 간 결정론 계약이다.
        for (let j = 0; j < hLen; j++) {
            const [baseV, baseOmega] = u[j]
            let accV = 0.0
            let accOmega = 0.0
            for (let k = 0; k < numSamples; k++) {
                const w = weights[k]
                const [epsV, epsOmega] = epsSamples[k][j]
                accV += w * epsV
                accOmega += w * epsOmega
            }
            u[j] = [clamp(baseV + accV, 0.0, vMax), clamp(baseOmega + accOmega, -omegaMax, omegaMax)]
        }
        controls = u

        let bestIndex = 0
        for (let k = 0; k < costs.length; k++) if (costs[k] === beta) { bestIndex = k; break }
        for (let k = 0; k < numSamples; k++) {
            tickEmit({
                event: "candidate_evaluated",
                state: terminals[k],
                cost: costs[k],
                data: {weight: weights[k], selected: k === bestIndex ? 1.0 : 0.0},
                rollout: rolloutsXy[k],
            })
        }
        const nominalTraj = rolloutControls(s0, u, h)
        emitBand(tickEmit, s0, nominalTraj, h, beta)

        // 실행 명령: 직전 실행 속도(RobotState.v, MPC/DWA처럼)에 대해 병진 가속을 clamp한 뒤
        // box-clamp해 항상 한계 이내로.
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

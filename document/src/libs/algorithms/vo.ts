import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {AgentCommandFn, AgentSpec, simulateAgents} from "./agent_sim";
import {EmitFn} from "./local_sim";
import {commandWithNeighbors, selectSampledVelocity} from "./velocity_obstacle";

// Velocity Obstacle(Fiorini & Shiller 1998, DOI 10.1177/027836499801700706) 브라우저
// 라이브 엔진. 저장소 velocity/vo.py를 그대로 미러한다 -- 매 tick 가까운
// obstacle마다 apex를 그 obstacle 자신의 속도에 두는 truncated cone을 만들고,
// 모든 원뿔 밖에 있는 후보 중 선호 속도에 가장 가까운 것을 고른다.
export interface VoOptions {
    map: GridMap;
    agents: AgentSpec[];
    maxSpeed: number;
    maxOmega: number;
    headingGain: number;
    agentRadius: number;
    neighborDist: number;
    timeHorizon: number;
    obstacleRadius: number;
    speedSamples: number;
    angleSamples: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

function makeVoCommandFn(map: GridMap, opts: VoOptions): AgentCommandFn {
    return (state, neighbors, goal, dt, emit) => commandWithNeighbors(
        (vPref, nb, st) => selectSampledVelocity(
            vPref, [...nb, ...st], [state.pose[0], state.pose[1]], opts.agentRadius, opts.neighborDist,
            opts.timeHorizon, opts.maxSpeed, opts.speedSamples, opts.angleSamples, (o) => o.velocity,
        ),
        opts, map, state, goal, neighbors, dt, emit,
    )
}

export function runVo(opts: VoOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "vo",
        params: {
            max_speed: opts.maxSpeed, max_omega: opts.maxOmega, heading_gain: opts.headingGain,
            agent_radius: opts.agentRadius, neighbor_dist: opts.neighborDist, time_horizon: opts.timeHorizon,
            obstacle_radius: opts.obstacleRadius, speed_samples: opts.speedSamples, angle_samples: opts.angleSamples,
            control_dt: opts.controlDt, max_steps: opts.maxSteps, goal_tolerance: opts.goalTolerance,
            footprint_radius: opts.footprintRadius, stall_window: opts.stallWindow, stall_distance: opts.stallDistance,
        },
    })

    const commandFn = makeVoCommandFn(opts.map, opts)
    const commandFns = opts.agents.map((a) => (a.scriptedVelocity === undefined ? commandFn : null))
    const results = simulateAgents(commandFns, opts.agents, opts.map, {
        controlDt: opts.controlDt, maxSteps: opts.maxSteps, goalTolerance: opts.goalTolerance,
        footprintRadius: opts.footprintRadius, stallWindow: opts.stallWindow, stallDistance: opts.stallDistance,
    }, emit)

    // agent_sim.py의 simulate_agents는 AgentResult 리스트만 반환하고(테스트 assertion용),
    // planning_finished는 방출하지 않는다 -- 이 이벤트는 브라우저 플레이어가 종료 상태
    // 배지를 그리기 위한 데모 전용 확장이다(단일 로봇 엔진들의 runClosedLoop와 동일한 필드).
    const ego = results[0]
    emit({
        event: "planning_finished",
        success: ego.status === "reached",
        metrics: {
            steps: ego.steps,
            collided: ego.status === "collision" ? 1 : 0,
            stalled: ego.status === "stalled" ? 1 : 0,
            min_pair_clearance: ego.minPairClearance,
        },
    })

    return events
}

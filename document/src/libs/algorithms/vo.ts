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
    // simulate_agents의 미러인 simulateAgents가 robot_moved와 ego 중심
    // planning_finished를 직접 방출한다(agent_sim.py와 동일).
    simulateAgents(commandFns, opts.agents, opts.map, {
        controlDt: opts.controlDt, maxSteps: opts.maxSteps, goalTolerance: opts.goalTolerance,
        footprintRadius: opts.footprintRadius, stallWindow: opts.stallWindow, stallDistance: opts.stallDistance,
    }, emit)

    return events
}

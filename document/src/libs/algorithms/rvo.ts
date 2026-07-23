import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {AgentCommandFn, AgentSpec, simulateAgents} from "./agent_sim";
import {EmitFn} from "./local_sim";
import {commandWithNeighbors, Point, rvoApex, selectSampledVelocity} from "./velocity_obstacle";

// Reciprocal Velocity Obstacle(van den Berg, Lin & Manocha 2008,
// DOI 10.1109/ROBOT.2008.4543489) 브라우저 라이브 엔진. 저장소 velocity/rvo.py를
// 그대로 미러한다 -- VO와 동일하되 각 원뿔의 apex를 상대 obstacle의 속도에서
// 두 agent 속도의 중점(reciprocity, 기본 0.5) 쪽으로 옮겨, 대칭적인 마주침에서
// 양쪽이 회피 부담을 절반씩 진다 -- VO의 상호 진동을 고치는 지점이다.
export interface RvoOptions {
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
    reciprocity: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

function makeRvoCommandFn(map: GridMap, opts: RvoOptions): AgentCommandFn {
    return (state, neighbors, goal, dt, emit) => commandWithNeighbors(
        (vPref, nb, st) => {
            const [x, y, theta] = state.pose
            const vSelf: Point = [state.v * Math.cos(theta), state.v * Math.sin(theta)]
            return selectSampledVelocity(
                vPref, [...nb, ...st], [x, y], opts.agentRadius, opts.neighborDist,
                opts.timeHorizon, opts.maxSpeed, opts.speedSamples, opts.angleSamples,
                (o) => rvoApex(vSelf, o.velocity, opts.reciprocity),
            )
        },
        opts, map, state, goal, neighbors, dt, emit,
    )
}

export function runRvo(opts: RvoOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "rvo",
        params: {
            max_speed: opts.maxSpeed, max_omega: opts.maxOmega, heading_gain: opts.headingGain,
            agent_radius: opts.agentRadius, neighbor_dist: opts.neighborDist, time_horizon: opts.timeHorizon,
            obstacle_radius: opts.obstacleRadius, speed_samples: opts.speedSamples, angle_samples: opts.angleSamples,
            reciprocity: opts.reciprocity, control_dt: opts.controlDt, max_steps: opts.maxSteps,
            goal_tolerance: opts.goalTolerance, footprint_radius: opts.footprintRadius,
            stall_window: opts.stallWindow, stall_distance: opts.stallDistance,
        },
    })

    const commandFn = makeRvoCommandFn(opts.map, opts)
    const commandFns = opts.agents.map((a) => (a.scriptedVelocity === undefined ? commandFn : null))
    const results = simulateAgents(commandFns, opts.agents, opts.map, {
        controlDt: opts.controlDt, maxSteps: opts.maxSteps, goalTolerance: opts.goalTolerance,
        footprintRadius: opts.footprintRadius, stallWindow: opts.stallWindow, stallDistance: opts.stallDistance,
    }, emit)

    // vo.ts와 동일한 데모 전용 확장 -- agent_sim.py에는 없는 종료 이벤트.
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

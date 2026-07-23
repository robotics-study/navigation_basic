import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {AgentCommandFn, AgentSpec, simulateAgents} from "./agent_sim";
import {EmitFn} from "./local_sim";
import {
    commandWithNeighbors, DynamicObstacle, halfplaneToConstraint, HalfPlane, linearProgram2d,
    linearProgram3d, orcaHalfPlane, Point,
} from "./velocity_obstacle";

// Optimal Reciprocal Collision Avoidance(van den Berg, Guy, Lin & Manocha 2011,
// DOI 10.1007/978-3-642-19457-3_1) 브라우저 라이브 엔진. 저장소 velocity/orca.py를
// 그대로 미러한다 -- VO/RVO의 표본 격자 대신 obstacle마다 정확한 half-plane을
// 만들고 결정적 2D 선형계획(RVO2의 linearProgram1/2)으로 푼다. 제약이 함께
// infeasible이면 침투 최소화 3D fallback으로 넘어간다. 정적 장애물은 동적
// 이웃과 별도로 설정된(더 짧은) time horizon을 쓴다 -- 벽의 "충돌" 긴급도는
// 다른 agent와 같은 lookahead로 다룰 대상이 아니기 때문이다.
export interface OrcaOptions {
    map: GridMap;
    agents: AgentSpec[];
    maxSpeed: number;
    maxOmega: number;
    headingGain: number;
    agentRadius: number;
    neighborDist: number;
    timeHorizon: number;
    timeHorizonObst: number;
    obstacleRadius: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

function halfPlanesFor(
    obstacles: DynamicObstacle[], pos: Point, vSelf: Point, dt: number, tau: number,
    neighborDist: number, agentRadius: number,
): HalfPlane[] {
    const planes: HalfPlane[] = []
    for (const o of obstacles) {
        const relPos: Point = [o.position[0] - pos[0], o.position[1] - pos[1]]
        if (Math.hypot(relPos[0], relPos[1]) >= neighborDist + o.radius) continue
        const relVel: Point = [vSelf[0] - o.velocity[0], vSelf[1] - o.velocity[1]]
        planes.push(orcaHalfPlane(relPos, relVel, vSelf, agentRadius + o.radius, tau, dt))
    }
    return planes
}

function makeOrcaCommandFn(map: GridMap, opts: OrcaOptions): AgentCommandFn {
    return (state, neighbors, goal, dt, emit) => commandWithNeighbors(
        (vPref, nb, st) => {
            const [x, y, theta] = state.pose
            const pos: Point = [x, y]
            const vSelf: Point = [state.v * Math.cos(theta), state.v * Math.sin(theta)]
            const planes = [
                ...halfPlanesFor(nb, pos, vSelf, dt, opts.timeHorizon, opts.neighborDist, opts.agentRadius),
                ...halfPlanesFor(st, pos, vSelf, dt, opts.timeHorizonObst, opts.neighborDist, opts.agentRadius),
            ]
            const [ok, vNewOk, fail] = linearProgram2d(planes, vPref, opts.maxSpeed)
            const vNew = ok ? vNewOk : linearProgram3d(planes, fail, vPref, opts.maxSpeed)
            const constraints = planes.map(halfplaneToConstraint)
            return [vNew, constraints]
        },
        opts, map, state, goal, neighbors, dt, emit,
    )
}

export function runOrca(opts: OrcaOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "orca",
        params: {
            max_speed: opts.maxSpeed, max_omega: opts.maxOmega, heading_gain: opts.headingGain,
            agent_radius: opts.agentRadius, neighbor_dist: opts.neighborDist, time_horizon: opts.timeHorizon,
            time_horizon_obst: opts.timeHorizonObst, obstacle_radius: opts.obstacleRadius,
            control_dt: opts.controlDt, max_steps: opts.maxSteps, goal_tolerance: opts.goalTolerance,
            footprint_radius: opts.footprintRadius, stall_window: opts.stallWindow, stall_distance: opts.stallDistance,
        },
    })

    const commandFn = makeOrcaCommandFn(opts.map, opts)
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

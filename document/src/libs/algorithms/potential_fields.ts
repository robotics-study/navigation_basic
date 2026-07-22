import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {occupiedWithin} from "./obstacle_grid";
import {ClosedLoopTick, Pose, RobotState3, headingCommand, runClosedLoop, wrapToPi} from "./local_sim";

// 브라우저 라이브 Potential Fields (Khatib 1986): 매 tick 목표 인력 + FIRAS 반발력을
// 합산해 그 방향으로 조향한다. 저장소 python/navigation/local_planning/reactive/
// potential_fields.py를 그대로 미러한다 (연산 순서 동일 — occupiedWithin의 row asc/
// col asc 열거 순서가 반발력 합산 순서를 결정하므로 parity 전제).
export interface PotentialFieldsOptions {
    map: GridMap;
    start: Pose;          // world (x, y, theta)
    goal: [number, number];
    kAtt: number;
    kRep: number;
    influenceRadius: number;
    kV: number;
    kOmega: number;
    maxSpeed: number;
    maxOmega: number;
    // 로봇 충돌 반경. 반발항 1/d 클램프의 d_min으로도 재사용된다(python 미러) —
    // 접촉면보다 가까운 거리를 반발력 계산에 넣지 않기 위한 것.
    footprintRadius: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    stallWindow: number;
    stallDistance: number;
}

export function runPotentialFields(opts: PotentialFieldsOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "potential_fields",
        params: {
            k_att: opts.kAtt, k_rep: opts.kRep, influence_radius: opts.influenceRadius,
            k_v: opts.kV, k_omega: opts.kOmega, max_speed: opts.maxSpeed, max_omega: opts.maxOmega,
        },
    })

    const dMin = opts.footprintRadius

    const tick: ClosedLoopTick = (state: RobotState3, _dt, emitTick) => {
        const [x, y, theta] = state.pose
        const [gx, gy] = opts.goal
        const fxAtt = opts.kAtt * (gx - x)
        const fyAtt = opts.kAtt * (gy - y)

        let fxRep = 0
        let fyRep = 0
        for (const [ox, oy] of occupiedWithin(opts.map, [x, y], opts.influenceRadius)) {
            const dx = x - ox
            const dy = y - oy
            const d = Math.max(Math.hypot(dx, dy), dMin)
            if (d >= opts.influenceRadius) continue
            const magnitude = opts.kRep * (1 / d - 1 / opts.influenceRadius) * (1 / (d * d))
            fxRep += magnitude * dx / d
            fyRep += magnitude * dy / d
        }

        const fx = fxAtt + fxRep
        const fy = fyAtt + fyRep

        // force_computed는 robot_moved보다 먼저 방출한다 — python도 compute_command
        // 안에서 planner가 방출한 뒤 시뮬레이터가 robot_moved를 방출하는 순서다.
        emitTick({
            event: "force_computed",
            state: [x, y, theta],
            data: {fx_att: fxAtt, fy_att: fyAtt, fx_rep: fxRep, fy_rep: fyRep, fx, fy},
        })

        const thetaD = Math.atan2(fy, fx)
        const vEff = Math.min(opts.maxSpeed, opts.kV * Math.hypot(fx, fy))
        return headingCommand(wrapToPi(thetaD - theta), opts.kOmega, vEff, opts.maxOmega)
    }

    runClosedLoop({
        map: opts.map, startPose: opts.start, goal: opts.goal,
        controlDt: opts.controlDt, maxSteps: opts.maxSteps, goalTolerance: opts.goalTolerance,
        footprintRadius: opts.footprintRadius, stallWindow: opts.stallWindow, stallDistance: opts.stallDistance,
    }, emit, tick)

    return events
}

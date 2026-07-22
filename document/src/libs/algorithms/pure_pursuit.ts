import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {Point} from "./sampling_space";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// 브라우저 라이브 Pure Pursuit (Coulter 1992, CMU-RI-TR-92-01). 저장소
// tracking/pure_pursuit.py를 그대로 미러한다: lookahead 원과 참조 경로의
// 교점을 매 tick 추종하며, 그 교점을 지나는 단일 상수-곡률 원호로 조향한다.
// 폐루프 적분·종료 판정은 local_sim.ts의 runClosedLoop 한 곳이 맡는다(3개
// local 엔진 공유 — DRY).
export interface PurePursuitOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    referencePath: Point[];
    lookaheadDistance: number;
    maxSpeed: number;
    maxOmega: number;
    slowRadius: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

// 이 아래로 곡률이 작으면 클램프-재계산 단계(v = omega / kappa)가 0에 가까운 값으로
// 나누게 된다. |kappa|가 이만큼 작을 때는 kappa*v가 이미 어떤 현실적인 max_omega보다도
// 훨씬 작아 클램프 분기가 실전에서 실행되지 않는다 — 실제 동작을 튜닝하는 값이 아니라
// 나눗셈을 방어적으로 가드할 뿐이다 (python 미러).
const KAPPA_EPS = 1e-9;

function closestPointOnSegment(p: Point, a: Point, b: Point): Point {
    const [ax, ay] = a
    const [bx, by] = b
    const dx = bx - ax, dy = by - ay
    const segLenSq = dx * dx + dy * dy
    if (segLenSq < 1e-12) return a
    const t = Math.max(0, Math.min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / segLenSq))
    return [ax + t * dx, ay + t * dy]
}

function sqDist(a: Point, b: Point): number {
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
}

// 로봇 중심 lookahead 원과 선분 a->b의 전방 교점 (매개변수 t in [0,1]), 없으면 null.
// |a + t*(b-a) - p|^2 = radius^2 (Coulter 1992 sec. 3의 원-직선 교차)을 풀고 범위 안의
// 큰 근(exit point, 경로를 따라 더 앞선 쪽)을 취한다 — 진입점(뒤쪽)이 아니라 항상
// 로봇을 앞으로 이끄는 점을 고르기 위해서다.
function segmentCircleForwardT(p: Point, a: Point, b: Point, radius: number): number | null {
    const dx = b[0] - a[0], dy = b[1] - a[1]
    const fx = a[0] - p[0], fy = a[1] - p[1]
    const aa = dx * dx + dy * dy
    if (aa < 1e-12) return null
    const bb = 2 * (fx * dx + fy * dy)
    const cc = fx * fx + fy * fy - radius * radius
    const disc = bb * bb - 4 * aa * cc
    if (disc < 0) return null
    const sq = Math.sqrt(disc)
    for (const t of [(-bb + sq) / (2 * aa), (-bb - sq) / (2 * aa)]) {
        if (t >= 0 && t <= 1) return t
    }
    return null
}

// 로봇이 추종 중인 참조 경로 구간의 index를 전진 전용으로 갱신한다. 단조 증가만
// 허용해, 자기 자신과 교차하는 경로에서 lookahead 점이 뒤쪽의 기하적으로 더 가까운
// 교차점으로 되돌아가지 않게 한다.
function advanceProgressIndex(path: Point[], robotXy: Point, progressIndex: number): number {
    if (path.length < 2) return progressIndex
    let bestIndex = progressIndex
    let bestSqDist = Infinity
    for (let i = progressIndex; i < path.length - 1; i++) {
        const closest = closestPointOnSegment(robotXy, path[i], path[i + 1])
        const d = sqDist(robotXy, closest)
        // <=, 아니라 <: 연속한 두 구간은 접점을 공유하므로, 로봇이 코너에 정확히 있으면
        // 그 접점에서 끝나거나 시작하는 모든 구간이 동률이 된다. 동률일 때 더 앞선(later)
        // 구간을 우선하면 코너를 지나며 진행이 계속 앞으로 가고, 방금 지나온 구간에
        // 눌러앉지 않는다.
        if (d <= bestSqDist) {
            bestSqDist = d
            bestIndex = i
        }
    }
    return bestIndex
}

function lookaheadPoint(path: Point[], startIndex: number, robotXy: Point, lookaheadDistance: number): Point {
    for (let i = startIndex; i < path.length - 1; i++) {
        const t = segmentCircleForwardT(robotXy, path[i], path[i + 1], lookaheadDistance)
        if (t !== null) {
            const a = path[i], b = path[i + 1]
            return [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
        }
    }
    // 어떤 구간도 lookahead 원과 만나지 않는다 — 남은 경로가 L_d보다 짧다는 뜻이므로
    // 경로 끝(goal)을 겨냥한다.
    return path[path.length - 1]
}

export function runPurePursuit(opts: PurePursuitOptions): TraceEvent[] {
    const {map, startPose, goal, referencePath, lookaheadDistance, maxSpeed, maxOmega, slowRadius,
           controlDt, maxSteps, goalTolerance, footprintRadius, stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "pure_pursuit",
        params: {
            lookahead_distance: lookaheadDistance, max_speed: maxSpeed, max_omega: maxOmega,
            slow_radius: slowRadius, control_dt: controlDt, max_steps: maxSteps,
            goal_tolerance: goalTolerance, footprint_radius: footprintRadius,
            stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // planner 인스턴스의 self._progress_index와 같은 역할 — 이 엔진 실행 하나에 묶인
    // tick 간 상태(로봇마다 reset()되는 python과 동치, 매 runPurePursuit 호출이 새 인스턴스).
    let progressIndex = 0
    const tick = (state: RobotState3, _dt: number, tickEmit: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose
        const robotXy: Point = [x, y]

        progressIndex = advanceProgressIndex(referencePath, robotXy, progressIndex)
        const target = lookaheadPoint(referencePath, progressIndex, robotXy, lookaheadDistance)

        const alpha = wrapToPi(Math.atan2(target[1] - y, target[0] - x) - theta)
        const kappa = 2 * Math.sin(alpha) / lookaheadDistance

        const remaining = Math.hypot(goal[0] - x, goal[1] - y)
        let v = maxSpeed * Math.min(1, remaining / slowRadius)

        const omegaRaw = kappa * v
        const omega = Math.max(-maxOmega, Math.min(maxOmega, omegaRaw))
        if (omega !== omegaRaw && Math.abs(kappa) > KAPPA_EPS) {
            // 클램프가 회전율을 바꿨다 -- 실행되는 (v, omega)가 명령된 곡률 kappa를
            // 그대로 그리도록 v를 다시 계산한다. 그렇지 않으면 lookahead 원호보다
            // 조용히 덜 도는(understeer) 결과가 된다.
            v = omega / kappa
        }

        tickEmit({event: "candidate_evaluated", state: [target[0], target[1]], cost: kappa, data: {alpha}})
        return {v, omega}
    }

    runClosedLoop({
        map, startPose, goal, controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

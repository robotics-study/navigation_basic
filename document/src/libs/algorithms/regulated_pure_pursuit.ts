import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {discCollides, Point} from "./sampling_space";
import {distanceToNearest} from "./obstacle_grid";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// 브라우저 라이브 Regulated Pure Pursuit (Macenski, Singh, Martin & Gines 2023,
// "Regulated Pure Pursuit for Robot Path Tracking"; Nav2 기본 local controller).
// 저장소 tracking/regulated_pure_pursuit.py를 문자 그대로 미러한다: Coulter 1992의
// 단순 lookahead-원호 조향(pure_pursuit.ts와 같은 기하)에 세 규제를 얹는다 — 속도에
// 비례하는 adaptive lookahead(Campbell 2007), 곡률에 반비례하는 속도 상한, 장애물
// 근접도에 비례하는 속도 상한. 명령 원호를 앞서 걸어보고 충돌이 예측되면 아예 정지한다.
// 경로 기하(progress index/lookahead 점)는 pure_pursuit.ts와 같은 식이다. python 쪽은
// tracking/_path 공용 모듈로 공유하지만, TS 엔진은 페이지 단위로 파일이 완결되는 기존
// 관례를 따라 이 파일 안에 자체 보유한다 — pure_pursuit.ts에서 import하지 않는다.
export interface RegulatedPurePursuitOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    referencePath: Point[];
    lookaheadTime: number;
    minLookahead: number;
    maxLookahead: number;
    regulatedMinRadius: number;
    proximityDistance: number;
    minRegulatedSpeed: number;
    collisionCheckStep: number;
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

// 이 아래로 곡률이 작으면 "규제 없음" 분기와 클램프-재계산(v = omega / kappa) 둘 다
// 0에 가까운 값으로 나누게 된다. 이 정도로 작은 |kappa|는 사실상 직선이라 규제를
// 건너뛰는 것과 나눗셈을 가드하는 것이 같은 임계값이다. python _KAPPA_EPS와 동일 값.
const KAPPA_EPS = 1e-9;

// lookahead 충돌 검사(아래 5단계)의 호길이 계산에서 alpha = 0과 alpha = ±π 둘레의
// 가드 폭(radian). L_d*alpha/sin(alpha)는 이 두 이웃 모두에서 퇴화한다 — alpha≈0
// 근방은 극한이 L_d로 잘 수렴하지만, alpha≈±π 근방은 alpha가 π 부근에 머무는 동안
// sin(alpha)만 0으로 줄어들어 호길이를 터무니없이 길게 계산한다. 로봇이 거의 일직선인
// 구간을 추종하며 lookahead 원이 그 구간의 끝점을 살짝 넘는 위치에서, 공유 lookahead
// 탐색이 전방 교점 대신 후방 교점을 반환하는 실제 관측 가능한 경우라 두 이웃 모두
// 같은 L_d 대체값을 쓴다(단순 0-나눗셈 가드가 아니라 폭을 가진 가드). python
// _ARC_ALPHA_MARGIN과 동일 값.
const ARC_ALPHA_MARGIN = 0.05;

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

// 로봇 중심 lookahead 원과 선분 a->b의 전방 교점(매개변수 t in [0,1]), 없으면 null.
// |a + t*(b-a) - p|^2 = radius^2 (Coulter 1992 sec. 3의 원-직선 교차)을 풀고 범위 안의
// 큰 근(exit point)을 취한다 — 항상 로봇을 앞으로 이끄는 점을 고르기 위해서다.
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
        // 동률이 된다. 동률일 때 더 앞선 구간을 우선해 진행이 코너를 지나 계속 앞으로 간다.
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

// 호길이 s만큼 상수-곡률 kappa를 따라 전진했을 때의 pose — closed-form 원호 적분
// (local_sim.ts의 integrateUnicycle과 같은 계열이지만, 시간이 아니라 호길이로
// 매개변수화한다: 충돌 검사는 tick이 아니라 거리를 따라 걷기 때문이다).
function propagateArc(pose: Pose, kappa: number, s: number): Pose {
    const [x, y, theta] = pose
    if (Math.abs(kappa) < KAPPA_EPS) {
        return [x + s * Math.cos(theta), y + s * Math.sin(theta), theta]
    }
    const newTheta = theta + kappa * s
    const x2 = x + (Math.sin(newTheta) - Math.sin(theta)) / kappa
    const y2 = y - (Math.cos(newTheta) - Math.cos(theta)) / kappa
    return [x2, y2, wrapToPi(newTheta)]
}

export function runRegulatedPurePursuit(opts: RegulatedPurePursuitOptions): TraceEvent[] {
    const {
        map, startPose, goal, referencePath, lookaheadTime, minLookahead, maxLookahead,
        regulatedMinRadius, proximityDistance, minRegulatedSpeed, collisionCheckStep,
        maxSpeed, maxOmega, slowRadius, controlDt, maxSteps, goalTolerance, footprintRadius,
        stallWindow, stallDistance,
    } = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "regulated_pure_pursuit",
        params: {
            lookahead_time: lookaheadTime, min_lookahead: minLookahead, max_lookahead: maxLookahead,
            regulated_min_radius: regulatedMinRadius, proximity_distance: proximityDistance,
            min_regulated_speed: minRegulatedSpeed, collision_check_step: collisionCheckStep,
            max_speed: maxSpeed, max_omega: maxOmega, slow_radius: slowRadius,
            control_dt: controlDt, max_steps: maxSteps, goal_tolerance: goalTolerance,
            footprint_radius: footprintRadius, stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // planner 인스턴스의 self._progress_index와 같은 역할.
    let progressIndex = 0
    const tick = (state: RobotState3, _dt: number, tickEmit: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose
        const robotXy: Point = [x, y]

        // 1. Adaptive lookahead(Macenski 2023 sec. 3.1; 계보 Campbell 2007): 속도에
        // 비례해 늘어나, 빠른 로봇은 더 멀리, 느린 로봇은 더 가까이 본다. clamp 순서는
        // min(max(.), max_lookahead) — min_lookahead가 max_lookahead보다 크게 선언되면
        // max_lookahead가 이긴다.
        const lookaheadDistance = Math.min(Math.max(lookaheadTime * state.v, minLookahead), maxLookahead)

        // 2. 목표점: Pure Pursuit과 같은 기하.
        progressIndex = advanceProgressIndex(referencePath, robotXy, progressIndex)
        const target = lookaheadPoint(referencePath, progressIndex, robotXy, lookaheadDistance)

        // 3. 명령 곡률(Coulter 1992 기하 — pure_pursuit.ts와 독립적으로 재계산한다.
        // 곡률 공식과 클램프-재계산은 비슷해 보이지만 공용화하면 이 규제 로직이 PP의
        // 평범한 명령에 결합돼 서로 독립적으로 진화하지 못하게 된다).
        const alpha = wrapToPi(Math.atan2(target[1] - y, target[0] - x) - theta)
        const kappa = 2 * Math.sin(alpha) / lookaheadDistance

        // 4. 속도 규제: v는 세 독립 상한의 최솟값이다.
        const remaining = Math.hypot(goal[0] - x, goal[1] - y)
        const vGoal = maxSpeed * Math.min(1, remaining / slowRadius)

        let vCurv: number
        if (Math.abs(kappa) <= KAPPA_EPS) {
            vCurv = maxSpeed
        } else {
            const radius = 1 / Math.abs(kappa)
            vCurv = radius < regulatedMinRadius ? maxSpeed * (radius / regulatedMinRadius) : maxSpeed
        }

        // 근접 휴리스틱(Macenski 2023 sec. 3.2): 원 논문은 costmap 비용에 비례해
        // 감속하지만, costmap 계층이 없는 이 구현은 EDT 거리에 비례해 감속하는
        // 단순화를 쓴다.
        const clearance = distanceToNearest(map, [x, y]) - footprintRadius
        const vProx = clearance < proximityDistance
            ? maxSpeed * Math.max(clearance, 0) / proximityDistance
            : maxSpeed

        // min_regulated_speed 바닥은 위 세 상한에만 적용된다 — 아래 5단계의 충돌
        // 정지는 이 바닥과 무관하게 항상 0까지 내려갈 수 있어야 한다.
        let v = Math.max(Math.min(vGoal, vCurv, vProx), minRegulatedSpeed)

        // 5. Lookahead 충돌 검사(Macenski 2023 sec. 3.3): 명령을 실행하기 전에 그
        // 원호를 앞서 걸어보고, 충돌이 예측되면 명령에 이미 발을 들이고 나서야
        // 알아차리는 대신 아예 멈춘다. sin(alpha)는 alpha->0과 alpha->±π 양쪽에서
        // 0으로 가므로 L_d*alpha/sin(alpha)는 두 지점 모두 퇴화한다 — 위
        // ARC_ALPHA_MARGIN 주석대로 두 이웃 모두 L_d로 대체한다.
        const arcLength = (Math.abs(alpha) < ARC_ALPHA_MARGIN || Math.abs(alpha) > Math.PI - ARC_ALPHA_MARGIN)
            ? lookaheadDistance
            : lookaheadDistance * alpha / Math.sin(alpha)

        let blocked = false
        for (let s = collisionCheckStep; s <= arcLength; s += collisionCheckStep) {
            const pose = propagateArc([x, y, theta], kappa, s)
            if (discCollides(map, footprintRadius, pose[0], pose[1])) {
                blocked = true
                break
            }
        }

        const emitCandidate = (blockedValue: number) => {
            tickEmit({
                event: "candidate_evaluated", state: [target[0], target[1]], cost: kappa,
                data: {
                    alpha, lookahead: lookaheadDistance,
                    curvature_scale: vCurv / maxSpeed, proximity_scale: vProx / maxSpeed,
                    blocked: blockedValue,
                },
            })
        }

        if (blocked) {
            emitCandidate(1)
            return {v: 0, omega: 0}
        }

        // 6. 각속도, 곡률-보존 재계산으로 클램프(pure_pursuit.ts와 같은 패턴).
        const omegaRaw = kappa * v
        const omega = Math.max(-maxOmega, Math.min(maxOmega, omegaRaw))
        if (omega !== omegaRaw && Math.abs(kappa) > KAPPA_EPS) {
            v = omega / kappa
        }

        emitCandidate(0)
        return {v, omega}
    }

    runClosedLoop({
        map, startPose, goal, controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

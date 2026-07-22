import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {Point} from "./sampling_space";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// 브라우저 라이브 Stanley (Thrun et al. 2006, DARPA Grand Challenge, sec. 9.2 +
// Hoffmann et al. 2007의 k_soft 저속 softening). 저장소 tracking/stanley.py를
// 그대로 미러한다: heading 오차와 crosstrack 오차를 전륜축에서 동시에 재고,
// 가상 축간거리 L을 통해 unicycle 명령으로 변환한다. 폐루프 적분·종료 판정은
// local_sim.ts의 runClosedLoop 한 곳이 맡는다(3개 local 엔진 공유 — DRY).
export interface StanleyOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    referencePath: Point[];
    kGain: number;
    kSoft: number;
    wheelbase: number;
    maxSteer: number;
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

// 클램프 시 재계산 단계(v = omega*L/tan(delta))가 0에 가까운 값으로 나누는 것을 막는
// 방어적 가드. max_steer는 pi/2 미만으로 제한돼 tan(delta) 자체는 발산하지 않는다
// (python stanley.py의 _TAN_EPS 미러).
const TAN_EPS = 1e-9;

// TS 엔진은 페이지 단위 교육 코드라 pure_pursuit.ts처럼 이 파일이 자체 보유한다
// (저장소 tracking/_path.py를 공용화하지 않는 기존 관례 — 3중 미러 유지비보다 파일
// 완결성이 우선이라는 판단).
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

// 전륜축 위치를 추종 중인 참조 경로 구간 index로 전진 전용 갱신한다. 단조 증가만
// 허용해, 자기 자신과 교차하는 경로에서 진행이 뒤쪽의 기하적으로 더 가까운
// 교차점으로 되돌아가지 않게 한다 (pure_pursuit.ts의 advanceProgressIndex와 동일 규칙,
// 기준점(probe)만 lookahead 원 위 점 대신 전륜축 점이다).
function advanceProgressIndex(path: Point[], probe: Point, startIndex: number): number {
    if (path.length < 2) return startIndex
    let bestIndex = startIndex
    let bestSqDist = Infinity
    for (let i = startIndex; i < path.length - 1; i++) {
        const closest = closestPointOnSegment(probe, path[i], path[i + 1])
        const d = sqDist(probe, closest)
        // <=, 아니라 <: 연속한 두 구간은 접점을 공유하므로, 전륜축 점이 코너에 정확히
        // 있으면 그 접점에서 끝나거나 시작하는 모든 구간이 동률이 된다. 동률일 때
        // 더 앞선(later) 구간을 우선하면 코너를 지나며 진행이 계속 앞으로 간다.
        if (d <= bestSqDist) {
            bestSqDist = d
            bestIndex = i
        }
    }
    return bestIndex
}

export function runStanley(opts: StanleyOptions): TraceEvent[] {
    const {map, startPose, goal, referencePath, kGain, kSoft, wheelbase, maxSteer, maxSpeed,
           maxOmega, slowRadius, controlDt, maxSteps, goalTolerance, footprintRadius,
           stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "stanley",
        params: {
            k_gain: kGain, k_soft: kSoft, wheelbase, max_steer: maxSteer, max_speed: maxSpeed,
            max_omega: maxOmega, slow_radius: slowRadius, control_dt: controlDt, max_steps: maxSteps,
            goal_tolerance: goalTolerance, footprint_radius: footprintRadius,
            stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // planner 인스턴스의 self._progress_index와 같은 역할 — 이 엔진 실행 하나에 묶인
    // tick 간 상태(로봇마다 reset()되는 python과 동치, 매 runStanley 호출이 새 인스턴스).
    let progressIndex = 0
    const tick = (state: RobotState3, _dt: number, tickEmit: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose

        // 속도 프로파일 먼저: 아래 조향 법칙이 (k_soft + v)로 나누므로 delta 계산 전에
        // v가 정해져 있어야 한다.
        const remaining = Math.hypot(goal[0] - x, goal[1] - y)
        let v = maxSpeed * Math.min(1, remaining / slowRadius)

        // 전륜축 지점: 원 논문이 전륜축 기준으로 두 오차를 정의한다(Thrun 2006 sec. 9.2),
        // 로봇의 후륜축/중심 pose가 아니다.
        const front: Point = [x + wheelbase * Math.cos(theta), y + wheelbase * Math.sin(theta)]

        progressIndex = advanceProgressIndex(referencePath, front, progressIndex)
        const i = progressIndex
        const a = referencePath[i], b = referencePath[i + 1]
        const segLen = Math.hypot(b[0] - a[0], b[1] - a[1])
        const tx = (b[0] - a[0]) / segLen, ty = (b[1] - a[1]) / segLen
        const thetaPath = Math.atan2(ty, tx)
        const psi = wrapToPi(thetaPath - theta)

        const foot = closestPointOnSegment(front, a, b)
        // 접선과 (front - foot)의 외적: 전륜축이 경로 좌측에 있으면 양수. 논문의 e(우측
        // 양수) 부호 규약을 거울상으로 뒤집은 것뿐 — 같은 조향 법칙이고 아래 부호만
        // 다르게 접힌다.
        const e = tx * (front[1] - foot[1]) - ty * (front[0] - foot[0])

        const deltaRaw = psi - Math.atan(kGain * e / (kSoft + v))
        const delta = Math.max(-maxSteer, Math.min(maxSteer, deltaRaw))

        // 후륜축 기준 기구학 bicycle theta_dot = (v/L)*tan(delta)는 omega = (v/L)*tan(delta)로
        // 두면 unicycle 방정식과 정확히 같다 — 근사가 아니라 같은 운동의 다른 매개변수화다
        // (이 기구학 모델을 넘어서는 타이어 슬립/차량 동역학은 다루지 않는다).
        const omegaRaw = v * Math.tan(delta) / wheelbase
        const omega = Math.max(-maxOmega, Math.min(maxOmega, omegaRaw))
        if (omega !== omegaRaw && Math.abs(Math.tan(delta)) > TAN_EPS) {
            // 클램프가 회전율을 바꿨다 -- 실행되는 (v, omega)가 delta로 명령한 곡률을
            // 그대로 그리도록 v를 다시 계산한다(PP의 clamp-recompute 패턴과 동일).
            v = omega * wheelbase / Math.tan(delta)
        }

        tickEmit({event: "candidate_evaluated", state: [foot[0], foot[1]], cost: delta, data: {e, psi, v}})
        return {v, omega}
    }

    runClosedLoop({
        map, startPose, goal, controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

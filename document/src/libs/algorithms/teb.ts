import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {Point} from "./sampling_space";
import {occupiedWithin} from "./obstacle_grid";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// Timed Elastic Band (Rösmann, Feiten, Wösch, Hoffmann & Bertram, ROBOTIK 2012;
// 확장 Rösmann et al., Robotics and Autonomous Systems 88:142-153, 2017,
// DOI 10.1016/j.robot.2016.11.007) 브라우저 라이브 엔진. 저장소
// local_planning/band/teb.py를 연산 순서까지 그대로 미러한다 -- (pose, 구간시간 dt)
// 열을 다목적 비용(추종·장애물·속도/가속·시간최적성·비홀로노믹 제약)에 대해 고정
// 반복 damped gradient descent로 최적화하고 첫 구간에서 명령을 뽑는다. 폐루프
// 적분·종료 판정은 local_sim.ts의 runClosedLoop이 맡는다.
export interface TebOptions {
    map: GridMap;
    startPose: Pose;
    goal: Pose;
    referencePath: Point[];
    wPath: number;
    wObstacle: number;
    wVelocity: number;
    wAcceleration: number;
    wTime: number;
    wKinematics: number;
    maxSpeed: number;
    maxOmega: number;
    aMax: number;
    minObstacleDist: number;
    dtRef: number;
    dtMin: number;
    horizon: number;
    iterations: number;
    stepAlpha: number;
    maxStepXy: number;
    maxStepTheta: number;
    maxStepDt: number;
    maxPoses: number;
    reinitDistance: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

// 이 아래 제곱노름/거리에서는 나눗셈(단위벡터, 세그먼트 방향)이 불안정해 해당 항을
// skip한다 -- tracking/_path.py의 세그먼트 퇴화 가드, elastic_bands.ts의 EPS_SQ와
// 같은 1e-12 상수 (py band/teb.py 미러).
const EPS_SQ = 1e-12;

function clampSym(value: number, bound: number): number {
    return Math.max(-bound, Math.min(bound, value))
}

function sqDist(a: Point, b: Point): number {
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
}

function closestPointOnSegment(p: Point, a: Point, b: Point): Point {
    const [ax, ay] = a
    const [bx, by] = b
    const dx = bx - ax, dy = by - ay
    const segLenSq = dx * dx + dy * dy
    if (segLenSq < EPS_SQ) return a
    const t = Math.max(0, Math.min(1, ((p[0] - ax) * dx + (p[1] - ay) * dy) / segLenSq))
    return [ax + t * dx, ay + t * dy]
}

// 참조 경로 위 로봇의 단조 진행 세그먼트 index -- 전진 전용이라 자기교차 경로에서도
// 뒤로 스냅되지 않는다 (py _geometry.advance_progress_index 미러).
function advanceProgressIndex(path: Point[], probe: Point, startIndex: number): number {
    if (path.length < 2) return startIndex
    let bestIndex = startIndex
    let bestSqDist = Infinity
    for (let i = startIndex; i < path.length - 1; i++) {
        const closest = closestPointOnSegment(probe, path[i], path[i + 1])
        const d = sqDist(probe, closest)
        if (d <= bestSqDist) {
            bestSqDist = d
            bestIndex = i
        }
    }
    return bestIndex
}

// 밴드의 첫 구간 a->b 위 probe의 (클램프하지 않은) 사영 매개변수 -- warm start가
// 로봇이 이 구간을 t>=1로 이미 지나쳤는지 알아야 하므로 clamp된 점이 아니라 t
// 자체가 필요하다 (py teb._segment_t 미러).
function segmentT(probe: Point, a: Point, b: Point): number {
    const dx = b[0] - a[0], dy = b[1] - a[1]
    const segLenSq = dx * dx + dy * dy
    if (segLenSq < EPS_SQ) return 0.0
    return ((probe[0] - a[0]) * dx + (probe[1] - a[1]) * dy) / segLenSq
}

// clip된 참조 경로 폴리라인 위 p의 최근접점 (advanceProgressIndex와 달리 단조
// 제약 없음 -- tick마다의 anchor 조회) (py teb._closest_point_on_polyline 미러).
function closestPointOnPolyline(points: Point[], p: Point): Point {
    let best = points[0]
    let bestSq = Infinity
    for (let i = 0; i < points.length - 1; i++) {
        const c = closestPointOnSegment(p, points[i], points[i + 1])
        const d = sqDist(p, c)
        if (d < bestSq) {
            bestSq = d
            best = c
        }
    }
    return best
}

function pointAtArcLength(points: Point[], s: number): Point {
    if (points.length === 1 || s <= 0.0) return points[0]
    let remaining = s
    for (let i = 0; i < points.length - 1; i++) {
        const a = points[i], b = points[i + 1]
        const segLen = Math.sqrt(sqDist(a, b))
        if (remaining <= segLen) {
            if (segLen < 1e-12) return a
            const t = remaining / segLen
            return [a[0] + t * (b[0] - a[0]), a[1] + t * (b[1] - a[1])]
        }
        remaining -= segLen
    }
    return points[points.length - 1]
}

function resamplePolyline(points: Point[], spacing: number): Point[] {
    if (points.length < 2) return points.slice()
    let total = 0
    for (let i = 0; i < points.length - 1; i++) total += Math.sqrt(sqDist(points[i], points[i + 1]))
    if (total < 1e-12) return [points[0], points[points.length - 1]]
    const nSegments = Math.max(1, Math.round(total / spacing))
    const step = total / nSegments
    const out: Point[] = [points[0]]
    for (let k = 1; k < nSegments; k++) out.push(pointAtArcLength(points, k * step))
    out.push(points[points.length - 1])
    return out
}

// radius 내 최근접 occupied cell 중심과 그 연속 거리 -- occupied_within의 row/col
// 오름차순 리스트에서 strict '<'로 첫 동률을 유지한다 (py _band.nearest_occupied 미러).
function nearestOccupied(map: GridMap, p: Point, radius: number): [Point | null, number] {
    let best: Point | null = null
    let bestSq = Infinity
    for (const o of occupiedWithin(map, p, radius)) {
        const d = sqDist(p, o)
        if (d < bestSq) {
            bestSq = d
            best = o
        }
    }
    if (best === null) return [null, Infinity]
    return [best, Math.sqrt(bestSq)]
}

export function runTeb(opts: TebOptions): TraceEvent[] {
    const {map, startPose, goal, referencePath, wPath, wObstacle, wVelocity, wAcceleration,
           wTime, wKinematics, maxSpeed, maxOmega, aMax, minObstacleDist, dtRef, dtMin,
           horizon, iterations, stepAlpha, maxStepXy, maxStepTheta, maxStepDt, maxPoses,
           reinitDistance, controlDt, maxSteps, goalTolerance, footprintRadius,
           stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "teb",
        params: {
            w_path: wPath, w_obstacle: wObstacle, w_velocity: wVelocity,
            w_acceleration: wAcceleration, w_time: wTime, w_kinematics: wKinematics,
            v_max: maxSpeed, omega_max: maxOmega, a_max: aMax, min_obstacle_dist: minObstacleDist,
            dt_ref: dtRef, dt_min: dtMin, horizon, iterations, step_alpha: stepAlpha,
            max_step_xy: maxStepXy, max_step_theta: maxStepTheta, max_step_dt: maxStepDt,
            max_poses: maxPoses, reinit_distance: reinitDistance,
            control_dt: controlDt, max_steps: maxSteps, goal_tolerance: goalTolerance,
            footprint_radius: footprintRadius, stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // 밴드 상태: poses[0]/poses[-1]은 매 tick 실행된 로봇 pose/현재 local goal로
    // 덮어써진다. 비어 있으면 재초기화 (py _poses/_dts 미러). progressIndex는
    // 밴드 자체의 warm-start 커서(segmentT)와 별개인, 참조 경로 위 전진 사영 커서.
    let poses: Pose[] = []
    let dts: number[] = []
    let progressIndex = 0

    // 참조 경로를 origin에서 arc-length horizon까지 잘라낸 조각과 그 끝점(local
    // goal)을 반환한다. 남은 경로가 horizon보다 짧으면 local goal은 최종 goal 자신
    // (theta는 goalTheta 인자) (py teb._clip 미러).
    const clip = (startIndex: number, origin: Point, goalTheta: number): [Pose, Point[]] => {
        const points: Point[] = [origin]
        let remaining = horizon
        let idx = startIndex
        let prev = origin
        while (idx < referencePath.length - 1) {
            const nxt = referencePath[idx + 1]
            const segLen = Math.sqrt(sqDist(prev, nxt))
            if (remaining <= segLen) {
                let gx: number, gy: number
                if (segLen < 1e-12) {
                    [gx, gy] = nxt
                } else {
                    const t = remaining / segLen
                    gx = prev[0] + t * (nxt[0] - prev[0])
                    gy = prev[1] + t * (nxt[1] - prev[1])
                }
                const theta = Math.atan2(nxt[1] - prev[1], nxt[0] - prev[0])
                points.push([gx, gy])
                return [[gx, gy, theta], points]
            }
            remaining -= segLen
            points.push(nxt)
            prev = nxt
            idx += 1
        }
        const last = referencePath[referencePath.length - 1]
        return [[last[0], last[1], goalTheta], points]
    }

    // 재초기화: clip된 경로를 v_max*dt_ref 간격으로 재샘플. dT_i = ell_i/(0.5*v_max)는
    // 의도적으로 보수적인(느린) 초기값 -- 시간 최적성 항이 이후 줄인다 (py
    // teb._init_band 미러).
    const initBand = (clipPoints: Point[], localGoalTheta: number): [Pose[], number[]] => {
        const spacing = maxSpeed * dtRef
        const pts = resamplePolyline(clipPoints, spacing)
        const newPoses: Pose[] = []
        for (let i = 0; i < pts.length - 1; i++) {
            const dx = pts[i + 1][0] - pts[i][0]
            const dy = pts[i + 1][1] - pts[i][1]
            const theta = dx * dx + dy * dy >= EPS_SQ ? Math.atan2(dy, dx) : 0.0
            newPoses.push([pts[i][0], pts[i][1], theta])
        }
        newPoses.push([pts[pts.length - 1][0], pts[pts.length - 1][1], localGoalTheta])
        const newDts: number[] = []
        for (let i = 0; i < newPoses.length - 1; i++) {
            const ell = Math.sqrt(sqDist(
                [newPoses[i][0], newPoses[i][1]], [newPoses[i + 1][0], newPoses[i + 1][1]]))
            newDts.push(ell / (0.5 * maxSpeed))
        }
        return [newPoses, newDts]
    }

    // Elastic Bands의 overlap 유지와 같은 forward while-loop: 너무 긴 구간은
    // wrap-aware 중간각 midpoint pose로 분할하고, 너무 짧은 구간은 이웃과 병합한다.
    // i+1이 범위를 벗어나면 병합하지 않아 마지막(고정) local-goal pose는 절대
    // 삭제되지 않는다 (py teb._resize 미러).
    const resize = (p: Pose[], d: number[]): void => {
        let i = 0
        while (i < d.length) {
            if (d[i] > 1.5 * dtRef && p.length < maxPoses) {
                const [x0, y0, th0] = p[i]
                const [x1, y1, th1] = p[i + 1]
                const mx = 0.5 * (x0 + x1)
                const my = 0.5 * (y0 + y1)
                const mth = wrapToPi(th0 + 0.5 * wrapToPi(th1 - th0))
                p.splice(i + 1, 0, [mx, my, mth])
                const half = 0.5 * d[i]
                d[i] = half
                d.splice(i + 1, 0, half)
                continue
            }
            if (d[i] < 0.5 * dtRef && p.length > 3 && i + 1 < d.length) {
                p.splice(i + 1, 1)
                d[i] += d[i + 1]
                d.splice(i + 1, 1)
                continue
            }
            i += 1
        }
    }

    // 고정 반복 damped gradient descent 1회: 항 순서 고정 (a) path -> (b) obstacle ->
    // (c) velocity -> (d) acceleration -> (f) kinematics -> (e) time, 각 항 내부는 i
    // 오름차순. 이 순서 자체가 언어 간 결정론 계약의 일부다 (py teb._gradient_step 미러).
    const gradientStep = (p: Pose[], d: number[], anchors: Point[]): void => {
        const n = p.length
        const gx = new Array<number>(n).fill(0.0)
        const gy = new Array<number>(n).fill(0.0)
        const gth = new Array<number>(n).fill(0.0)
        const gdt = new Array<number>(n - 1).fill(0.0)

        const dxs = new Array<number>(n - 1).fill(0.0)
        const dys = new Array<number>(n - 1).fill(0.0)
        const ell = new Array<number>(n - 1).fill(0.0)
        const hasPos = new Array<boolean>(n - 1).fill(false)
        const v = new Array<number>(n - 1).fill(0.0)
        const omega = new Array<number>(n - 1).fill(0.0)
        for (let i = 0; i < n - 1; i++) {
            const dx = p[i + 1][0] - p[i][0]
            const dy = p[i + 1][1] - p[i][1]
            dxs[i] = dx
            dys[i] = dy
            const dSq = dx * dx + dy * dy
            if (dSq < EPS_SQ) {
                ell[i] = 0.0
                v[i] = 0.0
                hasPos[i] = false
            } else {
                ell[i] = Math.sqrt(dSq)
                v[i] = ell[i] / d[i]
                hasPos[i] = true
            }
            omega[i] = wrapToPi(p[i + 1][2] - p[i][2]) / d[i]
        }

        // (a) reference-path attraction.
        for (let i = 1; i < n - 1; i++) {
            const [ax, ay] = anchors[i]
            const c = 2.0 * wPath
            gx[i] += c * (p[i][0] - ax)
            gy[i] += c * (p[i][1] - ay)
        }

        // (b) obstacle clearance -- 최근접 occupied cell 중심까지의 연속 거리
        // (양자화 distance_to_nearest가 아님, p에 대해 연속이라야 gradient가 성립).
        for (let i = 1; i < n - 1; i++) {
            const pi: Point = [p[i][0], p[i][1]]
            const [o, dTilde] = nearestOccupied(map, pi, minObstacleDist)
            if (o === null) continue
            const gI = minObstacleDist - dTilde
            if (gI <= 0.0) continue
            if (dTilde * dTilde < EPS_SQ) continue
            const c = -2.0 * wObstacle * gI / dTilde
            gx[i] += c * (pi[0] - o[0])
            gy[i] += c * (pi[1] - o[1])
        }

        // (c) velocity limits.
        for (let i = 0; i < n - 1; i++) {
            const eV = Math.max(0.0, v[i] - maxSpeed)
            if (eV > 0.0) {
                const c = 2.0 * wVelocity * eV
                if (hasPos[i]) {
                    const coeff = c / (ell[i] * d[i])
                    gx[i] -= coeff * dxs[i]
                    gy[i] -= coeff * dys[i]
                    gx[i + 1] += coeff * dxs[i]
                    gy[i + 1] += coeff * dys[i]
                }
                gdt[i] += c * (-v[i] / d[i])
            }
            const eW = Math.max(0.0, Math.abs(omega[i]) - maxOmega)
            if (eW > 0.0) {
                const sign = omega[i] > 0.0 ? 1.0 : (omega[i] < 0.0 ? -1.0 : 0.0)
                const c = 2.0 * wVelocity * eW * sign
                gth[i] -= c / d[i]
                gth[i + 1] += c / d[i]
                gdt[i] += c * (-omega[i] / d[i])
            }
        }

        // (d) translational acceleration limits.
        for (let i = 0; i < n - 2; i++) {
            const denom = 0.5 * (d[i] + d[i + 1])
            const aI = (v[i + 1] - v[i]) / denom
            const eA = Math.max(0.0, Math.abs(aI) - aMax)
            if (eA <= 0.0) continue
            const signA = aI > 0.0 ? 1.0 : (aI < 0.0 ? -1.0 : 0.0)
            const c = 2.0 * wAcceleration * eA * signA
            const dv1 = c / denom
            const dv0 = -c / denom
            if (hasPos[i]) {
                const coeff0 = dv0 / (ell[i] * d[i])
                gx[i] -= coeff0 * dxs[i]
                gy[i] -= coeff0 * dys[i]
                gx[i + 1] += coeff0 * dxs[i]
                gy[i + 1] += coeff0 * dys[i]
            }
            gdt[i] += dv0 * (-v[i] / d[i])
            if (hasPos[i + 1]) {
                const coeff1 = dv1 / (ell[i + 1] * d[i + 1])
                gx[i + 1] -= coeff1 * dxs[i + 1]
                gy[i + 1] -= coeff1 * dys[i + 1]
                gx[i + 2] += coeff1 * dxs[i + 1]
                gy[i + 2] += coeff1 * dys[i + 1]
            }
            gdt[i + 1] += dv1 * (-v[i + 1] / d[i + 1])
            gdt[i] += c * (-aI * 0.5 / denom)
            gdt[i + 1] += c * (-aI * 0.5 / denom)
        }

        // (f) nonholonomic two-pose-arc kinematics (Rösmann 2012).
        for (let i = 0; i < n - 1; i++) {
            const thI = p[i][2], thI1 = p[i + 1][2]
            const cosSum = Math.cos(thI) + Math.cos(thI1)
            const sinSum = Math.sin(thI) + Math.sin(thI1)
            const hI = cosSum * dys[i] - sinSum * dxs[i]
            const c = 2.0 * wKinematics * hI
            gx[i] += c * sinSum
            gx[i + 1] -= c * sinSum
            gy[i] -= c * cosSum
            gy[i + 1] += c * cosSum
            gth[i] += c * (-Math.sin(thI) * dys[i] - Math.cos(thI) * dxs[i])
            gth[i + 1] += c * (-Math.sin(thI1) * dys[i] - Math.cos(thI1) * dxs[i])
        }

        // (e) time optimality.
        for (let i = 0; i < n - 1; i++) gdt[i] += wTime

        for (let i = 1; i < n - 1; i++) {
            let [xI, yI, thI] = p[i]
            xI -= clampSym(stepAlpha * gx[i], maxStepXy)
            yI -= clampSym(stepAlpha * gy[i], maxStepXy)
            thI = wrapToPi(thI - clampSym(stepAlpha * gth[i], maxStepTheta))
            p[i] = [xI, yI, thI]
        }
        for (let i = 0; i < n - 1; i++) {
            const step = clampSym(stepAlpha * gdt[i], maxStepDt)
            d[i] = Math.max(dtMin, d[i] - step)
        }
    }

    // 최종 최적화 상태에서 모든 비용 항을 (gradient 없이) 재평가 -- band_updated의
    // total_cost 필드용. 솔버 hot loop와 분리된 별도 pass (py teb._total_cost 미러).
    const totalCost = (p: Pose[], d: number[], anchors: Point[]): number => {
        const n = p.length
        let total = 0.0
        for (let i = 1; i < n - 1; i++) {
            const [ax, ay] = anchors[i]
            total += wPath * ((p[i][0] - ax) ** 2 + (p[i][1] - ay) ** 2)
        }
        for (let i = 1; i < n - 1; i++) {
            const pi: Point = [p[i][0], p[i][1]]
            const [o, dTilde] = nearestOccupied(map, pi, minObstacleDist)
            if (o !== null) {
                const gI = Math.max(0.0, minObstacleDist - dTilde)
                total += wObstacle * gI * gI
            }
        }
        const v = new Array<number>(n - 1).fill(0.0)
        for (let i = 0; i < n - 1; i++) {
            const dx = p[i + 1][0] - p[i][0]
            const dy = p[i + 1][1] - p[i][1]
            const dSq = dx * dx + dy * dy
            v[i] = dSq >= EPS_SQ ? Math.sqrt(dSq) / d[i] : 0.0
            const omegaI = wrapToPi(p[i + 1][2] - p[i][2]) / d[i]
            const eV = Math.max(0.0, v[i] - maxSpeed)
            const eW = Math.max(0.0, Math.abs(omegaI) - maxOmega)
            total += wVelocity * (eV * eV + eW * eW)
        }
        for (let i = 0; i < n - 2; i++) {
            const denom = 0.5 * (d[i] + d[i + 1])
            const aI = (v[i + 1] - v[i]) / denom
            const eA = Math.max(0.0, Math.abs(aI) - aMax)
            total += wAcceleration * eA * eA
        }
        for (let i = 0; i < n - 1; i++) {
            const thI = p[i][2], thI1 = p[i + 1][2]
            const dx = p[i + 1][0] - p[i][0]
            const dy = p[i + 1][1] - p[i][1]
            const hI = (Math.cos(thI) + Math.cos(thI1)) * dy - (Math.sin(thI) + Math.sin(thI1)) * dx
            total += wKinematics * hI * hI
        }
        total += wTime * d.reduce((s, x) => s + x, 0.0)
        return total
    }

    const emitBand = (
        tickEmit: EmitFn, p: Pose[], d: number[], iters: number, cost: number,
    ): void => {
        const band = [[p[0][0], p[0][1], p[0][2], 0.0]]
        for (let i = 1; i < p.length; i++) band.push([p[i][0], p[i][1], p[i][2], d[i - 1]])
        tickEmit({
            event: "band_updated",
            band,
            data: {iterations: iters, poses: p.length, total_cost: cost,
                   horizon_time: d.reduce((s, x) => s + x, 0.0)},
        })
    }

    const tick = (state: RobotState3, _dt: number, tickEmit: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose
        const robotXy: Point = [x, y]

        progressIndex = advanceProgressIndex(referencePath, robotXy, progressIndex)
        const origin = closestPointOnSegment(
            robotXy, referencePath[progressIndex], referencePath[progressIndex + 1])
        const [localGoal, clipPoints] = clip(progressIndex, origin, goal[2])

        const needReinit = poses.length === 0 || Math.sqrt(sqDist(
            [poses[poses.length - 1][0], poses[poses.length - 1][1]],
            [localGoal[0], localGoal[1]])) > reinitDistance

        let p: Pose[], d: number[]
        if (needReinit) {
            [p, d] = initBand(clipPoints, localGoal[2])
        } else {
            p = poses
            d = dts
            // Warm start: 위 참조 경로 진행 index가 아니라 밴드 자신의 첫 구간을
            // 기준으로 한다 -- 밴드는 최적화가 남겨둔 속도로 로봇을 뒤따른다.
            while (p.length > 2 && segmentT(robotXy, [p[0][0], p[0][1]], [p[1][0], p[1][1]]) >= 1.0) {
                p.shift()
                d.shift()
            }
        }

        p[0] = [x, y, theta]
        p[p.length - 1] = localGoal

        if (p.length < 3) {
            // 퇴화 케이스: 로봇이 사실상 local goal에 도달해 최적화할 내부 pose가
            // 없다 -- 솔버를 생략하고 goal을 향해 비례 heading 제어로 직진한다
            // (Elastic Bands의 명령 추출과 같은 구조). 게인은 omega_max/pi -- TEB는
            // 전용 heading_gain을 선언하지 않으므로, |alpha|가 pi를 넘지 않아 클램프가
            // 구조적으로 no-op이 되게 하는 값을 쓴다 (임의 상수 도입 대신).
            poses = p
            dts = d
            emitBand(tickEmit, p, d, 0, 0.0)
            const [gx, gy] = localGoal
            const alpha = wrapToPi(Math.atan2(gy - y, gx - x) - theta)
            const vCmd = maxSpeed * Math.max(Math.cos(alpha), 0.0)
            const omegaCmd = clampSym((maxOmega / Math.PI) * alpha, maxOmega)
            return {v: vCmd, omega: omegaCmd}
        }

        resize(p, d)

        // 이번 tick 동안 고정되는 anchor: 최적화가 옮기기 전, 각 내부 pose의 초기
        // 위치에서 clip된 참조 경로 위 최근접점 -- 움직이는 목표라면 추종 항이
        // 쫓아가야 할 pose를 오히려 쫓게 된다.
        const anchors: Point[] = Array.from({length: p.length}, () => [0.0, 0.0] as Point)
        for (let i = 1; i < p.length - 1; i++) {
            anchors[i] = closestPointOnPolyline(clipPoints, [p[i][0], p[i][1]])
        }

        for (let k = 0; k < iterations; k++) gradientStep(p, d, anchors)

        poses = p
        dts = d

        const cost = totalCost(p, d, anchors)
        emitBand(tickEmit, p, d, iterations, cost)

        const dx0 = p[1][0] - p[0][0]
        const dy0 = p[1][1] - p[0][1]
        const ell0 = Math.sqrt(dx0 * dx0 + dy0 * dy0)
        const sigma = dx0 * Math.cos(theta) + dy0 * Math.sin(theta) >= 0.0 ? 1.0 : -1.0
        const vCmd = clampSym(sigma * ell0 / d[0], maxSpeed)
        const omegaCmd = clampSym(wrapToPi(p[1][2] - p[0][2]) / d[0], maxOmega)
        return {v: vCmd, omega: omegaCmd}
    }

    runClosedLoop({
        map, startPose, goal: [goal[0], goal[1]], controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

import {GridMap} from "../grid";
import {occupiedWithin} from "./obstacle_grid";
import {EmitFn, headingCommand, RobotState3, VelocityCommand, wrapToPi} from "./local_sim";

// VO/RVO/ORCA(Fiorini & Shiller 1998, DOI 10.1177/027836499801700706 / van den
// Berg, Lin & Manocha 2008, DOI 10.1109/ROBOT.2008.4543489 / van den Berg, Guy,
// Lin & Manocha 2011, DOI 10.1007/978-3-642-19457-3_1)이 공유하는 기하/LP
// 계산 — 저장소 velocity/_velocity_obstacle.py를 그대로 미러한다. DWA의 명령
// 공간 롤아웃 채점과 달리, 이 계열은 로봇의 *다음 절대 속도*를 속도 공간에서
// 바로 고른다: 이웃/장애물마다 금지 영역(원뿔 또는 half-plane)을 하나씩 만들고,
// 목표를 향하는 선호 속도에 가장 가까운 admissible 속도를 고른다.

export type Point = [number, number];

// 0으로 나누기 방지용 수치 guard — 물리 단위가 없는 튜닝 대상이 아니라서
// config 값이 아닌 모듈 상수다 (python _EPS 미러).
export const EPS = 1e-9;
// infeasible(원뿔/half-plane 위반) 후보 비용에 더하는 페널티 — feasible 후보가
// v_pref에 아무리 가까운 infeasible 후보보다도 항상 이기게 한다 (python _PENALTY 미러).
export const PENALTY = 1e6;

export interface DynamicObstacle {
    position: Point;
    velocity: Point;
    radius: number;
}

// Truncated velocity obstacle(Fiorini & Shiller 1998): apex를 tau초 유지하면
// obstacle의 radius 안으로 들어가는 (절대) 속도 집합. left/right는 apex에서
// 두 접선 방향의 단위 벡터. full은 이미 겹친 쌍(radius >= dist)이라 금지 영역이
// 속도 평면 전체임을 뜻한다.
export interface Cone {
    apex: Point;
    axis: Point;
    cosHalf: number;
    dist: number;
    radius: number;
    tau: number;
    full: boolean;
    left: Point;
    right: Point;
}

// ORCA half-plane: feasible 영역은 {v : dot(v - point, normal) >= 0}, normal은 단위벡터.
export type HalfPlane = [Point, Point];

// 목표를 향하는 속도: 목표까지 max_speed 미터 이내로 들어오면 속도를 줄여
// 에피소드가 goal 주변을 맴돌거나 지나치는 대신 REACHED로 정착하게 한다
// (RVO2의 goal heuristic).
export function preferredVelocity(pose: [number, number, number], goal: Point, maxSpeed: number): Point {
    const dx = goal[0] - pose[0]
    const dy = goal[1] - pose[1]
    const dist = Math.hypot(dx, dy)
    if (dist < EPS) return [0, 0]
    const speed = Math.min(maxSpeed, dist)
    return [dx / dist * speed, dy / dist * speed]
}

// 속도 공간에서 고른 목표를 (v, omega)로 투영 — reactive 계열의 제자리 회전 후
// 전진 조향 법칙을 재사용한다. 고른 속도의 크기 자체가 그 법칙의 속도 상한이 되고,
// heading 오차로 cos 게이트가 걸린다.
export function velocityToCommand(
    vNew: Point, theta: number, maxOmega: number, headingGain: number,
): VelocityCommand {
    const speed = Math.hypot(vNew[0], vNew[1])
    if (speed < EPS) return {v: 0, omega: 0}
    const desired = Math.atan2(vNew[1], vNew[0])
    const thetaErr = wrapToPi(desired - theta)
    return headingCommand(thetaErr, headingGain, speed, maxOmega)
}

export function truncatedVoCone(
    relPos: Point, combinedRadius: number, apexVel: Point, tau: number,
): Cone {
    const [px, py] = relPos
    const dist = Math.hypot(px, py)
    if (dist <= combinedRadius + EPS) {
        // 이미 겹친 상태: 어떤 상대 속도든 (더 깊은) 침투로 이어지므로 금지
        // 영역이 평면 전체다.
        return {
            apex: apexVel, axis: [1, 0], cosHalf: -1, dist, radius: combinedRadius, tau,
            full: true, left: [1, 0], right: [1, 0],
        }
    }
    const ux = px / dist
    const uy = py / dist
    const sinHalf = combinedRadius / dist
    const cosHalf = Math.sqrt(Math.max(0, 1 - sinHalf * sinHalf))
    const left: Point = [ux * cosHalf - uy * sinHalf, ux * sinHalf + uy * cosHalf]
    const right: Point = [ux * cosHalf + uy * sinHalf, -ux * sinHalf + uy * cosHalf]
    return {apex: apexVel, axis: [ux, uy], cosHalf, dist, radius: combinedRadius, tau, full: false, left, right}
}

export function inVelocityObstacle(v: Point, cone: Cone): boolean {
    if (cone.full) return true
    const wx = v[0] - cone.apex[0]
    const wy = v[1] - cone.apex[1]
    const wlen = Math.hypot(wx, wy)
    if (wlen < EPS) return false  // 상대 정지는 절대 충돌하지 않는다
    const wproj = wx * cone.axis[0] + wy * cone.axis[1]
    if (wproj <= 0) return false  // obstacle에서 멀어지는 방향
    const cosAng = wproj / wlen
    if (cosAng < cone.cosHalf) return false  // 원뿔의 각 범위 밖
    // tau-truncation(근사 평면화): 간격이 tau 안에 닫혀야 한다.
    if (wproj < (cone.dist - cone.radius) / cone.tau) return false
    return true
}

export function coneToConstraint(cone: Cone): number[] {
    return [cone.apex[0], cone.apex[1], cone.left[0], cone.left[1], cone.right[0], cone.right[1]]
}

export function halfplaneToConstraint(plane: HalfPlane): number[] {
    const [point, normal] = plane
    return [point[0], point[1], normal[0], normal[1]]
}

// RVO apex(van den Berg et al. 2008): 원뿔의 apex를 상대 obstacle의 속도에서
// 두 속도의 중점 쪽으로 옮겨, 대칭적인 마주침에서 양쪽이 회피 부담을 절반씩
// 진다. reciprocity=0은 순수 VO(상대가 전부 책임), 1은 원뿔이 자기 속도에 붕괴.
export function rvoApex(vSelf: Point, vOther: Point, reciprocity: number): Point {
    return [
        (1 - reciprocity) * vOther[0] + reciprocity * vSelf[0],
        (1 - reciprocity) * vOther[1] + reciprocity * vSelf[1],
    ]
}

// VO/RVO용 결정적 극좌표 후보 격자(난수 없음): speed-외측/angle-내측 순회라
// py/cpp/TS 채점·동률 처리가 bit-identical하게 유지된다. 후보 0은 v_pref
// 자체(max_speed로 클램프)라, 완전히 열린 tick은 정확히 비용 0으로 모든
// 동률에서 이긴다.
export function sampleReachableVelocities(
    vPref: Point, maxSpeed: number, speedSamples: number, angleSamples: number,
): Point[] {
    const speed = Math.hypot(vPref[0], vPref[1])
    const v0: Point = speed > maxSpeed ? [vPref[0] / speed * maxSpeed, vPref[1] / speed * maxSpeed] : vPref
    const out: Point[] = [v0]
    for (let si = 0; si <= speedSamples; si++) {
        const s = maxSpeed * si / speedSamples
        for (let ai = 0; ai < angleSamples; ai++) {
            const ang = 2 * Math.PI * ai / angleSamples
            out.push([s * Math.cos(ang), s * Math.sin(ang)])
        }
    }
    return out
}

// sensorRadius 안의 점유 셀을 velocity-0 obstacle로 접어 넣어, 정적 벽과 동적
// 이웃을 VO/RVO/ORCA 하나의 코드 경로로 다룬다. occupiedWithin이 이미
// row/col 오름차순(결정적)이다 (python static_obstacles 미러).
export function staticObstacles(
    map: GridMap, center: Point, sensorRadius: number, obstacleRadius: number,
): DynamicObstacle[] {
    return occupiedWithin(map, center, sensorRadius).map((p) => ({
        position: p, velocity: [0, 0] as Point, radius: obstacleRadius,
    }))
}

const dist = (a: Point, b: Point): number => Math.hypot(a[0] - b[0], a[1] - b[1])

export type ApexOf = (o: DynamicObstacle) => Point;

// VO와 RVO가 공유하는 후보-격자 속도 선택: 둘의 차이는 오직 각 obstacle의
// 원뿔 apex를 어디에 두는지(apex_of)뿐이라, 원뿔 구성/스캔/페널티 비용 루프를
// 여기 한 곳에 둔다(vo.ts/rvo.ts 사이 중복 스캔을 피한다).
export function selectSampledVelocity(
    vPref: Point, obstacles: DynamicObstacle[], pos: Point, agentRadius: number, neighborDist: number,
    timeHorizon: number, maxSpeed: number, speedSamples: number, angleSamples: number, apexOf: ApexOf,
): [Point, number[][]] {
    const cones = obstacles
        .filter((o) => dist(o.position, pos) < neighborDist + o.radius)
        .map((o) => truncatedVoCone(
            [o.position[0] - pos[0], o.position[1] - pos[1]], agentRadius + o.radius, apexOf(o), timeHorizon,
        ))
    const candidates = sampleReachableVelocities(vPref, maxSpeed, speedSamples, angleSamples)
    let best = candidates[0]
    let bestCost = Infinity
    for (const v of candidates) {  // 고정 순회 순서 -- sampleReachableVelocities 참고
        const violated = cones.some((c) => inVelocityObstacle(v, c))
        const cost = dist(v, vPref) + (violated ? PENALTY : 0)
        if (cost < bestCost) {  // strict <: 첫 후보(v_pref)가 동률에서 이긴다
            bestCost = cost
            best = v
        }
    }
    const constraints = cones.map(coneToConstraint)
    return [best, constraints]
}

// obstacle 하나에 대한 ORCA line(van den Berg et al. 2011; RVO2 레퍼런스
// 구현의 Agent::computeNewVelocity 미러). relPos = other - self, relVel =
// self - other(둘 다 절대 속도). feasible 영역이 dot(v - point, normal) >= 0인
// (point, normal)을 반환한다.
export function orcaHalfPlane(
    relPos: Point, relVel: Point, vSelf: Point, combinedRadius: number, tau: number, dt: number,
): HalfPlane {
    const [px, py] = relPos
    const [vx, vy] = relVel
    const distSq = px * px + py * py
    const r = combinedRadius
    const rSq = r * r
    let normal: Point
    let u: Point
    if (distSq > rSq) {  // 현재 충돌 중이 아님
        const invTau = 1 / tau
        const wx = vx - invTau * px
        const wy = vy - invTau * py
        const wLenSq = wx * wx + wy * wy
        const dot1 = wx * px + wy * py
        if (dot1 < 0 && dot1 * dot1 > rSq * wLenSq) {
            // truncated cutoff 원 앞쪽: 그 원으로 사영한다.
            const wLen = Math.sqrt(wLenSq)
            const unitWx = wx / wLen
            const unitWy = wy / wLen
            normal = [unitWx, unitWy]
            u = [(r * invTau - wLen) * unitWx, (r * invTau - wLen) * unitWy]
        } else {
            // 두 접선 leg 중 하나로 사영한다.
            const leg = Math.sqrt(distSq - rSq)
            let dirx: number
            let diry: number
            if (px * wy - py * wx > 0) {
                dirx = (px * leg - py * r) / distSq
                diry = (px * r + py * leg) / distSq
            } else {
                // RVO2의 실제 오른쪽 leg 접선은 왼쪽-leg 식을 단순 반전한 것의
                // 부호를 뒤집은 것이다(Agent.cpp computeNewVelocity) -- 이 flip이
                // 없으면 유도된 half-plane normal이 충돌 원뿔에서 멀어지는
                // 대신 그 안쪽을 향한다(ORCA LP 유닛 테스트로 확인됨).
                dirx = -(px * leg + py * r) / distSq
                diry = (px * r - py * leg) / distSq
            }
            const dot2 = vx * dirx + vy * diry
            u = [dot2 * dirx - vx, dot2 * diry - vy]
            normal = [-diry, dirx]
        }
    } else {  // 이미 충돌 중: 이번 tick의 cutoff 원으로 사영한다
        const invDt = 1 / dt
        const wx = vx - invDt * px
        const wy = vy - invDt * py
        const wLen = Math.hypot(wx, wy)
        let unitWx: number
        let unitWy: number
        if (wLen < EPS) {
            unitWx = 1
            unitWy = 0  // 완전히 겹치는 퇴화 케이스 fallback
        } else {
            unitWx = wx / wLen
            unitWy = wy / wLen
        }
        normal = [unitWx, unitWy]
        u = [(r * invDt - wLen) * unitWx, (r * invDt - wLen) * unitWy]
    }
    const point: Point = [vSelf[0] + 0.5 * u[0], vSelf[1] + 0.5 * u[1]]
    return [point, normal]
}

const det = (a: Point, b: Point): number => a[0] * b[1] - a[1] * b[0]
const vdot = (a: Point, b: Point): number => a[0] * b[0] + a[1] * b[1]
const vsub = (a: Point, b: Point): Point => [a[0] - b[0], a[1] - b[1]]
const vunit = (v: Point): Point => {
    const n = Math.hypot(v[0], v[1])
    if (n < EPS) return [0, 0]
    return [v[0] / n, v[1] / n]
}
// direction(RVO2의 Line.direction)은 normal을 -90도 회전한 것 -- feasibility
// det(direction, v - point) >= 0이 dot(v - point, normal) >= 0과 일치하도록.
const directionOf = (normal: Point): Point => [normal[1], -normal[0]]
const normalOf = (direction: Point): Point => [-direction[1], direction[0]]

// RVO2 linearProgram1을 (point, normal) 표현으로 옮긴 것: line_no 위에서
// max-speed 원과 그 이전 모든 line을 만족시키며 1차원 부분문제를 최적화한다.
// infeasible(빈) 구간에서는 (false, point)를 반환한다 -- 절대 raise하지 않는다.
function lp1(
    halfPlanes: HalfPlane[], lineNo: number, optVelocity: Point, maxSpeed: number, directionOpt: boolean,
): [boolean, Point] {
    const [point, normal] = halfPlanes[lineNo]
    const direction = directionOf(normal)
    const dotProduct = vdot(point, direction)
    const discriminant = dotProduct * dotProduct + maxSpeed * maxSpeed - vdot(point, point)
    if (discriminant < 0) return [false, point]
    const sqrtDiscriminant = Math.sqrt(discriminant)
    let tLeft = -dotProduct - sqrtDiscriminant
    let tRight = -dotProduct + sqrtDiscriminant
    for (let i = 0; i < lineNo; i++) {
        const [pI, nI] = halfPlanes[i]
        const dI = directionOf(nI)
        const denominator = det(direction, dI)
        const numerator = det(dI, vsub(point, pI))
        if (Math.abs(denominator) <= EPS) {
            if (numerator < 0) return [false, point]
            continue
        }
        const t = numerator / denominator
        if (denominator >= 0) tRight = Math.min(tRight, t)
        else tLeft = Math.max(tLeft, t)
        if (tLeft > tRight) return [false, point]
    }
    let t: number
    if (directionOpt) {
        t = vdot(optVelocity, direction) > 0 ? tRight : tLeft
    } else {
        t = Math.max(tLeft, Math.min(tRight, vdot(direction, vsub(optVelocity, point))))
    }
    return [true, [point[0] + t * direction[0], point[1] + t * direction[1]]]
}

// RVO2 linearProgram2: 위반되는 line마다 그 1차원 부분문제로 점증적으로
// 재최적화한다. (result, fail_index)를 반환하며, fail_index ==
// half_planes.length는 모든 line을 만족했다는 뜻이다.
function lp2(
    halfPlanes: HalfPlane[], optVelocity: Point, maxSpeed: number, directionOpt: boolean,
): [Point, number] {
    let result: Point
    if (directionOpt) {
        result = [optVelocity[0] * maxSpeed, optVelocity[1] * maxSpeed]
    } else if (Math.hypot(optVelocity[0], optVelocity[1]) > maxSpeed) {
        const u = vunit(optVelocity)
        result = [u[0] * maxSpeed, u[1] * maxSpeed]
    } else {
        result = optVelocity
    }
    for (let i = 0; i < halfPlanes.length; i++) {
        const [point, normal] = halfPlanes[i]
        if (vdot(vsub(result, point), normal) < 0) {
            const saved = result
            const [ok, candidate] = lp1(halfPlanes, i, optVelocity, maxSpeed, directionOpt)
            if (!ok) return [saved, i]
            result = candidate
        }
    }
    return [result, halfPlanes.length]
}

export function linearProgram2d(
    halfPlanes: HalfPlane[], vPref: Point, maxSpeed: number,
): [boolean, Point, number] {
    const [result, failIndex] = lp2(halfPlanes, vPref, maxSpeed, false)
    return [failIndex === halfPlanes.length, result, failIndex]
}

// RVO2 linearProgram3: 과제약(over-constrained) fallback. beginLine부터의 모든
// line에 걸친 총 침투를 최소화한다. 항상 Point를 반환한다 -- 절대 raise하지
// 않는다(hot path). beginLine 이전에 이미 만족된 prefix 결과는(linear_program_2d가
// 이미 계산한 것과 결정적으로 bit-identical하므로) 전달받지 않고 다시 계산한다.
export function linearProgram3d(
    halfPlanes: HalfPlane[], beginLine: number, vPref: Point, maxSpeed: number,
): Point {
    let [result] = lp2(halfPlanes.slice(0, beginLine), vPref, maxSpeed, false)
    let distance = 0
    for (let i = beginLine; i < halfPlanes.length; i++) {
        const [point, normal] = halfPlanes[i]
        const direction = directionOf(normal)
        if (vdot(vsub(result, point), normal) < -distance) {
            const projLines: HalfPlane[] = []
            for (let j = 0; j < i; j++) {
                const [pJ, nJ] = halfPlanes[j]
                const dJ = directionOf(nJ)
                const denominator = det(direction, dJ)
                let newPoint: Point
                if (Math.abs(denominator) <= EPS) {
                    if (vdot(direction, dJ) > 0) continue
                    newPoint = [0.5 * (point[0] + pJ[0]), 0.5 * (point[1] + pJ[1])]
                } else {
                    const t = det(dJ, vsub(point, pJ)) / denominator
                    newPoint = [point[0] + t * direction[0], point[1] + t * direction[1]]
                }
                const newDir = vunit([dJ[0] - direction[0], dJ[1] - direction[1]])
                projLines.push([newPoint, normalOf(newDir)])
            }
            const [candidate, failJ] = lp2(projLines, normal, maxSpeed, true)
            // 여기서의 실패는 direction-optimizing 1차원 부분문제조차 과제약이라는
            // 뜻이다 -- RVO2는 이를 이미 feasible한 점 주변의 부동소수 잡음으로
            // 보고 raise 대신 이전 result를 유지한다(이 함수의 hot-path 계약).
            if (failJ === projLines.length) result = candidate
            distance = -vdot(vsub(result, point), normal)
        }
    }
    return result
}

// `_select_velocity`가 반환하는 (다음 절대 속도, trace constraints) 쌍.
export type SelectVelocityFn = (
    vPref: Point, neighbors: DynamicObstacle[], statics: DynamicObstacle[], state: RobotState3, dt: number,
) => [Point, number[][]];

export interface VelocityObstacleOptions {
    maxSpeed: number;
    maxOmega: number;
    headingGain: number;
    agentRadius: number;
    neighborDist: number;
    obstacleRadius: number;
}

// VelocityObstaclePlanner.command_with_neighbors 미러: 정적 장애물 수집 +
// 선호 속도 계산 + strategy 위임(select_velocity) + velocity_obstacle trace
// 방출 + (v, omega) 투영까지, VO/RVO/ORCA 세 엔진이 공유하는 template method.
export function commandWithNeighbors(
    selectVelocity: SelectVelocityFn, opts: VelocityObstacleOptions, map: GridMap,
    state: RobotState3, goal: Point, neighbors: DynamicObstacle[], dt: number, emit?: EmitFn,
): VelocityCommand {
    const [x, y, theta] = state.pose
    const statics = staticObstacles(map, [x, y], opts.neighborDist, opts.obstacleRadius)
    const vPref = preferredVelocity(state.pose, goal, opts.maxSpeed)
    const [vNew, constraints] = selectVelocity(vPref, neighbors, statics, state, dt)
    if (emit) {
        emit({
            event: "velocity_obstacle",
            state: [x, y, theta],
            constraints,
            data: {pref_vx: vPref[0], pref_vy: vPref[1], new_vx: vNew[0], new_vy: vNew[1]},
        })
    }
    return velocityToCommand(vNew, theta, opts.maxOmega, opts.headingGain)
}

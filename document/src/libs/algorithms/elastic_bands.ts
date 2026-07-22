import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {Point} from "./sampling_space";
import {distanceToNearest, occupiedWithin} from "./obstacle_grid";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// Elastic Bands (Quinlan & Khatib, "Elastic bands: connecting path planning and
// control," ICRA 1993, DOI 10.1109/ROBOT.1993.291936) 브라우저 라이브 엔진. 저장소
// local_planning/band/elastic_bands.py를 연산 순서까지 그대로 미러한다 -- 참조 경로를
// 등간격 bubble(중심 + clearance 반경) 열로 초기화한 뒤, 매 tick 내부 수축력과 외부
// 반발력으로 변형하고 overlap 유지 후 폴리라인 위 lookahead 점을 비례 heading 제어로
// 추종한다. 폐루프 적분·종료 판정은 local_sim.ts의 runClosedLoop이 맡는다.
export interface ElasticBandsOptions {
    map: GridMap;
    startPose: Pose;
    goal: [number, number];
    referencePath: Point[];
    kContraction: number;
    kRepulsion: number;
    rhoMax: number;
    rhoInfluence: number;
    rhoMin: number;
    stepSize: number;
    deformIterations: number;
    repairIterations: number;
    repairStep: number;
    overlapFactor: number;
    maxBubbles: number;
    bubbleSpacing: number;
    lookaheadDistance: number;
    headingGain: number;
    maxSpeed: number;
    maxOmega: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

// 이 아래 제곱노름에서는 단위벡터 나눗셈이 불안정해 해당 항을 skip한다 --
// tracking/_path.py의 세그먼트 퇴화 가드와 같은 1e-12 상수 (py band/_band.py, /elastic_bands.py 미러).
const EPS_SQ = 1e-12;

function sqDist(a: Point, b: Point): number {
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
}

// 폴리라인 위 arc-length s 지점(끝에 도달하면 마지막 점으로 clamp) -- 밴드 초기화
// 재샘플과 명령 추출의 lookahead 점 둘 다가 쓴다 (py _band.point_at_arclength 미러).
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

// 폴리라인을 균등 arc-length 간격으로 재샘플 (시작/끝점 보존). 구간 수는
// max(1, round(total/spacing)) -- global_planning/sampling 계열과 같은 반올림 계약
// (py _band.resample_polyline 미러).
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

export function runElasticBands(opts: ElasticBandsOptions): TraceEvent[] {
    const {map, startPose, goal, referencePath, kContraction, kRepulsion, rhoMax, rhoInfluence,
           rhoMin, stepSize, deformIterations, repairIterations, repairStep, overlapFactor,
           maxBubbles, bubbleSpacing, lookaheadDistance, headingGain, maxSpeed, maxOmega,
           controlDt, maxSteps, goalTolerance, footprintRadius, stallWindow, stallDistance} = opts

    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "elastic_bands",
        params: {
            k_contraction: kContraction, k_repulsion: kRepulsion, rho_max: rhoMax,
            rho_influence: rhoInfluence, rho_min: rhoMin, step_size: stepSize,
            deform_iterations: deformIterations, repair_iterations: repairIterations,
            repair_step: repairStep, overlap_factor: overlapFactor, max_bubbles: maxBubbles,
            bubble_spacing: bubbleSpacing, lookahead_distance: lookaheadDistance,
            heading_gain: headingGain, v_max: maxSpeed, omega_max: maxOmega,
            control_dt: controlDt, max_steps: maxSteps, goal_tolerance: goalTolerance,
            footprint_radius: footprintRadius, stall_window: stallWindow, stall_distance: stallDistance,
        },
    })

    // 밴드 상태: 중심/반경 병렬 배열. 비어 있으면 "밴드 없음"(첫 tick / 직전 tick broken)
    // -- 다음 tick이 task.referencePath로부터 재초기화한다 (py _centers/_radii 미러).
    let centers: Point[] = []
    let radii: number[] = []

    const clearance = (p: Point): number => Math.min(distanceToNearest(map, p), rhoMax)

    // Jacobi 1 pass: 모든 힘을 pass 시작 시점 스냅샷에서 계산한 뒤 변위를 일괄
    // 적용한다 -- 순서 의존이 없어 py/C++/TS가 수치적으로 동일하다 (py _deform_once 미러).
    const deformOnce = (): void => {
        const n = centers.length
        const deltas: Point[] = Array.from({length: n}, () => [0.0, 0.0] as Point)
        for (let i = 1; i < n - 1; i++) {
            const [cx, cy] = centers[i]

            // 내부 수축력: 양 이웃 방향 단위벡터 합. 접선 성분은 간격 균등화에
            // 필요하므로 제거하지 않는다.
            let fcx = 0.0, fcy = 0.0
            for (const j of [i - 1, i + 1]) {
                const [nx, ny] = centers[j]
                const dx = nx - cx, dy = ny - cy
                const dSq = dx * dx + dy * dy
                if (dSq < EPS_SQ) continue
                const d = Math.sqrt(dSq)
                fcx += dx / d
                fcy += dy / d
            }
            fcx *= kContraction
            fcy *= kContraction

            // 외부 반발력: rho_influence 내 occupied cell 전체에 대한 합산
            // (occupied_within의 row/col 오름차순 순서 그대로 누적) -- 다중 cell
            // 장애물 내부에 갇힌 bubble도 최근접 경계 방향의 순 반발력을 느끼게 한다.
            let frx = 0.0, fry = 0.0
            for (const [ox, oy] of occupiedWithin(map, [cx, cy], rhoInfluence)) {
                const dx = cx - ox, dy = cy - oy
                const dSq = dx * dx + dy * dy
                if (dSq < EPS_SQ) continue
                const d = Math.sqrt(dSq)
                frx += (rhoInfluence - d) * dx / d
                fry += (rhoInfluence - d) * dy / d
            }
            frx *= kRepulsion
            fry *= kRepulsion

            // 접선 성분 제거는 외부력에만 적용 (Quinlan & Khatib 1993): 반발력의 접선
            // 성분은 bubble을 밴드 방향으로 미끄러뜨려 뭉치게 하므로 제거한다.
            let tx = centers[i + 1][0] - centers[i - 1][0]
            let ty = centers[i + 1][1] - centers[i - 1][1]
            const tSq = tx * tx + ty * ty
            if (tSq >= EPS_SQ) {
                const tNorm = Math.sqrt(tSq)
                tx /= tNorm
                ty /= tNorm
                const proj = frx * tx + fry * ty
                frx -= proj * tx
                fry -= proj * ty
            }

            let dxStep = stepSize * (fcx + frx)
            let dyStep = stepSize * (fcy + fry)
            // 이동량 clamp 하한 = repair_step -- rho=0 bubble(장애물 내부 시작)이
            // 영구 동결되지 않고 항상 움직이게 한다.
            const limit = Math.max(0.5 * radii[i], repairStep)
            const magSq = dxStep * dxStep + dyStep * dyStep
            if (magSq > limit * limit) {
                const scale = limit / Math.sqrt(magSq)
                dxStep *= scale
                dyStep *= scale
            }
            deltas[i] = [dxStep, dyStep]
        }

        for (let i = 1; i < n - 1; i++) {
            const [dxStep, dyStep] = deltas[i]
            const [cx, cy] = centers[i]
            const newCenter: Point = [cx + dxStep, cy + dyStep]
            centers[i] = newCenter
            radii[i] = clearance(newCenter)
        }
    }

    // Overlap 유지: 이웃의 성장으로 잉여가 된 bubble을 삭제하고, 충돌-free 보간을
    // 보장할 수 없을 만큼 벌어진 gap에 midpoint bubble을 삽입한다. maintenance가
    // 정상 종료하면 이어서 유효성 검사(모든 내부 bubble rho >= rho_min)를 수행한다
    // (py _maintain 미러).
    const maintain = (): boolean => {
        let i = 0
        while (i < centers.length - 1) {
            if (i + 2 < centers.length) {
                const gap = Math.sqrt(sqDist(centers[i], centers[i + 2]))
                if (gap <= overlapFactor * (radii[i] + radii[i + 2])) {
                    centers.splice(i + 1, 1)
                    radii.splice(i + 1, 1)
                    continue
                }
            }
            const gap = Math.sqrt(sqDist(centers[i], centers[i + 1]))
            if (gap > overlapFactor * (radii[i] + radii[i + 1])) {
                if (centers.length >= maxBubbles) return false
                const mid: Point = [
                    (centers[i][0] + centers[i + 1][0]) / 2.0,
                    (centers[i][1] + centers[i + 1][1]) / 2.0,
                ]
                const rhoMid = clearance(mid)
                if (rhoMid < rhoMin) return false
                centers.splice(i + 1, 0, mid)
                radii.splice(i + 1, 0, rhoMid)
                continue
            }
            i += 1
        }
        for (let k = 1; k < radii.length - 1; k++) {
            if (radii[k] < rhoMin) return false
        }
        return true
    }

    const emitBand = (tickEmit: EmitFn, broken: number): void => {
        const band = centers.map(([cx, cy], i) => [cx, cy, radii[i]])
        tickEmit({
            event: "band_updated",
            band,
            data: {iterations: deformIterations, bubbles: centers.length, broken},
        })
    }

    // 밴드는 방출 시점 상태를 그대로 직렬화한 뒤(마지막 유효 밴드로 되돌리지 않음)
    // 폐기한다 -- 다음 tick이 재초기화+repair로 회복을 시도한다 (py _on_broken 미러).
    const onBroken = (tickEmit: EmitFn): VelocityCommand => {
        emitBand(tickEmit, 1)
        centers = []
        radii = []
        return {v: 0.0, omega: 0.0}
    }

    const initialize = (robotXy: Point): void => {
        const resampled = resamplePolyline(referencePath, bubbleSpacing)
        const goalXy: Point = [goal[0], goal[1]]
        centers = [robotXy, ...resampled, goalXy]
        radii = centers.map((c) => clearance(c))
        for (let k = 0; k < repairIterations; k++) deformOnce()
    }

    const tick = (state: RobotState3, _dt: number, tickEmit: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose
        const robotXy: Point = [x, y]

        if (centers.length === 0) {
            initialize(robotXy)
            if (!maintain()) return onBroken(tickEmit)
        }

        // Front pruning: 로봇이 이미 지나친 bubble(다음 bubble의 clearance 원 안에
        // 위치)을 버린 뒤 c_0를 실행된 pose에 재고정한다.
        while (centers.length > 2 && Math.sqrt(sqDist(robotXy, centers[1])) <= radii[1]) {
            centers.shift()
            radii.shift()
        }
        centers[0] = robotXy
        radii[0] = clearance(robotXy)

        for (let k = 0; k < deformIterations; k++) deformOnce()
        if (!maintain()) return onBroken(tickEmit)

        emitBand(tickEmit, 0)

        // 명령 추출: 변형된 밴드 자신의 폴리라인 위 lookahead 점을 비례 heading
        // 제어로 추종한다 (밴드가 매 tick 통째로 재변형되므로 progress-indexed 경로
        // 추종은 적용하지 않는다).
        const target = pointAtArcLength(centers, lookaheadDistance)
        const alpha = wrapToPi(Math.atan2(target[1] - y, target[0] - x) - theta)
        const v = maxSpeed * Math.max(Math.cos(alpha), 0.0)
        const omega = Math.max(-maxOmega, Math.min(maxOmega, headingGain * alpha))
        return {v, omega}
    }

    runClosedLoop({
        map, startPose, goal, controlDt, maxSteps, goalTolerance,
        footprintRadius, stallWindow, stallDistance,
    }, emit, tick)

    return events
}

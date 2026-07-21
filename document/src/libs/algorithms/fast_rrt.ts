import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {pathLength, Tree} from "./rrt";
import {insertBestParent, nearRadius, rewireStep} from "./rrt_star";

// 브라우저 라이브 데모용 Fast-RRT (Wu et al. 2021, Applied Sciences 11(24):11777).
// 저장소 구현을 그대로 미러한다: RRT* 트리 위에 세 가지 가속을 얹는다 —
// Fast-Sampling(기존 노드 근방 표본 거부로 미탐사 공간 집중), Random Steering(직선
// 확장이 막히면 무작위 방향으로 재시도), Fast-Optimal(경로가 생기면 삼각 부등식으로
// 지름길 pruning). planner RNG와 map RNG가 같은 seed의 독립 numpy 스트림이라는 것,
// 그리고 goal_bias 동전과 random steering 각도는 planner RNG, 표본은 map RNG라는
// draw 순서까지 미러하므로 같은 seed면 python demo와 표본·트리·개선 열이 일치한다.
export interface FastRRTOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    maxIterations: number;
    stepSize: number;
    goalBias: number;
    goalTolerance: number;
    neighborRadius: number;
    radiusMode: "fixed" | "shrinking";
    rggGamma: number;
    reachedRadius: number;
    steeringAttempts: number;
    seed: number;
}

// Fast-Sampling (Wu et al. 2021): goal-biased 표본은 goal 도달을 위해 항상 받고,
// 자유 표본은 어떤 트리 노드의 reached_radius 안이면 (steering_attempts 상한 내)
// 거부해 미탐사 공간에 집중시킨다. 이로써 search-time variance가 줄어든다.
function fastSample(
    space: SamplingGrid, tree: Tree, goal: Point, goalBias: number,
    reachedRadius: number, steeringAttempts: number, rng: NumpyRandom,
): Point {
    if (rng.random() < goalBias) return goal
    let q = space.sample()
    for (let a = 0; a < steeringAttempts; a++) {
        if (tree.points.every((node) => space.distance(q, node) > reachedRadius)) break
        q = space.sample()
    }
    return q
}

// Random Steering (Wu et al. 2021): 직선 확장이 막히면 무작위 방향으로 한 스텝씩
// 시도해 첫 충돌 없는 스텝을 취한다 (좁은 통로 통과에 도움).
function randomSteer(
    space: SamplingGrid, qNear: Point, qRand: Point, stepSize: number,
    steeringAttempts: number, rng: NumpyRandom,
): Point | null {
    const qNew = space.steer(qNear, qRand, stepSize)
    if (space.isMotionValid(qNear, qNew)) return qNew
    for (let a = 0; a < steeringAttempts; a++) {
        const angle = rng.uniform(0, 2 * Math.PI)
        const cand: Point = [qNear[0] + stepSize * Math.cos(angle),
                             qNear[1] + stepSize * Math.sin(angle)]
        if (space.isMotionValid(qNear, cand)) return cand
    }
    return null
}

// Fast-Optimal (Wu et al. 2021): 유지된 각 waypoint에서 충돌 없는 직선 구간으로
// 닿는 가장 먼 이후 waypoint로 뛰는 greedy 지름길 pass. C++ shortcut_prune과
// i/j 스캔 순서·is_motion_valid 호출 열이 같아 양 언어가 동일 결과를 낸다.
function shortcut(space: SamplingGrid, path: Point[]): Point[] {
    if (path.length <= 2) return path
    const out: Point[] = [path[0]]
    let i = 0
    while (i < path.length - 1) {
        let j = path.length - 1
        while (j > i + 1 && !space.isMotionValid(path[i], path[j])) j--
        out.push(path[j])
        i = j
    }
    return out
}

export function runFastRRT(opts: FastRRTOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, stepSize, goalBias, goalTolerance,
           neighborRadius, radiusMode, rggGamma, reachedRadius, steeringAttempts, seed} = opts
    const space = new SamplingGrid(map, seed)
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "fast_rrt",
        params: {max_iterations: maxIterations, step_size: stepSize, goal_bias: goalBias,
                 goal_tolerance: goalTolerance, neighbor_radius: neighborRadius,
                 radius_mode: radiusMode, rgg_gamma: rggGamma, reached_radius: reachedRadius,
                 steering_attempts: steeringAttempts, seed},
    })

    const tree = new Tree(start)
    // goal은 트리에 넣지 않는다 (성장/rewire 후보가 되면 안 된다). 지름길 pruning된
    // 최선 경로와 그 비용만 추적한다 (anytime).
    let bestPath: Point[] = []
    let bestCost = Infinity
    let iterations = 0
    for (let it = 0; it < maxIterations; it++) {
        iterations++
        const qRand = fastSample(space, tree, goal, goalBias, reachedRadius, steeringAttempts, rng)
        emit({event: "sample_drawn", state: [qRand[0], qRand[1]]})
        const nearIdx = tree.nearest(qRand, space)
        const qNear = tree.points[nearIdx]
        const qNew = randomSteer(space, qNear, qRand, stepSize, steeringAttempts, rng)
        if (qNew === null) continue

        const radius = nearRadius(radiusMode, neighborRadius, rggGamma, tree.size)
        const neighborhood = tree.near(qNew, radius, space)
        const newIdx = insertBestParent(space, tree, qNew, nearIdx, neighborhood, emit)
        rewireStep(space, tree, newIdx, qNew, neighborhood, emit)

        if (space.distance(qNew, goal) <= goalTolerance && space.isMotionValid(qNew, goal)) {
            const raw = [...tree.pathTo(newIdx), goal]
            const pruned = shortcut(space, raw)
            const prunedCost = pathLength(space, pruned)
            if (prunedCost < bestCost) {
                bestCost = prunedCost
                bestPath = pruned
                emit({event: "path_found", path: pruned.map((p) => [p[0], p[1]]), cost: prunedCost})
            }
        }
    }

    const success = bestPath.length > 0
    const cost = success ? bestCost : 0
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: cost, expanded_nodes: tree.size - 1,
                  samples: iterations, tree_size: tree.size, iterations},
    })
    return events
}

import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {biasedSample, pathLength, Tree} from "./rrt";
import {insertBestParent, nearRadius, rewireStep} from "./rrt_star";

// 브라우저 라이브 데모용 Informed RRT* (Gammell, Srinivasa & Barfoot 2014).
// 트리 성장(choose-parent + rewire + anytime 현직 해)은 RRT*과 완전히 같고,
// 표본 추출만 바뀐다: 해가 하나라도 생기면 그 뒤로는 시작/goal을 초점으로 현재
// best 비용을 지름으로 하는 informed 타원 안에서만 표본을 뽑는다. 그래서 해 발견
// 이후의 표본이 현직 해를 개선할 수 있는 영역에만 떨어진다. planner RNG의 draw
// 순서(uniform 각도 → sqrt 반지름)까지 python 구현과 일치시켜야 같은 seed에서
// 표본열·트리·개선 열이 그대로 재현된다.
export interface InformedRRTStarOptions {
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
    seed: number;
}

// informed 타원 표본 (Gammell, Srinivasa & Barfoot 2014): 초점 start/goal, 횡축
// 지름 c_best 인 타원 안 균일 표본. 아직 해가 없거나(c_best=∞) 이론 하한 이하면
// 전 공간에서 뽑는다. RNG draw 순서(각도 uniform → 반지름 sqrt(random))는 저장소
// informed_sample을 그대로 미러한다.
export const informedSample = (
    space: SamplingGrid, start: Point, goal: Point, cBest: number, rng: NumpyRandom,
): Point => {
    const cMin = space.distance(start, goal)
    if (cBest >= Infinity || cBest <= cMin) return space.sample()
    const cx = (start[0] + goal[0]) / 2
    const cy = (start[1] + goal[1]) / 2
    const r1 = cBest / 2
    const r2 = Math.sqrt(Math.max(cBest * cBest - cMin * cMin, 0)) / 2
    const theta = Math.atan2(goal[1] - start[1], goal[0] - start[0])
    const ang = rng.uniform(0, 2 * Math.PI)
    const rad = Math.sqrt(rng.random())
    const ux = rad * Math.cos(ang) * r1
    const uy = rad * Math.sin(ang) * r2
    const x = cx + Math.cos(theta) * ux - Math.sin(theta) * uy
    const y = cy + Math.sin(theta) * ux + Math.cos(theta) * uy
    return [x, y]
}

export function runInformedRRTStar(opts: InformedRRTStarOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, stepSize, goalBias, goalTolerance,
           neighborRadius, radiusMode, rggGamma, seed} = opts
    const space = new SamplingGrid(map, seed)
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "informed_rrt_star",
        params: {max_iterations: maxIterations, step_size: stepSize, goal_bias: goalBias,
                 goal_tolerance: goalTolerance, neighbor_radius: neighborRadius,
                 radius_mode: radiusMode, rgg_gamma: rggGamma, seed},
    })

    const tree = new Tree(start)
    // goal은 트리에 넣지 않는다 (성장/rewire 후보가 되면 안 된다). 최선 부모와
    // 비용만 추적한다.
    let bestGoalParent = -1
    let bestCost = Infinity
    let iterations = 0
    for (let it = 0; it < maxIterations; it++) {
        iterations++
        // 해가 생기기 전엔 RRT*처럼 균일(goal-bias) 표본, 생긴 뒤엔 현직 해를
        // 개선할 수 있는 informed 타원 안에서만 표본을 뽑는다.
        const qRand = bestGoalParent !== -1
            ? informedSample(space, start, goal, bestCost, rng)
            : biasedSample(space, goal, goalBias, rng)
        emit({event: "sample_drawn", state: [qRand[0], qRand[1]]})
        const nearIdx = tree.nearest(qRand, space)
        const qNew = space.steer(tree.points[nearIdx], qRand, stepSize)
        if (!space.isMotionValid(tree.points[nearIdx], qNew)) continue

        const radius = nearRadius(radiusMode, neighborRadius, rggGamma, tree.size)
        const neighborhood = tree.near(qNew, radius, space)
        const newIdx = insertBestParent(space, tree, qNew, nearIdx, neighborhood, emit)
        rewireStep(space, tree, newIdx, qNew, neighborhood, emit)

        if (space.distance(qNew, goal) <= goalTolerance && space.isMotionValid(qNew, goal)) {
            const candCost = tree.cost[newIdx] + space.distance(qNew, goal)
            if (candCost < bestCost) {
                bestCost = candCost
                bestGoalParent = newIdx
                const path = [...tree.pathTo(newIdx), goal]
                emit({event: "path_found", path: path.map((p) => [p[0], p[1]]),
                      cost: pathLength(space, path)})
            }
        }
    }

    const success = bestGoalParent !== -1
    const path = success ? [...tree.pathTo(bestGoalParent), goal] : []
    const cost = success ? pathLength(space, path) : 0
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: cost, expanded_nodes: tree.size - 1,
                  samples: iterations, tree_size: tree.size, iterations},
    })
    return events
}

import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {biasedSample, pathLength, Tree} from "./rrt";
import {rggRadius} from "./prm";

// 브라우저 라이브 데모용 RRT* (Karaman & Frazzoli 2011). 저장소 구현을 그대로
// 미러한다: RRT에 choose-parent(근방 최소 비용 부모)와 rewire(새 노드를 거치면
// 더 싼 근방 노드의 재배선)를 더하고, 예산이 남는 한 goal 경로를 계속 개선한다
// (anytime). 같은 seed면 python demo와 표본·트리·개선 열까지 일치한다.
export interface RRTStarOptions {
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

type Emit = (ev: Omit<TraceEvent, "seq">) => void;

// RRT* 계열의 근방 반경: shrinking이면 트리 크기에 따라 RGG 반경으로 줄인다.
export const nearRadius = (
    mode: string, fixedRadius: number, gamma: number, n: number,
): number => mode === "shrinking" ? rggRadius(gamma, n) : fixedRadius

// choose-parent (Karaman & Frazzoli 2011): 근방에서 최소 비용의 실행 가능 부모.
export function insertBestParent(
    space: SamplingGrid, tree: Tree, qNew: Point, nearIdx: number,
    neighborhood: number[], emit: Emit,
): number {
    let bestParent = nearIdx
    let bestCost = tree.cost[nearIdx] + space.distance(tree.points[nearIdx], qNew)
    for (const j of neighborhood) {
        if (!space.isMotionValid(tree.points[j], qNew)) continue
        const c = tree.cost[j] + space.distance(tree.points[j], qNew)
        emit({event: "candidate_evaluated", state: [qNew[0], qNew[1]], cost: c})
        if (c < bestCost) {
            bestCost = c
            bestParent = j
        }
    }
    const newIdx = tree.add(qNew, bestParent, bestCost)
    emit({event: "edge_added", state: [qNew[0], qNew[1]],
          parent: [tree.points[bestParent][0], tree.points[bestParent][1]],
          cost: space.distance(tree.points[bestParent], qNew)})
    return newIdx
}

// rewire: 새 노드를 거치는 편이 싸면 근방 노드를 재배선한다.
export function rewireStep(
    space: SamplingGrid, tree: Tree, newIdx: number, qNew: Point,
    neighborhood: number[], emit: Emit,
): void {
    for (const j of neighborhood) {
        if (j === tree.parent[newIdx]) continue
        if (!space.isMotionValid(qNew, tree.points[j])) continue
        const rerouted = tree.cost[newIdx] + space.distance(qNew, tree.points[j])
        if (rerouted < tree.cost[j]) {
            tree.reparent(j, newIdx, rerouted, space)
            emit({event: "rewire", state: [tree.points[j][0], tree.points[j][1]],
                  parent: [qNew[0], qNew[1]]})
        }
    }
}

export function runRRTStar(opts: RRTStarOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, stepSize, goalBias, goalTolerance,
           neighborRadius, radiusMode, rggGamma, seed} = opts
    const space = new SamplingGrid(map, seed)
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit: Emit = (ev) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "rrt_star",
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
        const qRand = biasedSample(space, goal, goalBias, rng)
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

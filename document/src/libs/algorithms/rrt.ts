import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";

// 브라우저 라이브 데모용 RRT (LaValle 1998). 저장소 구현을 그대로 미러한다:
// goal-bias 표본 → nearest → steer 한 스텝 → 충돌 검사 → 트리 확장, goal 반경에
// 들면 종료. planner RNG와 map RNG가 같은 seed의 독립 numpy 스트림이라는 것까지
// 미러하므로 같은 seed면 python demo와 반복·트리 크기까지 일치한다.
export interface RRTOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    maxIterations: number;
    stepSize: number;
    goalBias: number;
    goalTolerance: number;
    seed: number;
}

// 저장소 _sampling.Tree 미러 — RRT 계열이 공유하는 병렬 배열 트리.
export class Tree {
    points: Point[];
    parent: number[];
    cost: number[];
    children: number[][];

    constructor(root: Point) {
        this.points = [root]
        this.parent = [-1]
        this.cost = [0]
        this.children = [[]]
    }

    get size(): number {
        return this.points.length
    }

    add(point: Point, parentIdx: number, cost: number): number {
        const idx = this.points.length
        this.points.push(point)
        this.parent.push(parentIdx)
        this.cost.push(cost)
        this.children.push([])
        if (parentIdx >= 0) this.children[parentIdx].push(idx)
        return idx
    }

    // rewire 시 cost 변화를 부분 트리 전체에 전파한다 — 후손이 낡은 누적 비용을
    // 유지하면 RRT*의 최적성 불변식이 깨진다 (Karaman & Frazzoli 2011).
    reparent(child: number, newParent: number, newCost: number, space: SamplingGrid): void {
        const old = this.parent[child]
        if (old >= 0) {
            this.children[old].splice(this.children[old].indexOf(child), 1)
        }
        this.parent[child] = newParent
        this.cost[child] = newCost
        this.children[newParent].push(child)
        const stack = [child]
        while (stack.length > 0) {
            const u = stack.pop()!
            for (const c of this.children[u]) {
                this.cost[c] = this.cost[u] + space.distance(this.points[u], this.points[c])
                stack.push(c)
            }
        }
    }

    nearest(p: Point, space: SamplingGrid): number {
        let bestIdx = 0
        let bestD = space.distance(this.points[0], p)
        for (let idx = 1; idx < this.points.length; idx++) {
            const d = space.distance(this.points[idx], p)
            if (d < bestD) {
                bestD = d
                bestIdx = idx
            }
        }
        return bestIdx
    }

    near(p: Point, radius: number, space: SamplingGrid): number[] {
        const out: number[] = []
        for (let i = 0; i < this.points.length; i++) {
            if (space.distance(this.points[i], p) <= radius) out.push(i)
        }
        return out
    }

    pathTo(idx: number): Point[] {
        const path: Point[] = []
        let node = idx
        while (node !== -1) {
            path.push(this.points[node])
            node = this.parent[node]
        }
        path.reverse()
        return path
    }
}

export const pathLength = (space: SamplingGrid, path: Point[]): number => {
    let total = 0
    for (let i = 0; i + 1 < path.length; i++) total += space.distance(path[i], path[i + 1])
    return total
}

// goal-bias 표본 (LaValle 1998): planner RNG 동전 → goal 또는 map RNG 균일 표본.
export const biasedSample = (
    space: SamplingGrid, goal: Point, goalBias: number, rng: NumpyRandom,
): Point => rng.random() < goalBias ? goal : space.sample()

export function runRRT(opts: RRTOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, stepSize, goalBias, goalTolerance, seed} = opts
    const space = new SamplingGrid(map, seed)
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "rrt",
        params: {max_iterations: maxIterations, step_size: stepSize, goal_bias: goalBias,
                 goal_tolerance: goalTolerance, seed},
    })

    const tree = new Tree(start)
    let goalIdx = -1
    let iterations = 0
    for (let it = 0; it < maxIterations; it++) {
        iterations++
        const qRand = biasedSample(space, goal, goalBias, rng)
        emit({event: "sample_drawn", state: [qRand[0], qRand[1]]})
        const nearIdx = tree.nearest(qRand, space)
        const qNear = tree.points[nearIdx]
        const qNew = space.steer(qNear, qRand, stepSize)
        if (!space.isMotionValid(qNear, qNew)) continue
        const stepCost = space.distance(qNear, qNew)
        const newIdx = tree.add(qNew, nearIdx, tree.cost[nearIdx] + stepCost)
        emit({event: "edge_added", state: [qNew[0], qNew[1]],
              parent: [qNear[0], qNear[1]], cost: stepCost})
        if (space.distance(qNew, goal) <= goalTolerance && space.isMotionValid(qNew, goal)) {
            goalIdx = tree.add(goal, newIdx, tree.cost[newIdx] + space.distance(qNew, goal))
            emit({event: "edge_added", state: [goal[0], goal[1]],
                  parent: [qNew[0], qNew[1]], cost: space.distance(qNew, goal)})
            break
        }
    }

    const success = goalIdx !== -1
    const path = success ? tree.pathTo(goalIdx) : []
    const cost = success ? pathLength(space, path) : 0
    if (success) {
        emit({event: "path_found", path: path.map((p) => [p[0], p[1]]), cost})
    }
    emit({
        event: "planning_finished",
        success,
        metrics: {
            path_cost: cost,
            expanded_nodes: tree.size - 1,
            samples: iterations,
            tree_size: tree.size,
            iterations,
        },
    })
    return events
}

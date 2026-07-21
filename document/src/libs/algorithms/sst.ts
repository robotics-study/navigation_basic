import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {discCollides, Point, SamplingGrid} from "./sampling_space";
import {biasedSample, pathLength} from "./rrt";

// 브라우저 라이브 데모용 SST (Li, Littlefield & Bekris 2016). 저장소 구현을 그대로
// 미러한다: goal-bias 표본 → BestNear(δ_BN 안 최소 cost active 노드) → 랜덤 control을
// 정방향 전파(unicycle)한 호 → witness 반경 δ_s로 지역 최고만 active로 남기고 나머지는
// 가지치기. planner RNG와 map RNG가 같은 seed의 독립 numpy 스트림이고, propagation의
// draw 순서(v → ω → duration)까지 미러하므로 같은 seed면 python demo와 확장 수·비용까지
// 일치한다.
export interface SSTOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    maxIterations: number;
    goalBias: number;
    goalTolerance: number;
    deltaBn: number;
    deltaS: number;
    maxVelocity: number;
    maxOmega: number;
    propDurationMin: number;
    propDurationMax: number;
    footprintRadius: number;
    sstStar: boolean;
    seed: number;
}

// 호 충돌 검사용 waypoint 간격 (m). 직선 chord가 한 셀보다 작아 supercover 검사가 곡선
// 호가 지나는 장애물을 놓치지 않는다 (저장소 구현과 동일 값).
const ARC_WAYPOINT_SPACING = 0.2;
// SST* 반경 감쇠 (반복 수 doubling마다). 초기엔 넓게 탐색하고 이후 조여 최적으로 수렴시킨다
// (Li, Littlefield & Bekris 2016 §V).
const SST_STAR_SHRINK = 0.9;

export function runSST(opts: SSTOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, goalBias, goalTolerance, deltaBn, deltaS,
           maxVelocity, maxOmega, propDurationMin, propDurationMax, footprintRadius, sstStar, seed} = opts
    const space = new SamplingGrid(map, seed)
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({
        event: "planning_started",
        algorithm: "sst",
        params: {max_iterations: maxIterations, goal_bias: goalBias, goal_tolerance: goalTolerance,
                 delta_bn: deltaBn, delta_s: deltaS, max_velocity: maxVelocity,
                 max_omega: maxOmega, prop_duration_min: propDurationMin,
                 prop_duration_max: propDurationMax, footprint_radius: footprintRadius, sst_star: sstStar, seed},
    })

    // 병렬 배열 트리 — SST는 shared Tree가 모르는 active/witness/가지치기 장부가 필요해
    // 자체 노드 배열을 든다.
    const pt: Point[] = [start]
    // 루트 heading은 goal을 향해 초기 전파가 생산적이게 한다.
    const th: number[] = [Math.atan2(goal[1] - start[1], goal[0] - start[0])]
    const parent: number[] = [-1]
    const cost: number[] = [0]
    const children: number[][] = [[]]
    // 노드로 들어오는 dense 호 (부모 제외 .. 노드 포함); 루트는 [].
    const arc: Point[][] = [[]]
    const activeIds = new Set<number>([0])

    // witness 집합: witness 점 + 그 active 대표 노드 index.
    const wpt: Point[] = [start]
    const wrep: number[] = [0]

    const radii = (it: number): [number, number] => {
        if (!sstStar) return [deltaBn, deltaS]
        const k = Math.floor(Math.log2(it + 2))
        const scale = SST_STAR_SHRINK ** k
        return [deltaBn * scale, deltaS * scale]
    }

    // BestNear: 표본의 δ_BN 안 최소 cost active 노드. 공이 비면 가장 가까운 active 노드로
    // 후퇴한다 (Li, Littlefield & Bekris 2016).
    const bestNear = (s: Point, dbn: number): number => {
        let best = -1
        let bestCost = Infinity
        for (const i of activeIds) {
            if (space.distance(pt[i], s) <= dbn && cost[i] < bestCost) {
                bestCost = cost[i]
                best = i
            }
        }
        if (best !== -1) return best
        let near = -1
        let nearD = Infinity
        for (const i of activeIds) {
            const d = space.distance(pt[i], s)
            if (d < nearD) {
                nearD = d
                near = i
            }
        }
        return near
    }

    const nearestWitness = (p: Point): [number, number] => {
        let best = 0
        let bestD = space.distance(wpt[0], p)
        for (let i = 1; i < wpt.length; i++) {
            const d = space.distance(wpt[i], p)
            if (d < bestD) {
                bestD = d
                best = i
            }
        }
        return [best, bestD]
    }

    // unicycle (x, y, theta)을 랜덤 상수 control (v, ω)로 랜덤 시간만큼 정방향 전파(Euler).
    // 모든 waypoint를 (x, y) 투영으로 충돌 검사하고 직전 점과의 chord를 supercover 검사한다.
    const propagate = (fromIdx: number): {theta: number; waypoints: Point[]} | null => {
        const v = rng.uniform(0, maxVelocity)
        const omega = rng.uniform(-maxOmega, maxOmega)
        const duration = rng.uniform(propDurationMin, propDurationMax)
        const nSub = Math.max(2, Math.ceil(v * duration / ARC_WAYPOINT_SPACING))
        const dt = duration / nSub
        let x = pt[fromIdx][0]
        let y = pt[fromIdx][1]
        let theta = th[fromIdx]
        let prev = pt[fromIdx]
        const waypoints: Point[] = []
        for (let k = 0; k < nSub; k++) {
            theta += omega * dt
            x += v * Math.cos(theta) * dt
            y += v * Math.sin(theta) * dt
            const p: Point = [x, y]
            // 웨이포인트 간격(0.2 m)이 footprint 반경 이하라 disc 사슬이 chord를 덮지만,
            // 얇은 벽 corner-cut은 supercover chord 검사로 함께 막는다.
            if (discCollides(map, footprintRadius, x, y)) return null
            if (!space.isStateValid(p) || !space.isMotionValid(prev, p)) return null
            waypoints.push(p)
            prev = p
        }
        return {theta, waypoints}
    }

    const addNode = (p: Point, theta: number, par: number, c: number, wps: Point[]): number => {
        const idx = pt.length
        pt.push(p)
        th.push(theta)
        parent.push(par)
        cost.push(c)
        children.push([])
        arc.push(wps)
        children[par].push(idx)
        activeIds.add(idx)
        return idx
    }

    // 지배당한 대표를 비활성화하고, 그 노드 + 비활성 leaf 조상을 트리에서 떼어내 active
    // 집합을 유계로 유지한다 ("sparse").
    const pruneLeafChain = (node: number): void => {
        activeIds.delete(node)
        let cur = node
        while (cur !== -1 && !activeIds.has(cur) && children[cur].length === 0) {
            const par = parent[cur]
            if (par !== -1) {
                const sib = children[par]
                sib.splice(sib.indexOf(cur), 1)
            }
            cur = par
        }
    }

    const reconstruct = (idx: number): Point[] => {
        const segs: Point[][] = []
        let node = idx
        while (parent[node] !== -1) {
            segs.push(arc[node])
            node = parent[node]
        }
        const path: Point[] = [start]
        for (let i = segs.length - 1; i >= 0; i--) path.push(...segs[i])
        return path
    }

    let bestGoal = -1
    let bestCost = Infinity
    let totalAdded = 0
    let iterations = 0
    for (let it = 0; it < maxIterations; it++) {
        iterations++
        const [dbn, ds] = radii(it)
        const sSample = biasedSample(space, goal, goalBias, rng)
        emit({event: "sample_drawn", state: [sSample[0], sSample[1]]})

        const selected = bestNear(sSample, dbn)
        const prop = propagate(selected)
        if (prop === null) continue
        const {theta: newTheta, waypoints} = prop
        const newPt = waypoints[waypoints.length - 1]
        const newCost = cost[selected] + pathLength(space, [pt[selected], ...waypoints])

        // IsNodeLocallyBest: 지배 witness를 찾거나 만들고, 그 witness의 현재 대표를 이겨야
        // 노드를 남긴다.
        let [wi, wd] = nearestWitness(newPt)
        if (wd > ds) {
            wi = wpt.length
            wpt.push(newPt)
            wrep.push(-1)
        }
        const peer = wrep[wi]
        if (peer !== -1 && newCost >= cost[peer]) continue

        const ci = addNode(newPt, newTheta, selected, newCost, waypoints)
        totalAdded++
        let prev = pt[selected]
        for (const w of waypoints) {
            emit({event: "edge_added", state: [w[0], w[1]],
                  parent: [prev[0], prev[1]], cost: space.distance(prev, w)})
            prev = w
        }
        wrep[wi] = ci
        if (peer !== -1) {
            // rewire는 witness 대표가 더 싼 노드로 옮겨감을 표시한다 (viz가 sparsification —
            // 옛 가지가 가지치기되는 것 — 을 보이게).
            emit({event: "rewire", state: [newPt[0], newPt[1]],
                  parent: [pt[peer][0], pt[peer][1]]})
            pruneLeafChain(peer)
        }

        if (space.distance(newPt, goal) <= goalTolerance && newCost < bestCost) {
            bestCost = newCost
            bestGoal = ci
            const path = reconstruct(ci)
            emit({event: "path_found", path: path.map((p) => [p[0], p[1]]),
                  cost: pathLength(space, path)})
        }
    }

    const success = bestGoal !== -1
    const path = success ? reconstruct(bestGoal) : []
    const resultCost = success ? pathLength(space, path) : 0
    const activeCount = activeIds.size
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: resultCost, expanded_nodes: totalAdded,
                  samples: iterations, tree_size: activeCount, iterations},
    })
    return events
}

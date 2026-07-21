import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {rggRadius} from "./prm";
import {informedSample} from "./informed_rrt_star";

// 브라우저 라이브 데모용 ABIT* — Advanced Batch Informed Trees (Strub & Gammell
// 2020). 저장소 python 구현을 연산 순서·RNG·tie-break·이벤트 순서까지 그대로
// 미러한다. BIT*의 배치 RGG + vertex/edge 큐 + lazy 충돌 검사 위에 두 가지를 얹는다:
// (1) 큐 키의 cost-to-go 항을 ε_infl(≥1)로 부풀려(weighted-A*/ARA* 순서; Likhachev,
// Gordon & Thrun 2003) 초반 배치가 goal 쪽으로 탐욕적으로 정렬돼 첫 해에 빨리 닿고,
// (2) 현재 해를 ε_trunc(≥1)배 이상 줄이지 못하는 간선 처리를 조기 종료(truncation)해
// 마지막 비싼 충돌 검사를 건너뛴다. 두 계수는 배치마다 각각 inflation_final, 1.0으로
// 단조 감소해 마지막 배치가 정확히 BIT*로 환원되므로 점근 최적성이 유지된다. 표본은
// space RNG(첫 해 이전 균일)와 planner RNG(informed 타원)에서 나오고 둘 다 같은 seed로
// numpy PCG64를 미러하므로, 같은 seed면 python demo와 표본·간선·확장 수까지 일치한다.
export interface ABITStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    batchSize: number;
    maxBatches: number;
    gamma: number;
    inflationFactor: number;
    inflationFinal: number;
    truncationFactor: number;
    seed: number;
}

// python heapq 미러: number[] 튜플을 사전순 비교하는 이진 최소 힙. q_v는 [key, v],
// q_e는 [key, vm, xm]을 담아 같은 key의 tie-break(정수 인덱스 오름차순)가 python heapq
// 와 정확히 일치한다.
class TupleHeap {
    private readonly items: number[][] = []

    get size(): number {
        return this.items.length
    }

    peek(): number[] {
        return this.items[0]
    }

    private less(a: number, b: number): boolean {
        const x = this.items[a]
        const y = this.items[b]
        for (let i = 0; i < x.length; i++) {
            if (x[i] < y[i]) return true
            if (x[i] > y[i]) return false
        }
        return false
    }

    private swap(a: number, b: number): void {
        [this.items[a], this.items[b]] = [this.items[b], this.items[a]]
    }

    push(item: number[]): void {
        this.items.push(item)
        let c = this.size - 1
        while (c > 0) {
            const p = (c - 1) >> 1
            if (!this.less(c, p)) break
            this.swap(c, p)
            c = p
        }
    }

    pop(): number[] {
        const top = this.items[0]
        const last = this.size - 1
        this.swap(0, last)
        this.items.pop()
        let p = 0
        const n = this.size
        for (;;) {
            const l = 2 * p + 1
            const r = l + 1
            let m = p
            if (l < n && this.less(l, m)) m = l
            if (r < n && this.less(r, m)) m = r
            if (m === p) break
            this.swap(p, m)
            p = m
        }
        return top
    }
}

// 배치마다 초기값 → 최종값으로 단조 선형 감소하는 ε 스케줄 (ARA* 방식; Likhachev,
// Gordon & Thrun 2003): 첫 배치는 부풀려 빠른 첫 해를 얻고, 마지막 배치에서 최종값으로
// 풀어 최적성을 회복한다. 배치가 하나뿐이면 초기값을 유지한다.
const schedule = (batch: number, maxBatches: number, initial: number, final: number): number => {
    if (maxBatches <= 1) return initial
    return initial + (final - initial) * (batch / (maxBatches - 1))
}

export function runABITStar(opts: ABITStarOptions): TraceEvent[] {
    const {map, start, goal, batchSize, maxBatches, gamma, inflationFactor,
           inflationFinal, truncationFactor, seed} = opts
    const space = new SamplingGrid(map, seed)
    // planner RNG: 첫 해가 생긴 뒤 informed 타원 draw 전용. 첫 해 이전 균일 표본은
    // space 자체 RNG(space.sample)를 쓰므로 두 스트림이 독립이다 (python 미러).
    const rng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "abit_star",
          params: {batch_size: batchSize, max_batches: maxBatches, gamma,
                   inflation_factor: inflationFactor, inflation_final: inflationFinal,
                   truncation_factor: truncationFactor, seed}})

    // 트리 상태: index 0 = start(root), 1 = goal(영구 표본). samples는 아직 트리에 없는
    // index 집합, g_t[i]는 cost-to-come(미연결이면 ∞).
    const points: Point[] = [start, goal]
    const gT: number[] = [0, Infinity]
    const parent: number[] = [-1, -1]
    const children: Array<Set<number>> = [new Set(), new Set()]
    const inTree: boolean[] = [true, false]
    const samples = new Set<number>([1])
    let cBest = Infinity
    let expanded = 0

    const addSample = (p: Point): number => {
        const idx = points.length
        points.push(p)
        gT.push(Infinity)
        parent.push(-1)
        children.push(new Set())
        inTree.push(false)
        samples.add(idx)
        return idx
    }

    const hHat = (i: number): number => space.distance(points[i], goal)
    const gHat = (i: number): number => space.distance(start, points[i])

    // rewire로 g_t[root]가 바뀌면 그 서브트리에 델타를 밀어 큐 키와 보고 비용을 일관되게
    // 유지한다.
    const propagate = (root: number): void => {
        const stack = [root]
        while (stack.length > 0) {
            const u = stack.pop() as number
            for (const c of children[u]) {
                gT[c] = gT[u] + space.distance(points[u], points[c])
                stack.push(c)
            }
        }
    }

    for (let batch = 0; batch < maxBatches; batch++) {
        // ε_infl은 inflation_final로, ε_trunc는 1.0(무-truncation)으로 감소해 마지막
        // 배치가 admissible·untruncated, 곧 정확히 BIT*가 된다.
        const epsInfl = schedule(batch, maxBatches, inflationFactor, inflationFinal)
        const epsTrunc = schedule(batch, maxBatches, truncationFactor, 1.0)

        // 현재 해를 더는 개선할 수 없는 표본을 쳐낸다. 경계는 un-inflated(admissible)라
        // 덜 부풀려질 다음 배치가 아직 경유할 표본을 떨구지 않는다.
        if (cBest < Infinity) {
            const toDrop: number[] = []
            for (const s of samples) {
                if (gHat(s) + hHat(s) >= cBest) toDrop.push(s)
            }
            for (const x of toDrop) samples.delete(x)
        }

        // 새 배치 draw (해가 있으면 informed 타원 안).
        let drawn = 0
        for (let k = 0; k < batchSize * 40; k++) {
            if (drawn >= batchSize) break
            const q = informedSample(space, start, goal, cBest, rng)
            if (!space.isStateValid(q)) continue
            addSample(q)
            drawn++
            emit({event: "sample_drawn", state: [q[0], q[1]]})
        }

        const n = points.length
        const radius = rggRadius(gamma, n)
        // 배치 반경 그래프를 한 번 미리 계산한다 (오름차순 index로 채워 tie-break가
        // python과 같다).
        const nbr: number[][] = Array.from({length: n}, () => [])
        for (let i = 0; i < n; i++) {
            const pi = points[i]
            for (let j = i + 1; j < n; j++) {
                if (space.distance(pi, points[j]) <= radius) {
                    nbr[i].push(j)
                    nbr[j].push(i)
                }
            }
        }

        // 큐: 확장할 정점 + 후보 간선. 키는 cost-to-go 항을 ε_infl로 부풀린다.
        const qV = new TupleHeap()
        const qE = new TupleHeap()
        for (let v = 0; v < n; v++) {
            if (inTree[v]) qV.push([gT[v] + epsInfl * hHat(v), v])
        }
        const expandedV = new Set<number>()

        const expandVertex = (v: number): void => {
            for (const x of nbr[v]) {
                const d = space.distance(points[v], points[x])
                if (samples.has(x)) {
                    // 미연결 표본으로의 후보 간선. enqueue 게이트는 admissible, 정렬 키만
                    // 부풀린다.
                    if (gHat(v) + d + hHat(x) < cBest) {
                        qE.push([gT[v] + d + epsInfl * hHat(x), v, x])
                    }
                } else if (inTree[x] && x !== parent[v]) {
                    // v를 통한 기존 정점 rewire 후보.
                    if (gHat(v) + d + hHat(x) < cBest && gT[v] + d < gT[x]) {
                        qE.push([gT[v] + d + epsInfl * hHat(x), v, x])
                    }
                }
            }
        }

        const bestV = (): number => {
            while (qV.size > 0) {
                const [key, v] = qV.peek()
                if (expandedV.has(v) || key > gT[v] + epsInfl * hHat(v) + 1e-9) {
                    qV.pop()
                    continue
                }
                return key
            }
            return Infinity
        }
        const bestE = (): number => (qE.size > 0 ? qE.peek()[0] : Infinity)

        // truncation 임계: 어떤 간선도 현직 해를 c_best / ε_trunc 아래로 못 끌면 배치를
        // 멈춘다 (Strub & Gammell 2020). ε_trunc = 1 → BIT*의 c_best.
        let truncBound = cBest / epsTrunc

        for (;;) {
            // 최선 간선을 이길 수 있는 정점 확장을 먼저 소진한다.
            while (qV.size > 0 && bestV() <= bestE()) {
                const [, v] = qV.pop()
                if (expandedV.has(v)) continue
                expandedV.add(v)
                expandVertex(v)
            }
            if (qE.size === 0) break
            const [, vm, xm] = qE.pop()
            const dVmXm = space.distance(points[vm], points[xm])
            // 이 간선을 통한 해의 admissible(un-inflated) 추정.
            const aKey = gT[vm] + dVmXm + hHat(xm)
            // truncation: 최선 정렬 간선조차 경계를 넘어 개선하지 못한다.
            if (aKey >= truncBound) break
            // 이 간선이 x_m의 cost-to-come을 조금이라도 낮추는가?
            if (gT[vm] + dVmXm >= gT[xm]) continue
            if (!space.isMotionValid(points[vm], points[xm])) continue
            const edgeCost = dVmXm
            const newG = gT[vm] + edgeCost
            if (newG + hHat(xm) >= cBest || newG >= gT[xm]) continue
            // 간선 채택: 표본을 잇거나 정점을 v_m 아래로 rewire.
            if (inTree[xm]) {
                children[parent[xm]].delete(xm)
            } else {
                samples.delete(xm)
                inTree[xm] = true
            }
            parent[xm] = vm
            gT[xm] = newG
            children[vm].add(xm)
            propagate(xm)
            expanded++
            expandedV.delete(xm)  // 개선됨: 이 배치에서 재확장 허용
            qV.push([gT[xm] + epsInfl * hHat(xm), xm])
            emit({event: "edge_added", state: [points[xm][0], points[xm][1]],
                  parent: [points[vm][0], points[vm][1]], cost: edgeCost})
            if (inTree[1] && gT[1] < cBest) {
                cBest = gT[1]
                truncBound = cBest / epsTrunc
                emit({event: "candidate_evaluated", state: [goal[0], goal[1]], cost: cBest})
            }
        }
    }

    const success = inTree[1] && gT[1] < Infinity
    const n = points.length
    if (success) {
        const path: number[][] = []
        let node = 1
        while (node !== -1) {
            path.push([points[node][0], points[node][1]])
            node = parent[node]
        }
        path.reverse()
        // 참 기하 길이 (rewire된 누적 비용의 stale 값을 피한다).
        let cost = 0
        for (let i = 0; i + 1 < path.length; i++) {
            cost += Math.hypot(path[i + 1][0] - path[i][0], path[i + 1][1] - path[i][1])
        }
        emit({event: "path_found", path, cost})
        emit({event: "planning_finished", success: true,
              metrics: {path_cost: cost, expanded_nodes: expanded, samples: n, tree_size: n}})
    } else {
        emit({event: "planning_finished", success: false,
              metrics: {path_cost: 0, expanded_nodes: expanded, samples: n, tree_size: n}})
    }
    return events
}

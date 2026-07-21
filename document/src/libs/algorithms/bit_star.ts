import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";
import {informedSample} from "./informed_rrt_star";
import {pathLength} from "./rrt";
import {rggRadius} from "./prm";

// 브라우저 라이브 데모용 BIT* — Batch Informed Trees (Gammell, Srinivasa &
// Barfoot 2015). 저장소 python 구현을 연산 순서·힙 tie-break·RNG draw 순서·이벤트
// 순서까지 그대로 미러한다. 한 배치마다 informed 표본을 뿌리고, 표본 위 암묵적
// RGG를 간선 큐로 best-first 확장하되(LPA*/A* 스타일), 충돌 검사는 간선을 큐에서
// 꺼내는 순간으로 미룬다(lazy). 해가 하나 생기면 이후 배치는 informed 타원에서만
// 표본을 뽑아 현직 해를 개선할 수 있는 영역에 집중한다(anytime). 표본은 두 개의
// numpy PCG64 스트림에서 나온다: 해가 없을 때의 균일 표본은 map RNG(space.sample),
// 타원 표본의 각도·반지름은 planner RNG. 같은 seed면 python demo와 표본·간선·확장
// 수까지 일치한다.
export interface BITStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    batchSize: number;
    maxBatches: number;
    gamma: number;
    seed: number;
}

// (key, a, b) 사전순 최소 힙 — python heapq의 튜플 pop 순서를 그대로 낸다. q_v 는
// (g+h, v) 2튜플이라 b=0 으로 밀어 넣어 (key, v) 비교로 환원되고, q_e는 (key, v, x)
// 3튜플이라 같은 key 일 때 v→x로 tie-break 한다. 항목이 유일하므로 (동일 v, 동일
// 간선 (v,x) 중복은 값이 같아 구별 불가) pop 하는 값의 순서가 결정적이다.
class Heap {
    private readonly key: number[] = []
    private readonly a: number[] = []
    private readonly b: number[] = []

    get size(): number {
        return this.key.length
    }

    peekKey(): number {
        return this.key[0]
    }

    peekA(): number {
        return this.a[0]
    }

    private less(i: number, j: number): boolean {
        if (this.key[i] !== this.key[j]) return this.key[i] < this.key[j]
        if (this.a[i] !== this.a[j]) return this.a[i] < this.a[j]
        return this.b[i] < this.b[j]
    }

    private swap(i: number, j: number): void {
        [this.key[i], this.key[j]] = [this.key[j], this.key[i]];
        [this.a[i], this.a[j]] = [this.a[j], this.a[i]];
        [this.b[i], this.b[j]] = [this.b[j], this.b[i]]
    }

    push(key: number, a: number, b: number): void {
        this.key.push(key)
        this.a.push(a)
        this.b.push(b)
        let c = this.size - 1
        while (c > 0) {
            const p = (c - 1) >> 1
            if (!this.less(c, p)) break
            this.swap(c, p)
            c = p
        }
    }

    pop(): [number, number, number] {
        const top: [number, number, number] = [this.key[0], this.a[0], this.b[0]]
        const last = this.size - 1
        this.swap(0, last)
        this.key.pop()
        this.a.pop()
        this.b.pop()
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

const INF = Infinity

export function runBITStar(opts: BITStarOptions): TraceEvent[] {
    const {map, start, goal, batchSize, maxBatches, gamma, seed} = opts
    const space = new SamplingGrid(map, seed)
    // planner RNG는 map RNG(space.sample이 쓰는 스트림)와 분리된다: 저장소는
    // np.random.default_rng(seed)를 planner 전용으로 따로 만들고, informed 타원
    // 표본만 그 스트림에서 뽑는다 (해가 없을 때의 균일 표본은 space RNG).
    const plannerRng = new NumpyRandom(seed)
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "bit_star",
          params: {batch_size: batchSize, max_batches: maxBatches, gamma, seed}})

    // 배치를 넘나드는 트리 상태 (병렬 배열). index 0 = start(root), 1 = goal(영구
    // 표본). g_t[i]는 cost-to-come(트리 밖이면 inf), samples는 아직 트리에 없는
    // 표본 index 집합.
    const points: Point[] = [start, goal]
    const gt: number[] = [0, INF]
    const parent: number[] = [-1, -1]
    const children: Array<Set<number>> = [new Set(), new Set()]
    const inTree: boolean[] = [true, false]
    const samples = new Set<number>([1])
    let cBest = INF
    let expanded = 0

    const addSample = (p: Point): number => {
        const idx = points.length
        points.push(p)
        gt.push(INF)
        parent.push(-1)
        children.push(new Set())
        inTree.push(false)
        samples.add(idx)
        return idx
    }

    const hHat = (i: number): number => space.distance(points[i], goal)
    const gHat = (i: number): number => space.distance(start, points[i])

    // rewire로 g_t[root]가 바뀌었을 때 그 델타를 부분트리로 밀어, 큐 key와 보고
    // 비용이 일관되게 유지되도록 한다.
    const propagate = (root: number): void => {
        const stack = [root]
        while (stack.length > 0) {
            const u = stack.pop()!
            for (const c of children[u]) {
                gt[c] = gt[u] + space.distance(points[u], points[c])
                stack.push(c)
            }
        }
    }

    for (let batch = 0; batch < maxBatches; batch++) {
        // --- 현직 해를 더는 개선할 수 없는 표본을 쳐낸다 -------------------------
        if (cBest < INF) {
            const prune: number[] = []
            for (const x of samples) if (gHat(x) + hHat(x) >= cBest) prune.push(x)
            for (const x of prune) samples.delete(x)
        }

        // --- 새 배치를 뽑는다 (해가 생긴 뒤엔 informed 타원) --------------------
        let drawn = 0
        for (let attempt = 0; attempt < batchSize * 40; attempt++) {
            if (drawn >= batchSize) break
            const q = informedSample(space, start, goal, cBest, plannerRng)
            if (!space.isStateValid(q)) continue
            addSample(q)
            drawn++
            emit({event: "sample_drawn", state: [q[0], q[1]]})
        }

        const n = points.length
        const radius = rggRadius(gamma, n)
        // 배치마다 전체 표본 위 반경 그래프를 한 번 다시 계산한다 (핫 루프가 near-set
        // 을 여러 번 훑으므로). 오름차순 index로 채워져 tie-break가 python과 같다.
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

        // --- 큐: 확장할 정점 + 후보 간선 --------------------------------------
        const qv = new Heap()
        const qe = new Heap()
        for (let v = 0; v < n; v++) {
            if (inTree[v]) qv.push(gt[v] + hHat(v), v, 0)
        }
        const expandedV = new Set<number>()

        const expandVertex = (v: number): void => {
            for (const x of nbr[v]) {
                if (samples.has(x)) {
                    // 아직 연결되지 않은 표본으로의 후보 간선.
                    if (gHat(v) + space.distance(points[v], points[x]) + hHat(x) < cBest) {
                        const key = gt[v] + space.distance(points[v], points[x]) + hHat(x)
                        qe.push(key, v, x)
                    }
                } else if (inTree[x] && x !== parent[v]) {
                    // v를 거치는 기존 정점의 후보 rewiring.
                    const cvw = space.distance(points[v], points[x])
                    if (gHat(v) + cvw + hHat(x) < cBest && gt[v] + cvw < gt[x]) {
                        qe.push(gt[v] + cvw + hHat(x), v, x)
                    }
                }
            }
        }

        // q_v 최상단의 유효 key를 반환하되, 이미 확장됐거나 개선으로 낡은 항목은
        // 꺼내 버린다 (lazy deletion). 비면 inf.
        const bestV = (): number => {
            while (qv.size > 0) {
                const key = qv.peekKey()
                const v = qv.peekA()
                if (expandedV.has(v) || key > gt[v] + hHat(v) + 1e-9) {
                    qv.pop()
                    continue
                }
                return key
            }
            return INF
        }
        const bestE = (): number => (qe.size > 0 ? qe.peekKey() : INF)

        for (;;) {
            // 최상단 간선을 이길 수 있는 정점을 모두 확장한다.
            while (qv.size > 0 && bestV() <= bestE()) {
                const [, v] = qv.pop()
                if (expandedV.has(v)) continue
                expandedV.add(v)
                expandVertex(v)
            }
            if (qe.size === 0) break
            const [edgeKey, vm, xm] = qe.pop()
            // 남은 최선 간선이 현직 해를 못 이기면 배치 종료.
            if (edgeKey >= cBest) break
            // 이 간선이 x_m의 트리 cost-to-come을 조금이라도 낮출 수 있는가?
            if (gt[vm] + space.distance(points[vm], points[xm]) >= gt[xm]) continue
            if (!space.isMotionValid(points[vm], points[xm])) continue
            const edgeCost = space.distance(points[vm], points[xm])
            const newG = gt[vm] + edgeCost
            if (newG + hHat(xm) >= cBest || newG >= gt[xm]) continue
            // 간선 채택: 표본을 연결하거나 v_m 아래로 정점을 rewire 한다.
            if (inTree[xm]) {
                children[parent[xm]].delete(xm)
            } else {
                samples.delete(xm)
                inTree[xm] = true
            }
            parent[xm] = vm
            gt[xm] = newG
            children[vm].add(xm)
            propagate(xm)
            expanded += 1
            expandedV.delete(xm)  // 개선됨: 이번 배치에서 재확장 허용
            qv.push(gt[xm] + hHat(xm), xm, 0)
            emit({event: "edge_added", state: [points[xm][0], points[xm][1]],
                  parent: [points[vm][0], points[vm][1]], cost: edgeCost})
            if (inTree[1] && gt[1] < cBest) {
                cBest = gt[1]
                emit({event: "candidate_evaluated", state: [goal[0], goal[1]], cost: cBest})
            }
        }
    }

    const success = inTree[1] && gt[1] < INF
    let path: number[][] = []
    if (success) {
        const poly: Point[] = []
        let node = 1
        while (node !== -1) {
            poly.push(points[node])
            node = parent[node]
        }
        poly.reverse()
        path = poly.map((p) => [p[0], p[1]])
        emit({event: "path_found", path})
    }
    const n = points.length
    const cost = success
        ? pathLength(space, path.map((p) => [p[0], p[1]] as Point))
        : 0
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: cost, expanded_nodes: expanded, samples: n, tree_size: n},
    })
    return events
}

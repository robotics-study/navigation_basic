import {GridMap, inBounds} from "../grid";
import {TraceEvent} from "../trace/types";
import {Cell} from "../trace/timeline";

// 브라우저 라이브 데모용 LPA* (lifelong replanner). 저장소 구현과 move-for-move
// bit-identical — 같은 DELTAS 순서, 같은 strict-< 최솟값 선택, 같은 heap tie-break
// (key 다음 삽입 counter)로 python trace와 이벤트 열까지 일치한다. LPA*는 고정
// start/goal의 forward 탐색: g/rhs는 시작 거리 추정이고 heuristic 기준(goal)이
// 움직이지 않으므로 D* Lite와 달리 k_m 오프셋이 없다 (Koenig & Likhachev 2001,
// "Incremental A*"; 저널 버전 Koenig, Likhachev & Furcy 2004).
export interface LpaStarRevision {
    cells: Cell[];      // 반전할 셀들 (world가 아니라 grid cell — 데모 시나리오와 동일)
    blocked: boolean;   // 반전 후 상태: true 면 벽 생성, false 면 free 복원
}

export interface LpaStarOptions {
    map: GridMap;                  // ground truth — LPA*는 round 0부터 이것을 안다
    start: Cell;
    goal: Cell;                    // 고정 goal (re-rooting은 D* Lite의 몫)
    revisions: Array<LpaStarRevision>;  // plan 라운드 사이에 "알려진" cost change 배치
}

export interface LpaStarRun {
    events: TraceEvent[];
}

const INF = Infinity;
const SQRT2 = Math.SQRT2;
type Key = [number, number];

// OccupancyGrid2D.heuristic과 같은 연산 순서 — sqrt가 아니라 hi-lo + √2*lo.
const octile = (a: Cell, b: Cell): number => {
    const dr = Math.abs(a[0] - b[0])
    const dc = Math.abs(a[1] - b[1])
    const lo = Math.min(dr, dc)
    const hi = Math.max(dr, dc)
    return (hi - lo) + SQRT2 * lo
}

const keyLess = (a: Key, b: Key): boolean =>
    a[0] < b[0] || (a[0] === b[0] && a[1] < b[1])

export function runLpaStar({map, start, goal, revisions}: LpaStarOptions): LpaStarRun {
    const events: TraceEvent[] = []
    let seq = 0
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev})
    emit({event: "planning_started", algorithm: "lpa_star", params: {}})

    const W = map.width
    const idx = (c: Cell) => c[0] * W + c[1]

    // 모델: 첫 plan()에서 ground truth 전체로 시드 — LPA*는 아는 것으로 계획한다.
    const blocked = new Set<number>()
    for (let i = 0; i < map.occupied.length; i++) if (map.occupied[i]) blocked.add(i)

    const g = new Map<number, number>()     // 시작 거리 추정 (없으면 INF)
    const rhs = new Map<number, number>()    // predecessor에 대한 one-step lookahead
    const keyOf = new Map<number, Key>()      // 큐에 있는 각 vertex의 현재 키
    const open: Array<{key: Key; id: number; cell: Cell}> = []
    let counter = 0     // 안정 tie-break — python heap tuple (key, counter) 순서와 동일
    let expanded = 0    // 모든 burst 누적 확장 수
    let replans = 0     // 완료된 re-plan 라운드 수

    const gOf = (c: Cell) => g.get(idx(c)) ?? INF
    const rhsOf = (c: Cell) => rhs.get(idx(c)) ?? INF
    const calcKey = (s: Cell): Key => {
        // CalculateKey(u) = [min(g,rhs) + h(goal,u); min(g,rhs)]. k_m 항 없음:
        // heuristic의 기준(goal)이 움직이지 않아 키는 replan 내내 비교 가능.
        const m = Math.min(gOf(s), rhsOf(s))
        return [m + octile(s, goal), m]
    }

    const DELTAS: Array<[number, number, number]> = [
        [-1, 0, 1], [1, 0, 1], [0, -1, 1], [0, 1, 1],
        [-1, -1, SQRT2], [-1, 1, SQRT2], [1, -1, SQRT2], [1, 1, SQRT2],
    ]
    // 모델 기준 통과 가능 이웃 (실제 지도가 아니라 플랜이 아는 세계). 대각은 양 옆이
    // 뚫려 있어야 한다 — 저장소 _MOVES_8/corner rule과 동일한 순서·판정.
    const passable = (u: Cell): Array<[Cell, number]> => {
        const out: Array<[Cell, number]> = []
        for (const [dr, dc, cost] of DELTAS) {
            const c: Cell = [u[0] + dr, u[1] + dc]
            if (!inBounds(map, c[0], c[1]) || blocked.has(idx(c))) continue
            if (dr !== 0 && dc !== 0) {
                const a: Cell = [u[0] + dr, u[1]]
                const b: Cell = [u[0], u[1] + dc]
                if ((inBounds(map, a[0], a[1]) && blocked.has(idx(a)))
                    || (inBounds(map, b[0], b[1]) && blocked.has(idx(b)))) continue
            }
            out.push([c, cost])
        }
        return out
    }

    const queueInsert = (u: Cell, key: Key) => {
        keyOf.set(idx(u), key)
        open.push({key, id: counter++, cell: u})
    }
    // 낡은 항목(저장 키가 현재 keyOf와 다른 항목)을 걷어내며 살아 있는 최소 키를
    // 찾는다. python heap은 (key, counter) 순서이므로 같은 키는 삽입 순서로 갈린다.
    const peekTop = (): {key: Key; cell: Cell} | null => {
        let best: {key: Key; id: number; cell: Cell} | null = null
        let bestAt = -1
        for (let i = 0; i < open.length; i++) {
            const e = open[i]
            const live = keyOf.get(idx(e.cell))
            if (!live || live[0] !== e.key[0] || live[1] !== e.key[1]) continue
            if (!best || keyLess(e.key, best.key) || (!keyLess(best.key, e.key) && e.id < best.id)) {
                best = e
                bestAt = i
            }
        }
        if (!best) {
            open.length = 0
            return null
        }
        open.splice(bestAt, 1)
        open.push(best)   // peek이므로 유지 (pop은 keyOf 삭제로 표현)
        return {key: best.key, cell: best.cell}
    }

    const updateVertex = (u: Cell) => {
        // UpdateVertex(u): rhs = predecessor에 대한 min (격자는 무향이라 u의 통과
        // 가능 이웃이 곧 predecessor). start는 rhs=0 고정.
        if (u[0] !== start[0] || u[1] !== start[1]) {
            let best = INF
            let sbest: Cell | null = null
            let bestEdge = 0
            for (const [s2, cost] of passable(u)) {
                const v = cost + gOf(s2)
                if (v < best) {          // strict < — python과 같은 첫 최솟값 선택
                    best = v
                    sbest = s2
                    bestEdge = cost
                }
            }
            const old = rhsOf(u)
            if (best !== old) {          // 시작 거리 추정의 실제 relaxation
                rhs.set(idx(u), best)
                if (sbest && best < INF) {
                    emit({event: "candidate_evaluated", state: u, cost: best})
                    emit({event: "edge_added", state: u, parent: sbest, cost: bestEdge})
                }
            }
        }
        keyOf.delete(idx(u))   // 큐에서 제거; 아래에서 inconsistent면 다시 삽입
        if (gOf(u) !== rhsOf(u)) queueInsert(u, calcKey(u))
    }

    const computeShortestPath = () => {
        // ComputeShortestPath(): goal이 locally consistent하고 살아 있는 최소 키가
        // goal의 키보다 커질 때까지 key 순서로 확장한다.
        for (;;) {
            const top = peekTop()
            if (!top) break
            if (!(keyLess(top.key, calcKey(goal)) || rhsOf(goal) !== gOf(goal))) break
            const u = top.cell
            keyOf.delete(idx(u))   // pop: 이 항목은 이제 낡은 항목이다
            expanded++
            emit({event: "node_expanded", state: u, cost: Math.min(gOf(u), rhsOf(u))})
            const kNew = calcKey(u)
            if (keyLess(top.key, kNew)) {
                // key-invariant (k_m 없음): popped key는 pop 자체로 낡은 것일 뿐이라
                // 재삽입은 증명된 dead code — 논문 pseudocode대로 그대로 둔다.
                queueInsert(u, kNew)
            } else if (gOf(u) > rhsOf(u)) {
                g.set(idx(u), rhsOf(u))   // over-consistent: 받아들이고 successor 갱신
                for (const [s2] of passable(u)) updateVertex(s2)
            } else {
                g.set(idx(u), INF)        // under-consistent: ∞로 올리고 재평가
                updateVertex(u)
                for (const [s2] of passable(u)) updateVertex(s2)
            }
        }
    }

    // parent chain을 goal에서 거꾸로 걷기: u에서 cost + g(p) == rhs(u)를 만족하는
    // 첫 neighbor p는 정확히 rhs(u)를 설정한 predecessor (연산자·순서 동일 → 등호가
    // bit-exact). 체인 vertex는 burst 종료 시 모두 locally consistent.
    const extractPath = (): {path: Cell[]; cost: number} => {
        if (gOf(goal) === INF) return {path: [], cost: 0}
        const rev: Cell[] = [goal]
        let cost = 0
        let cur = goal
        while (!(cur[0] === start[0] && cur[1] === start[1])) {
            const rhsCur = rhsOf(cur)
            let nxt: Cell | null = null
            let stepCost = 0
            for (const [s2, edge] of passable(cur)) {
                if (edge + gOf(s2) === rhsCur) {
                    nxt = s2
                    stepCost = edge
                    break
                }
            }
            if (!nxt) return {path: [], cost: 0}   // 올바른 실행에서는 도달 불가 — 정직하게 실패
            cost += stepCost
            rev.push(nxt)
            cur = nxt
        }
        rev.reverse()
        return {path: rev, cost}
    }

    // Initialize(): g/rhs는 기본 INF (빈 map), start의 rhs만 0으로 고정하고 큐에 삽입.
    rhs.set(idx(start), 0)
    queueInsert(start, calcKey(start))

    const round = () => {
        computeShortestPath()
        const {path, cost} = extractPath()
        const reached = path.length > 0
        if (reached) emit({event: "path_found", path})
        emit({
            event: "planning_finished", success: reached,
            metrics: {path_cost: cost, expanded_nodes: expanded, replan_count: replans},
        })
    }

    round()   // round 0 — 시드된 모델 위의 전체 burst
    for (const rev of revisions) {
        // 논문의 Main(): 열린 셀은 자기와 통과 가능 neighbor를 UpdateVertex, 막히는
        // 셀은 U에서 제거한다(막힌 vertex가 낡은 키로 pop 되어 헛확장하는 일 없이).
        // MODEL 상태가 무엇을 바꿨는지 결정한다 (ground truth가 아니라) 그래서
        // free→blocked→free 왕복이 합법이다. no-op 반전은 어떤 edge cost도 바꾸지
        // 않으므로 아무것도 방출하지 않는다.
        for (const c of rev.cells) {
            const wasBlocked = blocked.has(idx(c))
            if (wasBlocked === rev.blocked) continue
            emit({event: "obstacle_changed", state: [c[0], c[1]], blocked: rev.blocked})
            if (rev.blocked) {
                blocked.add(idx(c))
                keyOf.delete(idx(c))   // blocked vertex는 U에 남아 있으면 안 된다
            } else {
                blocked.delete(idx(c))
                updateVertex(c)         // c가 다시 vertex가 됐다: rhs 재계산
            }
            for (const [s2] of passable(c)) updateVertex(s2)
        }
        replans++
        round()
    }
    return {events}
}

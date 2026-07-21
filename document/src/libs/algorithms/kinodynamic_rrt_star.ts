import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";

// 브라우저 라이브 데모용 Kinodynamic RRT* (Webb & van den Berg 2013). 저장소 구현을
// 그대로 미러한다: 상태는 double integrator (x, y, vx, vy)이고, 간선은 직선이 아니라
// 고정 최종상태·자유 최종시간 최적 제어기의 궤적이다. 그 최적 도달 비용
// J = ∫(1 + r·uᵀu)dt이 nearest 척도이자 choose-parent/rewire 비용이 되어, 트리가
// 유클리드가 아니라 동역학이 유도하는 비용 기하에서 자란다. planner RNG와 map RNG가
// 같은 seed의 독립 numpy 스트림이고 draw 순서(goal 동전 → sample x,y → vx → vy)까지
// 미러하므로, 같은 seed면 python demo와 확장 수·비용까지 일치한다.
export interface KinodynamicRRTStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    maxIterations: number;
    goalBias: number;
    goalTolerance: number;
    neighborRadius: number;
    controlWeight: number;
    maxVelocity: number;
    seed: number;
}

// double integrator 계획 상태: world 위치 + 속도. planner만 이 4D 공간을 알고,
// SamplingSpace 맵에는 (x, y) 투영만 넘어간다.
type State4 = [number, number, number, number];

const EPS = 1e-9;
// nearest는 exact 최적 비용으로 고르지만, 매 반복 전 노드에 quartic을 푸는 것은
// 브라우저에서 비싸다. double-integrator 비용이 위치 격차에 지배되므로 유클리드로 K개를
// 선별한 뒤 그 안에서 exact 비용 최소를 택한다 (K-nearest RRT*, Karaman & Frazzoli 2011).
const NEAREST_CANDIDATES = 16;
// choose-parent/rewire 근방을 neighbor_radius 안 K개로 제한해 밀집 트리에서 반복당
// 작업량을 유계로 둔다.
const MAX_NEIGHBORS = 16;
// 궤적 충돌 표본: 직선 격차 이만큼(m)마다 waypoint 하나 (is_motion_valid supercover가
// 각 소구간을 채운다), 하한·상한 clamp.
const COLLISION_SPACING = 0.3;
const MIN_WAYPOINTS = 4;
const MAX_WAYPOINTS = 64;

// double-integrator cost-to-go c(t)의 축별 합 (C₁, C₂, C₃). a = 위치 격차,
// v0/v1 = 양끝 속도 (Webb & van den Berg 2013).
function steerCoeffs(x0: State4, x1: State4): [number, number, number] {
    let c1 = 0;
    let c2 = 0;
    let c3 = 0;
    const axes: Array<[number, number, number]> = [
        [x1[0] - x0[0], x0[2], x1[2]],
        [x1[1] - x0[1], x0[3], x1[3]],
    ];
    for (const [a, v0, v1] of axes) {
        c3 += 12.0 * a * a;
        c2 += -12.0 * a * (v0 + v1);
        c1 += 4.0 * (v0 * v0 + v0 * v1 + v1 * v1);
    }
    return [c1, c2, c3];
}

const costAt = (t: number, r: number, c1: number, c2: number, c3: number): number =>
    t + r * (c3 / (t * t * t) + c2 / (t * t) + c1 / t);

// 뿌리 하나에 Durand-Kerner를 돌리는 대신, 계수 실수 quartic의 모든 근을 한 번에
// 구한다. numpy.roots(companion 고유값)와 같은 근 집합에 수렴하므로, 아래 필터(양의
// 실근 중 c(t) 최소)가 python과 같은 τ*를 고른다.
type Complex = {re: number; im: number};

// 몫 t⁴ + a2·t² + a1·t + a0 (a3 = 0)의 네 근을 Durand-Kerner (Weierstrass) 반복으로
// 구한다. 초기값은 표준 회전 배치 (0.4 + 0.9i)^k. 저차라 수십 회면 배정밀도로 수렴한다.
function solveDepressedQuartic(a2: number, a1: number, a0: number): Complex[] {
    const coeffs = [1.0, 0.0, a2, a1, a0]; // 내림차순
    const evalPoly = (z: Complex): Complex => {
        let re = coeffs[0];
        let im = 0;
        for (let k = 1; k < coeffs.length; k++) {
            const nre = re * z.re - im * z.im + coeffs[k];
            const nim = re * z.im + im * z.re;
            re = nre;
            im = nim;
        }
        return {re, im};
    };
    const seed: Complex = {re: 0.4, im: 0.9};
    const roots: Complex[] = [];
    let pk: Complex = {re: 1, im: 0};
    for (let k = 0; k < 4; k++) {
        roots.push({re: pk.re, im: pk.im});
        const nre = pk.re * seed.re - pk.im * seed.im;
        const nim = pk.re * seed.im + pk.im * seed.re;
        pk = {re: nre, im: nim};
    }
    for (let iter = 0; iter < 100; iter++) {
        let maxDelta = 0;
        for (let i = 0; i < 4; i++) {
            const zi = roots[i];
            // 분모 = ∏_{j≠i} (z_i − z_j).
            let dre = 1;
            let dim = 0;
            for (let j = 0; j < 4; j++) {
                if (j === i) continue;
                const wre = zi.re - roots[j].re;
                const wim = zi.im - roots[j].im;
                const nre = dre * wre - dim * wim;
                const nim = dre * wim + dim * wre;
                dre = nre;
                dim = nim;
            }
            const p = evalPoly(zi);
            const den = dre * dre + dim * dim;
            if (den === 0) continue;
            // (p / denom)를 뺀다.
            const qre = (p.re * dre + p.im * dim) / den;
            const qim = (p.im * dre - p.re * dim) / den;
            roots[i] = {re: zi.re - qre, im: zi.im - qim};
            const delta = Math.hypot(qre, qim);
            if (delta > maxDelta) maxDelta = delta;
        }
        if (maxDelta < 1e-15) break;
    }
    return roots;
}

// 고정 최종상태·자유 최종시간 최적 비용과 도달 시간 τ* (Webb & van den Berg 2013).
// 상태가 일치하면 (0, 0).
function optimalCost(x0: State4, x1: State4, r: number): [number, number] {
    const [c1, c2, c3] = steerCoeffs(x0, x1);
    if (Math.abs(c1) < EPS && Math.abs(c2) < EPS && Math.abs(c3) < EPS) return [0.0, 0.0];
    // c'(t)=0 을 t⁴ 로 소거한 depressed quartic: t⁴ − r·C₁·t² − 2r·C₂·t − 3r·C₃ = 0.
    const roots = solveDepressedQuartic(-r * c1, -2.0 * r * c2, -3.0 * r * c3);
    let bestCost = Infinity;
    let bestTau = 0.0;
    for (const root of roots) {
        if (Math.abs(root.im) > EPS * (1.0 + Math.abs(root.re))) continue;
        const t = root.re;
        if (t <= EPS) continue;
        const c = costAt(t, r, c1, c2, c3);
        if (c < bestCost) {
            bestCost = c;
            bestTau = t;
        }
    }
    return [bestCost, bestTau];
}

// 정규화 시간 s∈[0,1]의 최적(min-∫‖u‖²) 3차 위치. 접선은 양끝 속도를 τ*로 스케일한
// 값 (모든 경계조건을 만족하는 유일 3차).
function hermiteXY(x0: State4, x1: State4, tau: number, s: number): Point {
    const s2 = s * s;
    const s3 = s2 * s;
    const h00 = 2.0 * s3 - 3.0 * s2 + 1.0;
    const h10 = s3 - 2.0 * s2 + s;
    const h01 = -2.0 * s3 + 3.0 * s2;
    const h11 = s3 - s2;
    const x = h00 * x0[0] + h10 * tau * x0[2] + h01 * x1[0] + h11 * tau * x1[2];
    const y = h00 * x0[1] + h10 * tau * x0[3] + h01 * x1[1] + h11 * tau * x1[3];
    return [x, y];
}

// 최적 궤적의 (x, y) waypoint, 부모 제외 (s>0..1).
function trajectoryXY(x0: State4, x1: State4, tau: number): Point[] {
    const gap = Math.hypot(x1[0] - x0[0], x1[1] - x0[1]);
    let n = Math.max(MIN_WAYPOINTS, Math.ceil(gap / COLLISION_SPACING));
    n = Math.min(MAX_WAYPOINTS, n);
    const out: Point[] = [];
    for (let k = 1; k <= n; k++) out.push(hermiteXY(x0, x1, tau, k / n));
    return out;
}

// double-integrator 상태 위 탐색 트리 (병렬 배열). 기하 RRT* 트리를 미러하되 유클리드
// 거리 대신 최적제어 비용으로 키운다. 각 노드는 들어오는 간선 비용 + dense 궤적을 들어,
// rewire가 궤적 재계산 없이 부분 트리의 누적 비용을 밀어내리고 경로 복원이 실제 곡선
// 궤적을 낼 수 있다.
class KinoTree {
    states: State4[];
    parent: number[];
    cost: number[];
    edgeCost: number[];
    incoming: Point[][];
    children: number[][];

    constructor(root: State4) {
        this.states = [root];
        this.parent = [-1];
        this.cost = [0.0];
        this.edgeCost = [0.0];
        this.incoming = [[]];
        this.children = [[]];
    }

    get size(): number {
        return this.states.length;
    }

    add(state: State4, parentIdx: number, edgeCost: number, traj: Point[]): number {
        const idx = this.states.length;
        this.states.push(state);
        this.parent.push(parentIdx);
        this.cost.push(this.cost[parentIdx] + edgeCost);
        this.edgeCost.push(edgeCost);
        this.incoming.push(traj);
        this.children.push([]);
        this.children[parentIdx].push(idx);
        return idx;
    }

    positions(): Point[] {
        return this.states.map((s) => [s[0], s[1]] as Point);
    }

    reparent(child: number, newParent: number, edgeCost: number, traj: Point[]): void {
        const old = this.parent[child];
        if (old >= 0) {
            const sib = this.children[old];
            sib.splice(sib.indexOf(child), 1);
        }
        this.parent[child] = newParent;
        this.edgeCost[child] = edgeCost;
        this.incoming[child] = traj;
        this.cost[child] = this.cost[newParent] + edgeCost;
        this.children[newParent].push(child);
        // 간선 비용은 조상 rewire로 바뀌지 않으므로 누적 합만 부분 트리로 밀어내린다.
        const stack = [child];
        while (stack.length > 0) {
            const u = stack.pop()!;
            for (const c of this.children[u]) {
                this.cost[c] = this.cost[u] + this.edgeCost[c];
                stack.push(c);
            }
        }
    }

    pathXYTo(idx: number): Point[] {
        const chain: number[] = [];
        let node = idx;
        while (node !== -1) {
            chain.push(node);
            node = this.parent[node];
        }
        chain.reverse();
        const root = this.states[chain[0]];
        const path: Point[] = [[root[0], root[1]]];
        for (let i = 1; i < chain.length; i++) path.push(...this.incoming[chain[i]]);
        return path;
    }
}

type Emit = (ev: Omit<TraceEvent, "seq">) => void;

// 최적 간선 x0→x1: 충돌 없으면 (edge_cost, dense 궤적), 아니면 null.
function connect(
    space: SamplingGrid, x0: State4, x1: State4, r: number,
): {edgeCost: number; traj: Point[]} | null {
    const [cost, tau] = optimalCost(x0, x1, r);
    if (!Number.isFinite(cost) || tau <= EPS) return null;
    const traj = trajectoryXY(x0, x1, tau);
    let prev: Point = [x0[0], x0[1]];
    for (const pt of traj) {
        if (!space.isMotionValid(prev, pt)) return null;
        prev = pt;
    }
    return {edgeCost: cost, traj};
}

export function runKinodynamicRRTStar(opts: KinodynamicRRTStarOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, goalBias, goalTolerance, neighborRadius,
           controlWeight: r, maxVelocity, seed} = opts;
    const space = new SamplingGrid(map, seed);
    const rng = new NumpyRandom(seed);
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit: Emit = (ev) => events.push({seq: seq++, ...ev});
    emit({
        event: "planning_started",
        algorithm: "kinodynamic_rrt_star",
        params: {max_iterations: maxIterations, goal_bias: goalBias,
                 goal_tolerance: goalTolerance, neighbor_radius: neighborRadius,
                 control_weight: r, max_velocity: maxVelocity, seed},
    });

    // 고정 최종상태: start/goal은 정지 상태(속도 0)로 들어 올린다.
    const xStart: State4 = [start[0], start[1], 0.0, 0.0];
    const xGoal: State4 = [goal[0], goal[1], 0.0, 0.0];
    const tree = new KinoTree(xStart);

    // goal-bias 표본은 goal 정지 상태를 직접 뽑고, 아니면 자유 위치 + [-v_max, v_max]²
    // 속도 (완전 4D 표본) (LaValle 1998).
    const sample = (): State4 => {
        if (rng.random() < goalBias) return [xGoal[0], xGoal[1], xGoal[2], xGoal[3]];
        const [px, py] = space.sample();
        const vx = rng.uniform(-maxVelocity, maxVelocity);
        const vy = rng.uniform(-maxVelocity, maxVelocity);
        return [px, py, vx, vy];
    };

    // 유클리드로 K개를 선별한 뒤 exact 최적 비용 최소를 택한다.
    const nearest = (positions: Point[], target: State4): number => {
        const n = tree.size;
        const order: number[] = positions
            .map((p, i): [number, number] => [Math.hypot(p[0] - target[0], p[1] - target[1]), i])
            .sort((a, b) => a[0] - b[0])
            .map(([, i]) => i);
        const k = Math.min(NEAREST_CANDIDATES, n);
        const candidates = k < n ? order.slice(0, k) : order;
        let bestIdx = -1;
        let bestCost = Infinity;
        for (const i of candidates) {
            const [cost] = optimalCost(tree.states[i], target, r);
            if (cost < bestCost) {
                bestCost = cost;
                bestIdx = i;
            }
        }
        return bestIdx;
    };

    // neighbor_radius 안 노드 (index 오름차순); 16 초과면 거리순 16개로 자른다. rewire가
    // 이 순서대로 greedy reparent 하므로 순서를 python과 맞춘다.
    const neighborhood = (positions: Point[], target: State4): number[] => {
        const within: number[] = [];
        for (let i = 0; i < positions.length; i++) {
            const d = Math.hypot(positions[i][0] - target[0], positions[i][1] - target[1]);
            if (d <= neighborRadius) within.push(i);
        }
        if (within.length > MAX_NEIGHBORS) {
            within.sort((a, b) => {
                const da = Math.hypot(positions[a][0] - target[0], positions[a][1] - target[1]);
                const db = Math.hypot(positions[b][0] - target[0], positions[b][1] - target[1]);
                return da - db;
            });
            return within.slice(0, MAX_NEIGHBORS);
        }
        return within;
    };

    // choose-parent: x_new을 근방 최소 비용의 실행 가능 부모에 붙인다 (Webb & van den
    // Berg 2013의 kinodynamic 형태, Karaman & Frazzoli 2011).
    const chooseParent = (
        xNew: State4, nearIdx: number, neighbors: number[],
    ): number => {
        const seen = new Set<number>();
        const cands: number[] = [];
        for (const j of [nearIdx, ...neighbors]) {
            if (j < 0 || seen.has(j)) continue;
            seen.add(j);
            cands.push(j);
        }
        let bestParent = -1;
        let bestEdge: Point[] = [];
        let bestTotal = Infinity;
        for (const j of cands) {
            const conn = connect(space, tree.states[j], xNew, r);
            if (conn === null) continue;
            const total = tree.cost[j] + conn.edgeCost;
            emit({event: "candidate_evaluated", state: [xNew[0], xNew[1]], cost: total});
            if (total < bestTotal) {
                bestTotal = total;
                bestParent = j;
                bestEdge = conn.traj;
            }
        }
        if (bestParent < 0) return -1;
        const edgeCost = bestTotal - tree.cost[bestParent];
        const newIdx = tree.add(xNew, bestParent, edgeCost, bestEdge);
        // 곡선 간선을 chord 열로 방출해 궤적이 그려지게 한다 (SamplingSpace viz는 직선
        // edge_added만 그린다).
        let prev: Point = [tree.states[bestParent][0], tree.states[bestParent][1]];
        for (const pt of bestEdge) {
            emit({event: "edge_added", state: [pt[0], pt[1]], parent: [prev[0], prev[1]]});
            prev = pt;
        }
        return newIdx;
    };

    const rewire = (newIdx: number, neighbors: number[]): void => {
        const xNew = tree.states[newIdx];
        for (const j of neighbors) {
            if (j === tree.parent[newIdx] || j === newIdx) continue;
            const conn = connect(space, xNew, tree.states[j], r);
            if (conn === null) continue;
            if (tree.cost[newIdx] + conn.edgeCost < tree.cost[j]) {
                tree.reparent(j, newIdx, conn.edgeCost, conn.traj);
                emit({event: "rewire", state: [tree.states[j][0], tree.states[j][1]],
                      parent: [xNew[0], xNew[1]]});
            }
        }
    };

    // x_new에서 goal 정지 상태에 닿는 비용 + 궤적. goal-bias 표본이 x_new을 goal 위에
    // 놓으면 자기연결이 퇴화(τ*=0)라 노드 자체가 도착이다 (궤적 비고, 비용은 누적 트리
    // 비용). 아니면 goal 정지 상태로 최적 간선을 날리고 충돌 없음을 요구한다.
    const goalArrival = (newIdx: number, xNew: State4): {cost: number; traj: Point[]} | null => {
        if (Math.abs(xNew[0] - xGoal[0]) <= EPS && Math.abs(xNew[1] - xGoal[1]) <= EPS
            && Math.abs(xNew[2]) <= EPS && Math.abs(xNew[3]) <= EPS) {
            return {cost: tree.cost[newIdx], traj: []};
        }
        const conn = connect(space, xNew, xGoal, r);
        if (conn === null) return null;
        return {cost: tree.cost[newIdx] + conn.edgeCost, traj: conn.traj};
    };

    // goal은 성장/rewire 노드가 아니다 (Karaman & Frazzoli 2011); 최선 부모 + 궤적만 든다.
    let bestGoalParent = -1;
    let bestGoalCost = Infinity;
    let iterations = 0;

    for (let it = 0; it < maxIterations; it++) {
        iterations++;
        const qRand = sample();
        emit({event: "sample_drawn", state: [qRand[0], qRand[1]]});
        const positions = tree.positions();
        const nearIdx = nearest(positions, qRand);
        // 최적 steering이 표본 상태에 정확히 도달하므로 (제어 가능 선형계라 절단 불필요)
        // x_new은 표본 자체다.
        const xNew = qRand;
        const neighbors = neighborhood(positions, xNew);
        const newIdx = chooseParent(xNew, nearIdx, neighbors);
        if (newIdx < 0) continue;
        rewire(newIdx, neighbors);

        if (Math.hypot(xNew[0] - goal[0], xNew[1] - goal[1]) <= goalTolerance) {
            const arrival = goalArrival(newIdx, xNew);
            if (arrival !== null && arrival.cost < bestGoalCost) {
                bestGoalCost = arrival.cost;
                bestGoalParent = newIdx;
                const path = [...tree.pathXYTo(newIdx), ...arrival.traj];
                emit({event: "path_found", path: path.map((p) => [p[0], p[1]])});
            }
        }
    }

    const success = bestGoalParent >= 0;
    // 최종 경로는 개선 때마다 path_found로 이미 방출됐다 (마지막 path_found = best).
    const cost = success ? bestGoalCost : 0;
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: cost, expanded_nodes: tree.size - 1,
                  samples: iterations, tree_size: tree.size, iterations},
    });
    return events;
}

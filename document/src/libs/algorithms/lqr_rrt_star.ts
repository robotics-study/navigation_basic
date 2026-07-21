import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {NumpyRandom} from "./numpy_rng";
import {Point, SamplingGrid} from "./sampling_space";

// 브라우저 라이브 데모용 LQR-RRT* (Perez, Platt, Konidaris, Kaelbling &
// Lozano-Pérez 2012). 저장소 python 구현을 그대로 미러한다. RRT*의 거리 metric과
// steering을 손으로 짜지 않고, double integrator를 선형화한 뒤 이차 비용
// J=∫(xᵀQx+uᵀRu)dt의 LQR 해에서 자동으로 끌어낸다. 거리 = LQR cost-to-go
// dist(a,b)=(a−b)ᵀS(a−b), steering = 피드백 u=−K(x−ref)를 정방향 적분한 rest→rest
// 궤적이다. numpy PCG64 RNG(표본)와 2×2 Riccati/궤적 적분의 연산 순서까지 미러하므로,
// 같은 seed면 python demo와 확장 수·비용까지 일치한다.
export interface LQRRRTStarOptions {
    map: GridMap;
    start: Point;
    goal: Point;
    maxIterations: number;
    stepSize: number;
    goalBias: number;
    goalTolerance: number;
    neighborRadius: number;
    qPos: number;
    qVel: number;
    rCtrl: number;
    lqrDt: number;
    controlLimit: number;
    maxVelocity: number;
    seed: number;
}

// double integrator 계획 상태: world 위치 + 속도 (x, y, vx, vy). (x, y) 투영만
// SamplingSpace 맵에 넘어간다 (맵은 속도를 모른다).
type State4 = [number, number, number, number];

const EPS = 1e-9;
// 정확한 LQR-metric 비교 전, 위치 유클리드로 K개 후보만 추리는 prefilter
// (k-nearest RRT* 변형, Karaman & Frazzoli 2011).
const NEAREST_CANDIDATES = 16;
// choose-parent / rewire 근방 크기 상한 — 반복당 작업을 유계로 유지한다.
const MAX_NEIGHBORS = 16;
// LQR steering 지평선: rest→rest 조절기는 점근 수렴하므로 적분 스텝 수를 캡한다.
const STEER_MAX_STEPS = 200;
// rest waypoint "도달" 판정 위치/속도 오차 임계.
const REACH_POS_TOL = 0.05;
const REACH_VEL_TOL = 0.05;
// DARE 고정점 반복 예산 + 수렴 tolerance (2×2 문제).
const DARE_MAX_ITERS = 1000;
const DARE_TOL = 1e-12;

type Mat2 = [number, number, number, number]; // [00, 01, 10, 11]

// 축별 이산 LQR (double integrator)을 Riccati(DARE) 반복으로 푼다. S(2×2 cost-to-go)와
// K(1×2 피드백)을 반환한다 (Perez et al. 2012). LTI + 대각 Q라 두 축이 이 값을 공유한다.
// numpy 행렬 연산 순서를 손으로 푼 산술로 그대로 재현한다 — S/K가 모든 roll에 쓰이므로
// 여기서 1비트라도 어긋나면 parity가 깨진다.
export function solveDLQR(
    qPos: number, qVel: number, rCtrl: number, dt: number,
): {s: Mat2; k: [number, number]} {
    // a=[[1,dt],[0,1]], b=[[dt²/2],[dt]], q=diag(qPos,qVel), r=[rCtrl].
    const a00 = 1.0, a01 = dt, a10 = 0.0, a11 = 1.0;
    const b0 = 0.5 * dt * dt, b1 = dt;
    const q00 = qPos, q11 = qVel;
    let p00 = q00, p01 = 0.0, p10 = 0.0, p11 = q11;

    // p_next 계산 한 스텝 — numpy의 좌결합 (a.T@p@a), ((a.T@p)@b)@gain 순서를 따른다.
    const iterate = (): {n00: number; n01: number; n10: number; n11: number;
                         g0: number; g1: number} => {
        // bt_p = b.T @ p  (1×2): b.T = [b0, b1]
        const btp0 = b0 * p00 + b1 * p10;
        const btp1 = b0 * p01 + b1 * p11;
        // rbtpb = r + bt_p @ b  (1×1)
        const rbtpb = rCtrl + (btp0 * b0 + btp1 * b1);
        // bt_p @ a  (1×2): a=[[1,dt],[0,1]]
        const btpa0 = btp0 * a00 + btp1 * a10;
        const btpa1 = btp0 * a01 + btp1 * a11;
        // gain = solve(rbtpb, bt_p@a) — 1×1 계에서는 나눗셈.
        const g0 = btpa0 / rbtpb;
        const g1 = btpa1 / rbtpb;
        // atp = a.T @ p  (2×2): a.T = [[1,0],[dt,1]]
        const atp00 = a00 * p00 + a10 * p10;
        const atp01 = a00 * p01 + a10 * p11;
        const atp10 = a01 * p00 + a11 * p10;
        const atp11 = a01 * p01 + a11 * p11;
        // atpa = atp @ a  (2×2)
        const atpa00 = atp00 * a00 + atp01 * a10;
        const atpa01 = atp00 * a01 + atp01 * a11;
        const atpa10 = atp10 * a00 + atp11 * a10;
        const atpa11 = atp10 * a01 + atp11 * a11;
        // atpb = atp @ b  (2×1)
        const atpb0 = atp00 * b0 + atp01 * b1;
        const atpb1 = atp10 * b0 + atp11 * b1;
        // (a.T@p@b) @ gain  (2×2 외적)
        const n00 = q00 + atpa00 - atpb0 * g0;
        const n01 = 0.0 + atpa01 - atpb0 * g1;
        const n10 = 0.0 + atpa10 - atpb1 * g0;
        const n11 = q11 + atpa11 - atpb1 * g1;
        return {n00, n01, n10, n11, g0, g1};
    };

    for (let i = 0; i < DARE_MAX_ITERS; i++) {
        const {n00, n01, n10, n11} = iterate();
        const maxDiff = Math.max(
            Math.abs(n00 - p00), Math.abs(n01 - p01),
            Math.abs(n10 - p10), Math.abs(n11 - p11));
        if (maxDiff < DARE_TOL) {
            p00 = n00; p01 = n01; p10 = n10; p11 = n11;
            break;
        }
        p00 = n00; p01 = n01; p10 = n10; p11 = n11;
    }
    // 수렴한 P로 gain을 한 번 더 (python은 루프 후 solve를 재실행).
    const {g0, g1} = iterate();
    return {s: [p00, p01, p10, p11], k: [g0, g1]};
}

// LQR 거리 metric dist(a,b)=(a−b)ᵀS(a−b)를 두 분리 축에 대해 더한 값 (Perez et al. 2012).
export function lqrCostToGo(a: State4, b: State4, s: Mat2): number {
    const dpx = a[0] - b[0], dpy = a[1] - b[1];
    const dvx = a[2] - b[2], dvy = a[3] - b[3];
    const s00 = s[0], s01 = s[1], s11 = s[3];
    return (
        s00 * (dpx * dpx + dpy * dpy)
        + 2.0 * s01 * (dpx * dvx + dpy * dvy)
        + s11 * (dvx * dvx + dvy * dvy)
    );
}

// double integrator rest 상태 위 탐색 트리 (병렬 배열). 각 노드는 들어오는 edge 비용과
// LQR roll의 dense (x, y) 궤적을 들어, rewire가 부분 트리 비용을 re-steer 없이 전파하고
// 경로 복원이 실제 곡선 궤적을 방출한다.
class LqrTree {
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
        // 조상 rewire는 edge 비용을 바꾸지 않고 누적합만 이동시키므로, 부분 트리로
        // 비용 delta를 밀어 후손이 정확한 비용을 유지하게 한다.
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
        for (let i = 1; i < chain.length; i++) {
            for (const p of this.incoming[chain[i]]) path.push(p);
        }
        return path;
    }
}

export function runLQRRRTStar(opts: LQRRRTStarOptions): TraceEvent[] {
    const {map, start, goal, maxIterations, stepSize, goalBias, goalTolerance,
           neighborRadius, qPos, qVel, rCtrl, lqrDt, controlLimit, maxVelocity, seed} = opts;
    const space = new SamplingGrid(map, seed);
    const rng = new NumpyRandom(seed);
    const events: TraceEvent[] = [];
    let seq = 0;
    const emit = (ev: Omit<TraceEvent, "seq">) => events.push({seq: seq++, ...ev});
    emit({
        event: "planning_started",
        algorithm: "lqr_rrt_star",
        params: {max_iterations: maxIterations, step_size: stepSize, goal_bias: goalBias,
                 goal_tolerance: goalTolerance, neighbor_radius: neighborRadius,
                 q_pos: qPos, q_vel: qVel, r_ctrl: rCtrl, lqr_dt: lqrDt,
                 control_limit: controlLimit, max_velocity: maxVelocity, seed},
    });

    // Riccati는 구성 시 한 번 — 이 LTI 계에서 S/K는 상태 독립이다.
    const {s, k} = solveDLQR(qPos, qVel, rCtrl, lqrDt);
    const k0 = k[0], k1 = k[1];
    const dt = lqrDt;
    const halfDt2 = 0.5 * dt * dt;
    const uMax = controlLimit;

    const xStart: State4 = [start[0], start[1], 0.0, 0.0];
    const xGoal: State4 = [goal[0], goal[1], 0.0, 0.0];
    const tree = new LqrTree(xStart);

    // goal-bias는 goal rest 상태를 직접 뽑고(LaValle 1998), 아니면 자유 위치 +
    // 랜덤 속도라 LQR nearest metric이 full-state가 된다. draw 순서: 동전(planner rng)
    // → space.sample(map rng x,y) → vx, vy(planner rng)까지 python과 동일.
    const sample = (): State4 => {
        if (rng.random() < goalBias) return [xGoal[0], xGoal[1], xGoal[2], xGoal[3]];
        const [px, py] = space.sample();
        const vx = rng.uniform(-maxVelocity, maxVelocity);
        const vy = rng.uniform(-maxVelocity, maxVelocity);
        return [px, py, vx, vy];
    };

    // sample 위치 방향으로 최대 step_size 떨어진 rest waypoint (RRT* eta 캡). 속도 0이라
    // LQR 피드백이 정확히 조절되는 rest 평형점이다.
    const restTarget = (xFrom: State4, s2: State4): State4 => {
        const dx = s2[0] - xFrom[0], dy = s2[1] - xFrom[1];
        const dist = Math.hypot(dx, dy);
        if (dist <= stepSize || dist < EPS) return [s2[0], s2[1], 0.0, 0.0];
        const scale = stepSize / dist;
        return [xFrom[0] + dx * scale, xFrom[1] + dy * scale, 0.0, 0.0];
    };

    // LQR 피드백 u=−K(x−target)를 clamp해 x_from에서 target(rest)까지 적분한다.
    // 충돌 없이 도달하면 (edge_cost, dense (x, y) 궤적), 아니면 null. edge_cost는 실현
    // LQR 비용 Σ(xᵀQx+uᵀRu)·dt (Perez et al. 2012).
    const roll = (xFrom: State4, target: State4): {cost: number; traj: Point[]} | null => {
        let px = xFrom[0], py = xFrom[1], vx = xFrom[2], vy = xFrom[3];
        const tx = target[0], ty = target[1], tvx = target[2], tvy = target[3];
        let cost = 0.0;
        const traj: Point[] = [];
        let prev: Point = [px, py];
        for (let step = 0; step < STEER_MAX_STEPS; step++) {
            const exP = px - tx, exV = vx - tvx;
            const eyP = py - ty, eyV = vy - tvy;
            let ux = -(k0 * exP + k1 * exV);
            let uy = -(k0 * eyP + k1 * eyV);
            ux = Math.max(-uMax, Math.min(uMax, ux));
            uy = Math.max(-uMax, Math.min(uMax, uy));
            cost += (
                qPos * (exP * exP + eyP * eyP)
                + qVel * (exV * exV + eyV * eyV)
                + rCtrl * (ux * ux + uy * uy)
            ) * dt;
            px = px + dt * vx + halfDt2 * ux;
            py = py + dt * vy + halfDt2 * uy;
            vx = vx + dt * ux;
            vy = vy + dt * uy;
            const cur: Point = [px, py];
            if (!space.isMotionValid(prev, cur)) return null;
            traj.push(cur);
            prev = cur;
            if (Math.abs(px - tx) <= REACH_POS_TOL
                && Math.abs(py - ty) <= REACH_POS_TOL
                && Math.abs(vx - tvx) <= REACH_VEL_TOL
                && Math.abs(vy - tvy) <= REACH_VEL_TOL) {
                traj[traj.length - 1] = [tx, ty];
                return {cost, traj};
            }
        }
        return null;
    };

    // 위치 유클리드로 K개 후보를 추린 뒤 정확한 LQR cost-to-go 최소값. 후보 집합은
    // 거리 오름차순(동거리는 index 오름차순)으로 결정론화한다.
    const nearest = (target: State4): number => {
        const n = tree.size;
        const kk = Math.min(NEAREST_CANDIDATES, n);
        let candidates: number[];
        if (kk < n) {
            const idx = Array.from({length: n}, (_, i) => i);
            const d = idx.map((i) => Math.hypot(
                tree.states[i][0] - target[0], tree.states[i][1] - target[1]));
            idx.sort((x, y) => d[x] - d[y] || x - y);
            candidates = idx.slice(0, kk);
        } else {
            candidates = Array.from({length: n}, (_, i) => i);
        }
        let bestIdx = -1;
        let bestCost = Infinity;
        for (const i of candidates) {
            const c = lqrCostToGo(tree.states[i], target, s);
            if (c < bestCost) {
                bestCost = c;
                bestIdx = i;
            }
        }
        return bestIdx;
    };

    // 위치 반경 안 노드. MAX_NEIGHBORS 초과면 거리 오름차순으로 잘라내고, 아니면
    // index 오름차순(np.nonzero 순서)을 유지한다 — rewire 비용 전파가 순서 의존이다.
    const neighborhood = (target: State4): number[] => {
        const n = tree.size;
        const within: number[] = [];
        const dist: number[] = [];
        for (let i = 0; i < n; i++) {
            const di = Math.hypot(
                tree.states[i][0] - target[0], tree.states[i][1] - target[1]);
            if (di <= neighborRadius) {
                within.push(i);
                dist.push(di);
            }
        }
        if (within.length > MAX_NEIGHBORS) {
            const order = within.map((_, j) => j);
            order.sort((x, y) => dist[x] - dist[y] || within[x] - within[y]);
            return order.slice(0, MAX_NEIGHBORS).map((j) => within[j]);
        }
        return within;
    };

    // choose-parent (Karaman & Frazzoli 2011): 근방에서 최소 비용의 실행 가능 부모.
    // near_idx의 이미 rolled된 near_edge가 기본값이다.
    const chooseParent = (
        xNew: State4, nearIdx: number, nearEdge: {cost: number; traj: Point[]},
        nbhd: number[],
    ): number => {
        let bestParent = nearIdx;
        let bestEdge = nearEdge.traj;
        let bestTotal = tree.cost[nearIdx] + nearEdge.cost;
        for (const j of nbhd) {
            if (j === nearIdx) continue;
            const r = roll(tree.states[j], xNew);
            if (r === null) continue;
            const total = tree.cost[j] + r.cost;
            emit({event: "candidate_evaluated", state: [xNew[0], xNew[1]], cost: total});
            if (total < bestTotal) {
                bestTotal = total;
                bestParent = j;
                bestEdge = r.traj;
            }
        }
        const edgeCost = bestTotal - tree.cost[bestParent];
        const newIdx = tree.add(xNew, bestParent, edgeCost, bestEdge);
        // 곡선 edge를 chord 사슬로 방출해 궤적이 렌더된다.
        let prev: Point = [tree.states[bestParent][0], tree.states[bestParent][1]];
        for (const pt of bestEdge) {
            emit({event: "edge_added", state: [pt[0], pt[1]],
                  parent: [prev[0], prev[1]]});
            prev = pt;
        }
        return newIdx;
    };

    const rewire = (newIdx: number, nbhd: number[]): void => {
        const xNew = tree.states[newIdx];
        for (const j of nbhd) {
            if (j === tree.parent[newIdx] || j === newIdx) continue;
            const r = roll(xNew, tree.states[j]);
            if (r === null) continue;
            if (tree.cost[newIdx] + r.cost < tree.cost[j]) {
                tree.reparent(j, newIdx, r.cost, r.traj);
                emit({event: "rewire", state: [tree.states[j][0], tree.states[j][1]],
                      parent: [xNew[0], xNew[1]]});
            }
        }
    };

    // goal은 성장/rewire 노드가 아니다 (Karaman & Frazzoli 2011). 최선 부모 +
    // 들어오는 궤적만 추적한다.
    const goalArrival = (
        newIdx: number, xNew: State4,
    ): {cost: number; traj: Point[]} | null => {
        // goal-bias 표본이 x_new를 goal(이미 rest)에 올리면 self-roll이 퇴화한다 —
        // 노드 자체가 도착(빈 궤적)이다.
        if (Math.abs(xNew[0] - xGoal[0]) <= EPS
            && Math.abs(xNew[1] - xGoal[1]) <= EPS
            && Math.abs(xNew[2]) <= EPS
            && Math.abs(xNew[3]) <= EPS) {
            return {cost: tree.cost[newIdx], traj: []};
        }
        const r = roll(xNew, xGoal);
        if (r === null) return null;
        return {cost: tree.cost[newIdx] + r.cost, traj: r.traj};
    };

    let bestGoalParent = -1;
    let bestGoalCost = Infinity;
    let iterations = 0;

    for (let it = 0; it < maxIterations; it++) {
        iterations++;
        const qRand = sample();
        emit({event: "sample_drawn", state: [qRand[0], qRand[1]]});
        const nearIdx = nearest(qRand);
        const xNew = restTarget(tree.states[nearIdx], qRand);
        const nearRoll = roll(tree.states[nearIdx], xNew);
        if (nearRoll === null) continue;
        const nbhd = neighborhood(xNew);
        const newIdx = chooseParent(xNew, nearIdx, nearRoll, nbhd);
        rewire(newIdx, nbhd);

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
    let cost = 0.0;
    if (success) {
        cost = bestGoalCost;
    }
    emit({
        event: "planning_finished",
        success,
        metrics: {path_cost: cost, expanded_nodes: tree.size - 1,
                  samples: iterations, tree_size: tree.size, iterations},
    });
    return events;
}

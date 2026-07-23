import {GridMap} from "../grid";
import {Point} from "./sampling_space";
import {nearestOccupied} from "./obstacle_grid";
import {integrateUnicycle, Pose} from "./local_sim";
import {NumpyRandom} from "./numpy_rng";

// predictive 패밀리(MPC, MPPI) 공용 rollout + 비용. 두 planner는 완전히 같은 유한
// horizon 최적제어 문제 J(U)를 풀고 결정변수도 제어열 U=[(v,ω)...]로 동일하다 --
// 다른 것은 옵티마이저뿐(MPC=유한차분 경사하강, MPPI=Gauss 표본 softmax). 그 대비가
// 유일한 차이가 되도록 원호 rollout과 스칼라 비용을 여기 free 함수로 모아 두 엔진이
// 그대로 호출한다. 저장소 local_planning/predictive/_rollout.py를 연산 순서까지 미러한다.

// 장애물 페널티 활성 거리 너머로 더 질의하는 여유. hinge 항은 clearance c_k <
// min_obstacle_dist인 곳(최근접 occupied가 min_obstacle_dist + footprint_radius 안)에서만
// nonzero라, 반 셀 더 질의해야 유한차분 gradient가 장애물이 활성 대역에 막 들어오는
// 순간을 본다 (py _rollout._QUERY_MARGIN 미러).
const QUERY_MARGIN = 0.5

// box 투영/가속 clamp 공용 헬퍼 (py _rollout.clamp 미러).
export function clamp(value: number, lo: number, hi: number): number {
    return Math.max(lo, Math.min(hi, value))
}

// s0에서 각 제어를 순서대로 적용한 상태열 s_1 .. s_H (s0는 포함하지 않는다 -- 호출자가
// band 방출 시 직접 앞에 붙인다). integrateUnicycle은 시뮬레이터/DWA/TEB와 같은 closed-form
// 원호라 예측과 실행 이산화가 어긋나지 않는다 (py _rollout.rollout 미러).
export function rolloutControls(s0: Pose, controls: Array<[number, number]>, h: number): Pose[] {
    const states: Pose[] = []
    let s = s0
    for (const [v, omega] of controls) {
        s = integrateUnicycle(s, {v, omega}, h)
        states.push(s)
    }
    return states
}

// MPC와 MPPI가 공유하는 receding-horizon 비용 J(U):
//   J = Σ_{k=1..H} [ w_goal·‖p_k − g‖² + w_obstacle·max(0, d_min − c_k)² + w_control·(v² + ω²) ]
// c_k는 p_k에서 최근접 occupied cell까지의 연속 거리 − footprint_radius. 장애물 항은
// 셀 양자화 distance_to_nearest(셀 내부 상수라 유한차분 gradient가 0)가 아니라 연속
// nearestOccupied 거리를 쓴다 -- TEB gradient 솔버가 같은 연속 clearance를 쓰는 이유와
// 같다. 명시 스칼라 누적(numpy 벡터화 없음)으로 py/cpp/TS fold 순서를 일치시킨다
// (py _rollout.sequence_cost 미러).
export function sequenceCost(
    map: GridMap,
    traj: Pose[],
    controls: Array<[number, number]>,
    goal: [number, number],
    footprintRadius: number,
    wGoal: number,
    wObstacle: number,
    minObstacleDist: number,
    wControl: number,
): number {
    const gx = goal[0], gy = goal[1]
    const rQuery = minObstacleDist + footprintRadius + QUERY_MARGIN
    let total = 0.0
    for (let k = 0; k < traj.length; k++) {
        const [x, y] = traj[k]
        const dx = x - gx
        const dy = y - gy
        total += wGoal * (dx * dx + dy * dy)
        const [, dTilde] = nearestOccupied(map, [x, y] as Point, rQuery)
        if (dTilde !== Infinity) {
            const cK = dTilde - footprintRadius
            const hinge = minObstacleDist - cK
            if (hinge > 0.0) {
                total += wObstacle * hinge * hinge
            }
        }
        const [v, omega] = controls[k]
        total += wControl * (v * v + omega * omega)
    }
    return total
}

// PCG64 uniform 스트림 위 Box-Muller 한 쌍은 표준정규 둘을 내는데, MPPI는 (v, ω) 한
// 쌍당 정확히 한 Box-Muller 쌍을 소비한다 -- 첫 정규는 지금 반환하고 둘째는 rng별로
// 캐싱했다가 다음 gaussian 호출이 반환한다. 이 소비 순서가 py↔TS 재현성 계약이라, py
// MppiPlanner의 _spare 캐시(np.random.default_rng)를 그대로 미러하려고 rng 객체를 키로
// 스페어를 둔다. runMppi는 실행마다 새 NumpyRandom을 만들므로 실행 간 스페어가 섞이지
// 않는다 (py _gaussian 미러).
const spareByRng = new WeakMap<NumpyRandom, number>()

// 1.0 − rng.random()으로 (0, 1] 매핑해 log(0)를 피한다 (py와 동일).
export function gaussian(rng: NumpyRandom): number {
    const spare = spareByRng.get(rng)
    if (spare !== undefined) {
        spareByRng.delete(rng)
        return spare
    }
    const u1 = 1.0 - rng.random()
    const u2 = rng.random()
    const magnitude = Math.sqrt(-2.0 * Math.log(u1))
    spareByRng.set(rng, magnitude * Math.sin(2.0 * Math.PI * u2))
    return magnitude * Math.cos(2.0 * Math.PI * u2)
}

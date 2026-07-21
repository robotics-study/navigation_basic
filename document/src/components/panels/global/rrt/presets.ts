import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// RRT 전용: bug trap. 시작이 오른쪽이 막힌 C자 함정 안에 있어, 트리가 입구를
// 찾아 왼쪽으로 빠져나온 뒤 goal로 돌아가야 한다. goal 쪽 직진은 벽에 막히므로
// 탐색(Voronoi bias)이 일하는 모습이 잘 보인다 (8 seed 실측: bias 0 → 평균
// 2100 반복, bias 0.05 → 850 반복).
export const TRAP_N = 20;
export const TRAP_START: Point = [8.5, 10.0];
export const TRAP_GOAL: Point = [17.5, 10.0];

export function bugTrapMap(): GridMap {
    const map = emptyGrid("bug_trap", TRAP_N, TRAP_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * TRAP_N + c] = true
        }
    }
    fill(6, 6, 5, 12)     // 위 벽
    fill(13, 13, 5, 12)   // 아래 벽
    fill(6, 13, 12, 12)   // 오른 벽 — goal 쪽을 막는다
    return map
}

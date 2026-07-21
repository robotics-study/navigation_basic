import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// RRT-Connect 전용: S자 복도. 시작→goal 직선이 두 번 꺾이는 긴 뱀길이라 한쪽
// 트리는 전체를 걸어야 하지만, 양쪽에서 자라면 가운데에서 만난다 (6 seed 실측:
// RRT-Connect 평균 528 반복/285 노드 vs 단일 트리 RRT 982 반복/601 노드).
export const SCURVE_N = 20;
export const SCURVE_START: Point = [2.5, 17.5];
export const SCURVE_GOAL: Point = [17.5, 2.5];

export function sCurveMap(): GridMap {
    const map = emptyGrid("s_corridor", SCURVE_N, SCURVE_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * SCURVE_N + c] = true
        }
    }
    fill(5, 6, 0, 13)     // 위 벽 (왼쪽 벽에서 뻗음)
    fill(12, 13, 6, 19)   // 아래 벽 (오른쪽 벽에서 뻗음)
    return map
}

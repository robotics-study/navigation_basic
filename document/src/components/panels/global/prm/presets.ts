import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// PRM 전용: 좁은 문. 두 방을 잇는 유일한 통로가 얇은 벽의 4칸 문이라, 표본이
// 적으면 양쪽 문가에 동시에 표본이 떨어질 확률이 낮아 roadmap이 끊긴다.
// 표본 수를 늘리면 연결된다 — 확률적 완전성이 눈으로 보이는 배치다.
// (10 seed 실측: n=80 → 0/10, n=160 → 8/10, n=300 → 10/10, r=2.5)
export const GATE_N = 20;
export const GATE_START: Point = [3.5, 3.5];
export const GATE_GOAL: Point = [16.5, 16.5];

export function gateMap(): GridMap {
    const map = emptyGrid("narrow_gate", GATE_N, GATE_N)
    for (let r = 0; r < GATE_N; r++) {
        if (r >= 8 && r <= 11) continue    // 문
        map.occupied[r * GATE_N + 9] = true
    }
    return map
}

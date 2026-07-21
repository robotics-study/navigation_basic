import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Point} from "../../../../libs/algorithms/sampling_space";

// FMT* 전용: 벽 두 개가 통로를 어긋나게 낸 S자 지형. 어느 표본 수에서도 양쪽이
// 연결되므로 관심사는 연결 여부가 아니라 트리를 세우는 데 든 일(간선/충돌 검사)
// 이다. 같은 표본 위에서 FMT*와 PRM*를 돌린 seed 1 실측(engine):
//   n=150  FMT* 간선 143 · 비용 31.77   vs  PRM* 간선 1340 · 비용 31.77
//   n=300  FMT* 간선 280 · 비용 30.95   vs  PRM* 간선 3463 · 비용 30.95
//   n=600  FMT* 간선 548 · 비용 30.39   vs  PRM* 간선 8499 · 비용 30.39
// 비용은 자릿수까지 같은데 FMT*가 세우는 간선은 12~15배 적다. lazy 충돌 검사가
// 후보마다 근방 최소비용 간선 하나만 검사하기 때문이다 (Janson et al. 2015).
export const FMT_N = 24;
export const FMT_START: Point = [1.5, 1.5];
export const FMT_GOAL: Point = [22.5, 22.5];

export function zigMap(): GridMap {
    const map = emptyGrid("fmt_zig", FMT_N, FMT_N)
    const fill = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) {
            for (let c = c0; c <= c1; c++) map.occupied[r * FMT_N + c] = true
        }
    }
    // 위에서 내려오는 벽(아래에 gap)과 아래에서 올라오는 벽(위에 gap)이 어긋나
    // 붙어 경로를 S자로 강제한다.
    fill(0, 15, 8, 9)
    fill(8, 23, 15, 16)
    return map
}

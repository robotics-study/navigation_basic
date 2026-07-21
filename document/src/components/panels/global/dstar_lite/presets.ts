import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";
import {mulberry32} from "../../../../libs/rng";

// maps/grid/dstar_trap01 과 같은 배치 — 목표 쪽으로 열린 C자 함정. 로봇은 지도를
// 모른 채 함정 안으로 들어갔다가, 벽을 발견하며 국소 수리로 빠져나온다.
export const TRAP_N = 17;
export const TRAP_START: Cell = [15, 1];
export const TRAP_GOAL: Cell = [1, 15];

export function trapMap(): GridMap {
    const map = emptyGrid("dstar_trap", TRAP_N, TRAP_N)
    const set = (row: number, col: number) => {
        map.occupied[row * TRAP_N + col] = true
    }
    for (let c = 5; c <= 12; c++) set(4, c)
    for (let r = 5; r <= 11; r++) set(r, 12)
    for (let c = 5; c <= 12; c++) set(12, c)
    return map
}

// 흩어진 소형 장애물 — 발견 하나가 국소 수리로 끝나는 일반적인 상황. 이런 지형에서
// incremental 수리가 매번 A* 재실행을 크게 이긴다 (trap은 반대의 최악 케이스).
export const SCATTER_N = 24;
export const SCATTER_START: Cell = [21, 2];
export const SCATTER_GOAL: Cell = [2, 21];

export function scatterMap(): GridMap {
    const map = emptyGrid("dstar_scatter", SCATTER_N, SCATTER_N)
    const rand = mulberry32(23)
    let placed = 0
    let guard = 0
    while (placed < 26 && guard++ < 500) {
        const r = 1 + Math.floor(rand() * (SCATTER_N - 2))
        const c = 1 + Math.floor(rand() * (SCATTER_N - 2))
        // 시작/목표 주변은 비워 둔다.
        if (Math.hypot(r - SCATTER_START[0], c - SCATTER_START[1]) < 4) continue
        if (Math.hypot(r - SCATTER_GOAL[0], c - SCATTER_GOAL[1]) < 4) continue
        if (map.occupied[r * SCATTER_N + c]) continue
        map.occupied[r * SCATTER_N + c] = true
        // 절반은 두 칸짜리 (이웃 하나 추가)
        if (rand() < 0.5) {
            const c2 = Math.min(SCATTER_N - 2, c + 1)
            map.occupied[r * SCATTER_N + c2] = true
        }
        placed++
    }
    return map
}

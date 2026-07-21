import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";
import {mulberry32} from "../../../../libs/rng";

// Theta* 전용: 넓은 마당에 기둥 몇 개. grid 경로는 45°에 갇혀 지그재그가 되고,
// any-angle 은 기둥 모서리만 스치는 긴 직선을 긋는다 — 두 경로의 차이가 가장 잘 보이는 지형.
export const PILLAR_N = 22;
export const PILLAR_START: Cell = [19, 2];
export const PILLAR_GOAL: Cell = [2, 19];

export function pillarsMap(): GridMap {
    const map = emptyGrid("pillars", PILLAR_N, PILLAR_N)
    const block = (r0: number, c0: number, h: number, w: number) => {
        for (let r = r0; r < r0 + h; r++) {
            for (let c = c0; c < c0 + w; c++) map.occupied[r * PILLAR_N + c] = true
        }
    }
    block(6, 5, 3, 3)
    block(13, 10, 3, 3)
    block(5, 14, 3, 3)
    return map
}

// Lazy Theta* 전용: 잘게 흩어진 잔해. LOS 검사 대상 edge 가 많아져 "edge 마다 검사"와
// "확장마다 한 번"의 횟수 차이가 크게 벌어진다.
export const RUBBLE_N = 22;
export const RUBBLE_START: Cell = [19, 2];
export const RUBBLE_GOAL: Cell = [2, 19];

export function rubbleMap(): GridMap {
    const map = emptyGrid("rubble", RUBBLE_N, RUBBLE_N)
    const rand = mulberry32(7)
    let placed = 0
    let guard = 0
    while (placed < 34 && guard++ < 600) {
        const r = 1 + Math.floor(rand() * (RUBBLE_N - 2))
        const c = 1 + Math.floor(rand() * (RUBBLE_N - 2))
        if (Math.hypot(r - RUBBLE_START[0], c - RUBBLE_START[1]) < 3.5) continue
        if (Math.hypot(r - RUBBLE_GOAL[0], c - RUBBLE_GOAL[1]) < 3.5) continue
        if (map.occupied[r * RUBBLE_N + c]) continue
        map.occupied[r * RUBBLE_N + c] = true
        placed++
    }
    return map
}

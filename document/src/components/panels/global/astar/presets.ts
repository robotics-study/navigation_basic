import {emptyGrid, GridMap} from "../../../../libs/grid";
import {Cell} from "../../../../libs/trace/timeline";

export const SANDBOX_N = 20;
export const SANDBOX_START: Cell = [SANDBOX_N - 2, 1];
export const SANDBOX_GOAL: Cell = [1, SANDBOX_N - 2];

// sandbox/히어로 기본 벽: 시작→목표 대각선을 막아서는 ⌐ 모양 pocket.
// 입구가 시작점 쪽으로 열려 있어 과대평가 heuristic(greedy)은 안으로 뛰어들었다가
// 더 긴 경로를 내놓는다 — heuristic 별 차이가 한눈에 보이는 배치다.
export function pocketMap(): GridMap {
    const map = emptyGrid("sandbox", SANDBOX_N, SANDBOX_N)
    const set = (row: number, col: number) => {
        if (row >= 0 && row < SANDBOX_N && col >= 0 && col < SANDBOX_N)
            map.occupied[row * SANDBOX_N + col] = true
    }
    for (let c = 5; c <= 14; c++) set(6, c)
    for (let r = 6; r <= 14; r++) set(r, 14)
    for (let r = 10; r <= 18; r++) set(r, 7)
    return map
}

// maps/grid/bfs_hopcost01과 같은 배치 — 벽 두 칸이 hop-최단 경로를 대각선 지그재그로
// 밀어 넣어, BFS(최소 hop)와 Dijkstra(최소 비용)의 경로 비용이 갈라진다.
export const HOPCOST_START: Cell = [0, 4];
export const HOPCOST_GOAL: Cell = [11, 8];

export function hopcostMap(): GridMap {
    const map = emptyGrid("hopcost", 12, 13)
    map.occupied[1 * 12 + 4] = true
    map.occupied[3 * 12 + 3] = true
    return map
}

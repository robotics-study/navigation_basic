import {cellCenterWorld, GridMap, isOccupied, worldToCellIndex} from "../grid";
import {Point} from "./sampling_space";

// 저장소 OccupancyGrid2D의 ObstacleQuery capability 미러 (local planner용 장애물 질의:
// 최근접 clearance + 근방 점유 열거). world<->cell 변환은 grid.ts의 공용 헬퍼만 쓴다
// (worldToCellIndex/cellCenterWorld) — 새 좌표 변환을 만들지 않는다.

// EDT의 "무한대" sentinel — 실제 inf를 쓰면 lower-envelope 교차식이 0/0=NaN을 낼 수
// 있어, 그보다 훨씬 크되 유한한 값을 쓴다 (저장소 occupancy_grid.py 미러).
const EDT_INF = 1e15;

// 1D exact squared-distance lower envelope (Felzenszwalb & Huttenlocher 2004,
// "Distance Transforms of Sampled Functions", §2). f[q]는 source에서 0, 그 외는
// EDT_INF. 결과 d[q] = min_{q'} (q-q')^2 + f[q'].
function dt1d(f: Float64Array): Float64Array {
    const n = f.length
    const d = new Float64Array(n)
    const v = new Int32Array(n)          // envelope에 남는 포물선들의 q-index
    const z = new Float64Array(n + 1)    // 포물선 사이 envelope 경계
    let k = 0
    v[0] = 0
    z[0] = -EDT_INF
    z[1] = EDT_INF
    for (let q = 1; q < n; q++) {
        let s: number
        for (;;) {
            s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0 * q - 2.0 * v[k])
            if (s <= z[k]) {
                k--
            } else {
                break
            }
        }
        k++
        v[k] = q
        z[k] = s
        z[k + 1] = EDT_INF
    }
    k = 0
    for (let q = 0; q < n; q++) {
        while (z[k + 1] < q) k++
        const dq = q - v[k]
        d[q] = dq * dq + f[v[k]]
    }
    return d
}

// 2-pass exact squared EDT(Felzenszwalb & Huttenlocher 2004): 열 방향 1D 변환 후
// 행 방향 1D 변환. cell 단위 제곱거리(정수)까지는 결정론적이고, 호출부의 sqrt 한 번만
// 부동소수 — python/C++ 미러와 그 한 스텝까지만 일치하면 된다.
function squaredEdt(nonFree: Uint8Array, h: number, w: number): Float64Array {
    const f = new Float64Array(h * w)
    for (let i = 0; i < h * w; i++) f[i] = nonFree[i] ? 0.0 : EDT_INF
    const partial = new Float64Array(h * w)
    const col = new Float64Array(h)
    for (let c = 0; c < w; c++) {
        for (let r = 0; r < h; r++) col[r] = f[r * w + c]
        const dCol = dt1d(col)
        for (let r = 0; r < h; r++) partial[r * w + c] = dCol[r]
    }
    const out = new Float64Array(h * w)
    const row = new Float64Array(w)
    for (let r = 0; r < h; r++) {
        for (let c = 0; c < w; c++) row[c] = partial[r * w + c]
        const dRow = dt1d(row)
        for (let c = 0; c < w; c++) out[r * w + c] = dRow[c]
    }
    return out
}

// map 객체별 lazy EDT 캐시(cell 단위 제곱거리) — python의 인스턴스별 `_edt_sq` 캐시 미러.
// sandbox는 페인팅 시 새 map 객체를 만들므로(불변 갱신) WeakMap 키가 자동으로 새로 잡힌다.
const edtCache = new WeakMap<GridMap, Float64Array>()

function edtSquared(map: GridMap): Float64Array {
    let cached = edtCache.get(map)
    if (cached) return cached
    const h = map.height
    const w = map.width
    // 1셀 non-free 테두리 패딩 — 맵 경계 자체가 source가 되어, 경계 인접 로봇도
    // 장애물을 보는 것과 같이 처리된다(occupied OR out-of-bounds = non-free 규약과 일관).
    const ph = h + 2
    const pw = w + 2
    const padded = new Uint8Array(ph * pw).fill(1)
    for (let r = 0; r < h; r++) {
        for (let c = 0; c < w; c++) {
            padded[(r + 1) * pw + (c + 1)] = map.occupied[r * w + c] ? 1 : 0
        }
    }
    const paddedSq = squaredEdt(padded, ph, pw)
    const out = new Float64Array(h * w)
    for (let r = 0; r < h; r++) {
        for (let c = 0; c < w; c++) out[r * w + c] = paddedSq[(r + 1) * pw + (c + 1)]
    }
    edtCache.set(map, out)
    return out
}

// p에서 가장 가까운 비자유(점유/경계 밖) 셀 중심까지 euclidean 거리 (m).
export function distanceToNearest(map: GridMap, p: Point): number {
    const [row, col] = worldToCellIndex(map, p[0], p[1])
    if (row < 0 || row >= map.height || col < 0 || col >= map.width) {
        // p 소속 셀 자체가 비자유(경계 밖)이므로 스스로가 최근접 비자유 셀이다.
        return 0.0
    }
    const sq = edtSquared(map)[row * map.width + col]
    return Math.sqrt(sq) * map.resolution
}

// center 반경 radius 내 비자유 셀 중심(world). 경계 밖 셀 포함. row asc, col asc 순 —
// 이 열거 순서가 PF 힘 합산·VFH bin 누적의 부동소수 합 순서를 결정하므로 python과
// 정확히 같아야 parity가 성립한다(occupancy_grid.py occupied_within 미러).
export function occupiedWithin(map: GridMap, center: Point, radius: number): Point[] {
    const [x, y] = center
    const r2 = radius * radius
    const [loRow, loCol] = worldToCellIndex(map, x - radius, y + radius)  // y+r → 작은 row
    const [hiRow, hiCol] = worldToCellIndex(map, x + radius, y - radius)
    const out: Point[] = []
    for (let row = loRow; row <= hiRow; row++) {
        for (let col = loCol; col <= hiCol; col++) {
            if (!isOccupied(map, row, col)) continue   // 자유 & 경계 안 → source 아님
            const [cx, cy] = cellCenterWorld(map, [row, col])
            const dx = cx - x
            const dy = cy - y
            if (dx * dx + dy * dy <= r2) out.push([cx, cy])
        }
    }
    return out
}

// radius 내 최근접 occupied cell 중심과 그 연속 거리 — occupiedWithin의 row/col 오름차순
// 리스트에서 strict '<'로 첫 동률을 유지한다(3언어 결정론). radius 내 occupied가 없으면
// [null, Infinity]. 셀 양자화 계단인 distanceToNearest와 달리 질의점 p에 대해 연속이라,
// 유한차분/gradient 옵티마이저(teb·predictive)가 셀 내부에서도 유효한 clearance를 얻는다.
// band 패밀리(teb) 전용이던 것을 predictive가 두 번째 소비자로 필요로 해, 격자 질의
// primitive occupiedWithin/distanceToNearest 옆으로 승격했다(py local_planning/_geometry
// nearest_occupied 미러).
export function nearestOccupied(map: GridMap, p: Point, radius: number): [Point | null, number] {
    let best: Point | null = null
    let bestSq = Infinity
    for (const o of occupiedWithin(map, p, radius)) {
        const dx = p[0] - o[0]
        const dy = p[1] - o[1]
        const d = dx * dx + dy * dy
        if (d < bestSq) {
            bestSq = d
            best = o
        }
    }
    if (best === null) return [null, Infinity]
    return [best, Math.sqrt(bestSq)]
}

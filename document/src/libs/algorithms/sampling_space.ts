import {GridMap} from "../grid";
import {NumpyRandom} from "./numpy_rng";

// 저장소 OccupancyGrid2D의 SamplingSpace capability 미러. sampling 계열 라이브
// 엔진들이 공유한다. 표본 좌표는 world (x, y)이고, python demo와 같은 seed 면
// 같은 표본열이 나온다 (NumpyRandom이 numpy PCG64를 그대로 재현).
export type Point = [number, number];

// 내접원 footprint의 disc-셀 겹침 충돌 — 저장소 OccupancyGrid2D.is_collision 미러.
// theta 불변(disc)이라 (x, y)만 본다. SE(2) planner(Hybrid A*, SST)가 공유한다.
export function discCollides(map: GridMap, radius: number, x: number, y: number): boolean {
    const res = map.resolution
    const half = res * 0.5
    const worldToCell = (wx: number, wy: number): [number, number] => [
        map.height - 1 - Math.floor((wy - map.originY) / res),
        Math.floor((wx - map.originX) / res),
    ]
    const freeCell = (r: number, c: number) =>
        r >= 0 && r < map.height && c >= 0 && c < map.width && !map.occupied[r * map.width + c]
    const [loRow, loCol] = worldToCell(x - radius, y + radius)   // y+r → 작은 row
    const [hiRow, hiCol] = worldToCell(x + radius, y - radius)
    const r2 = radius * radius
    for (let row = loRow; row <= hiRow; row++) {
        for (let col = loCol; col <= hiCol; col++) {
            if (freeCell(row, col)) continue
            const cx = map.originX + (col + 0.5) * res
            const cy = map.originY + (map.height - 1 - row + 0.5) * res
            const dx = x - Math.min(Math.max(x, cx - half), cx + half)
            const dy = y - Math.min(Math.max(y, cy - half), cy + half)
            if (dx * dx + dy * dy <= r2) return true
        }
    }
    return false
}

export class SamplingGrid {
    readonly map: GridMap;
    private readonly rng: NumpyRandom;

    constructor(map: GridMap, seed: number) {
        this.map = map
        this.rng = new NumpyRandom(seed)
    }

    sample(): Point {
        const m = this.map
        const x = m.originX + this.rng.uniform(0, m.width * m.resolution)
        const y = m.originY + this.rng.uniform(0, m.height * m.resolution)
        return [x, y]
    }

    // goal-bias 등 planner 자체 draw 용 (map RNG와 같은 스트림을 공유하지 않는
    // planner는 별도 NumpyRandom을 만든다).
    static plannerRng(seed: number): NumpyRandom {
        return new NumpyRandom(seed)
    }

    private freeCell(row: number, col: number): boolean {
        const m = this.map
        return row >= 0 && row < m.height && col >= 0 && col < m.width
            && !m.occupied[row * m.width + col]
    }

    worldToCell(x: number, y: number): [number, number] {
        const m = this.map
        const col = Math.floor((x - m.originX) / m.resolution)
        const row = (m.height - 1) - Math.floor((y - m.originY) / m.resolution)
        return [row, col]
    }

    isStateValid(s: Point): boolean {
        const [row, col] = this.worldToCell(s[0], s[1])
        return this.freeCell(row, col)
    }

    // (u, v)는 origin 기준 아래에서 위로 세는 grid 좌표. row는 위에서 아래다.
    private isFreeUV(iu: number, iv: number): boolean {
        return this.freeCell(this.map.height - 1 - iv, iu)
    }

    // supercover 통과 검사 (Amanatides & Woo 1987) — 저장소 is_motion_valid 미러.
    isMotionValid(a: Point, b: Point): boolean {
        const m = this.map
        const u0 = (a[0] - m.originX) / m.resolution
        const v0 = (a[1] - m.originY) / m.resolution
        const u1 = (b[0] - m.originX) / m.resolution
        const v1 = (b[1] - m.originY) / m.resolution
        let iu = Math.floor(u0)
        let iv = Math.floor(v0)
        const ju = Math.floor(u1)
        const jv = Math.floor(v1)
        if (!this.isFreeUV(iu, iv)) return false
        const du = u1 - u0
        const dv = v1 - v0
        const stepU = du > 0 ? 1 : -1
        const stepV = dv > 0 ? 1 : -1
        const tDeltaU = du !== 0 ? Math.abs(1 / du) : Infinity
        const tDeltaV = dv !== 0 ? Math.abs(1 / dv) : Infinity
        let tMaxU = du !== 0
            ? (du > 0 ? Math.floor(u0) + 1 - u0 : u0 - Math.floor(u0)) * tDeltaU
            : Infinity
        let tMaxV = dv !== 0
            ? (dv > 0 ? Math.floor(v0) + 1 - v0 : v0 - Math.floor(v0)) * tDeltaV
            : Infinity
        while (iu !== ju || iv !== jv) {
            if (iv === jv || tMaxU < tMaxV) {
                iu += stepU
                tMaxU += tDeltaU
            } else if (iu === ju || tMaxV < tMaxU) {
                iv += stepV
                tMaxV += tDeltaV
            } else {
                // 정확한 corner 교차: 양쪽 직교 셀이 모두 free 여야 통과.
                if (!(this.isFreeUV(iu + stepU, iv) && this.isFreeUV(iu, iv + stepV))) return false
                iu += stepU
                iv += stepV
                tMaxU += tDeltaU
                tMaxV += tDeltaV
            }
            if (!this.isFreeUV(iu, iv)) return false
        }
        return true
    }

    distance(a: Point, b: Point): number {
        return Math.hypot(b[0] - a[0], b[1] - a[1])
    }

    steer(a: Point, b: Point, eta: number): Point {
        const dist = this.distance(a, b)
        if (dist <= eta || dist === 0) return b
        const scale = eta / dist
        return [a[0] + (b[0] - a[0]) * scale, a[1] + (b[1] - a[1]) * scale]
    }
}

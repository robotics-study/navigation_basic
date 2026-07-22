import {GridMap} from "../grid";
import {TraceEvent} from "../trace/types";
import {occupiedWithin} from "./obstacle_grid";
import {EmitFn, Pose, RobotState3, VelocityCommand, runClosedLoop, wrapToPi} from "./local_sim";

// Vector Field Histogram (Borenstein & Koren 1991) 브라우저 라이브 엔진. 저장소
// reactive/vfh.py를 그대로 미러한다 — valley 선택의 폭 tie-break, target-inside-valley
// 진입 판정(margin), wide/narrow 조향 규칙까지 포함해 python과 동일한 결정을 낸다
// (parity 전제 — tick 순서·occupied_within 열거 순서는 local_sim/obstacle_grid가 이미
// python과 일치하게 만들어 두었다).

// (startSector, endSector, width): 스무딩된 히스토그램이 threshold 미만인 극대 원형
// 연속 구간. endSector는 startSector에서 +1(mod n)을 width번 걸어 도달하므로,
// 구간이 sector n-1에서 0으로 넘어갈 수 있어 start<=end가 보장되지 않는다.
type Valley = [start: number, end: number, width: number];

export interface VfhOptions {
    map: GridMap;
    start: Pose;
    goal: [number, number];
    numSectors: number;
    windowRadius: number;
    threshold: number;
    smoothingWindow: number;
    wideValleySectors: number;
    hM: number;
    kOmega: number;
    maxSpeed: number;
    maxOmega: number;
    controlDt: number;
    maxSteps: number;
    goalTolerance: number;
    footprintRadius: number;
    stallWindow: number;
    stallDistance: number;
}

// VFH는 sector를 [0, 2pi) 위에 배치한다 — PF/PP가 공유하는 (-pi,pi] 범위의
// wrapToPi와 별도로 이 범위가 필요하다(python _wrap_2pi 미러, reactive 계열 중
// VFH만 쓰는 로컬 유틸이라 local_sim에 얹지 않는다).
function wrap2pi(angle: number): number {
    let wrapped = angle % (2 * Math.PI)
    if (wrapped < 0) wrapped += 2 * Math.PI
    return wrapped
}

// python `%`(항상 [0,n) 반환)과 일치시키는 정수 모듈러 — JS `%`는 음수 피연산자에서
// 음수를 낼 수 있어 valley 폭·circular distance 계산에 그대로 쓰면 어긋난다.
const mod = (a: number, n: number): number => ((a % n) + n) % n

// heading-command 법칙(저장소 reactive/_steering.py 미러 — PF와 공유하는 3줄짜리
// 순수 함수라 별도 공용 모듈 없이 여기 인라인한다). omega는 게인 클램프, v는
// cos(theta_err) 게이트로 목표가 뒤에 있으면 제자리 회전한다.
function headingCommand(thetaErr: number, gain: number, maxSpeed: number, maxOmega: number): VelocityCommand {
    const omega = Math.max(-maxOmega, Math.min(maxOmega, gain * thetaErr))
    const v = maxSpeed * Math.max(0, Math.cos(thetaErr))
    return {v, omega}
}

export function runVfh(opts: VfhOptions): TraceEvent[] {
    const events: TraceEvent[] = []
    let seq = 0
    const emit: EmitFn = (ev) => { events.push({seq: seq++, ...ev}) }
    emit({
        event: "planning_started",
        algorithm: "vfh",
        params: {
            num_sectors: opts.numSectors, window_radius: opts.windowRadius, threshold: opts.threshold,
            smoothing_window: opts.smoothingWindow, wide_valley_sectors: opts.wideValleySectors,
            h_m: opts.hM, k_omega: opts.kOmega, max_speed: opts.maxSpeed, max_omega: opts.maxOmega,
        },
    })

    const n = opts.numSectors
    const delta = (2 * Math.PI) / n
    const half = Math.floor(opts.smoothingWindow / 2)

    const sectorOf = (angle: number): number => Math.min(Math.floor(wrap2pi(angle) / delta), n - 1)
    // `index`는 미리 mod n 하지 않아도 된다 — wrapToPi가 2pi 합동인 각을 전부 같은
    // (-pi,pi] 값으로 정규화한다.
    const sectorCenter = (index: number): number => wrapToPi((index + 0.5) * delta)

    const buildHistogram = (x: number, y: number): number[] => {
        const h = new Array<number>(n).fill(0)
        for (const [ox, oy] of occupiedWithin(opts.map, [x, y], opts.windowRadius)) {
            const beta = Math.atan2(oy - y, ox - x)
            const k = sectorOf(beta)
            const d = Math.hypot(ox - x, oy - y)
            // c^2*(a - b*d), a=1, b=1/window_radius, c=1 (Borenstein & Koren 1991 eq. 5의
            // known/static map 특수형으로 정규화) — d=0이면 m=1, d=window_radius면 m=0,
            // 가까운 장애물일수록 표가 커진다.
            const m = (1 - d / opts.windowRadius) ** 2
            h[k] += m
        }
        return h
    }

    const smooth = (h: number[]): number[] => {
        const out = new Array<number>(n).fill(0)
        for (let k = 0; k < n; k++) {
            let total = 0
            for (let j = -half; j <= half; j++) total += h[mod(k + j, n)]
            out[k] = total / opts.smoothingWindow
        }
        return out
    }

    const findValleys = (below: boolean[]): Valley[] => {
        if (below.every((b) => b)) return [[0, n - 1, n]]
        if (!below.some((b) => b)) return []
        // known-false sector 바로 다음부터 원형 스캔을 시작해, 선형 패스 하나가
        // n-1 -> 0으로 넘어가는 run을 특수 케이스로 다루지 않게 한다.
        const cut = below.findIndex((b) => !b)
        const order = Array.from({length: n}, (_, i) => (cut + 1 + i) % n)
        const valleys: Valley[] = []
        let runStart: number | null = null
        let runEnd = -1
        for (const idx of order) {
            if (below[idx]) {
                if (runStart === null) runStart = idx
                runEnd = idx
            } else if (runStart !== null) {
                const width = mod(runEnd - runStart, n) + 1
                valleys.push([runStart, runEnd, width])
                runStart = null
            }
        }
        return valleys
    }

    const valleyContains = (valley: Valley, k: number): boolean => mod(k - valley[0], n) < valley[2]
    const circularDist = (a: number, b: number): number => Math.min(mod(a - b, n), mod(b - a, n))

    const nearestSectorGap = (valley: Valley, kTarget: number): number => {
        if (valleyContains(valley, kTarget)) return 0
        return Math.min(circularDist(valley[0], kTarget), circularDist(valley[1], kTarget))
    }

    const candidateDirection = (valley: Valley, kTarget: number, goalDir: number): number => {
        const [start, end, width] = valley
        // wide-valley 임계값의 절반이 target bearing을 "goal 직진 신뢰"로 받아들이기
        // 위한 최소 border standoff도 겸한다 — 막힌 이웃 바로 옆 target sector는
        // 로봇의 다음 위치 갱신 한 번으로 valley 밖으로 밀려날 수 있어(장애물 경계는
        // 로봇의 이산화와 함께 움직이지 않는다), valley 밖 target과 같은
        // border-hugging 취급을 받는다.
        const margin = Math.min(Math.floor(opts.wideValleySectors / 2), Math.floor((width - 1) / 2))
        if (valleyContains(valley, kTarget)
            && Math.min(circularDist(start, kTarget), circularDist(end, kTarget)) >= margin) {
            return goalDir
        }
        if (width >= opts.wideValleySectors) {
            // Wide (Borenstein & Koren 1991 §IV): 폭 전체의 중심이 아니라, goal
            // bearing에 가까운 border에서 wide-valley 임계값의 절반만큼 안쪽을
            // 조향점으로 잡아 opening 쪽으로 붙는다.
            const offset = Math.floor(opts.wideValleySectors / 2)
            const idx = circularDist(start, kTarget) <= circularDist(end, kTarget) ? start + offset : end - offset
            return sectorCenter(idx)
        }
        // Narrow: valley 자체의 중심을 조향 방향으로 삼는다.
        return sectorCenter(start + (width - 1) / 2)
    }

    const tick = (state: RobotState3, _dt: number, emitEv: EmitFn): VelocityCommand => {
        const [x, y, theta] = state.pose
        const h = buildHistogram(x, y)
        const smoothed = smooth(h)
        const below = smoothed.map((v) => v < opts.threshold)
        let valleys = findValleys(below)
        // 스무딩 커널보다 좁은 run은 히스토그램 자체 해상도보다 미세해(이동평균이
        // 스무딩 잔물결과 구분 못 함) 노이즈로 버린다 — 유일한 opening이면 예외적으로
        // 그대로 보고하는 편이 전 sector 폐색 선언보다 낫다.
        const wideEnough = valleys.filter((v) => v[2] >= opts.smoothingWindow)
        if (wideEnough.length > 0) valleys = wideEnough

        const goalDir = Math.atan2(opts.goal[1] - y, opts.goal[0] - x)

        if (valleys.length === 0) {
            // 전 sector가 threshold 이상: 실행 가능한 방향이 없다. cos-gate는
            // theta_err=0일 수도 있어 저절로 발동하지 않으므로 여기서 명시적으로
            // 로봇을 세워 시뮬레이터의 정체 판정이 에피소드를 끝내게 한다.
            emitEv({
                event: "histogram_updated", state: [x, y, theta], bins: smoothed,
                data: {threshold: opts.threshold, target_direction: goalDir, selected_direction: goalDir},
            })
            return {v: 0, omega: 0}
        }

        const kTarget = sectorOf(goalDir)
        const candidates = valleys.map((v) => candidateDirection(v, kTarget, goalDir))
        const costs = candidates.map((d) => Math.abs(wrapToPi(d - goalDir)))
        const gaps = valleys.map((v) => nearestSectorGap(v, kTarget))
        // 동률로 가까운 valley는 폭(넓을수록 여유 공간)으로 tie-break한다 — 원시 스캔
        // 순서가 아니라. python min(key=(gap, -width))의 첫 최솟값 선택과 동일하게,
        // 엄격한 개선(<)일 때만 갱신해 동률 시 더 낮은 index를 유지한다.
        let selected = 0
        for (let i = 1; i < valleys.length; i++) {
            if (gaps[i] < gaps[selected]
                || (gaps[i] === gaps[selected] && valleys[i][2] > valleys[selected][2])) {
                selected = i
            }
        }
        const thetaSel = candidates[selected]

        emitEv({
            event: "histogram_updated", state: [x, y, theta], bins: smoothed,
            data: {threshold: opts.threshold, target_direction: goalDir, selected_direction: thetaSel},
        })
        candidates.forEach((direction, i) => {
            const probe: [number, number] = [
                x + opts.windowRadius * Math.cos(direction), y + opts.windowRadius * Math.sin(direction),
            ]
            emitEv({
                event: "candidate_evaluated", state: probe, cost: costs[i],
                data: {direction, selected: i === selected ? 1 : 0},
            })
        })

        const hSel = smoothed[sectorOf(thetaSel)]
        const vEff = opts.maxSpeed * (1 - Math.min(hSel, opts.hM) / opts.hM)
        return headingCommand(wrapToPi(thetaSel - theta), opts.kOmega, vEff, opts.maxOmega)
    }

    runClosedLoop(
        {
            map: opts.map, startPose: opts.start, goal: opts.goal, controlDt: opts.controlDt,
            maxSteps: opts.maxSteps, goalTolerance: opts.goalTolerance, footprintRadius: opts.footprintRadius,
            stallWindow: opts.stallWindow, stallDistance: opts.stallDistance,
        },
        emit,
        tick,
    )

    return events
}

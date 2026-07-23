import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runTeb} from "../../../../libs/algorithms/teb";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// 두 프리셋 모두 20x20 @ 0.5m 해상도(10m x 10m 방) -- ElasticBandsSandbox와 같은
// "world 좌표 술어로 벽을 찍는" 관례. footprintRadius 0.3(지름 0.6) 규칙에 맞춰 모든
// 통로 폭을 0.9m 이상으로 잡는다.
const RES = 0.5
const buildGrid = (name: string, width: number, height: number, isWall: (x: number, y: number) => boolean): GridMap => {
    const occupied = new Array(width * height).fill(false)
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            const x = (col + 0.5) * RES
            const y = (height - 1 - row + 0.5) * RES
            occupied[row * width + col] = isWall(x, y)
        }
    }
    return {name, width, height, occupied, resolution: RES, originX: 0, originY: 0}
}

// 시작/목표는 경계 벽에서 안쪽으로 두 칸(1.25m, 벽면까지 0.75m 여유) 떨어뜨린다 --
// 한 칸(0.75m, 벽면까지 0.25m)이면 footprintRadius 0.3보다 벽까지 여유가 좁아 첫
// tick부터 즉시 충돌 처리된다(discCollides는 로봇 중심-셀 사각형 최근접점 거리로
// 판정하므로, 셀 하나 두께의 경계벽에서 0.25m는 0.3m 반경보다 작다).

// "Corner cutting": 빈 방(경계만 벽) 위에 3단 지그재그 reference path -- 장애물의
// 간섭 없이 시간 최적성 항(w_time)이 코너를 안쪽으로 당기는 모습만 순수하게 보여준다.
const cornerCuttingMap = (): GridMap => buildGrid("teb_corner_cutting", 20, 20, (x, y) =>
    x < 0.5 || x >= 9.5 || y < 0.5 || y >= 9.5)
const CORNER_CUTTING_PATH: Point[] = [[1.25, 1.25], [1.25, 5.0], [5.0, 5.0], [5.0, 8.75], [8.75, 8.75]]
const CORNER_CUTTING_START: Pose = [1.25, 1.25, Math.PI / 2]
const CORNER_CUTTING_GOAL: [number, number] = [8.75, 8.75]

// "Sharp corner": 폭 1.5m짜리 L자 복도(직선 뒤 90도 급코너, 코너 뒤 통로도 0.9m 규칙을
// 만족)만 남기고 나머지는 전부 벽으로 채운다 -- 회전율 한계 + kinematics 항이 코너에서
// ΔT를 늘려 v = ell/ΔT가 떨어지는 감속을 trail 의 옅어지는 색으로 보여준다(직선 구간 자체는
// 어떤 비용 항도 감속시키지 않는다). 폭 1.0m(0.9m 규칙의 최소치에 가까움)에서는 TEB가
// 코너를 자르며 중심선에서 0.2m 이상 벗어나 실제로 벽에 닿는 것을 실행 검증으로
// 확인했다 -- 1.5m로 여유를 둔다.
const CORRIDOR_HALF = 0.75
const sharpCornerMap = (): GridMap => buildGrid("teb_sharp_corner", 20, 20, (x, y) => {
    const horizontalArm = y >= 5.0 - CORRIDOR_HALF && y < 5.0 + CORRIDOR_HALF && x >= 0.5 && x < 5.0 + CORRIDOR_HALF
    const verticalArm = x >= 5.0 - CORRIDOR_HALF && x < 5.0 + CORRIDOR_HALF && y >= 5.0 - CORRIDOR_HALF && y < 9.5
    return !(horizontalArm || verticalArm)
})
const SHARP_CORNER_PATH: Point[] = [[1.25, 5.0], [5.0, 5.0], [5.0, 8.75]]
const SHARP_CORNER_START: Pose = [1.25, 5.0, 0]
const SHARP_CORNER_GOAL: [number, number] = [5.0, 8.75]

// "Homotopy trap": 중앙 블록(x=[4,6), y=[2.5,6.5))의 아래로 폭 2m 통로가 곧게 열려
// 있는데 reference path는 위로 크게 돈다. 밴드 변형은 연속이라 장애물을 "건너뛰어"
// 반대편 homotopy로 넘어갈 수 없고, 최적화는 reference가 정한 위쪽 부류 안의 국소
// 최적만 찾는다 — 실행 검증: 아래 직선 약 7.5m가 열려 있어도 위로 약 19m를 돌아
// REACHED. 원 논문 계열의 알려진 한계로, 후속 연구가 서로 다른 homotopy의 TEB 여러
// 개를 병렬 최적화해 고친다(Rösmann 2017).
const homotopyMap = (): GridMap => buildGrid("teb_homotopy", 20, 20, (x, y) =>
    x >= 4.0 && x < 6.0 && y >= 2.5 && y < 6.5)
const HOMOTOPY_PATH: Point[] = [[1.25, 1.5], [1.25, 7.5], [8.75, 7.5], [8.75, 1.5]]
const HOMOTOPY_START: Pose = [1.25, 1.5, Math.PI / 2]
const HOMOTOPY_GOAL: [number, number] = [8.75, 1.5]

type PresetId = "corner_cutting" | "sharp_corner" | "homotopy_trap";

interface Preset { map: () => GridMap; path: Point[]; start: Pose; goal: [number, number] }

const PRESETS: Record<PresetId, Preset> = {
    corner_cutting: {
        map: cornerCuttingMap, path: CORNER_CUTTING_PATH, start: CORNER_CUTTING_START, goal: CORNER_CUTTING_GOAL,
    },
    sharp_corner: {
        map: sharpCornerMap, path: SHARP_CORNER_PATH, start: SHARP_CORNER_START, goal: SHARP_CORNER_GOAL,
    },
    homotopy_trap: {
        map: homotopyMap, path: HOMOTOPY_PATH, start: HOMOTOPY_START, goal: HOMOTOPY_GOAL,
    },
}

// configs/local_planning/teb.yaml 기본값 -- 슬라이더가 없는 나머지 파라미터와 sim 6종은
// 이 값으로 고정한다. max_steps는 yaml 기본(1000)보다 낮춰 sandbox 재생 이벤트 수를
// 억제한다(다른 local sandbox와 같은 관례).
const W_PATH = 1.0
const W_OBSTACLE = 15.0
const W_VELOCITY = 10.0
const W_ACCELERATION = 5.0
const W_KINEMATICS = 50.0
// yaml 기본값(1.5)에서는 두 프리셋 모두 실제 최적화된 궤적의 회전율이 한계에 거의
// 닿지 않아(실행 검증 결과 최대 ~1.1 rad/s) sharp corner에서 trail 색 차이가
// 거의 안 보인다 -- 0.5로 낮춰야 코너 부근 평균 속도가 far-from-corner 대비 뚜렷하게
// 떨어지는(실행 검증: near 0.39 m/s vs far 0.71 m/s) 것을 확인했다. 두 프리셋 모두 이
// 값에서 REACHED로 끝난다.
const OMEGA_MAX = 0.5
const A_MAX = 1.5
const MIN_OBSTACLE_DIST = 0.8
const DT_REF = 0.85
const DT_MIN = 0.05
const HORIZON = 4.0
const ITERATIONS = 40
const STEP_ALPHA = 0.02
const MAX_STEP_XY = 0.05
const MAX_STEP_THETA = 0.1
const MAX_STEP_DT = 0.02
const MAX_POSES = 40
const REINIT_DISTANCE = 1.0
const FOOTPRINT_RADIUS = 0.3
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const TebScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("corner_cutting")
    const preset = PRESETS[presetId]
    const [map, setMap] = useState<GridMap>(preset.map)
    const [start, setStart] = useState<Pose>(preset.start)
    const [wTime, setWTime] = useState(1.0)
    const [vMax, setVMax] = useState(0.8)

    const events = useMemo(() => runTeb({
        map, startPose: start, goal: [preset.goal[0], preset.goal[1], 0], referencePath: preset.path,
        wPath: W_PATH, wObstacle: W_OBSTACLE, wVelocity: W_VELOCITY, wAcceleration: W_ACCELERATION,
        wTime, wKinematics: W_KINEMATICS, maxSpeed: vMax, maxOmega: OMEGA_MAX, aMax: A_MAX,
        minObstacleDist: MIN_OBSTACLE_DIST, dtRef: DT_REF, dtMin: DT_MIN, horizon: HORIZON,
        iterations: ITERATIONS, stepAlpha: STEP_ALPHA, maxStepXy: MAX_STEP_XY,
        maxStepTheta: MAX_STEP_THETA, maxStepDt: MAX_STEP_DT, maxPoses: MAX_POSES,
        reinitDistance: REINIT_DISTANCE, controlDt: CONTROL_DT, maxSteps: MAX_STEPS,
        goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, preset, wTime, vMax])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const switchPreset = (id: PresetId) => {
        const next = PRESETS[id]
        setPresetId(id)
        setMap(next.map())
        setStart(next.start)
    }

    const presetLabel = (id: PresetId): string => ({
        corner_cutting: t("corner cutting", "corner cutting"),
        sharp_corner: t("sharp corner", "sharp corner"),
        homotopy_trap: t("homotopy trap", "homotopy 함정"),
    })[id]

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS} durationMs={8000}
            map={map} events={events} startPose={start} goal={preset.goal}
            referencePath={preset.path} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={() => switchPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs">
                        {(["corner_cutting", "sharp_corner", "homotopy_trap"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        presetId === id
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border text-muted hover:bg-surface",
                                    )}>
                                {presetLabel(id)}
                            </button>
                        ))}
                    </div>
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="w_time" value={wTime}
                                     min={0} max={5.0} step={0.1} onCommit={setWTime}/>
                        <ParamSlider label="v_max" value={vMax}
                                     min={0.3} max={1.3} step={0.05} onCommit={setVMax}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "w_time — weight on finishing fast; raise it and the pose chain pulls tighter, cutting corners closer",
                            "w_time: 빨리 끝내려는 가중치. 올리면 pose 열이 더 바짝 당겨져 코너를 가깝게 자른다",
                        )}</span>
                        <span>{t(
                            "v_max — top speed; raise it to move faster, but the turn-rate and accel limits bind more often",
                            "v_max: 상한 속도. 올리면 빨라지되 회전율·가속 한계에 더 자주 걸린다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center max-w-[24rem]">
                        {t(
                            "blue chain = the plan (band) re-optimized this tick, running ahead of the robot · segment thickness = its ΔT (thicker = slower stretch) · gray dashed = reference path · trail fading = the robot actually slowing down",
                            "파란 사슬 = 이번 tick 다시 최적화된 계획(밴드)으로, 로봇보다 앞서 달린다 · 세그먼트 굵기 = 그 구간의 ΔT (굵을수록 느린 구간) · 회색 점선 = reference path · trail이 옅어지면 로봇이 실제로 감속 중",
                        )}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "raise w_time to see the pose chain pull tighter into each corner, or watch the sharp corner preset slow down honestly where the turn rate limit binds",
                            "w_time을 올리면 pose 열이 각 코너로 더 바짝 당겨지는 모습을, sharp corner 프리셋에서는 회전율 한계가 걸리는 지점에서 정직하게 느려지는 모습을 볼 수 있다",
                        )}
                    </div>
                    <div className="text-xs text-muted text-center max-w-[24rem]">
                        {t(
                            "homotopy trap = the weakness preset: a straight 2 m-wide corridor is open below the block, but because the reference path rounds it above, the band can only deform continuously and never jumps to the other side — the robot detours roughly 19 m where 7.5 m would do",
                            "homotopy 함정 = 약점 프리셋. 블록 아래로 폭 2m 직선 통로가 열려 있지만 reference path가 위로 돌기 때문에, 연속으로만 변형되는 밴드는 반대편으로 건너뛰지 못한다. 7.5m면 될 길을 로봇이 약 19m 돌아간다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const TebSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live TEB: the pose chain drawn with segment thickness proportional to its ΔT is the band optimizing this tick — raise w_time to pull it tighter through corners, or switch to sharp corner to see the turn-rate limit force a real slowdown",
            "라이브 TEB. ΔT에 비례한 굵기로 그려지는 pose 열이 이번 tick 최적화된 밴드다. w_time을 올리면 코너를 더 바짝 당겨 지나가고, sharp corner로 전환하면 회전율 한계가 실제로 감속을 강제하는 모습을 볼 수 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<TebScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <TebScene panel={340}/>
    </CanvasFigure>
}

export default TebSandbox

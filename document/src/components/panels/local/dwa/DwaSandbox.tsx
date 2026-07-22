import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runDwa} from "../../../../libs/algorithms/dwa";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, gridFromPgmRows} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// "sudden wall" 프리셋: 로봇이 정면으로 향한 벽이 남쪽 gap 하나만 열어 둔다 -- dynamic
// window가 admissible 정지거리 부등식에 걸려 급격히 좁아지는 모습과, workspace에서
// 롤아웃 부채꼴이 벽 앞에서 잘려나가는 모습을 보여준다(인트로의 LocalVelocityWindow는
// (v, ω) 평면에서 같은 현상을 보여준다 -- 이 프리셋은 workspace 쪽 대응물).
const WALL_COL = 10
const suddenWallMap = (): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let r = 0; r < 16; r++) occupied[r * width + WALL_COL] = true  // rows 0..15: 벽
    // rows 16..19: 남쪽 전역 gap -- 로봇이 커브를 틀어 통과할 수 있는 유일한 경로.
    return {name: "sudden_wall", width, height, occupied, resolution: 0.5, originX: 0, originY: 0}
}
const SUDDEN_WALL_START: Pose = [1.25, 4.75, 0]
const SUDDEN_WALL_GOAL: [number, number] = [8.75, 1.25]

// "clutter" 프리셋: 균일 간격 2x2 블록(1m) 밭. footprint 0.3(지름 0.6)에서 통로 폭이
// 지름 + 0.3m 여유를 넘도록 블록 사이 간격을 1.0m로 잡고 3행으로 엇갈려 배치한다.
// 저장소 clutter01.pgm은 yaml 기본 footprint 0.2용으로 튜닝돼 통로가 좁아, 0.3에서는
// 첫 tick 충돌/정체가 나므로(footprint 0.2로만 REACHED) 0.3 규칙에 맞춰 새로 구성했다.
type Rows = string[];
const CLUTTER_BLOCKS: [number, number][] = [
    [3, 3], [9, 3], [15, 3], [6, 8], [12, 8], [3, 13], [9, 13], [15, 13],
]
const clutterMap = (): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let r = 0; r < height; r++)
        for (let c = 0; c < width; c++)
            if (r === 0 || r === height - 1 || c === 0 || c === width - 1) occupied[r * width + c] = true
    for (const [c0, r0] of CLUTTER_BLOCKS)
        for (let r = r0; r < r0 + 2; r++) for (let c = c0; c < c0 + 2; c++) occupied[r * width + c] = true
    return {name: "clutter", width, height, occupied, resolution: 0.5, originX: 0, originY: 0}
}
const CLUTTER_START: Pose = [1.25, 1.25, 0]
const CLUTTER_GOAL: [number, number] = [8.75, 8.75]
// "dead end" 프리셋: 저장소 pf_trap01 맵(maps/grid/pf_trap01.pgm) 그대로 -- ㄷ자 함정의
// 등 쪽에 goal을 두어, dynamic window 전체가 admissible을 잃고 STALLED로 끝나는 한계를
// 정직하게 보여준다(python/tests/test_dwa.py의 local-minima 케이스와 같은 시나리오).
const TRAP_ROWS: Rows = [
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 0 0 0 0 0 0 0 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 0 0 0 0 0 0 0 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
]
const TRAP_START: Pose = [1.25, 4.75, 0]
const TRAP_GOAL: [number, number] = [8.75, 4.75]

type PresetId = "sudden_wall" | "clutter" | "dead_end";

interface Preset { map: () => GridMap; start: Pose; goal: [number, number] }

const PRESETS: Record<PresetId, Preset> = {
    sudden_wall: {map: suddenWallMap, start: SUDDEN_WALL_START, goal: SUDDEN_WALL_GOAL},
    clutter: {map: clutterMap, start: CLUTTER_START, goal: CLUTTER_GOAL},
    dead_end: {map: () => gridFromPgmRows("pf_trap01", TRAP_ROWS, 0.5), start: TRAP_START, goal: TRAP_GOAL},
}

// configs/local_planning/dwa.yaml 기본값 -- sandbox 슬라이더가 없는 나머지 파라미터와
// sim 6종은 이 값으로 고정한다. omega_samples는 후보 렌더 가독성을 위해 9로 고정
// (yaml 기본값과 동일), max_steps는 sandbox 재생 성능을 위해 400으로 낮춘다(yaml은 1000).
const MIN_SPEED = 0.0
const MAX_OMEGA = 1.35
const ACCEL = 1.92
const ACCEL_OMEGA = 6.98
const V_SAMPLES = 5
const OMEGA_SAMPLES = 9
const SIM_STEPS = 12
const HEADING_WEIGHT = 0.15
const VELOCITY_WEIGHT = 0.23
const CLEARANCE_LIMIT = 0.68
const SLOW_RADIUS = 0.5
// 데모 룰상 차량이 잘 보이도록 footprint는 0.3 이상으로 둔다. 세 프리셋의 통로 폭·시작
// 위치를 이 반경으로 REACHED(sudden_wall/clutter)/STALLED(dead_end)가 나도록 재튜닝했다.
const FOOTPRINT_RADIUS = 0.3
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const DwaScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("sudden_wall")
    const preset = PRESETS[presetId]
    const [map, setMap] = useState<GridMap>(preset.map)
    const [start, setStart] = useState<Pose>(preset.start)
    const [goal, setGoal] = useState<[number, number]>(preset.goal)
    const [simTime, setSimTime] = useState(1.66)
    const [clearanceWeight, setClearanceWeight] = useState(0.14)
    const [maxSpeed, setMaxSpeed] = useState(0.77)

    const events = useMemo(() => runDwa({
        map, startPose: start, goal,
        maxSpeed, minSpeed: MIN_SPEED, maxOmega: MAX_OMEGA, accel: ACCEL, accelOmega: ACCEL_OMEGA,
        vSamples: V_SAMPLES, omegaSamples: OMEGA_SAMPLES, simTime, simSteps: SIM_STEPS,
        headingWeight: HEADING_WEIGHT, clearanceWeight, velocityWeight: VELOCITY_WEIGHT,
        clearanceLimit: CLEARANCE_LIMIT, slowRadius: SLOW_RADIUS, footprintRadius: FOOTPRINT_RADIUS,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, goal, simTime, clearanceWeight, maxSpeed])

    const applyPreset = (id: PresetId) => {
        setPresetId(id)
        setMap(PRESETS[id].map())
        setStart(PRESETS[id].start)
        setGoal(PRESETS[id].goal)
    }

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }

    const presetLabel = (id: PresetId): string => ({
        sudden_wall: t("sudden wall", "급정지 벽"),
        clutter: t("clutter", "산개 장애물"),
        dead_end: t("dead end", "막다른 길"),
    })[id]

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS}
            map={map} events={events} startPose={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onMoveGoal={setGoal}
            onReset={() => applyPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs">
                        {(["sudden_wall", "clutter", "dead_end"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => applyPreset(id)}
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
                        <ParamSlider label={t("sim time", "sim time")} value={simTime}
                                     min={0.5} max={3.0} step={0.1} onCommit={setSimTime}/>
                        <ParamSlider label="β" value={clearanceWeight}
                                     min={0} max={0.6} step={0.02} onCommit={setClearanceWeight}/>
                        <ParamSlider label={t("max speed", "max speed")} value={maxSpeed}
                                     min={0.2} max={1.3} step={0.05} onCommit={setMaxSpeed}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "sim time — how far each candidate arc is rolled out; longer looks further ahead but reacts slower",
                            "sim time: 각 후보 원호를 굴려보는 예측 구간. 길수록 멀리 내다보지만 반응이 굼뜬다",
                        )}</span>
                        <span>{t(
                            "β — weight on staying clear of obstacles; raise it and the fan hugs walls less, zero lets arcs graze them",
                            "β: 장애물에서 멀어지려는 가중치. 올리면 부채꼴이 벽에서 더 멀어지고, 0이면 원호가 벽을 스친다",
                        )}</span>
                        <span>{t(
                            "max speed — top of the dynamic window; higher is faster but the stopping-distance limit prunes more arcs",
                            "max speed: dynamic window 상한 속도. 올리면 빨라지되 정지거리 제약이 커져 후보가 더 많이 걸러진다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "walk through the wall gap, weave the clutter, or watch the dead end stall the robot honestly",
                            "벽 gap을 통과하거나 산개 장애물 사이를 누비거나, 막다른 길에서 로봇이 정직하게 멈추는 모습을 보라",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const DwaSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live DWA: every candidate arc rolled out this tick is drawn as a thin fan, the chosen one thick and bright — raise the clearance weight and watch the fan hug the wall less, or push clearance to zero in the dead end and watch every arc lose admissibility at once",
            "라이브 DWA. 이번 tick이 굴려본 모든 후보 원호가 가는 부채꼴로, 선택된 것은 굵고 밝게 그려진다. clearance 가중치를 올리면 부채꼴이 벽에서 더 멀어지고, 막다른 길에서는 모든 원호가 한꺼번에 admissible을 잃는 모습을 볼 수 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<DwaScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <DwaScene panel={340}/>
    </CanvasFigure>
}

export default DwaSandbox

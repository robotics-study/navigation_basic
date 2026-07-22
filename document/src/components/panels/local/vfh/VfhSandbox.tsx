import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runVfh} from "../../../../libs/algorithms/vfh";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// 두 프리셋 모두 20x20 @ 0.5m 해상도(10m x 10m 방). footprint 0.3(지름 0.6) 규칙에
// 맞춰 통로 폭을 1.0m 이상으로 잡는다.
const RES = 0.5
const buildGrid = (name: string, isWall: (x: number, y: number) => boolean): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            if (row === 0 || row === height - 1 || col === 0 || col === width - 1) {
                occupied[row * width + col] = true
                continue
            }
            const x = (col + 0.5) * RES
            const y = (height - 1 - row + 0.5) * RES
            occupied[row * width + col] = isWall(x, y)
        }
    }
    return {name, width, height, occupied, resolution: RES, originX: 0, originY: 0}
}

// "좁은 통로": 폭 2.0m 통로에 중앙 1.0m pinch. 양쪽 벽이 항상 window 안에 있어
// 히스토그램이 양쪽으로 채워지므로, 열린 방에서 나타나던 조향 진동이 없다. 그 진동은
// 장애물이 window 밖으로 사라진 near-open 구간에서 전방 valley가 goal 방향을 인접 sector
// 사이에서 뒤집으며 생기는 VFH의 sector 이산화 한계이고(VFH+가 hysteresis로 억제), 벽이
// 방향을 가둔 통로에서는 나타나지 않는다. pinch에서 valley 밀도가 올라 감속을 보여준다.
const narrowMap = (): GridMap => buildGrid("vfh_narrow", (x, y) => {
    if (!(y >= 4.0 && y < 6.0)) return true
    return Math.abs(x - 5.0) < 0.25 && !(y >= 4.5 && y < 5.5)
})
const NARROW_START: Pose = [1.25, 5.0, 0]
const NARROW_GOAL: [number, number] = [8.75, 5.0]

// "밀집": 균일 간격 2x2 블록(1m) 밭. footprint 0.3에 통로 폭 1.0m를 보장하도록 3행
// 엇갈림 배치. 저장소 clutter01.pgm은 yaml 기본 footprint 0.2용이라 0.3에서 통로가 좁아
// 첫 tick 충돌이 나므로, 0.3 규칙에 맞춰 새로 구성했다.
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

type Preset = "narrow" | "clutter";

const presetMap = (preset: Preset): GridMap => preset === "narrow" ? narrowMap() : clutterMap()
const presetStart = (preset: Preset): Pose => preset === "narrow" ? NARROW_START : CLUTTER_START
const presetGoal = (preset: Preset): [number, number] => preset === "narrow" ? NARROW_GOAL : CLUTTER_GOAL

// configs/local_planning/vfh.yaml 기본값 — sandbox 슬라이더가 없는 나머지 파라미터
// (smoothing_window, wide_valley_sectors, h_m, k_omega, max_speed, max_omega)와
// sim 6종은 이 값으로 고정한다.
const SMOOTHING_WINDOW = 3
const WIDE_VALLEY_SECTORS = 20
const H_M = 0.0905
const K_OMEGA = 1.54
const MAX_SPEED = 0.44
const MAX_OMEGA = 2.86
const FOOTPRINT_RADIUS = 0.3
const CONTROL_DT = 0.1
const MAX_STEPS = 1000
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const VfhScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [preset, setPreset] = useState<Preset>("clutter")
    const [map, setMap] = useState<GridMap>(() => presetMap("clutter"))
    const [start, setStart] = useState<Pose>(CLUTTER_START)
    const [goal, setGoal] = useState<[number, number]>(CLUTTER_GOAL)
    const [threshold, setThreshold] = useState(0.02)
    const [windowRadius, setWindowRadius] = useState(1.8)
    const [numSectors, setNumSectors] = useState(60)

    const events = useMemo(() => runVfh({
        map, start, goal,
        numSectors, windowRadius, threshold,
        smoothingWindow: SMOOTHING_WINDOW, wideValleySectors: WIDE_VALLEY_SECTORS, hM: H_M,
        kOmega: K_OMEGA, maxSpeed: MAX_SPEED, maxOmega: MAX_OMEGA,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, goal, numSectors, windowRadius, threshold])

    const applyPreset = (next: Preset) => {
        setPreset(next)
        setMap(presetMap(next))
        setStart(presetStart(next))
        setGoal(presetGoal(next))
    }

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS}
            map={map} events={events} startPose={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onMoveGoal={setGoal}
            onReset={() => applyPreset(preset)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center gap-1.5 text-xs">
                        {(["narrow", "clutter"] as const).map((p) => (
                            <button key={p} type="button" onClick={() => applyPreset(p)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        preset === p
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {p === "narrow" ? t("narrow passage", "좁은 통로") : t("dense clutter", "밀집")}
                            </button>
                        ))}
                    </div>
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label={t("threshold", "threshold")} value={threshold}
                                     min={0.01} max={0.12} step={0.005} onCommit={setThreshold}/>
                        <ParamSlider label={t("window", "window")} value={windowRadius}
                                     min={0.6} max={2.4} step={0.02} onCommit={setWindowRadius}/>
                        <ParamSlider label={t("sectors", "sector 수")} value={numSectors}
                                     min={20} max={100} step={4} onCommit={setNumSectors}
                                     format={(v) => String(Math.round(v))}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "threshold — density that counts a direction as blocked; raise it and valleys close one by one",
                            "threshold: 한 방향을 막힘으로 보는 밀도 기준. 올리면 valley가 하나씩 닫힌다",
                        )}</span>
                        <span>{t(
                            "window — obstacle radius folded into the histogram; wider looks further ahead but can read a real gap as blocked",
                            "window: 히스토그램에 담는 장애물 반경. 넓히면 더 멀리 보지만 실제 틈을 막힘으로 볼 수 있다",
                        )}</span>
                        <span>{t(
                            "sectors — how finely direction is discretized; more is smoother, fewer coarsens steering and worsens the wobble",
                            "sector 수: 방향 이산화 해상도. 많을수록 조향이 매끄럽고, 적으면 거칠어져 흔들림이 커진다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center">
                        {t("draw walls, drag the endpoints, or raise threshold until a valley closes",
                            "벽을 그리거나 시작/목표를 끌어 보라. threshold를 올리면 valley가 하나씩 닫힌다")}
                    </div>
                </div>
            }
        />
    )
}

const VfhSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live VFH: the rose around the robot is the smoothed polar histogram, thickest where obstacles crowd a direction — the robot steers for the nearest open valley toward the goal and slows as that valley's density rises. In the narrow corridor the walls pin the valley so it tracks straight",
            "라이브 VFH. 로봇 주변의 장미 모양이 스무딩된 폴라 히스토그램이고, 장애물이 몰린 방향일수록 두껍다. goal에 가장 가까운 열린 valley로 조향하고, 그 valley의 밀도가 오를수록 느려진다. 좁은 통로에서는 양쪽 벽이 valley를 가둬 곧게 따라간다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<VfhScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <VfhScene panel={340}/>
    </CanvasFigure>
}

export default VfhSandbox

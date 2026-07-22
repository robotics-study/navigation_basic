import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runVfh} from "../../../../libs/algorithms/vfh";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, gridFromPgmRows} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// 산개 블록 프리셋은 저장소 실제 demo 맵(maps/grid/clutter01.pgm)을 그대로 재사용한다 —
// configs/local_planning/vfh.yaml 기본값이 정확히 이 스케일로 튜닝되어 있어(clutter01_s1,
// steps=315), 손으로 새 지형을 지어 균형을 잘못 잡을 위험이 없다(Potential Fields
// sandbox와 같은 관례). 좁은 통로 프리셋은 대응하는 저장소 맵이 없어(VFH 전용
// 시나리오) 같은 20x20/0.5m 스케일로 새로 구성했다 — valley가 하나만 열려
// narrow-valley 조향 규칙이 항상 발동하는 것을 보여준다.
type Rows = string[];   // 각 행: 공백 구분 pixel 값(0=occupied, 255=free), pgm 그대로.


// 전체 높이를 가로지르는 벽 하나에 gap 두 칸(로봇 지름 대비 넉넉한 폭)뿐이라, 매 tick
// valley가 하나만 열린다.
const NARROW_ROWS: Rows = [
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 255 255 255 255 255 255 255 255 0",
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
]
const NARROW_START: Pose = [1.25, 5.0, 0]
const NARROW_GOAL: [number, number] = [8.75, 5.0]

// maps/grid/clutter01.pgm 그대로(2x2 블록 산개, 사방 border) — clutter01_s1과 동일한
// 대각선 start/goal.
const CLUTTER_ROWS: Rows = [
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 0 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 0 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 0 0 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 0 0 255 255 255 255 255 255 0 0 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 0 0 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 0 0 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 0 0 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 0 0 255 255 255 255 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 0 0 255 255 255 255 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
]
const CLUTTER_START: Pose = [1.25, 1.25, 0]
const CLUTTER_GOAL: [number, number] = [9.0, 7.75]

type Preset = "narrow" | "clutter";

const presetMap = (preset: Preset): GridMap => preset === "narrow"
    ? gridFromPgmRows("vfh_narrow", NARROW_ROWS, 0.5)
    : gridFromPgmRows("clutter01", CLUTTER_ROWS, 0.5)
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
            "Live VFH: the rose around the robot is the smoothed polar histogram, thickest where obstacles crowd a direction — the robot steers for the nearest open valley toward the goal and slows as that valley's density rises",
            "라이브 VFH. 로봇 주변의 장미 모양이 스무딩된 폴라 히스토그램이고, 장애물이 몰린 방향일수록 두껍다. goal에 가장 가까운 열린 valley로 조향하고, 그 valley의 밀도가 오를수록 느려진다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<VfhScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <VfhScene panel={340}/>
    </CanvasFigure>
}

export default VfhSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runStanley} from "../../../../libs/algorithms/stanley";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {emptyGrid, GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// S-curve 프리셋: maps/scenarios/open01_s3.yaml이 참조하는 grid(maps/grid/open01.yaml)와
// reference_path를 그대로 미러한다 -- start만 경로 첫 점에서 측방 1.5m 오프셋을 둬
// crosstrack 수렴을 시연한다(python 시나리오는 1.2m 오프셋, 여기서는 pure_pursuit.ts의
// S-curve 맵 정의를 이 파일이 독립적으로 소유하는 관례와 같은 이유로 다시 적는다).
// 추종 데모는 장애물 없는 빈 맵(0.5 m/셀) — Stanley 는 회피 능력이 없는 추종기다.
const openHalfGrid = (name: string): GridMap => ({...emptyGrid(name, 20, 20), resolution: 0.5})

const OFFSET_PATH: Point[] = [
    [1.0, 1.0], [2.0, 1.0], [3.5, 1.2], [5.0, 1.5], [5.0, 3.5],
    [5.0, 6.0], [5.0, 8.5], [7.0, 8.8], [8.7, 9.0], [9.0, 9.0],
]
const OFFSET_START: Pose = [1.0, 2.5, 0]
const OFFSET_GOAL: [number, number] = [9.0, 9.0]

// 급코너 프리셋: 벽이 없는 빈 맵 위에 연속된 직각 코너를 둔 경로 -- heading 오차가
// 코너마다 크게 튀는 상황에서 psi/e 두 항이 함께 조향을 정정하는 모습을 보여준다.
const SHARP_PATH: Point[] = [[1, 1], [1, 7], [4, 7], [4, 2], [7, 2], [7, 8], [9, 8]]
const SHARP_START: Pose = [1, 1, 0]
const SHARP_GOAL: [number, number] = [9, 8]

type PresetId = "offset_start" | "sharp_corner" | "high_gain";

interface Preset { map: () => GridMap; path: Point[]; start: Pose; goal: [number, number]; kGain: number }

// "high gain" 프리셋은 offset_start와 같은 시나리오에서 k_gain만 정상 범위(2.5) 밖으로
// 크게 잡아 -- Stanley의 알려진 한계인 진동(overshoot → 반대편 overshoot)을 정직하게
// 드러낸다. 다른 두 프리셋의 gain은 configs/local_planning/stanley.yaml 기본값(2.5).
const PRESETS: Record<PresetId, Preset> = {
    offset_start: {map: () => openHalfGrid("s_curve"), path: OFFSET_PATH, start: OFFSET_START, goal: OFFSET_GOAL, kGain: 2.5},
    sharp_corner: {
        map: () => openHalfGrid("sharp_turn"), path: SHARP_PATH, start: SHARP_START,
        goal: SHARP_GOAL, kGain: 2.5,
    },
    high_gain: {map: () => openHalfGrid("s_curve"), path: OFFSET_PATH, start: OFFSET_START, goal: OFFSET_GOAL, kGain: 14},
}

// configs/local_planning/stanley.yaml의 공유 폐루프 시뮬레이터 블록 기본값
// (k_gain/k_soft/wheelbase만 sandbox에서 조절한다). max_steps는 yaml 기본(1000)보다
// 낮춰 sandbox 재생 성능을 지킨다.
const SIM_DEFAULTS = {
    maxSteer: 1.2, maxSpeed: 0.8, maxOmega: 1.5, slowRadius: 0.5, controlDt: 0.1, maxSteps: 400,
    goalTolerance: 0.3, footprintRadius: 0.35, stallWindow: 20, stallDistance: 0.05,
}

const StanleyScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("offset_start")
    const preset = PRESETS[presetId]
    const [map, setMap] = useState<GridMap>(preset.map)
    const [start, setStart] = useState<Pose>(preset.start)
    const [kGain, setKGain] = useState(preset.kGain)
    const [kSoft, setKSoft] = useState(0.5)
    const [wheelbase, setWheelbase] = useState(0.3)

    const events = useMemo(() => runStanley({
        map, startPose: start, goal: preset.goal, referencePath: preset.path,
        kGain, kSoft, wheelbase, ...SIM_DEFAULTS,
    }), [map, start, preset, kGain, kSoft, wheelbase])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const switchPreset = (id: PresetId) => {
        setPresetId(id)
        setMap(PRESETS[id].map())
        setStart(PRESETS[id].start)
        setKGain(PRESETS[id].kGain)
    }

    return (
        <LocalTracePlayer footprintRadius={SIM_DEFAULTS.footprintRadius} showCrosstrack
            map={map} events={events} startPose={start} goal={preset.goal}
            referencePath={preset.path} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={() => switchPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="k_gain" value={kGain} min={0.5} max={18} step={0.5}
                                     onCommit={setKGain}/>
                        <ParamSlider label="k_soft" value={kSoft} min={0.1} max={2} step={0.05}
                                     onCommit={setKSoft}/>
                        <ParamSlider label="L" value={wheelbase} min={0.15} max={0.6} step={0.01}
                                     onCommit={setWheelbase}/>
                    </div>
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {(["offset_start", "sharp_corner", "high_gain"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        presetId === id
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {id === "offset_start" ? t("offset start", "오프셋 시작")
                                    : id === "sharp_corner" ? t("sharp corner", "급코너")
                                        : t("high gain", "과대 gain")}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("watch the crosstrack error converge as the robot steers back onto the path",
                            "로봇이 경로로 다시 조향해 crosstrack 오차가 수렴하는 모습을 보라")}
                        {" · "}{t("raise k_gain to see it overshoot and oscillate",
                        "k_gain을 올려 overshoot와 진동을 관찰해 보라")}
                    </div>
                </div>
            }
        />
    )
}

const StanleySandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Stanley: converges from an offset start, negotiates sharp corners, and — pushed to a high gain — overshoots the path and oscillates",
            "라이브 Stanley. 오프셋 시작에서 수렴하고, 급코너를 통과하며, gain을 과도하게 올리면 경로를 지나쳐 진동한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<StanleyScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <StanleyScene panel={340}/>
    </CanvasFigure>
}

export default StanleySandbox

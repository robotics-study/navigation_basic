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

// 추종 데모는 장애물 없는 빈 맵(0.5 m/셀) — Stanley 는 회피 능력이 없는 추종기다.
const openHalfGrid = (name: string): GridMap => ({...emptyGrid(name, 20, 20), resolution: 0.5})

// S-curve 경로(maps/scenarios/open01_s3.yaml이 참조하는 grid의 reference_path 미러).
// start만 경로 첫 점에서 측방 1.5m 오프셋을 둬 crosstrack 수렴을 시연한다.
const OFFSET_PATH: Point[] = [
    [1.0, 1.0], [2.0, 1.0], [3.5, 1.2], [5.0, 1.5], [5.0, 3.5],
    [5.0, 6.0], [5.0, 8.5], [7.0, 8.8], [8.7, 9.0], [9.0, 9.0],
]
const OFFSET_START: Pose = [1.0, 2.5, 0]
const OFFSET_GOAL: [number, number] = [9.0, 9.0]

// 급코너 프리셋: 벽이 없는 빈 맵 위에 연속된 직각 코너를 둔 경로 -- heading 오차가
// 코너마다 크게 튀는 상황에서 psi(heading)/e(crosstrack) 두 항이 함께 조향을 정정하는
// 모습을 보여준다.
const SHARP_PATH: Point[] = [[1, 1], [1, 7], [4, 7], [4, 2], [7, 2], [7, 8], [9, 8]]
const SHARP_START: Pose = [1, 1, Math.PI / 2]
const SHARP_GOAL: [number, number] = [9, 8]

type PresetId = "weak_gain" | "strong_gain" | "sharp_corner";

interface Preset { map: () => GridMap; path: Point[]; start: Pose; goal: [number, number]; kGain: number }

// weak/strong 프리셋은 같은 오프셋 시작에서 k_gain만 바꿔 crosstrack 보정 세기를 대비시킨다.
// 엔진 스윕 결과 약한 gain(0.8)은 경로에 x=4.3 부근에서야 느슨하게 붙고, 강한 gain(8)은
// x=1.6 부근에서 곧바로 되돌아 붙는다. clamp-recompute 폐루프라 gain을 크게 올려도
// 오버슈트로 진동하지 않고 수렴 속도만 빨라진다. 급코너 프리셋은 기본 gain(2.5).
const PRESETS: Record<PresetId, Preset> = {
    weak_gain: {map: () => openHalfGrid("s_curve"), path: OFFSET_PATH, start: OFFSET_START, goal: OFFSET_GOAL, kGain: 0.8},
    strong_gain: {map: () => openHalfGrid("s_curve"), path: OFFSET_PATH, start: OFFSET_START, goal: OFFSET_GOAL, kGain: 8},
    sharp_corner: {map: () => openHalfGrid("sharp_turn"), path: SHARP_PATH, start: SHARP_START, goal: SHARP_GOAL, kGain: 2.5},
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
    const [presetId, setPresetId] = useState<PresetId>("weak_gain")
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
            crosstrackWheelbase={wheelbase}
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
                    <ul className="text-xs text-muted text-left max-w-[28rem] list-none space-y-0.5">
                        <li>{t("k_gain — crosstrack correction strength: higher snaps back onto the path faster, lower drifts back gently.",
                            "k_gain은 crosstrack 보정 세기다. 올리면 경로로 더 빠르게 되돌아 붙고, 내리면 느슨하게 천천히 붙는다.")}</li>
                        <li>{t("k_soft — softens the steering law at low speed so it does not spike as speed approaches zero.",
                            "k_soft는 저속에서 조향 법칙을 완화해 속도가 0에 가까워질 때 조향이 튀지 않게 한다.")}</li>
                        <li>{t("L — the virtual wheelbase used to turn steering angle into a turn rate: larger L turns more gently.",
                            "L은 조향각을 회전율로 바꾸는 가상 축간거리다. L이 클수록 더 완만하게 돈다.")}</li>
                    </ul>
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {(["weak_gain", "strong_gain", "sharp_corner"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        presetId === id
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {id === "weak_gain" ? t("weak k_gain", "약한 k_gain")
                                    : id === "strong_gain" ? t("strong k_gain", "강한 k_gain")
                                        : t("sharp corner", "급코너")}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("weak k_gain drifts back to the path slowly · strong k_gain snaps back fast",
                            "약한 k_gain은 경로로 천천히 되돌아 붙고, 강한 k_gain은 곧바로 되돌아 붙는다")}
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
            "Live Stanley: from an offset start, a weak k_gain drifts back to the path slowly while a strong k_gain snaps back fast; the sharp-corner preset shows the heading and crosstrack terms steering through right angles together",
            "라이브 Stanley. 오프셋 시작에서 약한 k_gain은 경로로 천천히 되돌아 붙고 강한 k_gain은 곧바로 붙는다. 급코너 프리셋은 heading 항과 crosstrack 항이 직각 코너를 함께 조향하는 모습을 보여준다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<StanleyScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <StanleyScene panel={340}/>
    </CanvasFigure>
}

export default StanleySandbox

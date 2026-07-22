import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runPurePursuit} from "../../../../libs/algorithms/pure_pursuit";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {emptyGrid, GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// S-curve 프리셋: maps/scenarios/open01_s2.yaml이 참조하는 grid(maps/grid/open01.yaml)와
// reference_path를 그대로 미러한다 -- python demo와 같은 입력이라 결과를 나란히 비교할 수 있다.
const S_CURVE_PATH: Point[] = [
    [1.0, 1.0], [2.0, 1.0], [3.5, 1.2], [5.0, 1.5], [5.0, 3.5],
    [5.0, 6.0], [5.0, 8.5], [7.0, 8.8], [8.7, 9.0], [9.0, 9.0],
]
// 추종 데모는 장애물 없는 빈 맵(0.5 m/셀)에서 경로 기하만 보여준다 — pure pursuit 은
// 회피 능력이 없는 추종기라 장애물을 두면 오해를 부른다.
const openHalfGrid = (name: string): GridMap => ({...emptyGrid(name, 20, 20), resolution: 0.5})

const S_CURVE_START: Pose = [1.0, 1.0, 0]
const S_CURVE_GOAL: [number, number] = [9.0, 9.0]

// 급커브 프리셋: 벽이 없는 빈 맵 위에 연속된 직각 코너를 둔 경로 -- lookahead가 코너를
// "잘라가는" 모습(corner cutting)을 L_d를 키우며 보여주기 위한 기하 데모다.
const SHARP_PATH: Point[] = [[1, 1], [1, 7], [4, 7], [4, 2], [7, 2], [7, 8], [9, 8]]
const SHARP_START: Pose = [1, 1, 0]
const SHARP_GOAL: [number, number] = [9, 8]

type PresetId = "s_curve" | "sharp";

interface Preset { map: () => GridMap; path: Point[]; start: Pose; goal: [number, number] }

const PRESETS: Record<PresetId, Preset> = {
    s_curve: {map: () => openHalfGrid("s_curve"), path: S_CURVE_PATH, start: S_CURVE_START, goal: S_CURVE_GOAL},
    sharp: {map: () => openHalfGrid("sharp_turn"), path: SHARP_PATH, start: SHARP_START, goal: SHARP_GOAL},
}

// configs/local_planning/pure_pursuit.yaml의 공유 폐루프 시뮬레이터 블록 기본값
// (lookahead_distance만 sandbox에서 조절한다).
const SIM_DEFAULTS = {
    maxSpeed: 0.8, maxOmega: 1.5, slowRadius: 0.5, controlDt: 0.1, maxSteps: 1000,
    goalTolerance: 0.3, footprintRadius: 0.35, stallWindow: 20, stallDistance: 0.05,
}

const PurePursuitScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("s_curve")
    const preset = PRESETS[presetId]
    const [map, setMap] = useState<GridMap>(preset.map)
    const [start, setStart] = useState<Pose>(preset.start)
    const [lookahead, setLookahead] = useState(0.6)

    const events = useMemo(() => runPurePursuit({
        map, startPose: start, goal: preset.goal, referencePath: preset.path,
        lookaheadDistance: lookahead, ...SIM_DEFAULTS,
    }), [map, start, preset, lookahead])

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
    }

    return (
        <LocalTracePlayer footprintRadius={SIM_DEFAULTS.footprintRadius} showLookahead
            map={map} events={events} startPose={start} goal={preset.goal}
            referencePath={preset.path} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={() => switchPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="L_d" value={lookahead} min={0.1} max={1.5} step={0.05}
                                     onCommit={setLookahead}/>
                        <span className="mx-1" aria-hidden="true">·</span>
                        {(["s_curve", "sharp"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        presetId === id
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {id === "s_curve" ? t("S-curve", "S-곡선") : t("sharp turn", "급커브")}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("drag the robot off the path and watch it converge back",
                            "로봇을 경로 밖으로 끌어 다시 붙는 모습을 보라")}
                        {" · "}{t("draw walls to force a collision", "벽을 그려 충돌을 유도해 보라")}
                    </div>
                </div>
            }
        />
    )
}

const PurePursuitSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Pure Pursuit: drag the robot off the reference path and watch the lookahead circle pull it back — raise L_d to see it cut corners wider",
            "라이브 Pure Pursuit. 로봇을 참조 경로 밖으로 끌면 lookahead 원이 다시 경로로 끌어당긴다. L_d를 올리면 코너를 더 크게 잘라가는 것을 볼 수 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<PurePursuitScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <PurePursuitScene panel={340}/>
    </CanvasFigure>
}

export default PurePursuitSandbox

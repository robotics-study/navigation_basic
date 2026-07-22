import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runRegulatedPurePursuit} from "../../../../libs/algorithms/regulated_pure_pursuit";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {emptyGrid, GridMap, gridFromPgmRows} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// "hairpin" 프리셋: 벽 없는 빈 맵 위에 90도 급코너 3개를 긴 직선 구간(4~6m)으로
// 띄엄띄엄 배치한 경로 -- 장애물의 간섭 없이 곡률 규제(regulated_min_radius)만
// 순수하게 보여주기 위한 기하 데모다. 직선 구간이 넉넉해 로봇이 코너 사이마다
// 최고 속도로 회복했다가 다음 코너에서 다시 감속하는 모습이 뚜렷이 보인다(코너를
// 촘촘히 붙이면 규제가 거의 끊임없이 걸려 "코너에서만 느려진다"는 대비가 흐려진다
// -- 실측: max_steps=400 예산 안에서 완주하는지도 이 간격으로 확인했다). 경로가
// 경계에서 항상 1m 이상 떨어져 있어(footprint를 빼도 proximity_distance 기본값
// 0.6m보다 크다) 근접 규제는 우연히도 발동하지 않는다.
const HAIRPIN_PATH: Point[] = [
    [1.2, 1.2], [1.2, 7.2], [5.2, 7.2], [5.2, 1.2], [7.8, 1.2],
]
const HAIRPIN_START: Pose = [1.2, 1.2, 0]
const HAIRPIN_GOAL: [number, number] = [7.8, 1.2]
const hairpinMap = (): GridMap => emptyGrid("hairpin", 9, 9)

// "narrow gap" 프리셋: maps/grid/clutter01.pgm과 maps/scenarios/clutter01_s2.yaml의
// reference_path를 그대로 재현한다(VFH/Potential Fields sandbox와 같은 관례 --
// 실제 저장소 시나리오라 근접 규제가 실제로 발동하는 구간임이 보장된다). 상단
// 가로 구간이 우상단 장애물(world x[7.5,8.5] y[7.5,8.5])의 위쪽 가장자리를
// 스치며 지나가 근접 규제를, 좌측->상단 코너가 곡률 규제를 함께 발동시킨다.
type Rows = string[];
const CLUTTER_ROWS: Rows = [
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 0 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 0 255 255 0",
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
const NARROW_GAP_PATH: Point[] = [
    [0.75, 0.75], [1.0, 4.0], [1.0, 7.5], [1.6, 8.6], [4.0, 8.75], [7.3, 8.75], [9.25, 9.25],
]
const NARROW_GAP_START: Pose = [0.75, 0.75, 0]
const NARROW_GAP_GOAL: [number, number] = [9.25, 9.25]
const narrowGapMap = (): GridMap => gridFromPgmRows("clutter01", CLUTTER_ROWS, 0.5)

type PresetId = "hairpin" | "narrow_gap" | "plain_pursuit";

interface Preset {
    map: () => GridMap;
    path: Point[];
    start: Pose;
    goal: [number, number];
    // 프리셋 전환 시 규제 슬라이더의 초기값 -- "plain pursuit"는 두 슬라이더 모두
    // yaml 선언 최소값 근방으로 낮춰 규제가 사실상 발동하지 않게 한다(threshold를
    // hairpin 경로의 실제 회전 반경보다 한참 작게 두면 "r < threshold" 조건이 결코
    // 참이 되지 않는다는 뜻 -- 규제식 자체는 그대로 두고 문턱만 끈다).
    regulatedMinRadius: number;
    proximityDistance: number;
}

const PRESETS: Record<PresetId, Preset> = {
    hairpin: {
        map: hairpinMap, path: HAIRPIN_PATH, start: HAIRPIN_START, goal: HAIRPIN_GOAL,
        regulatedMinRadius: 0.9, proximityDistance: 0.6,
    },
    narrow_gap: {
        map: narrowGapMap, path: NARROW_GAP_PATH, start: NARROW_GAP_START, goal: NARROW_GAP_GOAL,
        regulatedMinRadius: 0.9, proximityDistance: 0.6,
    },
    plain_pursuit: {
        map: hairpinMap, path: HAIRPIN_PATH, start: HAIRPIN_START, goal: HAIRPIN_GOAL,
        regulatedMinRadius: 0.05, proximityDistance: 0.05,
    },
}

// configs/local_planning/regulated_pure_pursuit.yaml의 공유 폐루프 시뮬레이터 블록 +
// 슬라이더로 노출하지 않는 알고리즘 파라미터는 이 값으로 고정한다. max_steps는 yaml
// 기본(1000)보다 낮춰 sandbox 재생 이벤트 수를 억제한다(다른 local sandbox와 같은 관례).
const FIXED = {
    minLookahead: 0.25, maxLookahead: 1.0, minRegulatedSpeed: 0.1, collisionCheckStep: 0.05,
    maxSpeed: 0.8, maxOmega: 1.5, slowRadius: 0.5,
    controlDt: 0.1, maxSteps: 400, goalTolerance: 0.3, footprintRadius: 0.35,
    stallWindow: 20, stallDistance: 0.05,
}

const RegulatedPurePursuitScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("hairpin")
    const preset = PRESETS[presetId]
    const [map, setMap] = useState<GridMap>(preset.map)
    const [start, setStart] = useState<Pose>(preset.start)
    const [lookaheadTime, setLookaheadTime] = useState(1.0)
    const [regulatedMinRadius, setRegulatedMinRadius] = useState(preset.regulatedMinRadius)
    const [proximityDistance, setProximityDistance] = useState(preset.proximityDistance)

    const events = useMemo(() => runRegulatedPurePursuit({
        map, startPose: start, goal: preset.goal, referencePath: preset.path,
        lookaheadTime, regulatedMinRadius, proximityDistance, ...FIXED,
    }), [map, start, preset, lookaheadTime, regulatedMinRadius, proximityDistance])

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
        setRegulatedMinRadius(next.regulatedMinRadius)
        setProximityDistance(next.proximityDistance)
    }

    const presetLabel = (id: PresetId): string => ({
        hairpin: t("hairpin", "hairpin"),
        narrow_gap: t("narrow gap", "narrow gap"),
        plain_pursuit: t("plain pursuit", "plain pursuit"),
    })[id]

    return (
        <LocalTracePlayer footprintRadius={FIXED.footprintRadius} showLookahead
            map={map} events={events} startPose={start} goal={preset.goal}
            referencePath={preset.path} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={() => switchPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="t_l" value={lookaheadTime} min={0.2} max={2.0} step={0.05}
                                     onCommit={setLookaheadTime}/>
                        <ParamSlider label="r_min" value={regulatedMinRadius} min={0.05} max={1.5} step={0.05}
                                     onCommit={setRegulatedMinRadius}/>
                        <ParamSlider label="d_prox" value={proximityDistance} min={0.05} max={1.2} step={0.05}
                                     onCommit={setProximityDistance}/>
                    </div>
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        {(["hairpin", "narrow_gap", "plain_pursuit"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        presetId === id
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {presetLabel(id)}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("watch the robot slow into sharp turns and near obstacles, then speed back up",
                            "급코너와 장애물 근처에서 느려졌다가 다시 속도를 회복하는 모습을 보라")}
                        {" · "}{t("plain pursuit turns the regulations off for comparison",
                            "plain pursuit는 비교를 위해 규제를 껐다")}
                    </div>
                </div>
            }
        />
    )
}

const RegulatedPurePursuitSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Regulated Pure Pursuit: raise r_min or d_prox to see the robot brake earlier into corners and near obstacles — the plain pursuit preset turns both regulations off for comparison",
            "라이브 Regulated Pure Pursuit. r_min이나 d_prox를 올리면 코너와 장애물 근처에서 더 일찍 감속하는 것을 볼 수 있다. plain pursuit 프리셋은 비교를 위해 두 규제를 모두 껐다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<RegulatedPurePursuitScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <RegulatedPurePursuitScene panel={340}/>
    </CanvasFigure>
}

export default RegulatedPurePursuitSandbox

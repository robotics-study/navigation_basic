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
// 최고 속도로 회복했다가 다음 코너에서 다시 감속하는 모습이 뚜렷이 보인다. 벽이 없어
// 근접 규제는 발동하지 않는다.
const HAIRPIN_PATH: Point[] = [
    [1.2, 1.2], [1.2, 7.2], [5.2, 7.2], [5.2, 1.2], [7.8, 1.2],
]
const HAIRPIN_START: Pose = [1.2, 1.2, 0]
const HAIRPIN_GOAL: [number, number] = [7.8, 1.2]
const hairpinMap = (): GridMap => emptyGrid("hairpin", 9, 9)

// "narrow gap" 프리셋: 열린 맵 한가운데에 위/아래 장애물 블록으로 좁은 통로(폭 1.6m)를
// 만든 gate 맵. 직선 경로가 그 틈을 그대로 통과한다 -- 곡률이 거의 0이라 곡률 규제는
// 걸리지 않고 근접 규제(proximity_distance)만 순수하게 발동한다. footprint 0.3 기준
// 통로 중앙에서 clearance가 약 0.2m라 근접 규제가 확실히 발동하며, 통로 밖에서는
// 최고 속도로 회복한다. 저장소 clutter01 맵은 시작점이 벽에서 0.25m뿐이라 footprint
// 0.3에서 첫 tick부터 충돌하므로, footprint 0.3을 지키면서 근접 규제만 깔끔히 보이도록
// 이 gate 맵을 쓴다.
type Rows = string[];
const GATE_ROWS: Rows = [
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 0 0 0 0 255 255 255 255 255 255 255 0",
    "0 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0",
    "0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0 0",
]
const NARROW_GAP_PATH: Point[] = [[1.0, 5.0], [9.0, 5.0]]
const NARROW_GAP_START: Pose = [1.0, 5.0, 0]
const NARROW_GAP_GOAL: [number, number] = [9.0, 5.0]
const narrowGapMap = (): GridMap => gridFromPgmRows("gate", GATE_ROWS, 0.5)

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
// footprintRadius는 0.3으로 둔다 -- narrow_gap의 gate 통로(폭 1.6m)와 시작점(벽에서 0.5m)이
// 이 반경을 안전하게 수용하고, hairpin/plain_pursuit는 벽 없는 맵이라 영향이 없다.
const FIXED = {
    minLookahead: 0.25, maxLookahead: 1.0, minRegulatedSpeed: 0.1, collisionCheckStep: 0.05,
    maxSpeed: 0.8, maxOmega: 1.5, slowRadius: 0.5,
    controlDt: 0.1, maxSteps: 400, goalTolerance: 0.3, footprintRadius: 0.3,
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
            auxCircleRadius={proximityDistance}
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
                    <ul className="text-xs text-muted text-left max-w-[28rem] list-none space-y-0.5">
                        <li>{t("t_l — lookahead time: the aim point sits t_l seconds of travel ahead, so higher t_l looks farther when moving fast.",
                            "t_l은 lookahead 시간이다. 겨냥점을 t_l초 앞에 두어, t_l이 클수록 빠를 때 더 멀리 본다.")}</li>
                        <li>{t("r_min — the turn radius below which the robot brakes for curvature: raise it to slow into gentler corners too (the dashed lookahead circle marks the aim point).",
                            "r_min은 이 회전 반경보다 급하면 곡률 때문에 감속하는 문턱이다. 올리면 더 완만한 코너에서도 감속한다(파선 lookahead 원이 겨냥점을 표시한다).")}</li>
                        <li>{t("d_prox — obstacle clearance below which the robot brakes for proximity: raise it to start braking farther from walls (the dotted circle around the robot marks this radius).",
                            "d_prox는 이 장애물 clearance보다 가까우면 근접 감속하는 문턱이다. 올리면 벽에서 더 멀리서부터 감속한다(로봇 둘레의 점선 원이 이 반경이다).")}</li>
                    </ul>
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
                        {t("the trail fades where the robot slows: hairpin brakes into corners, narrow gap brakes at the gap, plain pursuit turns both off for comparison",
                            "속도가 줄면 궤적 색이 옅어진다. hairpin은 코너에서, narrow gap은 통로에서 감속하고, plain pursuit는 비교를 위해 두 규제를 껐다")}
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
            "Live Regulated Pure Pursuit: raise r_min or d_prox to see the robot brake earlier into corners and near obstacles — the trail fades where it slows, and the plain pursuit preset turns both regulations off for comparison",
            "라이브 Regulated Pure Pursuit. r_min이나 d_prox를 올리면 코너와 장애물 근처에서 더 일찍 감속하는 것을 볼 수 있다. 궤적은 감속 구간에서 색이 옅어지고, plain pursuit 프리셋은 비교를 위해 두 규제를 모두 껐다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<RegulatedPurePursuitScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <RegulatedPurePursuitScene panel={340}/>
    </CanvasFigure>
}

export default RegulatedPurePursuitSandbox

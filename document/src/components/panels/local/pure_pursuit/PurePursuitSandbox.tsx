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

// 추종 데모는 장애물 없는 빈 맵(0.5 m/셀)에서 경로 기하만 보여준다. pure pursuit 은
// 회피 능력이 없는 추종기라 장애물을 두면 오해를 부른다.
const openHalfGrid = (name: string): GridMap => ({...emptyGrid(name, 20, 20), resolution: 0.5})

// 코너가 촘촘한 지그재그 경로. 두 프리셋이 이 한 경로를 공유하고 L_d(lookahead)만
// 바꿔, lookahead 트레이드오프를 나란히 대비시킨다. start heading은 첫 구간(위로 향함)에
// 맞춰 초기 heading 오차 없이 시작한다.
const CORNER_PATH: Point[] = [[1, 1], [1, 7], [4, 7], [4, 2], [7, 2], [7, 8], [9, 8]]
const CORNER_START: Pose = [1, 1, Math.PI / 2]
const CORNER_GOAL: [number, number] = [9, 8]

// 짧은/긴 L_d는 엔진 스윕으로 고른 값이다. maxSpeed=0.8(yaml 기본)에서 짧은 L_d(0.2)는
// 코너를 거의 정확히 추종하되 코너마다 속도를 0.15까지 떨어뜨리고, 긴 L_d(1.1)는 속도를
// 유지한 채 코너를 크게 벌린다 (1.4 이상은 되돌아오는 코너에서 lookahead 가
// 다음 구간으로 건너뛰어 제자리 orbit 이 생긴다). clamp-recompute 폐루프라 짧은 L_d에서도
// 오버슈트로 발산하지 않아, 대비는 "바짝 추종+감속" 대 "부드럽지만 코너 컷"으로 나타난다.
const SHORT_LOOKAHEAD = 0.2
const LONG_LOOKAHEAD = 1.1

type PresetId = "short" | "long";

interface Preset { lookahead: number }

const PRESETS: Record<PresetId, Preset> = {
    short: {lookahead: SHORT_LOOKAHEAD},
    long: {lookahead: LONG_LOOKAHEAD},
}

// configs/local_planning/pure_pursuit.yaml의 공유 폐루프 시뮬레이터 블록 기본값
// (lookahead_distance만 sandbox에서 조절한다).
const SIM_DEFAULTS = {
    maxSpeed: 0.8, maxOmega: 1.5, slowRadius: 0.5, controlDt: 0.1, maxSteps: 1000,
    // goalTolerance 는 긴 L_d 프리셋에서도 goal 을 스쳐 지나 주위를 도는 orbit 이 생기지
    // 않는 값 — lookahead 원이 goal 을 물고 있는 동안 종료 판정이 잡히게 한다.
    goalTolerance: 0.55, footprintRadius: 0.35, stallWindow: 20, stallDistance: 0.05,
}

const PurePursuitScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("short")
    const [map, setMap] = useState<GridMap>(() => openHalfGrid("corners"))
    const [start, setStart] = useState<Pose>(CORNER_START)
    const [lookahead, setLookahead] = useState(SHORT_LOOKAHEAD)

    const events = useMemo(() => runPurePursuit({
        map, startPose: start, goal: CORNER_GOAL, referencePath: CORNER_PATH,
        lookaheadDistance: lookahead, ...SIM_DEFAULTS,
    }), [map, start, lookahead])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const switchPreset = (id: PresetId) => {
        setPresetId(id)
        setMap(openHalfGrid("corners"))
        setStart(CORNER_START)
        setLookahead(PRESETS[id].lookahead)
    }

    return (
        <LocalTracePlayer footprintRadius={SIM_DEFAULTS.footprintRadius} showLookahead showHeadingRay
            map={map} events={events} startPose={start} goal={CORNER_GOAL}
            referencePath={CORNER_PATH} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={() => switchPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="L_d" value={lookahead} min={0.1} max={1.5} step={0.05}
                                     onCommit={setLookahead}/>
                        <span className="mx-1" aria-hidden="true">·</span>
                        {(["short", "long"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        presetId === id
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border hover:bg-surface",
                                    )}>
                                {id === "short" ? t("short L_d", "짧은 L_d") : t("long L_d", "긴 L_d")}
                            </button>
                        ))}
                    </div>
                    <div className="text-xs text-muted text-center max-w-[26rem]">
                        {t("L_d = how far ahead the robot aims. Raise it for a smoother, faster line that cuts corners wide; lower it to hug the path but slow sharply into each corner.",
                            "L_d는 로봇이 앞을 얼마나 멀리 겨냥하는지다. 올리면 부드럽고 빠르지만 코너를 크게 벌리고, 내리면 경로에 바짝 붙되 코너마다 크게 감속한다.")}
                    </div>
                    <div className="text-xs text-muted text-center max-w-[26rem]">
                        {t("teal arrow = current heading · blue dashed line = aim at the lookahead point; the gap between them is the angle α that sets the commanded curvature",
                            "청록 화살표 = 현재 heading · 파란 점선 = lookahead 점 겨냥. 둘 사이 벌어진 각이 명령 곡률을 정하는 α다.")}
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t("short L_d hugs every corner and slows into it · long L_d stays fast but cuts corners wide",
                            "짧은 L_d는 코너마다 바짝 붙어 감속하고, 긴 L_d는 빠르지만 코너를 크게 벌린다")}
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
            "Live Pure Pursuit: the two presets share one path and differ only in L_d — short L_d hugs every corner (slowing into it), long L_d stays smooth and fast but cuts corners wide",
            "라이브 Pure Pursuit. 두 프리셋은 한 경로를 공유하고 L_d만 다르다. 짧은 L_d는 코너마다 바짝 붙어 감속하고, 긴 L_d는 부드럽고 빠르지만 코너를 크게 벌린다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<PurePursuitScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <PurePursuitScene panel={340}/>
    </CanvasFigure>
}

export default PurePursuitSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import VelocityObstaclePlayer from "../VelocityObstaclePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runVo} from "../../../../libs/algorithms/vo";
import {crossingAgents, headOnAgents, openArenaMap} from "../velocity/scenarios";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// configs/local_planning/vo.yaml 기본값 -- sandbox 슬라이더가 없는 나머지 파라미터는
// 이 값으로 고정한다. max_steps는 sandbox 재생 성능을 위해 400으로 낮춘다(yaml은 800).
const MAX_OMEGA = 4.0
const HEADING_GAIN = 6.0
const AGENT_RADIUS = 0.3
const NEIGHBOR_DIST = 4.0
const OBSTACLE_RADIUS = 0.25
const SPEED_SAMPLES = 10
const ANGLE_SAMPLES = 16
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const FOOTPRINT_RADIUS = 0.3
const STALL_WINDOW = 60
const STALL_DISTANCE = 0.15

type PresetId = "crossing" | "head_on";

const PRESET_BUILDERS: Record<PresetId, (radius: number) => ReturnType<typeof crossingAgents>> = {
    crossing: crossingAgents,
    head_on: headOnAgents,
}

const VoScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("head_on")
    const [maxSpeed, setMaxSpeed] = useState(1.2)
    const [timeHorizon, setTimeHorizon] = useState(2.0)
    const map = useMemo(openArenaMap, [])
    const agents = useMemo(() => PRESET_BUILDERS[presetId](AGENT_RADIUS), [presetId])

    const events = useMemo(() => runVo({
        map, agents, maxSpeed, maxOmega: MAX_OMEGA, headingGain: HEADING_GAIN, agentRadius: AGENT_RADIUS,
        neighborDist: NEIGHBOR_DIST, timeHorizon, obstacleRadius: OBSTACLE_RADIUS,
        speedSamples: SPEED_SAMPLES, angleSamples: ANGLE_SAMPLES, controlDt: CONTROL_DT, maxSteps: MAX_STEPS,
        goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW,
        stallDistance: STALL_DISTANCE,
    }), [map, agents, maxSpeed, timeHorizon])

    const presetLabel = (id: PresetId): string => ({
        crossing: t("crossing", "교차"),
        head_on: t("head-on", "정면 마주침"),
    })[id]

    return (
        <VelocityObstaclePlayer
            map={map} events={events} panel={panel} maxSpeedScale={maxSpeed}
            agents={agents.map((a) => ({start: a.start.pose, goal: a.goal, radius: a.radius}))}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs">
                        {(["crossing", "head_on"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => setPresetId(id)}
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
                        <ParamSlider label={t("max speed", "max speed")} value={maxSpeed}
                                     min={0.4} max={1.8} step={0.05} onCommit={setMaxSpeed}/>
                        <ParamSlider label="τ" value={timeHorizon}
                                     min={0.3} max={4.0} step={0.1} onCommit={setTimeHorizon}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "τ — how many seconds ahead the cone is truncated; larger reacts earlier but shrinks the reachable set more",
                            "τ: 원뿔을 잘라내는 시간 지평. 크면 더 일찍 반응하지만 그만큼 도달 가능 집합을 더 좁힌다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "crossing shows the honest single-robot case; head-on shows both agents mirroring each other's evasive swerve every tick",
                            "crossing은 정직한 단일 로봇 케이스를 보여주고, head-on은 두 agent가 매 tick 서로의 회피 동작을 거울처럼 따라 하는 모습을 보여준다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const VoSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live VO: the crossing preset shows the honest single-robot-vs-mover case, and head-on shows two agents facing off nearly symmetrically — watch the ego inset's chosen velocity (solid) swerve away from the preferred one (dashed) as the cone reappears tick after tick",
            "라이브 VO. crossing 프리셋은 정직한 단일 로봇 대 이동장애물 케이스를, head-on은 거의 대칭으로 마주 보는 두 agent를 보여준다. 매 tick 원뿔이 다시 나타날 때마다 ego inset의 선택 속도(실선)가 선호 속도(파선)에서 벗어나는 모습을 보라",
        )}
        tight bodyClassName="w-fit max-w-full" className="w-full"
        modal={<VoScene panel={Math.min(modalCanvasSize(1).width, 520)}/>}
    >
        <VoScene panel={300}/>
    </CanvasFigure>
}

export default VoSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import VelocityObstaclePlayer from "../VelocityObstaclePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runRvo} from "../../../../libs/algorithms/rvo";
import {circleSwapAgents, headOnAgents, openArenaMap} from "../velocity/scenarios";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// configs/local_planning/rvo.yaml 기본값 -- sandbox 슬라이더가 없는 나머지 파라미터는
// 이 값으로 고정한다. max_steps는 sandbox 재생 성능을 위해 400으로 낮춘다(yaml은 800).
const MAX_OMEGA = 4.0
const HEADING_GAIN = 6.0
const AGENT_RADIUS = 0.3
const NEIGHBOR_DIST = 4.0
const TIME_HORIZON = 2.0
const OBSTACLE_RADIUS = 0.25
const SPEED_SAMPLES = 10
const ANGLE_SAMPLES = 16
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const FOOTPRINT_RADIUS = 0.3
const STALL_WINDOW = 60
const STALL_DISTANCE = 0.15

type PresetId = "head_on" | "circle_swap";

const PRESET_BUILDERS: Record<PresetId, (radius: number) => ReturnType<typeof headOnAgents>> = {
    head_on: headOnAgents,
    circle_swap: circleSwapAgents,
}

const RvoScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("head_on")
    const [maxSpeed, setMaxSpeed] = useState(1.2)
    const [reciprocity, setReciprocity] = useState(0.5)
    const map = useMemo(openArenaMap, [])
    const agents = useMemo(() => PRESET_BUILDERS[presetId](AGENT_RADIUS), [presetId])

    const events = useMemo(() => runRvo({
        map, agents, maxSpeed, maxOmega: MAX_OMEGA, headingGain: HEADING_GAIN, agentRadius: AGENT_RADIUS,
        neighborDist: NEIGHBOR_DIST, timeHorizon: TIME_HORIZON, obstacleRadius: OBSTACLE_RADIUS,
        speedSamples: SPEED_SAMPLES, angleSamples: ANGLE_SAMPLES, reciprocity, controlDt: CONTROL_DT,
        maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, agents, maxSpeed, reciprocity])

    const presetLabel = (id: PresetId): string => ({
        head_on: t("head-on", "정면 마주침"),
        circle_swap: t("circle swap", "원형 스왑"),
    })[id]

    return (
        <VelocityObstaclePlayer
            map={map} events={events} panel={panel} maxSpeedScale={maxSpeed}
            agents={agents.map((a) => ({start: a.start.pose, goal: a.goal, radius: a.radius}))}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs">
                        {(["head_on", "circle_swap"] as const).map((id) => (
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
                        <ParamSlider label="reciprocity" value={reciprocity}
                                     min={0} max={1} step={0.05} onCommit={setReciprocity}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "reciprocity — 0 recovers plain VO (watch the head-on oscillation return), 0.5 splits the avoidance evenly, 1 pushes the whole cone onto this agent alone",
                            "reciprocity: 0이면 VO 그대로(head-on 진동이 되살아난다), 0.5는 회피를 절반씩 나누고, 1은 원뿔 전체를 이 agent 혼자 짊어진다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "head-on now resolves smoothly at reciprocity 0.5; circle swap shows four agents crossing without deadlocking",
                            "head-on은 reciprocity 0.5에서 부드럽게 풀리고, circle swap은 네 agent가 교착 없이 교차하는 모습을 보여준다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const RvoSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live RVO: drag reciprocity down to 0 on the head-on preset and watch VO's oscillation come back, or up to 0.5 and watch it settle — circle swap shows four-body reciprocal avoidance resolving without a deadlock",
            "라이브 RVO. head-on 프리셋에서 reciprocity를 0으로 내리면 VO의 진동이 되살아나고, 0.5로 올리면 가라앉는다. circle swap은 네 몸체의 상호 회피가 교착 없이 풀리는 모습을 보여준다",
        )}
        tight bodyClassName="w-fit max-w-full" className="w-full"
        modal={<RvoScene panel={Math.min(modalCanvasSize(1).width, 520)}/>}
    >
        <RvoScene panel={300}/>
    </CanvasFigure>
}

export default RvoSandbox

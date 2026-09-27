import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import VelocityObstaclePlayer from "../VelocityObstaclePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runOrca} from "../../../../libs/algorithms/orca";
import {circleSwapAgents, headOnAgents, openArenaMap} from "../velocity/scenarios";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// configs/local_planning/orca.yaml 기본값 -- sandbox 슬라이더가 없는 나머지 파라미터는
// 이 값으로 고정한다. max_steps는 sandbox 재생 성능을 위해 400으로 낮춘다(yaml은 800).
const MAX_OMEGA = 4.0
const HEADING_GAIN = 6.0
const AGENT_RADIUS = 0.3
const NEIGHBOR_DIST = 4.0
const TIME_HORIZON_OBST = 1.0
const OBSTACLE_RADIUS = 0.25
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

const OrcaScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("head_on")
    const [maxSpeed, setMaxSpeed] = useState(1.2)
    const [timeHorizon, setTimeHorizon] = useState(1.0)
    const map = useMemo(openArenaMap, [])
    const agents = useMemo(() => PRESET_BUILDERS[presetId](AGENT_RADIUS), [presetId])

    const events = useMemo(() => runOrca({
        map, agents, maxSpeed, maxOmega: MAX_OMEGA, headingGain: HEADING_GAIN, agentRadius: AGENT_RADIUS,
        neighborDist: NEIGHBOR_DIST, timeHorizon, timeHorizonObst: TIME_HORIZON_OBST,
        obstacleRadius: OBSTACLE_RADIUS, controlDt: CONTROL_DT, maxSteps: MAX_STEPS,
        goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW,
        stallDistance: STALL_DISTANCE,
    }), [map, agents, maxSpeed, timeHorizon])

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
                        <ParamSlider label="τ" value={timeHorizon}
                                     min={0.3} max={3.0} step={0.1} onCommit={setTimeHorizon}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "τ — the ORCA time horizon for other agents; the ego inset now shows a single half-plane line per neighbor instead of a sampled cone",
                            "τ: 다른 agent에 대한 ORCA 시간 지평. ego inset에는 이제 표본화된 원뿔 대신 이웃마다 half-plane 선 하나가 뜬다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "both presets resolve every time — the linear program always returns a velocity, feasible or the closest penetration-minimizing one",
                            "두 프리셋 모두 매번 풀린다. 선형계획은 feasible하든 침투 최소화든 항상 속도 하나를 반환한다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const OrcaSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live ORCA: the ego inset now draws one exact half-plane line per neighbor instead of VO/RVO's sampled wedge, and the chosen velocity always lands exactly on its boundary or corner when constraints bind",
            "라이브 ORCA. ego inset에는 VO/RVO의 표본 쐐기 대신 이웃마다 정확한 half-plane 선 하나가 그려지고, 제약이 걸리면 선택 속도가 항상 그 경계나 모서리 위에 정확히 놓인다",
        )}
        tight bodyClassName="w-fit max-w-full" className="w-full"
        modal={<OrcaScene panel={Math.min(modalCanvasSize(1).width, 520)}/>}
    >
        <OrcaScene panel={300}/>
    </CanvasFigure>
}

export default OrcaSandbox

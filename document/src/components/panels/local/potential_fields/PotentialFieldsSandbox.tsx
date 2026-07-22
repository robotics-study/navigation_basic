import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runPotentialFields} from "../../../../libs/algorithms/potential_fields";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap, gridFromPgmRows} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// 저장소 실제 demo 맵을 그대로 재사용한다(좌표는 maps/grid/{pf_trap01,clutter01}.pgm +
// maps/scenarios/{pf_trap01_s1,clutter01_s1}.yaml 그대로) — 손으로 새 지형을 지어 균형을
// 잘못 잡을 위험 없이, 알고리즘의 알려진 함정과 성공 사례를 검증된 형태로 보여준다.
type Rows = string[];   // 각 행: 공백 구분 pixel 값(0=occupied, 255=free), pgm 그대로.


// ㄷ자 함정: 입구가 서쪽(start), 동쪽이 막힌 back wall — 직선 인력이 벽에서 stall된다.
const PF_TRAP_ROWS: Rows = [
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 0 0 0 0 0 0 0 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 0 0 0 0 0 0 0 0 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
    "255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255 255",
]
const PF_TRAP_START: Pose = [1.25, 4.75, 0]
const PF_TRAP_GOAL: [number, number] = [8.75, 4.75]

// 산개 블록 지형: 벽을 완전히 두르지 않아, 벽이 미는 힘으로 회피해 목표에 닿는다. footprint
// 0.35에 통로 폭이 넉넉하도록 6개 블록(페이지 산문의 "six scattered blocks"와 일치)을
// 2x2(1m)로 성기게 흩는다. 저장소 clutter01.pgm은 yaml 기본 footprint 0.2용이라 0.35에서는
// 통로가 좁아 벽에 스치므로, 0.3+ 규칙에 맞춰 새로 구성했다. 인력이 goal로 곧장 당기는
// 사이 벽 반발이 로봇을 블록마다 우회시키는 모습을 보여준다.
const CLUTTER_BLOCKS: [number, number][] = [
    [6, 4], [4, 9], [11, 8], [14, 12], [8, 13], [15, 7],
]
const clutterMap = (): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let r = 0; r < height; r++)
        for (let c = 0; c < width; c++)
            if (r === 0 || r === height - 1 || c === 0 || c === width - 1) occupied[r * width + c] = true
    for (const [c0, r0] of CLUTTER_BLOCKS)
        for (let r = r0; r < r0 + 2; r++) for (let c = c0; c < c0 + 2; c++) occupied[r * width + c] = true
    return {name: "clutter", width, height, occupied, resolution: 0.5, originX: 0, originY: 0}
}
const CLUTTER_START: Pose = [1.25, 1.25, 0]
const CLUTTER_GOAL: [number, number] = [8.75, 8.75]

type Preset = "trap" | "bypass";

const presetMap = (preset: Preset): GridMap => preset === "trap"
    ? gridFromPgmRows("pf_trap01", PF_TRAP_ROWS, 0.5)
    : clutterMap()
const presetStart = (preset: Preset): Pose => preset === "trap" ? PF_TRAP_START : CLUTTER_START
const presetGoal = (preset: Preset): [number, number] => preset === "trap" ? PF_TRAP_GOAL : CLUTTER_GOAL

// configs/local_planning/potential_fields.yaml 기본값 — sandbox 슬라이더가 없는 나머지
// 파라미터(k_v, k_omega, max_speed, max_omega)와 sim 6종은 이 값으로 고정한다.
const K_V = 1.0
const K_OMEGA = 4.0
const MAX_SPEED = 0.8
const MAX_OMEGA = 5.0
const FOOTPRINT_RADIUS = 0.35
const CONTROL_DT = 0.1
const MAX_STEPS = 1000
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const PotentialFieldsScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [preset, setPreset] = useState<Preset>("trap")
    const [map, setMap] = useState<GridMap>(() => presetMap("trap"))
    const [start, setStart] = useState<Pose>(PF_TRAP_START)
    const [goal, setGoal] = useState<[number, number]>(PF_TRAP_GOAL)
    const [kAtt, setKAtt] = useState(0.5)
    const [kRep, setKRep] = useState(3.0)
    // 영향 반경 밖 장애물은 반발력에 잡히지 않는다. 기본값을 키워 벽을 그렸을 때 힘이
    // 곧바로 반응하는 것이 보이게 하고, bypass 프리셋이 산개 블록을 스치지 않고 돌게 한다.
    const [influenceRadius, setInfluenceRadius] = useState(1.0)

    const events = useMemo(() => runPotentialFields({
        map, start, goal,
        kAtt, kRep, influenceRadius,
        kV: K_V, kOmega: K_OMEGA, maxSpeed: MAX_SPEED, maxOmega: MAX_OMEGA,
        footprintRadius: FOOTPRINT_RADIUS,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, goal, kAtt, kRep, influenceRadius])

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
            map={map} events={events} panel={panel}
            startPose={start} goal={goal}
            auxCircleRadius={influenceRadius}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], 0])}
            onMoveGoal={setGoal}
            onReset={() => applyPreset(preset)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs">
                        {(["trap", "bypass"] as const).map((p) => (
                            <button key={p} type="button" onClick={() => applyPreset(p)}
                                    className={cn(
                                        "px-2 py-0.5 rounded border",
                                        preset === p
                                            ? "border-[var(--accent)] text-[var(--accent)] font-semibold"
                                            : "border-border text-muted hover:bg-surface",
                                    )}>
                                {p === "trap"
                                    ? t("U-trap", "U-trap 갇힘")
                                    : t("bypass", "우회 탈출")}
                            </button>
                        ))}
                    </div>
                    <div className="flex items-center justify-center gap-1.5 text-xs text-muted flex-wrap tabular-nums">
                        <ParamSlider label="k_att" value={kAtt} min={0.05} max={2} step={0.05} onCommit={setKAtt}/>
                        <ParamSlider label="k_rep" value={kRep} min={0} max={10} step={0.1} onCommit={setKRep}/>
                        <ParamSlider label={t("ρ₀", "ρ₀")} value={influenceRadius} min={0.1} max={2}
                                     step={0.05} onCommit={setInfluenceRadius}/>
                    </div>
                    <div className="flex flex-col gap-0.5 text-[11px] text-muted text-left max-w-[20rem]">
                        <span>{t(
                            "k_att — pull toward the goal; raise it to head straight for the goal and detour less",
                            "k_att: goal로 당기는 인력. 올리면 goal로 곧장 향하고 덜 우회한다",
                        )}</span>
                        <span>{t(
                            "k_rep — push from walls; raise it to swing wider, too high and a narrow gap traps it as both sides cancel",
                            "k_rep: 벽이 미는 반발력. 올리면 더 크게 돌고, 과하면 좁은 틈에서 양쪽 반발이 상쇄돼 갇힌다",
                        )}</span>
                        <span>{t(
                            "ρ₀ — the dashed circle where repulsion acts; walls outside it exert no force, so raise it to react sooner",
                            "ρ₀: 반발이 작동하는 점선 원. 이 원 밖 장애물은 힘에 안 잡히므로, 올리면 더 일찍 반응한다",
                        )}</span>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "drag start/goal or paint walls, then compare the U-trap against the clutter field",
                            "start/goal를 끌거나 벽을 그려 보라. U-trap과 산개 지형을 비교해 보라",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const PotentialFieldsSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Potential Fields: raise k_rep and the U-trap traps it for good — the resultant force hits zero before the goal does, so the robot honestly stops",
            "라이브 Potential Fields. k_rep를 올리면 U-trap에서 완전히 갇힌다. 목표에 닿기 전에 합력이 0이 되어, 로봇이 정직하게 멈춘다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<PotentialFieldsScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <PotentialFieldsScene panel={340}/>
    </CanvasFigure>
}

export default PotentialFieldsSandbox

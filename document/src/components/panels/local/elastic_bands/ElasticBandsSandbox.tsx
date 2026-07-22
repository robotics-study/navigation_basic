import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import ParamSlider from "../../../player/ParamSlider";
import {runElasticBands} from "../../../../libs/algorithms/elastic_bands";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";
import cn from "../../../../libs/cn";

// 두 프리셋 모두 20x20 @ 0.5m 해상도(10m x 10m 방) -- 경계 한 칸은 항상 벽이고, 내부
// 장애물은 world 좌표 술어로 직접 찍는다(DwaSandbox의 sudden_wall처럼 GridMap을 손으로
// 짓는 관례). footprintRadius 0.3(지름 0.6) 규칙에 맞춰 모든 통로 폭을 0.9m 이상으로
// 잡는다.
const RES = 0.5
const buildGrid = (name: string, width: number, height: number, isWall: (x: number, y: number) => boolean): GridMap => {
    const occupied = new Array(width * height).fill(false)
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            if (row === 0 || row === height - 1 || col === 0 || col === width - 1) {
                occupied[row * width + col] = true
                continue
            }
            const x = (col + 0.5) * RES
            const y = (height - 1 - row + 0.5) * RES
            occupied[row * width + col] = isWall(x, y)
        }
    }
    return {name, width, height, occupied, resolution: RES, originX: 0, originY: 0}
}

// "Blocked path": 곧은 reference path(y=5.0)의 중앙에 장애물 블록을 놓는다. 블록을
// 진행 방향(x)으로 한 칸(0.5m)만 좁게, 옆(y)으로는 길게 잡는다 -- 진행 방향으로 두꺼운
// 블록(1m x 1m)을 곧장 관통시키면 bubble 여럿이 한꺼번에 내부에 갇혀 repair 반복으로도
// 못 빠져나오는 것을 실행 검증으로 확인했다(밀도 높은 solid 구간이 길수록 이웃
// bubble끼리 서로 다시 끌어당겨 탈출을 방해한다). 블록을 y로 대칭이 아니라 위로
// 치우치게 걸쳐 놓아(§1.2의 대칭 정지점 회피) 첫 반복부터 아래쪽으로 밀리게 한다.
// 시작/목표는 경계 벽에서 안쪽으로 두 칸(1.25m, 벽면까지 0.75m 여유) 떨어뜨린다 --
// 한 칸(0.75m, 벽면까지 0.25m)이면 footprintRadius 0.3보다 벽까지 여유가 좁아 첫
// tick부터 즉시 충돌 처리된다(discCollides는 로봇 중심-셀 사각형 최근접점 거리로
// 판정하므로, 셀 하나 두께의 경계벽에서 0.25m는 0.3m 반경보다 작다).
const blockedPathMap = (): GridMap => buildGrid("eb_blocked", 20, 20, (x, y) =>
    x >= 4.75 && x < 5.25 && y >= 4.4 && y < 5.9)
const BLOCKED_PATH: Point[] = [[1.25, 5.0], [8.75, 5.0]]
const BLOCKED_START: Pose = [1.25, 5.0, 0]
const BLOCKED_GOAL: [number, number] = [8.75, 5.0]

// "Narrow gap": x=[4.5,5.5) 세로 벽 하나에 셀 경계에 맞춘 틈을 뚫어 둔다. 점유 판정은
// 셀 중심 단위로 양자화되므로 GAP_LO/GAP_HI를 셀 경계(0.5의 배수)에 맞추면 틈 폭이
// 항상 정확히 두 칸(1.0m, 0.9m 규칙에 여유)이 된다.
const GAP_LO = 4.5, GAP_HI = 5.5
const narrowGapMap = (): GridMap => buildGrid("eb_gap", 20, 20, (x, y) =>
    x >= 4.5 && x < 5.5 && !(y >= GAP_LO && y < GAP_HI))
const NARROW_GAP_PATH: Point[] = [[1.25, 5.0], [8.75, 5.0]]
const NARROW_GAP_START: Pose = [1.25, 5.0, 0]
const NARROW_GAP_GOAL: [number, number] = [8.75, 5.0]

type PresetId = "blocked_path" | "narrow_gap";

interface Preset { map: () => GridMap; path: Point[]; start: Pose; goal: [number, number] }

const PRESETS: Record<PresetId, Preset> = {
    blocked_path: {map: blockedPathMap, path: BLOCKED_PATH, start: BLOCKED_START, goal: BLOCKED_GOAL},
    narrow_gap: {map: narrowGapMap, path: NARROW_GAP_PATH, start: NARROW_GAP_START, goal: NARROW_GAP_GOAL},
}

// configs/local_planning/elastic_bands.yaml 기본값 -- 슬라이더가 없는 나머지 파라미터와
// sim 6종은 이 값으로 고정한다. max_steps는 yaml 기본(1000)보다 낮춰 sandbox 재생
// 이벤트 수를 억제한다(다른 local sandbox와 같은 관례).
const K_CONTRACTION = 1.0
const RHO_MAX = 1.5
const RHO_MIN = 0.35
const STEP_SIZE = 0.2
const DEFORM_ITERATIONS = 5
const REPAIR_ITERATIONS = 20
const REPAIR_STEP = 0.2
const OVERLAP_FACTOR = 0.7
const MAX_BUBBLES = 80
const BUBBLE_SPACING = 0.4
const LOOKAHEAD_DISTANCE = 0.7
const HEADING_GAIN = 2.0
const V_MAX = 0.8
const OMEGA_MAX = 1.5
const FOOTPRINT_RADIUS = 0.3
const CONTROL_DT = 0.1
const MAX_STEPS = 400
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const ElasticBandsScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [presetId, setPresetId] = useState<PresetId>("blocked_path")
    const preset = PRESETS[presetId]
    const [map, setMap] = useState<GridMap>(preset.map)
    const [start, setStart] = useState<Pose>(preset.start)
    const [kRepulsion, setKRepulsion] = useState(2.0)
    const [rhoInfluence, setRhoInfluence] = useState(1.0)

    const events = useMemo(() => runElasticBands({
        map, startPose: start, goal: preset.goal, referencePath: preset.path,
        kContraction: K_CONTRACTION, kRepulsion, rhoMax: RHO_MAX, rhoInfluence, rhoMin: RHO_MIN,
        stepSize: STEP_SIZE, deformIterations: DEFORM_ITERATIONS, repairIterations: REPAIR_ITERATIONS,
        repairStep: REPAIR_STEP, overlapFactor: OVERLAP_FACTOR, maxBubbles: MAX_BUBBLES,
        bubbleSpacing: BUBBLE_SPACING, lookaheadDistance: LOOKAHEAD_DISTANCE, headingGain: HEADING_GAIN,
        maxSpeed: V_MAX, maxOmega: OMEGA_MAX, controlDt: CONTROL_DT, maxSteps: MAX_STEPS,
        goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, preset, kRepulsion, rhoInfluence])

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
    }

    const presetLabel = (id: PresetId): string => ({
        blocked_path: t("blocked path", "blocked path"),
        narrow_gap: t("narrow gap", "narrow gap"),
    })[id]

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS}
            map={map} events={events} startPose={start} goal={preset.goal}
            referencePath={preset.path} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={() => switchPreset(presetId)}
            footer={
                <div className="flex flex-col items-center gap-1.5">
                    <div className="flex items-center justify-center gap-1.5 text-xs">
                        {(["blocked_path", "narrow_gap"] as const).map((id) => (
                            <button key={id} type="button" onClick={() => switchPreset(id)}
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
                        <ParamSlider label="k_repulsion" value={kRepulsion}
                                     min={0.5} max={5.0} step={0.1} onCommit={setKRepulsion}/>
                        <ParamSlider label="rho_influence" value={rhoInfluence}
                                     min={0.3} max={2.0} step={0.05} onCommit={setRhoInfluence}/>
                    </div>
                    <div className="text-xs text-muted text-center tabular-nums">
                        {t(
                            "draw a wall across the band mid-flight and watch it deform live, or erase it to see the band recover",
                            "재생 중에 밴드 위로 벽을 그려 실시간으로 밀려나는 모습을 보거나, 지워서 밴드가 회복되는 모습을 보라",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const ElasticBandsSandbox = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Live Elastic Bands: the translucent chain of bubbles is the band deforming this tick — raise k_repulsion to push it farther from obstacles, or paint a wall across it mid-replay to watch it shove the band aside in real time",
            "라이브 Elastic Bands. 반투명 bubble 사슬이 이번 tick 변형된 밴드다. k_repulsion을 올리면 장애물에서 더 멀리 밀려나고, 재생 도중 밴드 위에 벽을 그리면 실시간으로 밀려나는 모습을 볼 수 있다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<ElasticBandsScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <ElasticBandsScene panel={340}/>
    </CanvasFigure>
}

export default ElasticBandsSandbox

import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import {runElasticBands} from "../../../../libs/algorithms/elastic_bands";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {Point} from "../../../../libs/algorithms/sampling_space";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// 대칭 정체 약점 전용 데모: 밴드(y=5.0)를 정확히 반으로 가르는 블록. 점유 셀 중심이
// y=4.75/5.25 두 줄이라 셀 합산 반발이 밴드 중심선 위에서 위아래로 정확히 상쇄되고,
// 변형 힘이 0이 되어 밴드가 블록에서 빠져나오지 못한다 — 실행 검증: STALLED. 셀 하나만
// 칠해 대칭을 깨면 같은 파라미터로 REACHED가 된다(메인 데모의 블록을 비대칭으로 걸쳐
// 놓은 이유이기도 하다). '정확한' 대칭이 필요하므로 블록 경계는 셀 경계(0.5 배수)에
// 맞춘다 — 반 칸이라도 어긋나면 합산이 상쇄되지 않는다.
const RES = 0.5
const symmetricMap = (): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let row = 0; row < height; row++) {
        for (let col = 0; col < width; col++) {
            if (row === 0 || row === height - 1 || col === 0 || col === width - 1) {
                occupied[row * width + col] = true
                continue
            }
            const x = (col + 0.5) * RES
            const y = (height - 1 - row + 0.5) * RES
            occupied[row * width + col] = x >= 4.75 && x < 5.25 && y >= 4.5 && y < 5.5
        }
    }
    return {name: "eb_symmetric", width, height, occupied, resolution: RES, originX: 0, originY: 0}
}
const REF_PATH: Point[] = [[1.25, 5.0], [8.75, 5.0]]
const START: Pose = [1.25, 5.0, 0]
const GOAL: [number, number] = [8.75, 5.0]

// configs/local_planning/elastic_bands.yaml 기본값 + 메인 sandbox 슬라이더 시작값 —
// 이 데모는 시나리오만 다르고 파라미터는 동일해야 "대칭"만 변수가 된다.
const K_CONTRACTION = 1.0
const K_REPULSION = 2.0
const RHO_MAX = 1.5
const RHO_MIN = 0.35
const RHO_INFLUENCE = 1.0
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

const SymmetryStallScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(symmetricMap)
    const [start, setStart] = useState<Pose>(START)

    const events = useMemo(() => runElasticBands({
        map, startPose: start, goal: GOAL, referencePath: REF_PATH,
        kContraction: K_CONTRACTION, kRepulsion: K_REPULSION, rhoMax: RHO_MAX,
        rhoInfluence: RHO_INFLUENCE, rhoMin: RHO_MIN,
        stepSize: STEP_SIZE, deformIterations: DEFORM_ITERATIONS, repairIterations: REPAIR_ITERATIONS,
        repairStep: REPAIR_STEP, overlapFactor: OVERLAP_FACTOR, maxBubbles: MAX_BUBBLES,
        bubbleSpacing: BUBBLE_SPACING, lookaheadDistance: LOOKAHEAD_DISTANCE, headingGain: HEADING_GAIN,
        maxSpeed: V_MAX, maxOmega: OMEGA_MAX, controlDt: CONTROL_DT, maxSteps: MAX_STEPS,
        goalTolerance: GOAL_TOLERANCE, footprintRadius: FOOTPRINT_RADIUS,
        stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const reset = () => {
        setMap(symmetricMap())
        setStart(START)
    }

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS}
            map={map} events={events} startPose={start} goal={GOAL}
            referencePath={REF_PATH} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onReset={reset}
            footer={
                <div className="flex flex-col items-center gap-1.5 text-xs text-muted text-center">
                    <div className="max-w-[24rem]">
                        {t(
                            "the block splits the band dead-center, so the summed cell repulsion cancels exactly on the centerline: the deformation force is zero, the band never escapes the block, and the robot stalls at the start",
                            "블록이 밴드를 정확히 반으로 가르므로 셀 합산 반발이 중심선 위에서 정확히 상쇄된다. 변형 힘이 0이라 밴드가 블록에서 빠져나오지 못하고, 로봇은 출발점에서 정체된다",
                        )}
                    </div>
                    <div className="max-w-[24rem]">
                        {t(
                            "paint a single extra cell on either side of the block and replay: the symmetry breaks and the same parameters reach the goal",
                            "블록 위나 아래에 셀 하나만 더 칠하고 다시 재생해 보라. 대칭이 깨지고, 같은 파라미터로 goal에 닿는다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const EbSymmetryStallDemo = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Elastic Bands' honest weakness: a band pierced dead-center by an obstacle feels zero net repulsion on the centerline and stalls — one painted cell of asymmetry frees it",
            "Elastic Bands의 정직한 약점. 장애물이 밴드를 정확히 반으로 가르면 중심선의 합산 반발이 0이 되어 정체한다. 셀 하나만큼의 비대칭이면 풀려난다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<SymmetryStallScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <SymmetryStallScene panel={340}/>
    </CanvasFigure>
}

export default EbSymmetryStallDemo

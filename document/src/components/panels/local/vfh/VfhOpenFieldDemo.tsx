import {useMemo, useState} from "react";
import CanvasFigure, {modalCanvasSize} from "../../../CanvasFigure";
import LocalTracePlayer from "../LocalTracePlayer";
import {runVfh} from "../../../../libs/algorithms/vfh";
import {Pose} from "../../../../libs/algorithms/local_sim";
import {GridMap} from "../../../../libs/grid";
import {useTr} from "../../../../libs/i18n";

// 개활지 지그재그 약점 전용 데모: 경계 벽뿐인 빈 방을 정동쪽으로 가로지르는 최악
// 케이스만 고정해 보여준다. 전 sector가 자유이면 valley가 [0..n-1]로 한 바퀴 감기며
// 인위적 경계가 seam(sector 0/n-1)에 생기고, 정동쪽 goal bearing이 바로 그 경계에 걸려
// goal-직진 조건이 실패한다. 경계 기준 wide-valley 조향이 tick마다 양쪽으로 뒤집혀
// 로봇이 지그재그로 긴다(cos-gate가 전진도 깎는다). 실측: 같은 7.5m 직선이 경로 옆에
// 벽을 그려 주면 165 tick·ω 부호 반전 27회, 완전 개활지에서는 408 tick·반전 100회다.
const RES = 0.5
const openMap = (): GridMap => {
    const width = 20, height = 20
    const occupied = new Array(width * height).fill(false)
    for (let r = 0; r < height; r++)
        for (let c = 0; c < width; c++)
            if (r === 0 || r === height - 1 || c === 0 || c === width - 1) occupied[r * width + c] = true
    return {name: "vfh_open", width, height, occupied, resolution: RES, originX: 0, originY: 0}
}
const START: Pose = [1.25, 5.0, 0]
const GOAL: [number, number] = [8.75, 5.0]

// configs/local_planning/vfh.yaml의 고정 게인 + 메인 sandbox와 같은 데모용
// threshold/window 시작값 — 이 데모는 시나리오만 다르고 파라미터는 동일해야
// "지형 차이"만 보인다.
const NUM_SECTORS = 60
const WINDOW_RADIUS = 1.8
const THRESHOLD = 0.02
const SMOOTHING_WINDOW = 3
const WIDE_VALLEY_SECTORS = 20
const H_M = 0.0905
const K_OMEGA = 1.54
const MAX_SPEED = 0.44
const MAX_OMEGA = 2.86
const FOOTPRINT_RADIUS = 0.3
const CONTROL_DT = 0.1
const MAX_STEPS = 1000
const GOAL_TOLERANCE = 0.3
const STALL_WINDOW = 20
const STALL_DISTANCE = 0.05

const OpenFieldScene = ({panel = 340}: {panel?: number}) => {
    const t = useTr()
    const [map, setMap] = useState<GridMap>(openMap)
    const [start, setStart] = useState<Pose>(START)
    const [goal, setGoal] = useState<[number, number]>(GOAL)

    const events = useMemo(() => runVfh({
        map, start, goal,
        numSectors: NUM_SECTORS, windowRadius: WINDOW_RADIUS, threshold: THRESHOLD,
        smoothingWindow: SMOOTHING_WINDOW, wideValleySectors: WIDE_VALLEY_SECTORS, hM: H_M,
        kOmega: K_OMEGA, maxSpeed: MAX_SPEED, maxOmega: MAX_OMEGA,
        controlDt: CONTROL_DT, maxSteps: MAX_STEPS, goalTolerance: GOAL_TOLERANCE,
        footprintRadius: FOOTPRINT_RADIUS, stallWindow: STALL_WINDOW, stallDistance: STALL_DISTANCE,
    }), [map, start, goal])

    const paintCell = (row: number, col: number, occupied: boolean) => {
        setMap((prev) => {
            const next = {...prev, occupied: [...prev.occupied]}
            next.occupied[row * prev.width + col] = occupied
            return next
        })
    }
    const reset = () => {
        setMap(openMap())
        setStart(START)
        setGoal(GOAL)
    }

    return (
        <LocalTracePlayer footprintRadius={FOOTPRINT_RADIUS} durationMs={8000}
            map={map} events={events} startPose={start} goal={goal} panel={panel}
            onPaintCell={paintCell}
            onMoveStart={(xy) => setStart([xy[0], xy[1], start[2]])}
            onMoveGoal={setGoal}
            onReset={reset}
            footer={
                <div className="flex flex-col items-center gap-1.5 text-xs text-muted text-center">
                    <div className="max-w-[24rem]">
                        {t(
                            "nothing in the window pins the valley, so the steering target flickers across the sector grid every tick — the trail zigzags and the run takes about 2.4x the ticks it needs beside a wall",
                            "window 안에 valley를 고정해 줄 것이 없어 조향 목표가 매 tick sector 격자 위에서 흔들린다. trail이 지그재그가 되고, 벽 옆에서 달릴 때의 2.4배쯤 tick을 쓴다",
                        )}
                    </div>
                    <div className="max-w-[24rem]">
                        {t(
                            "draw a wall one meter beside the route and watch the same run settle down and finish in far fewer ticks",
                            "경로 옆 1m쯤에 벽을 한 줄 그려 보라. 같은 주행이 차분해지며 훨씬 적은 tick으로 끝난다",
                        )}
                    </div>
                </div>
            }
        />
    )
}

const VfhOpenFieldDemo = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "VFH's honest failure case: crossing an empty room straight toward the goal, the sector-quantized steering target flips every tick and the robot zigzags and crawls — the oscillation VFH+ later damped with hysteresis",
            "VFH의 정직한 실패 사례. 빈 방을 goal 쪽으로 곧장 가로지르면 sector로 양자화된 조향 목표가 매 tick 뒤집혀 로봇이 지그재그로 기어간다. VFH+가 나중에 hysteresis로 억제한 바로 그 진동이다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<OpenFieldScene panel={Math.min(modalCanvasSize(1).width, 640)}/>}
    >
        <OpenFieldScene panel={340}/>
    </CanvasFigure>
}

export default VfhOpenFieldDemo

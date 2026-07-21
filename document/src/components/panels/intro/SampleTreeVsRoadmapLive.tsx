import {useMemo} from "react";
import CanvasFigure, {modalCanvasSize} from "../../CanvasFigure";
import DualReplay from "./DualReplay";
import {runRRT} from "../../../libs/algorithms/rrt";
import {runPRM} from "../../../libs/algorithms/prm";
import {buildGridTimeline} from "../../../libs/trace/timeline";
import {emptyGrid, GridMap} from "../../../libs/grid";
import {Point} from "../../../libs/algorithms/sampling_space";
import {useTr} from "../../../libs/i18n";

// 같은 맵·같은 표본원에서 두 방식이 자란다: RRT는 시작점에서 트리 하나를 뻗어
// 목표에 닿자마자 멈추고(단일 질의), PRM은 공간 전체에 표본을 뿌려 이웃끼리 촘촘히
// 이은 roadmap을 세운 뒤 그 위에서 최단 경로를 뽑는다(다중 질의). 실제 저장소 미러
// 엔진을 브라우저에서 그대로 돌린다.
// seed 1 실측(24×24, 두 블록): RRT 트리 154간선 첫 경로 37.41(삐죽함),
// PRM 180표본 roadmap 1185간선 경로 28.78.
const N = 24;

function twoBlockMap(): GridMap {
    const map = emptyGrid("tree_vs_roadmap", N, N)
    const block = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) for (let c = c0; c <= c1; c++) map.occupied[r * N + c] = true
    }
    block(10, 13, 6, 8)
    block(6, 9, 15, 17)
    return map
}

const START: Point = [2.5, 2.5];
const GOAL: Point = [21.5, 21.5];

const Scene = ({panel = 236}: {panel?: number}) => {
    const map = useMemo(() => twoBlockMap(), [])
    const rrt = useMemo(
        () => buildGridTimeline(runRRT({
            map, start: START, goal: GOAL, maxIterations: 1500, stepSize: 1.5,
            goalBias: 0.05, goalTolerance: 0.7, seed: 1,
        })),
        [map],
    )
    const prm = useMemo(
        () => buildGridTimeline(runPRM({
            map, start: START, goal: GOAL, numSamples: 180, connectionRadius: 4, seed: 1,
        })),
        [map],
    )
    const t = useTr()
    return (
        <DualReplay
            panel={panel}
            left={{label: "RRT  ·  tree", map, timeline: rrt,
                   start: [START[0], START[1]], goal: [GOAL[0], GOAL[1]], showTree: true}}
            right={{label: "PRM  ·  roadmap", map, timeline: prm,
                    start: [START[0], START[1]], goal: [GOAL[0], GOAL[1]], showTree: true}}
            caption={t(
                "The tree reaches the goal with one jagged branch (cost 37.41) and stops; the roadmap wires 180 samples into a mesh and returns a much straighter path (28.78). Single query versus many.",
                "트리는 삐죽한 가지 하나로 목표에 닿아(비용 37.41) 멈추고, roadmap은 표본 180개를 그물로 엮어 훨씬 곧은 경로(28.78)를 돌려준다. 단일 질의 대 다중 질의.",
            )}
        />
    )
}

const SampleTreeVsRoadmapLive = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "Two ways to wire random samples, live on the same map: a tree grows outward from the start until it touches the goal, a roadmap connects every sample to its neighbors first",
            "무작위 표본을 잇는 두 방식을 같은 맵에서 라이브로. 트리는 시작점에서 자라 목표에 닿을 때까지 뻗고, roadmap은 모든 표본을 먼저 이웃과 잇는다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene panel={Math.min(modalCanvasSize(2).width / 2 - 12, 420)}/>}
    >
        <Scene panel={236}/>
    </CanvasFigure>
}

export default SampleTreeVsRoadmapLive

import {useMemo} from "react";
import CanvasFigure, {modalCanvasSize} from "../../CanvasFigure";
import DualReplay from "./DualReplay";
import {runAStar} from "../../../libs/algorithms/astar";
import {runRRTStar} from "../../../libs/algorithms/rrt_star";
import {buildGridTimeline, Cell} from "../../../libs/trace/timeline";
import {emptyGrid, GridMap} from "../../../libs/grid";
import {Point} from "../../../libs/algorithms/sampling_space";
import {useTr} from "../../../libs/i18n";

// 같은 맵·같은 시작/목표를 두 계열이 나란히 푼다: A*는 시작점을 중심으로 비용
// 등고선을 넓혀 128칸을 확장하고, RRT*는 무작위 표본으로 트리를 뻗는다. 8-connected
// 격자는 A*를 45° 이동에 묶어 그 경로(29.80)가 RRT*의 any-angle 경로(28.84)보다
// 오히려 길다 — any-angle planner가 존재하는 이유의 예고편이다.
// seed 1 실측(24×24, 아래 블록 배치): A* expanded=128 cost=29.80,
// RRT* 900 iters edges≈1677 cost=28.84, 직선 하한 26.87.
const N = 24;

function compareMap(): GridMap {
    const map = emptyGrid("search_vs_sample", N, N)
    const block = (r0: number, r1: number, c0: number, c1: number) => {
        for (let r = r0; r <= r1; r++) for (let c = c0; c <= c1; c++) map.occupied[r * N + c] = true
    }
    block(8, 15, 9, 11)     // 가운데 세로 벽
    block(4, 6, 15, 20)     // 우상단 블록
    return map
}

const A_START: Cell = [N - 3, 2];
const A_GOAL: Cell = [2, N - 3];
// grid 셀 (row, col) → world (x, y): x = col + 0.5, y = height - row - 0.5.
const cellToWorld = (c: Cell): Point => [c[1] + 0.5, N - c[0] - 0.5];
const R_START = cellToWorld(A_START);
const R_GOAL = cellToWorld(A_GOAL);

const Scene = ({panel = 236}: {panel?: number}) => {
    const map = useMemo(() => compareMap(), [])
    const aStar = useMemo(
        () => buildGridTimeline(runAStar({
            map, start: A_START, goal: A_GOAL, heuristicWeight: 1, connectivity: 8,
        })),
        [map],
    )
    const rrtStar = useMemo(
        () => buildGridTimeline(runRRTStar({
            map, start: R_START, goal: R_GOAL, maxIterations: 900, stepSize: 1.5,
            goalBias: 0.05, goalTolerance: 0.6, neighborRadius: 3,
            radiusMode: "fixed", rggGamma: 2, seed: 1,
        })),
        [map],
    )
    const t = useTr()
    return (
        <DualReplay
            panel={panel}
            left={{label: "A*  ·  grid search", map, timeline: aStar, start: A_START, goal: A_GOAL}}
            right={{label: "RRT*  ·  sampling", map, timeline: rrtStar,
                    start: [R_START[0], R_START[1]], goal: [R_GOAL[0], R_GOAL[1]], showTree: true}}
            caption={t(
                "Same map, same start and goal. A* floods a cost contour outward from the start; RRT* throws a tree at the space. The 8-connected grid even ties A*'s path (29.80) slightly above RRT*'s any-angle route (28.84).",
                "같은 맵, 같은 시작·목표. A*는 시작점에서 비용 등고선을 바깥으로 넓히고, RRT*는 공간에 트리를 던진다. 8-connected 격자가 A*를 45° 이동에 묶어, 그 경로(29.80)가 RRT*의 any-angle 경로(28.84)보다 오히려 조금 길다.",
            )}
        />
    )
}

const SearchVsSampleLive = () => {
    const t = useTr()
    return <CanvasFigure
        label={t(
            "One problem, two paradigms: A*'s expanding cost contour versus RRT*'s reaching tree, replayed side by side",
            "한 문제, 두 패러다임. A*의 넓어지는 비용 등고선과 RRT*의 뻗어 가는 트리를 나란히 재생한다",
        )}
        tight bodyClassName="w-fit" className="w-full"
        modal={<Scene panel={Math.min(modalCanvasSize(2).width / 2 - 12, 420)}/>}
    >
        <Scene panel={236}/>
    </CanvasFigure>
}

export default SearchVsSampleLive

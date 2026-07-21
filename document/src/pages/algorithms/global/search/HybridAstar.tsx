import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import PrimitiveFan from "../../../../components/panels/global/hybrid_astar/PrimitiveFan";
import HybridSandbox from "../../../../components/panels/global/hybrid_astar/HybridSandbox";
import hybridPy from "../../../../../../python/navigation/global_planning/search/hybrid_astar.py?raw";
import hybridCpp from "../../../../../../cpp/src/global_planning/search/hybrid_astar.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const HybridAstar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every planner so far answers "which cells?" — but a car cannot teleport
                    between cells. It has a heading, a minimum turn radius, and a steering wheel;
                    a 90° grid corner is simply not a motion it can make. Hybrid A* (Dolgov,
                    Thrun, Montemerlo &amp; Diebel, 2008), built for Stanford's DARPA Urban
                    Challenge entry, moves the search itself into the vehicle's continuous pose
                    space — every edge it considers is an arc the car can actually drive.
                </p>}
                ko={<p>
                    지금까지의 planner는 "어느 셀인가"에 답한다. 그런데 자동차는 셀 사이를
                    순간이동하지 못한다. heading이 있고, 최소 회전 반경이 있고, 조향각이 있다.
                    90° 격자 모서리는 애초에 만들 수 없는 동작이다. Hybrid A*(Dolgov, Thrun,
                    Montemerlo &amp; Diebel, 2008)는 Stanford의 DARPA Urban Challenge 출전작을
                    위해 만들어졌고, 탐색 자체를 차량의 연속 pose 공간으로 옮긴다. 고려하는 모든
                    edge가 차가 실제로 달릴 수 있는 arc다.
                </p>}
            />

            <h2>{t("Leaving the Grid", "격자를 떠나며")}</h2>
            <T
                en={<>
                    <p>
                        The state becomes a continuous pose{" "}
                        <InlineMath math="(x, y, \theta) \in SE(2)"/>, and successors are{" "}
                        <strong>motion primitives</strong>: constant-curvature arcs of a fixed
                        length, one per steering angle, capped by the minimum turn radius —
                        optionally mirrored in reverse. Integrating an arc is exact:
                    </p>
                    <BlockMath math="\theta' = \theta + \kappa \ell, \qquad x' = x + \tfrac{\sin\theta' - \sin\theta}{\kappa}, \qquad y' = y - \tfrac{\cos\theta' - \cos\theta}{\kappa}"/>
                    <Terms items={[
                        ["(x, y, \\theta)", <>the current pose: position plus heading — <InlineMath math="\theta"/> is <strong>the new state dimension</strong> grid planners never had</>],
                        ["\\kappa", <>curvature of the arc, the steering command; bounded by <InlineMath math="|\kappa| \le 1/R_{\min}"/> (minimum turn radius)</>],
                        ["\\ell", "signed arc length driven along the primitive (negative = reverse)"],
                        ["(x', y', \\theta')", <>the pose after driving the arc — the successor state</>],
                    ]}/>
                    <p>
                        A continuous space would make A*'s closed set useless — no two poses ever
                        repeat exactly. The <em>hybrid</em> in the name is the fix: costs and
                        poses stay continuous, but the closed set and the g-table key on a
                        discretized bin <InlineMath math="(\lfloor x \rfloor, \lfloor y \rfloor, \lfloor \theta \rfloor)"/>,
                        each bin remembering the best continuous pose that reached it. The search
                        is finite; the paths are smooth.
                    </p>
                </>}
                ko={<>
                    <p>
                        상태가 연속 pose <InlineMath math="(x, y, \theta) \in SE(2)"/>가 되고,
                        successor는 <strong>motion primitive</strong>다. 조향각마다 하나씩,
                        최소 회전 반경이 한계 짓는 고정 길이의 일정 곡률 arc 들이고, 원하면
                        후진으로도 뒤집는다. arc 적분은 정확하다:
                    </p>
                    <BlockMath math="\theta' = \theta + \kappa \ell, \qquad x' = x + \tfrac{\sin\theta' - \sin\theta}{\kappa}, \qquad y' = y - \tfrac{\cos\theta' - \cos\theta}{\kappa}"/>
                    <Terms items={[
                        ["(x, y, \\theta)", <>현재 pose. 위치에 heading이 더해진 것으로, <InlineMath math="\theta"/>가 grid planner에는 없던 <strong>새 상태 차원</strong>이다</>],
                        ["\\kappa", <>arc의 곡률, 즉 조향 명령. <InlineMath math="|\kappa| \le 1/R_{\min}"/> (최소 회전 반경)으로 유계</>],
                        ["\\ell", "primitive를 따라 달리는 부호 있는 arc 길이 (음수 = 후진)"],
                        ["(x', y', \\theta')", "arc를 달린 뒤의 pose, 곧 successor 상태"],
                    ]}/>
                    <p>
                        연속 공간에서는 A*의 closed set이 무력해진다. 어떤 두 pose도 정확히
                        겹치지 않기 때문이다. 이름의 <em>hybrid</em>가 그 해법이다. 비용과 pose
                        는 연속으로 두되, closed set과 g-테이블은 이산화된 bin{" "}
                        <InlineMath math="(\lfloor x \rfloor, \lfloor y \rfloor, \lfloor \theta \rfloor)"/>
                        을 키로 쓰고, 각 bin은 자기에게 도달한 최선의 연속 pose를 기억한다.
                        탐색은 유한해지고, 경로는 매끄럽다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Kinematically feasible by construction</strong> — every path
                        segment respects the turn radius; goals carry a heading and are reached
                        within position + heading tolerances.</li>
                    <li><strong>Resolution-complete, resolution-suboptimal</strong>: guarantees
                        are relative to the bin resolution and primitive set, not the continuous
                        optimum.</li>
                    <li><strong>This implementation is the honest core</strong> of Dolgov et al.:
                        Euclidean heuristic (admissible) and no analytic Reeds–Shepp expansion —
                        the paper's non-holonomic heuristic and shot-to-goal accelerate the same
                        search without changing its character.</li>
                    <li><strong>Cost</strong>: the state space is 3D
                        (<InlineMath math="x \times y \times \theta"/>), so expansions run far
                        higher than 2D grid searches on the same map — the price of feasibility.</li>
                </ul>}
                ko={<ul>
                    <li><strong>구성상 기구학적으로 주행 가능하다.</strong> 모든 경로 구간이 회전
                        반경을 지키고, goal은 heading을 가지며 위치·heading 허용 오차 안에서
                        도달한다.</li>
                    <li><strong>Resolution-complete, resolution-suboptimal</strong>: 보장은 연속
                        최적이 아니라 bin 해상도와 primitive 집합에 상대적이다.</li>
                    <li><strong>이 구현은 Dolgov et al. 의 정직한 코어다.</strong> admissible 한
                        Euclidean heuristic을 쓰고 Reeds–Shepp analytic expansion은 없다.
                        논문의 non-holonomic heuristic과 shot-to-goal은 같은 탐색을 가속할 뿐
                        성격을 바꾸지 않는다.</li>
                    <li><strong>비용</strong>: 상태 공간이 3차원
                        (<InlineMath math="x \times y \times \theta"/>)이라 같은 맵의 2D grid
                        탐색보다 확장 수가 훨씬 크다. 주행 가능성의 값이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    A* over bins, with continuous poses riding along. Collision checking
                    sub-samples each arc densely enough that consecutive footprint discs overlap:
                </p>}
                ko={<p>
                    bin 위의 A*에 연속 pose가 함께 실려 다닌다. 충돌 검사는 각 arc를 footprint
                    disc들이 겹칠 만큼 촘촘히 나눠 본다:
                </p>}
            />
            <Pseudocode code={`g[bin(start)] ← 0;  pose_of[bin(start)] ← start;  push with key h(start)
while OPEN is not empty:
    b ← pop_min(OPEN);  skip if closed;  close b
    p ← pose_of[b]                                             # 1
    if p is within goal tolerances (position and heading):     # 2
        return densified arcs start → p
    for each primitive (κ, ℓ, reverse?):
        sub-sample the arc from p; discard if any pose collides  # 3
        child ← arc endpoint;  b' ← bin(child)
        cost ← |ℓ|·(reverse penalty?) + steer_penalty·|κ|·|ℓ|    # 4
        if g[b] + cost < g[b']:
            g[b'] ← g[b] + cost;  pose_of[b'] ← child            # 5
            push b' with key g[b'] + euclidean(child, goal)`}/>
            <T
                en={<ol>
                    <li>Expand the bin's <em>best continuous pose</em>, not the bin center — this
                        is what keeps chained arcs drivable across bins.</li>
                    <li>Goal test with tolerances: an exact continuous pose is measure-zero, so
                        "close enough in position and heading" is the correct notion.</li>
                    <li>Sub-sampling spacing is tied to the footprint radius so discs overlap —
                        an arc cannot tunnel through a thin wall between samples.</li>
                    <li>Costs shape behavior: reverse costs extra, steering costs extra — the
                        planner prefers straight forward driving when it can.</li>
                    <li>Only a cheaper route may claim a bin, and the bin adopts the new route's
                        endpoint pose.</li>
                </ol>}
                ko={<ol>
                    <li>bin의 중심이 아니라 <em>최선의 연속 pose</em>를 확장한다. arc들이 bin
                        을 넘나들며 이어져도 주행 가능함을 지키는 장치다.</li>
                    <li>허용 오차 있는 goal 검사. 정확한 연속 pose는 measure-zero라 "위치와
                        heading이 충분히 가깝다"가 옳은 개념이다.</li>
                    <li>부표본 간격을 footprint 반지름에 묶어 disc들이 겹치게 한다. arc가 표본
                        사이의 얇은 벽을 뚫고 지나갈 수 없다.</li>
                    <li>비용이 행동을 빚는다. 후진에 벌점, 조향에 벌점. planner는 가능하면 곧게
                        전진하는 쪽을 고른다.</li>
                    <li>더 싼 경로만 bin을 차지할 수 있고, bin은 그 경로의 끝 pose를 넘겨받는다.</li>
                </ol>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    First the primitive fan — the literal successor set of one pose; squeeze the
                    turn radius and the fan narrows. Then the parking sandbox: the goal requires
                    entering the bay <em>facing up</em>, a constraint no cell planner can even
                    express. Forbid reverse or tighten the radius and the swing changes; block
                    the approach and the maneuver reroutes. The replay below is the repository
                    demo on the benchmark maps: arc trees in continuous space.
                </p>}
                ko={<p>
                    먼저 primitive 부채꼴. 문자 그대로 pose 하나의 successor 집합이고, 회전
                    반경을 조이면 부채가 좁아진다. 다음은 주차 sandbox다. goal이 베이에{" "}
                    <em>위를 향한 채</em> 들어가기를 요구하는데, 셀 planner로는 이 제약을 표현할
                    방법조차 없다. 후진을 끄거나 반경을 조이면 진입 궤적이 달라지고, 진입로를
                    막으면 기동이 우회한다. 아래 replay는 벤치마크 맵에서의 저장소 demo다.
                    연속 공간의 arc 트리가 그대로 보인다.
                </p>}
            />
            <PrimitiveFan/>
            <HybridSandbox/>
            <TraceReplay vehicle algo="hybrid_astar" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's Hybrid A* demo — pose dots and arc chords instead of cells",
                "저장소 Hybrid A* demo의 실제 trace. 셀 대신 pose 점과 arc 현이 그려진다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation carries its own vehicle model (primitives) and keys
                    everything on pose bins; the map only answers footprint collision queries.
                    Embedded below in full.
                </p>}
                ko={<p>
                    구현은 자체 차량 모델(primitive)을 갖고 모든 것을 pose bin으로 키잉한다.
                    맵은 footprint 충돌 질의에만 답한다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/search/hybrid_astar.py",
                            code: hybridPy,
                            href: `${REPO}/python/navigation/global_planning/search/hybrid_astar.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/search/hybrid_astar.cpp",
                            code: hybridCpp,
                            href: `${REPO}/cpp/src/global_planning/search/hybrid_astar.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Hybrid A* implementation, embedded from the repository sources",
                    "Hybrid A* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel,{" "}
                    <a href="https://ai.stanford.edu/~ddolgov/papers/dolgov_gpp_stair08.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Practical Search Techniques in Path Planning for Autonomous
                            Driving</em>
                    </a>,
                    AAAI Workshop on Search Techniques in AI and Robotics, 2008.
                </li>
                <li>
                    D. Dolgov, S. Thrun, M. Montemerlo, J. Diebel,{" "}
                    <a href="https://doi.org/10.1177/0278364909359210" target="_blank"
                       rel="noopener noreferrer">
                        <em>Path Planning for Autonomous Vehicles in Unknown Semi-structured
                            Environments</em>
                    </a>,
                    The International Journal of Robotics Research, 2010.
                </li>
                <li>
                    J. A. Reeds, L. A. Shepp,{" "}
                    <a href="https://doi.org/10.2140/pjm.1990.145.367" target="_blank"
                       rel="noopener noreferrer">
                        <em>Optimal Paths for a Car That Goes Both Forwards and Backwards</em>
                    </a>,
                    Pacific Journal of Mathematics, 1990.
                </li>
            </ol>
        </>
    )
}

export default HybridAstar

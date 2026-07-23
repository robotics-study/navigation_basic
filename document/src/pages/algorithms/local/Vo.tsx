import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import VoSandbox from "../../../components/panels/local/vo/VoSandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import voPy from "../../../../../python/navigation/local_planning/velocity/vo.py?raw";
import velocityObstaclePy from "../../../../../python/navigation/local_planning/velocity/_velocity_obstacle.py?raw";
import voHpp from "../../../../../cpp/include/navigation/local_planning/velocity/vo.hpp?raw";
import voCpp from "../../../../../cpp/src/local_planning/velocity/vo.cpp?raw";
import velocityObstacleHpp from "../../../../../cpp/include/navigation/local_planning/velocity/velocity_obstacle.hpp?raw";
import velocityObstacleCpp from "../../../../../cpp/src/local_planning/velocity/velocity_obstacle.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록 -- DWA/VFH 등 다른 local planner 페이지의 Proof 관례를 그대로 따른다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Vo = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    DWA and VFH both ask "which command looks safe over the next second or two" by
                    rolling out or sensing around the robot's own body. Neither has any notion of
                    where a moving obstacle will <em>be</em> a moment from now — a pedestrian or
                    another robot is just terrain that happens to have shifted the next time it is
                    sensed. Velocity Obstacles, introduced by Paolo Fiorini and Zvi Shiller in 1998,
                    change the question: instead of asking "is this position safe," it asks "which of
                    my velocities, held for a few seconds, would put me on a collision course with
                    something that is also moving." That question is answered directly in velocity
                    space, and it is the first idea in this family of local planners — Reciprocal
                    Velocity Obstacles and ORCA, covered next, both refine the same forbidden-region
                    picture rather than replacing it.
                </p>}
                ko={<p>
                    DWA와 VFH는 둘 다 로봇 자신의 몸 주변을 굴려보거나 감지해서 "다음 1~2초 동안 이
                    명령이 안전해 보이는가"를 묻는다. 둘 다 움직이는 장애물이 잠시 후 <em>어디에</em>{" "}
                    있을지에 대한 개념이 전혀 없다. 보행자나 다른 로봇은 그저 다음에 감지할 때 위치가
                    바뀐 지형일 뿐이다. Paolo Fiorini와 Zvi Shiller가 1998년 내놓은 Velocity
                    Obstacle(VO)은 질문 자체를 바꾼다. "이 위치가 안전한가" 대신 "내 속도 중 어느
                    것을 몇 초간 유지하면 마찬가지로 움직이는 무언가와 충돌 코스에 들어서는가"를
                    묻는다. 이 질문은 속도 공간에서 직접 답해진다. 그리고 이는 이번 계열의 첫
                    아이디어다. 이어지는 Reciprocal Velocity Obstacles와 ORCA는 이 금지 영역이라는
                    그림 자체를 대체하는 게 아니라 다듬어 나간다.
                </p>}
            />

            <h2>{t("A Forbidden Region in Velocity Space", "속도 공간에 생기는 금지 영역")}</h2>
            <T
                en={<p>
                    Take one nearby obstacle: a disc of radius <InlineMath math="r_o"/> sitting at
                    position <InlineMath math="p_o"/>, moving at velocity <InlineMath math="v_o"/>{" "}
                    (a stationary wall cell is simply <InlineMath math="v_o = 0"/>). Growing the
                    obstacle by the robot's own radius and shrinking the robot to a point is the
                    usual Minkowski-sum trick, and it turns the question into: from the robot's
                    position, which of <em>my</em> absolute velocities <InlineMath math="v"/>,
                    held for <InlineMath math="\tau"/> seconds, land inside a disc of the combined
                    radius growing out from the obstacle's own path? Every such <InlineMath math="v"/>{" "}
                    forms a truncated cone in velocity space, apex at the obstacle's own
                    velocity <InlineMath math="v_o"/> — because two things moving at the same
                    velocity never get any closer — opening outward along the bearing to the
                    obstacle:
                </p>}
                ko={<p>
                    근처 obstacle 하나를 보자. 위치 <InlineMath math="p_o"/>에 있고 속도{" "}
                    <InlineMath math="v_o"/>로 움직이는 반경 <InlineMath math="r_o"/>인 원판이다
                    (정지한 벽 셀은 그냥 <InlineMath math="v_o = 0"/>이다). obstacle을 로봇 반경만큼
                    부풀리고 로봇을 점으로 줄이는 것은 흔한 Minkowski 합 트릭이고, 이건 질문을
                    이렇게 바꾼다. 로봇 위치에서, <em>내</em> 절대 속도 <InlineMath math="v"/> 중
                    어느 것을 <InlineMath math="\tau"/>초 유지하면 obstacle 자신의 경로에서 자라난
                    결합 반경의 원판 안으로 들어가는가? 그런 <InlineMath math="v"/> 전부가 속도
                    공간에서 하나의 truncated cone을 이룬다. apex는 obstacle 자신의 속도{" "}
                    <InlineMath math="v_o"/>에 있다. 같은 속도로 움직이는 둘은 절대 가까워지지
                    않기 때문이다. 그리고 obstacle 방위를 따라 바깥으로 벌어진다:
                </p>}
            />
            <BlockMath math="\sin\phi = \frac{r_o + r_{\text{self}}}{\lVert p_o - p_{\text{self}} \rVert}"/>
            <T
                en={<Terms items={[
                    ["\\phi", "the cone's half-angle — how wide the forbidden wedge opens from its apex"],
                    ["r_o,\\ r_{\\text{self}}", "the obstacle's radius and the robot's own radius — their sum is the combined (Minkowski) radius"],
                    ["p_o,\\ p_{\\text{self}}", "world positions of the obstacle and the robot — their difference sets the cone's axis"],
                ]}/>}
                ko={<Terms items={[
                    ["\\phi", "원뿔의 반각. apex에서 금지 쐐기가 얼마나 넓게 벌어지는지"],
                    ["r_o,\\ r_{\\text{self}}", "obstacle 반경과 로봇 자신의 반경. 둘의 합이 결합(Minkowski) 반경이다"],
                    ["p_o,\\ p_{\\text{self}}", "obstacle과 로봇의 world 위치. 그 차가 원뿔의 축 방향을 정한다"],
                ]}/>}
            />
            <T
                en={<p>
                    A finite window rather than "forever" matters too: without it, almost every
                    direction is eventually forbidden by something far away that will not matter for
                    minutes. The cone is truncated at <InlineMath math="\tau"/> seconds — a
                    candidate only counts as violating it if the closing gap would actually run out
                    within that horizon, not merely if the angle lines up:
                </p>}
                ko={<p>
                    "영원히"가 아니라 유한한 창을 쓰는 것도 중요하다. 그게 없으면 몇 분 뒤에나
                    문제될 먼 무언가 때문에 거의 모든 방향이 결국 금지된다. 원뿔은{" "}
                    <InlineMath math="\tau"/>초에서 잘린다. 각도가 맞는 것만으로는 위반이 아니고,
                    그 지평 안에 실제로 간격이 다 좁혀질 때만 위반으로 친다:
                </p>}
            />
            <BlockMath math="w_{\parallel} \ge \frac{d - r_o - r_{\text{self}}}{\tau}"/>
            <T
                en={<Terms items={[
                    ["w_{\\parallel}", <>the component of relative approach velocity{" "}
                        <InlineMath math="v - v_o"/> projected onto the bearing toward the
                        obstacle — how fast the gap is actually closing</>],
                    ["d", <>current distance <InlineMath math="\lVert p_o - p_{\\text{self}} \\rVert"/> between centers</>],
                    ["r_o,\\ r_{\\text{self}}", "as above — their sum is subtracted to leave the actual gap between surfaces"],
                    ["\\tau", "the time horizon: only closing speeds fast enough to collide within this many seconds count as a violation"],
                ]}/>}
                ko={<Terms items={[
                    ["w_{\\parallel}", <>상대 접근 속도 <InlineMath math="v - v_o"/>를 obstacle
                        방위로 사영한 성분. 실제로 간격이 얼마나 빠르게 좁혀지는지</>],
                    ["d", <>중심 사이 현재 거리 <InlineMath math="\\lVert p_o - p_{\\text{self}} \\rVert"/></>],
                    ["r_o,\\ r_{\\text{self}}", "위와 동일. 둘의 합을 빼면 실제 표면 사이 간격이 남는다"],
                    ["\\tau", "시간 지평. 이 초 안에 실제로 충돌할 만큼 빠르게 좁혀지는 접근 속도만 위반으로 친다"],
                ]}/>}
            />
            <T
                en={<p>
                    A robot's velocity is admissible only if it sits outside <em>every</em> nearby
                    obstacle's cone at once (VO(A|B), one cone per obstacle B). The planner's whole
                    job each tick is picking, among the admissible ones, whichever comes closest to
                    where the robot actually wants to go.
                </p>}
                ko={<p>
                    로봇의 속도는 근처 <em>모든</em> obstacle의 원뿔(obstacle B마다 하나씩인
                    VO(A|B)) 밖에 동시에 있을 때만 admissible하다. planner가 매 tick 하는 일은
                    그 admissible한 속도들 중 로봇이 실제로 가려는 곳에 가장 가까운 것을 고르는
                    것뿐이다.
                </p>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Cost per tick: <InlineMath math="O(m \cdot n_s)"/></strong>, for{" "}
                        <InlineMath math="m"/> obstacles (neighbors plus occupied cells) within{" "}
                        <InlineMath math="\text{neighbor\_dist}"/> and <InlineMath math="n_s"/>{" "}
                        sampled candidate velocities — building one cone per obstacle is
                        <InlineMath math="O(m)"/>, checking every candidate against every cone is
                        <InlineMath math="O(m \cdot n_s)"/>.</li>
                    <li><strong>Deterministic, not exhaustive.</strong> Candidates come from a fixed
                        polar grid (speed rings times bearing steps), never randomly sampled, but a
                        real admissible velocity that falls between grid points is invisible to the
                        search — resolution is a tuning knob, not a guarantee.</li>
                    <li><strong>Free space always costs exactly zero.</strong> Candidate 0 is{" "}
                        <InlineMath math="v_{\text{pref}}"/> itself; when nothing is in the way it
                        wins outright, so an unobstructed robot drives a perfectly straight line to
                        the goal with no jitter from the sampling grid at all.</li>
                    <li><strong>Assumes the obstacle holds its course.</strong> The cone's apex is
                        the obstacle's <em>current</em> velocity, extrapolated forward unchanged for
                        the full <InlineMath math="\tau"/> seconds. For a wall that is exactly true;
                        for another agent that is also avoiding <em>this</em> robot, it is a fiction
                        both sides believe at once — the source of the failure mode below.</li>
                    <li><strong>No completeness or optimality guarantee.</strong> VO reports
                        whichever admissible candidate is closest to <InlineMath math="v_{\text{pref}}"/>{" "}
                        on the sampled grid this tick; it has no path-level guarantee of ever
                        reaching the goal, and (unlike DWA) no explicit clearance margin beyond
                        whatever <InlineMath math="\tau"/> and the combined radius already encode.</li>
                </ul>}
                ko={<ul>
                    <li><strong>tick당 비용 <InlineMath math="O(m \cdot n_s)"/></strong>.{" "}
                        <InlineMath math="\text{neighbor\_dist}"/> 이내 obstacle(이웃 +
                        점유 셀) <InlineMath math="m"/>개, 표본 후보 속도{" "}
                        <InlineMath math="n_s"/>개일 때 obstacle마다 원뿔 하나 만드는 데{" "}
                        <InlineMath math="O(m)"/>, 후보마다 모든 원뿔과 대조하는 데{" "}
                        <InlineMath math="O(m \cdot n_s)"/>가 든다.</li>
                    <li><strong>결정적이지만 전수 탐색은 아니다.</strong> 후보는 고정된 극좌표
                        격자(speed 링 x bearing 스텝)에서 나오고 절대 난수로 뽑지 않지만, 격자
                        점 사이에 있는 실제 admissible 속도는 탐색에 보이지 않는다. 해상도는
                        보장이 아니라 튜닝 손잡이다.</li>
                    <li><strong>열린 공간은 항상 정확히 비용 0이다.</strong> 후보 0이{" "}
                        <InlineMath math="v_{\text{pref}}"/> 자신이라, 앞을 막는 게 없으면 그대로
                        이긴다. 그래서 아무것도 없는 공간에서는 로봇이 격자 표본 때문에
                        떨리는 일 없이 목표까지 완전히 직선으로 달린다.</li>
                    <li><strong>obstacle이 진로를 유지한다고 가정한다.</strong> 원뿔의 apex는
                        obstacle의 <em>현재</em> 속도를 <InlineMath math="\tau"/>초 내내 그대로
                        외삽한 값이다. 벽이라면 정확히 참이지만, 마찬가지로 <em>이</em> 로봇을
                        피하고 있는 다른 agent라면 양쪽 모두가 동시에 믿는 허구다. 아래 실패
                        모드의 원인이다.</li>
                    <li><strong>완전성도 최적성 보장도 없다.</strong> VO는 이번 tick 표본 격자
                        위에서 <InlineMath math="v_{\text{pref}}"/>에 가장 가까운 admissible
                        후보를 낼 뿐, 결국 goal에 도달한다는 경로 수준 보장이 없고 (DWA와 달리){" "}
                        <InlineMath math="\tau"/>와 결합 반경이 이미 담고 있는 것 이상의 명시적
                        여유 마진도 없다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Every tick starts clean: gather this instant's obstacles, compute a goal-seeking
                    preferred velocity, then scan a fixed candidate grid for the admissible one
                    closest to it. Nothing about the search carries over from the previous tick.
                </p>}
                ko={<p>
                    매 tick은 깨끗하게 시작한다. 이 순간의 obstacle을 모으고, goal을 향하는 선호
                    속도를 계산한 뒤, 고정된 후보 격자를 훑어 그중 가장 가까운 admissible 속도를
                    찾는다. 탐색의 어떤 것도 직전 tick에서 이어지지 않는다.
                </p>}
            />
            <Pseudocode code={`statics ← occupied_cells_within(neighbor_dist) as velocity-0 obstacles     # 1
obstacles ← neighbors + statics                                            # 2
v_pref ← toward(goal), capped at max_speed, tapered near goal              # 3
cones ← [truncated_vo_cone(o.pos − pos, r_self + o.r, o.velocity, tau)     # 4
         for o in obstacles if dist(o.pos, pos) < neighbor_dist + o.r]
candidates ← [v_pref] + polar_grid(max_speed, speed_samples, angle_samples) # 5
best ← v_pref, best_cost ← 0                                               # 6
for v in candidates:                                                        # 7
    violated ← any(in_velocity_obstacle(v, c) for c in cones)               # 8
    cost ← dist(v, v_pref) + (PENALTY if violated else 0)                  # 9
    if cost < best_cost: best, best_cost ← v, cost                         # 10
return velocity_to_command(best, theta, max_omega, heading_gain)           # 11`}/>
            <T
                en={<ol>
                    <li>Static obstacles are not a separate code path: every occupied cell within
                        sensing range becomes a zero-velocity <InlineMath math="\text{DynamicObstacle}"/>,
                        so a wall is just an obstacle that happens to never move.</li>
                    <li>Neighbors (other agents) and statics get concatenated — the cone-building and
                        scan below treat every source of danger identically.</li>
                    <li>The preferred velocity points straight at the goal at full speed, slowing
                        linearly once within <InlineMath math="\text{max\_speed}"/> meters of it so
                        the episode settles into REACHED rather than orbiting.</li>
                    <li>One truncated cone per obstacle inside range — this is exactly the geometry
                        derived above, apex at the obstacle's own velocity.</li>
                    <li>The candidate grid is deterministic and fixed: <InlineMath math="v_{\text{pref}}"/>{" "}
                        itself first, then a speed-outer / angle-inner polar sweep — never randomly
                        drawn, so the same scene always produces the same choice.</li>
                    <li><strong>The trap in waiting:</strong> initializing the running best to{" "}
                        <InlineMath math="v_{\text{pref}}"/> with cost 0 means an unobstructed tick
                        never even has to evaluate a single cone to know it has already won —
                        candidate 0 always wins every tie against a violating candidate of equal
                        distance, because ties break in favor of whichever came first and{" "}
                        <InlineMath math="v_{\text{pref}}"/> is always first.</li>
                    <li>Every candidate gets scored on its own: distance to{" "}
                        <InlineMath math="v_{\text{pref}}"/>, plus a fixed penalty if it violates any
                        cone at all.</li>
                    <li>Strict less-than keeps the running best from being displaced by a
                        later candidate at the exact same cost — this, not just step 5's ordering,
                        is what makes tie-breaking reproducible.</li>
                    <li>A cost-minimizing but still-violating candidate can win only when{" "}
                        <em>every</em> sampled velocity violates some cone — the penalty guarantees
                        any truly admissible candidate always outranks it first.</li>
                    <li>The chosen absolute velocity is projected onto a differential-drive
                        command exactly like DWA's steering law: turn toward it, with the chosen
                        speed itself becoming the speed cap.</li>
                </ol>}
                ko={<ol>
                    <li>정적 장애물은 별도 코드 경로가 아니다. 감지 범위 안의 점유 셀 전부가
                        속도 0인 <InlineMath math="\text{DynamicObstacle}"/>이 된다. 벽은 그냥
                        절대 움직이지 않는 obstacle일 뿐이다.</li>
                    <li>이웃(다른 agent)과 정적 obstacle을 이어 붙인다. 아래 원뿔 구성과 스캔은
                        위험의 출처를 전부 동일하게 다룬다.</li>
                    <li>선호 속도는 goal을 향해 최고 속도로 곧장 향하되,{" "}
                        <InlineMath math="\text{max\_speed}"/> 미터 이내로 들어오면 선형으로
                        느려져 에피소드가 맴돌지 않고 REACHED로 정착한다.</li>
                    <li>범위 안 obstacle마다 truncated cone 하나. 위에서 유도한 기하 그대로이며,
                        apex는 obstacle 자신의 속도에 있다.</li>
                    <li>후보 격자는 결정적이고 고정돼 있다. <InlineMath math="v_{\text{pref}}"/>{" "}
                        자신이 먼저, 그다음 speed-외측/angle-내측 극좌표 스캔이다. 절대 난수로
                        뽑지 않아 같은 장면은 항상 같은 선택을 낸다.</li>
                    <li><strong>기다리고 있는 함정.</strong> 지금까지의 최선을 비용 0인{" "}
                        <InlineMath math="v_{\text{pref}}"/>로 초기화해 두면, 막힌 게 없는
                        tick은 원뿔을 단 하나도 검사하지 않고도 이미 이겼다는 걸 안다. 후보 0은
                        같은 거리의 위반 후보와의 어떤 동률에서도 항상 이긴다. 동률은 먼저 온
                        쪽이 이기고 <InlineMath math="v_{\text{pref}}"/>는 항상 제일 먼저이기
                        때문이다.</li>
                    <li>모든 후보는 각자 채점된다. <InlineMath math="v_{\text{pref}}"/>까지의
                        거리에, 원뿔을 하나라도 위반하면 고정 페널티가 더해진다.</li>
                    <li>strict less-than이 지금까지의 최선이 정확히 같은 비용의 나중 후보로
                        바뀌는 것을 막는다. 5단계의 순서만이 아니라 이것이 동률 처리를 재현
                        가능하게 만드는 지점이다.</li>
                    <li>비용은 최소지만 여전히 위반인 후보가 이기는 건 표본으로 뽑힌{" "}
                        <em>모든</em> 속도가 어떤 원뿔이든 위반할 때뿐이다. 페널티가 진짜
                        admissible한 후보라면 반드시 먼저 이기도록 보장한다.</li>
                    <li>고른 절대 속도는 DWA의 조향 법칙과 똑같이 차동 구동 명령으로
                        투영된다. 그 방향으로 돌되, 고른 속도 자체가 속도 상한이 된다.</li>
                </ol>}
            />
            <T
                en={<p>
                    Step 4's cone construction is a right-triangle tangent-line argument — worth
                    spelling out once here since RVO and ORCA both reuse the same combined-radius
                    geometry underneath their own apex placements.
                </p>}
                ko={<p>
                    4단계의 원뿔 구성은 직각삼각형 접선 논증이다. RVO와 ORCA 둘 다 apex를 두는
                    방식만 다를 뿐 이 결합 반경 기하 자체는 그대로 재사용하니, 여기서 한 번 풀어볼
                    가치가 있다.
                </p>}
            />
            <Proof title={t("Derivation (tangent half-angle of the truncated cone)", "유도 (truncated cone의 접선 반각)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Place the robot at the origin of a frame centered
                            on itself, so the obstacle sits at relative
                            position <InlineMath math="\text{rel} = p_o - p_{\text{self}}"/> at
                            distance <InlineMath math="d = \lVert \text{rel} \rVert"/>. The forbidden
                            disc has combined radius <InlineMath math="r = r_o + r_{\text{self}}"/>.
                            The two boundary rays of the cone are exactly the two lines through the
                            origin tangent to that disc.
                        </p>
                        <BlockMath math="\sin\phi = \frac{r}{d}"/>
                        <Terms items={[
                            ["\\phi", "half-angle between the cone's axis and either tangent boundary"],
                            ["r", "combined radius — the tangent line's perpendicular distance from the disc's center at the point of tangency"],
                            ["d", "distance from the robot (origin) to the obstacle's center"],
                        ]}/>
                        <p>
                            This is immediate from the tangent-line right triangle: the radius to the
                            point of tangency is perpendicular to the tangent line itself, so the
                            triangle formed by the origin, the disc center, and the tangent point has
                            a right angle at the tangent point, hypotenuse <InlineMath math="d"/>, and
                            opposite side <InlineMath math="r"/>. The cosine follows from the same
                            triangle (adjacent over hypotenuse), taking the non-negative root since{" "}
                            <InlineMath math="\phi \in [0, \pi/2]"/> for any obstacle outside the
                            combined radius:
                        </p>
                        <BlockMath math="\cos\phi = \sqrt{1 - \sin^2\phi} = \sqrt{1 - \frac{r^2}{d^2}}"/>
                        <Terms items={[
                            ["\\phi,\\ r,\\ d", "as above"],
                        ]}/>
                        <p>
                            The axis itself is just the unit bearing <InlineMath math="u = \text{rel}/d"/>.
                            Rotating <InlineMath math="u"/> by <InlineMath math="\pm\phi"/> gives the
                            two tangent directions directly, without ever computing an angle and
                            calling <InlineMath math="\sin"/>/<InlineMath math="\cos"/> on it — a
                            standard rotation-by-known-sine-and-cosine identity:
                        </p>
                        <BlockMath math="\text{left} = (u_x\cos\phi - u_y\sin\phi,\ u_x\sin\phi + u_y\cos\phi), \quad \text{right} = (u_x\cos\phi + u_y\sin\phi,\ -u_x\sin\phi + u_y\cos\phi)"/>
                        <Terms items={[
                            ["\\text{left},\\ \\text{right}", "unit vectors along the cone's two boundary rays, from the apex"],
                            ["u_x,\\ u_y", "components of the unit bearing u toward the obstacle"],
                            ["\\phi", "the half-angle derived above"],
                        ]}/>
                        <p>
                            When the obstacle already overlaps the robot's own disc
                            (<InlineMath math="d \le r"/>), the right triangle above no longer exists —
                            there is no external tangent line at all, and the honest forbidden region
                            is every velocity whatsoever, which is exactly the <InlineMath math="\text{full}"/>{" "}
                            flag's fallback case rather than a divide-by-zero. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>설정.</strong> 로봇을 자기 자신 중심의 좌표계 원점에 두면
                            obstacle은 상대 위치 <InlineMath math="\text{rel} = p_o - p_{\text{self}}"/>,
                            거리 <InlineMath math="d = \lVert \text{rel} \rVert"/>에 있다. 금지 원판의
                            결합 반경은 <InlineMath math="r = r_o + r_{\text{self}}"/>다. 원뿔의 두
                            경계 광선은 정확히 원점을 지나 그 원판에 접하는 두 직선이다.
                        </p>
                        <BlockMath math="\sin\phi = \frac{r}{d}"/>
                        <Terms items={[
                            ["\\phi", "원뿔의 축과 각 접선 경계 사이의 반각"],
                            ["r", "결합 반경. 접점에서 접선까지의 수직 거리"],
                            ["d", "로봇(원점)에서 obstacle 중심까지 거리"],
                        ]}/>
                        <p>
                            접선 직각삼각형에서 바로 나온다. 접점까지의 반지름은 접선 자체와
                            수직이므로, 원점·원판 중심·접점이 이루는 삼각형은 접점에서 직각이고
                            빗변이 <InlineMath math="d"/>, 대변이 <InlineMath math="r"/>이다.
                            코사인도 같은 삼각형에서(인접변/빗변) 나오는데, obstacle이 결합 반경
                            밖에 있는 한 <InlineMath math="\phi \in [0, \pi/2]"/>이므로 음이 아닌
                            근을 취한다:
                        </p>
                        <BlockMath math="\cos\phi = \sqrt{1 - \sin^2\phi} = \sqrt{1 - \frac{r^2}{d^2}}"/>
                        <Terms items={[
                            ["\\phi,\\ r,\\ d", "위와 동일"],
                        ]}/>
                        <p>
                            축 자체는 그냥 단위 방위 <InlineMath math="u = \text{rel}/d"/>다.{" "}
                            <InlineMath math="u"/>를 <InlineMath math="\pm\phi"/>만큼 회전하면
                            각도를 따로 계산해 <InlineMath math="\sin"/>/<InlineMath math="\cos"/>를
                            부르지 않고도 두 접선 방향이 바로 나온다. sine·cosine을 이미 아는
                            회전의 표준 항등식이다:
                        </p>
                        <BlockMath math="\text{left} = (u_x\cos\phi - u_y\sin\phi,\ u_x\sin\phi + u_y\cos\phi), \quad \text{right} = (u_x\cos\phi + u_y\sin\phi,\ -u_x\sin\phi + u_y\cos\phi)"/>
                        <Terms items={[
                            ["\\text{left},\\ \\text{right}", "apex에서 뻗는 원뿔의 두 경계 광선을 따르는 단위 벡터"],
                            ["u_x,\\ u_y", "obstacle을 향한 단위 방위 u의 성분"],
                            ["\\phi", "위에서 유도한 반각"],
                        ]}/>
                        <p>
                            obstacle이 이미 로봇 자신의 원판과 겹친 경우
                            (<InlineMath math="d \le r"/>)에는 위 직각삼각형 자체가 존재하지
                            않는다. 바깥 접선이 아예 없고, 정직한 금지 영역은 속도 전체다. 이는
                            0으로 나누기가 아니라 정확히 <InlineMath math="\text{full}"/> 플래그의
                            fallback 케이스다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("The Reciprocal Dance", "상호 회피의 진동")}</h2>
            <T
                en={<p>
                    The cone's apex assumption — "this obstacle will keep going exactly as it is
                    right now" — is harmless for a wall and false for another VO-driven agent that
                    is, at the very same instant, making the identical assumption about
                    <em>this</em> robot. Put two such agents on a perfectly symmetric head-on
                    course and watch what that mutual fiction does.
                </p>}
                ko={<p>
                    원뿔 apex의 가정, 곧 "이 obstacle은 지금 이대로 계속 갈 것이다"는 벽에는
                    무해하지만, 바로 같은 순간 <em>이</em> 로봇에 대해 똑같은 가정을 하고 있는
                    다른 VO 기반 agent에는 거짓이다. 그런 agent 둘을 완전히 대칭인 정면 마주침
                    코스에 세우고 이 상호 허구가 무슨 일을 벌이는지 보자.
                </p>}
            />
            <Proof title={t("Why a symmetric encounter oscillates instead of settling", "대칭 마주침이 정착 대신 진동하는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Two agents A and B, identical parameters,
                            positioned as exact mirror images through the midpoint between them,
                            each with <InlineMath math="v_{\text{pref}}"/> pointing straight at the
                            other's start (a head-on line). At tick <InlineMath math="t"/>, suppose
                            both are still running the straight-line <InlineMath math="v_{\text{pref}}"/>.
                        </p>
                        <BlockMath math="\text{cone}_A^{(t)} = \text{cone}(\text{apex} = v_B^{(t)}), \qquad \text{cone}_B^{(t)} = \text{cone}(\text{apex} = v_A^{(t)})"/>
                        <Terms items={[
                            ["\\text{cone}_A^{(t)}", "A's forbidden region at tick t, as computed from B's current velocity"],
                            ["v_A^{(t)},\\ v_B^{(t)}", "A's and B's actual velocities at tick t"],
                        ]}/>
                        <p>
                            By the mirror symmetry, <InlineMath math="v_A^{(t)} = -v_B^{(t)}"/>{" "}
                            (equal speed, opposite heading) whenever both are still on the straight
                            line, and the two cones are mirror images of each other. Since{" "}
                            <InlineMath math="v_{\text{pref}}"/> lies exactly on the axis of both
                            cones, it now violates both — the straight line is blocked for both agents
                            at once, symmetrically.
                        </p>
                        <p>
                            <strong>The swerve.</strong> Both scan the identical mirrored candidate
                            grid and, by symmetry of the cost function
                            <InlineMath math="\text{dist}(v, v_{\text{pref}})"/>, both pick the
                            evasive candidate on the same relative side (say, each swerves to its own
                            right) — a coordinated-looking maneuver that emerges from identical
                            geometry, not communication.
                        </p>
                        <BlockMath math="v_A^{(t+1)} \ne v_{\text{pref}}, \qquad v_B^{(t+1)} \ne v_{\text{pref}}, \qquad v_A^{(t+1)} = -v_B^{(t+1)}"/>
                        <Terms items={[
                            ["v_A^{(t+1)},\\ v_B^{(t+1)}", "the swerved velocities each agent commits to for tick t+1"],
                        ]}/>
                        <p>
                            <strong>The relapse.</strong> At tick <InlineMath math="t+2"/>, each
                            agent rebuilds its cone with apex at the <em>other's just-swerved</em>{" "}
                            velocity — which by construction now points safely off to the side. The
                            straight-line <InlineMath math="v_{\text{pref}}"/> that was blocked one
                            tick ago is, from this new apex, no longer inside either cone at all:
                        </p>
                        <BlockMath math="v_{\text{pref}} \notin \text{cone}_A(\text{apex} = v_B^{(t+1)}), \qquad v_{\text{pref}} \notin \text{cone}_B(\text{apex} = v_A^{(t+1)})"/>
                        <p>
                            So both agents revert straight back onto the collision line, and tick{" "}
                            <InlineMath math="t+2"/> looks exactly like tick <InlineMath math="t"/>{" "}
                            all over again — the same symmetric block, the same coordinated swerve,
                            the same relapse. Nothing in VO's apex rule ever anchors on the fact that
                            the other agent's swerve was itself a <em>reaction</em>, so the cycle has
                            no reason to break. <InlineMath math="\blacksquare"/>
                        </p>
                        <p>
                            Van den Berg, Lin and Manocha's 2008 Reciprocal Velocity Obstacle is
                            built to fix exactly this — see the next page.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>설정.</strong> agent A와 B, 동일한 파라미터, 둘 사이 중점을
                            기준으로 정확히 거울상인 위치, 각자의 <InlineMath math="v_{\text{pref}}"/>가
                            상대의 시작점을 곧장 겨눈다(정면 마주침 직선). tick{" "}
                            <InlineMath math="t"/>에서 둘 다 아직 직선의{" "}
                            <InlineMath math="v_{\text{pref}}"/>를 그대로 쓰고 있다고 하자.
                        </p>
                        <BlockMath math="\text{cone}_A^{(t)} = \text{cone}(\text{apex} = v_B^{(t)}), \qquad \text{cone}_B^{(t)} = \text{cone}(\text{apex} = v_A^{(t)})"/>
                        <Terms items={[
                            ["\\text{cone}_A^{(t)}", "tick t에 A의 금지 영역. B의 현재 속도로부터 계산된다"],
                            ["v_A^{(t)},\\ v_B^{(t)}", "tick t에서 A와 B의 실제 속도"],
                        ]}/>
                        <p>
                            거울 대칭에 의해 둘 다 아직 직선 위에 있는 한{" "}
                            <InlineMath math="v_A^{(t)} = -v_B^{(t)}"/>(같은 속력, 반대 heading)이고,
                            두 원뿔은 서로의 거울상이다. <InlineMath math="v_{\text{pref}}"/>가 두
                            원뿔 모두의 축 위에 정확히 놓여 있으므로, 이제 둘 다를 위반한다. 직선은
                            두 agent 모두에게 동시에, 대칭으로 막혀 있다.
                        </p>
                        <p>
                            <strong>회피.</strong> 둘 다 동일하게 거울상인 후보 격자를 훑고, 비용
                            함수 <InlineMath math="\text{dist}(v, v_{\text{pref}})"/>의 대칭성에
                            의해 둘 다 같은 상대 쪽(가령 각자 자기 오른쪽)의 회피 후보를 고른다.
                            통신 없이 동일한 기하에서 나온, 마치 조율된 듯한 움직임이다.
                        </p>
                        <BlockMath math="v_A^{(t+1)} \ne v_{\text{pref}}, \qquad v_B^{(t+1)} \ne v_{\text{pref}}, \qquad v_A^{(t+1)} = -v_B^{(t+1)}"/>
                        <Terms items={[
                            ["v_A^{(t+1)},\\ v_B^{(t+1)}", "tick t+1에 각 agent가 실행하기로 한 회피 속도"],
                        ]}/>
                        <p>
                            <strong>재발.</strong> tick <InlineMath math="t+2"/>에서 각 agent는
                            <em>상대가 방금 회피한</em> 속도를 apex로 원뿔을 다시 만든다. 구성상 그
                            속도는 이제 안전하게 옆으로 비켜나 있다. 한 tick 전에 막혀 있던 직선{" "}
                            <InlineMath math="v_{\text{pref}}"/>는 이 새 apex 기준으로는 어느 쪽
                            원뿔에도 더는 들어있지 않다:
                        </p>
                        <BlockMath math="v_{\text{pref}} \notin \text{cone}_A(\text{apex} = v_B^{(t+1)}), \qquad v_{\text{pref}} \notin \text{cone}_B(\text{apex} = v_A^{(t+1)})"/>
                        <p>
                            그래서 둘 다 곧장 충돌 직선으로 되돌아가고, tick{" "}
                            <InlineMath math="t+2"/>는 tick <InlineMath math="t"/>와 완전히
                            똑같아 보인다. 같은 대칭적 봉쇄, 같은 조율된 회피, 같은 재발. VO의
                            apex 규칙 어디에도 상대의 회피가 애초에 <em>반응</em>이었다는 사실을
                            붙잡아 두는 부분이 없어, 이 순환이 끊길 이유가 없다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                        <p>
                            van den Berg, Lin, Manocha의 2008년 Reciprocal Velocity Obstacle이
                            바로 이 문제를 고치기 위해 나온다. 다음 페이지에서 다룬다.
                        </p>
                    </>}
                />
            </Proof>
            <T
                en={<p>
                    Try it below: the head-on preset drops two agents on a near-symmetric course
                    (offset just enough to keep the encounter deterministic). Watch the ego inset's
                    chosen velocity (solid) swing away from the preferred one (dashed) and back,
                    tick after tick, closing to a hair's-breadth pass before the two are finally far
                    enough apart for either cone to stop engaging.
                </p>}
                ko={<p>
                    아래에서 직접 보자. head-on 프리셋이 거의 대칭인 코스(마주침이 결정적으로
                    갈리도록 아주 조금만 어긋나 있다)에 agent 둘을 놓는다. ego inset의 선택
                    속도(실선)가 선호 속도(파선)에서 벗어났다가 돌아오길 tick마다 반복하다가,
                    둘 다 원뿔이 더는 걸리지 않을 만큼 멀어지고 나서야 아슬아슬한 간격으로
                    스쳐 지나간다.
                </p>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    The crossing preset is VO's honest single-robot case: one planner-driven agent
                    against one scripted, non-cooperative mover that never yields, so all the
                    avoiding falls on the planner. The head-on preset is the near-symmetric two-agent
                    course from the derivation above — watch the wedge in the ego inset reappear on
                    the same side tick after tick as the two agents scrape past each other.
                </p>}
                ko={<p>
                    crossing 프리셋은 VO의 정직한 단일 로봇 케이스다. planner가 구동하는 agent
                    하나가 절대 양보하지 않는 스크립트된 비협조적 mover 하나를 상대한다. 회피는
                    전부 planner의 몫이다. head-on 프리셋은 위 유도에서 쓴 거의 대칭인 두 agent
                    코스다. 두 agent가 서로를 아슬아슬하게 스쳐 지나가는 동안 ego inset의 쐐기가
                    매 tick 같은 쪽에서 다시 나타나는 모습을 보라.
                </p>}
            />
            <VoSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    <InlineMath math="\text{vo.py}"/>/<InlineMath math="\text{vo.cpp}"/> only wire
                    up the parameters and pick which agent's velocity each cone's apex tracks —
                    the cone geometry, candidate grid, and cost-with-penalty scan it calls into
                    live in <InlineMath math="\text{\_velocity\_obstacle.py}"/>/{" "}
                    <InlineMath math="\text{velocity\_obstacle.cpp}"/>, shared with RVO.
                </p>}
                ko={<p>
                    <InlineMath math="\text{vo.py}"/>/<InlineMath math="\text{vo.cpp}"/>는 파라미터를
                    연결하고 각 원뿔의 apex가 어느 agent의 속도를 따라갈지 고르기만 한다. 실제
                    원뿔 기하, 후보 격자, 페널티 포함 비용 스캔은{" "}
                    <InlineMath math="\text{\_velocity\_obstacle.py}"/>/{" "}
                    <InlineMath math="\text{velocity\_obstacle.cpp}"/>에 있고 RVO와 공유한다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/velocity/vo.py",
                                code: voPy,
                                href: `${REPO}/python/navigation/local_planning/velocity/vo.py`,
                            },
                            {
                                name: "python/navigation/local_planning/velocity/_velocity_obstacle.py",
                                code: velocityObstaclePy,
                                href: `${REPO}/python/navigation/local_planning/velocity/_velocity_obstacle.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/velocity/vo.hpp",
                                code: voHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/velocity/vo.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/velocity/vo.cpp",
                                code: voCpp,
                                href: `${REPO}/cpp/src/local_planning/velocity/vo.cpp`,
                            },
                            {
                                name: "cpp/include/navigation/local_planning/velocity/velocity_obstacle.hpp",
                                code: velocityObstacleHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/velocity/velocity_obstacle.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/velocity/velocity_obstacle.cpp",
                                code: velocityObstacleCpp,
                                href: `${REPO}/cpp/src/local_planning/velocity/velocity_obstacle.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The cone geometry, candidate grid, and VO's apex-at-the-obstacle's-own-velocity rule, embedded from the repository sources",
                    "원뿔 기하, 후보 격자, obstacle 자신의 속도에 apex를 두는 VO 규칙. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    P. Fiorini, Z. Shiller,{" "}
                    <a href="https://doi.org/10.1177/027836499801700706" target="_blank" rel="noopener noreferrer">
                        <em>Motion Planning in Dynamic Environments Using Velocity Obstacles</em>
                    </a>, The International Journal of Robotics Research, vol. 17, no. 7, pp. 760–772, 1998.
                </li>
            </ol>
        </>
    )
}

export default Vo

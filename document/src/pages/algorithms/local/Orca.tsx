import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import OrcaSandbox from "../../../components/panels/local/orca/OrcaSandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import orcaPy from "../../../../../python/navigation/local_planning/velocity/orca.py?raw";
import velocityObstaclePy from "../../../../../python/navigation/local_planning/velocity/_velocity_obstacle.py?raw";
import orcaHpp from "../../../../../cpp/include/navigation/local_planning/velocity/orca.hpp?raw";
import orcaCpp from "../../../../../cpp/src/local_planning/velocity/orca.cpp?raw";
import velocityObstacleHpp from "../../../../../cpp/include/navigation/local_planning/velocity/velocity_obstacle.hpp?raw";
import velocityObstacleCpp from "../../../../../cpp/src/local_planning/velocity/velocity_obstacle.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Orca = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    VO and RVO both answer "which velocities are forbidden" with a cone, and then
                    fall back on sampling a finite candidate grid because a nonconvex cone has no
                    closed-form closest point. Every sample is a resolution trade-off — coarser is
                    cheaper and jitters more, finer is smoother and costs more — and no amount of
                    samples ever gives an exact answer. Jur van den Berg, Stephen Guy, Ming Lin and
                    Dinesh Manocha's 2011 Optimal Reciprocal Collision Avoidance replaces the cone
                    with something a linear program can solve exactly: one half-plane per obstacle,
                    already split reciprocally. This is also the last stop in local planning before
                    the next category — once avoidance between independently-moving bodies is
                    solved reactively, tick by tick, the natural next question is the one{" "}
                    <strong>multi-agent planning</strong> asks instead: what full paths should a
                    whole fleet have committed to in the first place.
                </p>}
                ko={<p>
                    VO와 RVO는 둘 다 "어떤 속도가 금지인가"에 원뿔로 답한 뒤, 비볼록 원뿔에는
                    닫힌 형태의 최근접점이 없어 유한한 후보 격자를 표본화하는 데 기댄다. 표본
                    하나하나가 해상도 타협이다. 성기면 싸지만 더 떨리고, 촘촘하면 매끄럽지만
                    더 비싸다. 표본을 아무리 늘려도 정확한 답은 나오지 않는다. Jur van den Berg,
                    Stephen Guy, Ming Lin, Dinesh Manocha의 2011년 Optimal Reciprocal Collision
                    Avoidance(ORCA)는 원뿔을 선형계획으로 정확히 풀 수 있는 것으로 바꾼다.
                    obstacle마다 half-plane 하나, 그것도 이미 상호적으로 나뉜 half-plane이다.
                    이는 local planning 마지막 정거장이기도 하다. 독립적으로 움직이는 몸체
                    사이의 회피를 매 tick 반응적으로 풀고 나면, 자연스러운 다음 질문은{" "}
                    <strong>multi-agent planning</strong>이 대신 던지는 질문이다. 함대 전체가
                    애초에 어떤 전체 경로에 처음부터 정착했어야 하는가.
                </p>}
            />

            <h2>{t("From Cones to Half-Planes", "원뿔에서 half-plane으로")}</h2>
            <T
                en={<p>
                    Look at the relative velocity <InlineMath math="v_A - v_B"/> instead of the
                    absolute one. VO/RVO's forbidden cone becomes, in relative-velocity terms, the
                    minimal displacement <InlineMath math="u"/> that would push the current relative
                    velocity back onto the boundary of that cone — either the near tangent leg, or
                    the truncation circle at <InlineMath math="\tau"/>, whichever is closer:
                </p>}
                ko={<p>
                    절대 속도 대신 상대 속도 <InlineMath math="v_A - v_B"/>를 보자. VO/RVO의
                    금지 원뿔은 상대 속도 관점에서, 지금의 상대 속도를 그 원뿔의 경계로 되돌리는
                    데 필요한 최소 변위 <InlineMath math="u"/>가 된다. 가까운 접선 leg든{" "}
                    <InlineMath math="\tau"/>에서의 truncation 원이든, 더 가까운 쪽이다:
                </p>}
            />
            <BlockMath math="u = \operatorname*{arg\,min}_{w \in \partial\,VO_A^\tau(B)} \lVert w - (v_A - v_B) \rVert - \big(v_A - v_B\big)"/>
            <T
                en={<Terms items={[
                    ["u", "the minimal vector that would move the current relative velocity onto the VO's boundary — zero-length only when already outside the cone"],
                    ["VO_A^\\tau(B)", <>the same truncated velocity obstacle from VO's page — apex
                        implicitly at 0 in relative-velocity terms, half-angle from{" "}
                        <InlineMath math="r_A + r_B"/> and distance</>],
                    ["\\partial\\, VO_A^\\tau(B)", "the cone's boundary — either of the two tangent legs, or the truncation arc at τ"],
                    ["v_A,\\ v_B", "A's and B's current absolute velocities"],
                ]}/>}
                ko={<Terms items={[
                    ["u", "현재 상대 속도를 VO의 경계로 옮기는 데 필요한 최소 벡터. 이미 원뿔 밖이면 길이 0"],
                    ["VO_A^\\tau(B)", <>VO 페이지와 같은 truncated velocity obstacle. 상대 속도 관점에서
                        apex는 암묵적으로 0이고, 반각은 <InlineMath math="r_A + r_B"/>와 거리에서
                        나온다</>],
                    ["\\partial\\, VO_A^\\tau(B)", "원뿔의 경계. 두 접선 leg 중 하나 또는 τ에서의 truncation 호"],
                    ["v_A,\\ v_B", "A와 B의 현재 절대 속도"],
                ]}/>}
            />
            <T
                en={<p>
                    That displacement's direction is the half-plane's outward normal, and the
                    reciprocal split happens in exactly one place: instead of asking A to absorb all
                    of <InlineMath math="u"/> itself, the plane is anchored halfway between A's
                    current velocity and its corrected one — each side takes half the correction,
                    with no separate reciprocity parameter to tune (unlike RVO, this 50/50 split is
                    baked into where the plane sits, not into where a cone's apex sits):
                </p>}
                ko={<p>
                    그 변위의 방향이 half-plane의 바깥 방향 법선이고, 상호 분담은 정확히 한
                    곳에서 일어난다. A에게 <InlineMath math="u"/> 전체를 혼자 흡수하라고 하는
                    대신, 평면을 A의 현재 속도와 보정된 속도의 중간에 고정한다. 각자 보정의
                    절반씩을 진다. 튜닝할 별도의 reciprocity 파라미터가 없다(RVO와 달리, 이
                    50/50 분담은 원뿔 apex가 아니라 평면이 놓이는 위치 자체에 이미 박혀
                    있다):
                </p>}
            />
            <BlockMath math="ORCA_A^\tau(B) = \left\{\, v \;\middle|\; \big(v - (v_A + \tfrac{1}{2}u)\big) \cdot n \ge 0 \,\right\}"/>
            <T
                en={<Terms items={[
                    ["v", "any candidate absolute velocity for agent A being tested for feasibility"],
                    ["v_A", "A's current absolute velocity — same quantity as RVO's v_self, read from forward speed and heading"],
                    ["u", "the minimal boundary-correcting displacement derived above"],
                    ["n", "unit vector along u — the half-plane's outward normal, pointing away from the forbidden region"],
                ]}/>}
                ko={<Terms items={[
                    ["v", "feasibility를 검사할 A의 임의의 후보 절대 속도"],
                    ["v_A", "A의 현재 절대 속도. RVO의 v_self와 같은 값. 전진 속력과 heading에서 읽는다"],
                    ["u", "위에서 유도한 경계-보정 최소 변위"],
                    ["n", "u 방향의 단위 벡터. half-plane의 바깥 법선. 금지 영역에서 멀어지는 쪽을 향한다"],
                ]}/>}
            />
            <T
                en={<p>
                    Stack one such half-plane per nearby obstacle and the whole avoidance problem
                    collapses into a textbook shape: find the point closest
                    to <InlineMath math="v_{\text{pref}}"/>, inside the max-speed disc, that
                    satisfies every half-plane at once — a 2D linear program. RVO2's incremental
                    algorithm (successively re-optimizing along each violated line) solves it in a
                    handful of lines, and when the half-planes are jointly infeasible — too many
                    obstacles crowding at once for any velocity to satisfy them all — a second,
                    penetration-minimizing 3D linear program takes over instead of the planner simply
                    having no answer.
                </p>}
                ko={<p>
                    근처 obstacle마다 이런 half-plane을 하나씩 쌓으면 회피 문제 전체가 교과서적인
                    모양으로 접힌다. max-speed 원판 안에서 모든 half-plane을 동시에 만족하며{" "}
                    <InlineMath math="v_{\text{pref}}"/>에 가장 가까운 점을 찾는 2D 선형계획이다.
                    RVO2의 점증적 알고리즘(위반된 line을 따라 차례로 재최적화)이 몇 줄만에 이를
                    푼다. half-plane들이 함께 infeasible할 때, 곧 너무 많은 obstacle이 한꺼번에
                    몰려 어떤 속도도 전부를 만족시키지 못할 때는, planner가 그냥 답이 없다고
                    포기하는 대신 침투를 최소화하는 두 번째 3D 선형계획이 대신 나선다.
                </p>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Exact, not sampled.</strong> Given the half-planes, the linear
                        program finds the true closest feasible point (or true minimum-penetration
                        point) — no grid resolution, no candidate count to tune, and no jitter from
                        a fixed sample spacing the way VO/RVO can show near a cone boundary.</li>
                    <li><strong>Cost per tick: <InlineMath math="O(m)"/> to build the half-planes,
                        <InlineMath math="O(m)"/> amortized (worst case <InlineMath math="O(m^2)"/>)
                        for the 2D linear program</strong> over <InlineMath math="m"/> obstacles —
                        each violated line triggers a bounded 1D sub-problem against every earlier
                        line. Typically cheaper than VO/RVO once the sample count needed for a
                        comparably smooth result is factored in.</li>
                    <li><strong>Never fails to return a velocity.</strong> The 3D fallback is a hot-path
                        contract, not a rare exception handler — it always produces a finite point
                        even when every half-plane cannot be satisfied at once, by minimizing total
                        penetration instead of giving up.</li>
                    <li><strong>A provable guarantee, under assumptions VO/RVO never make.</strong>{" "}
                        Van den Berg et al. prove ORCA collision-free for any number of agents that
                        all run the identical algorithm with matching parameters and perceive each
                        other exactly — a real step up from RVO's two-body-only argument, but one
                        that only holds inside those same assumptions (mixed-behavior fleets, sensing
                        noise, or one agent that reacts differently all fall outside it).</li>
                    <li><strong>Still holonomic in its own velocity space.</strong> The half-plane
                        geometry and the linear program both reason about an agent that can be
                        instantaneously commanded to any velocity in the disc — see the section
                        below for what that costs a real differential-drive robot.</li>
                </ul>}
                ko={<ul>
                    <li><strong>표본이 아니라 정확하다.</strong> half-plane이 주어지면 선형계획은
                        진짜 최근접 feasible 점(또는 진짜 최소 침투 점)을 찾는다. 격자 해상도도,
                        튜닝할 후보 개수도 없고, VO/RVO가 원뿔 경계 근처에서 보일 수 있는 고정
                        표본 간격발 떨림도 없다.</li>
                    <li><strong>tick당 비용: half-plane 구성에 <InlineMath math="O(m)"/>, 2D
                        선형계획에 상각 <InlineMath math="O(m)"/>(최악 <InlineMath math="O(m^2)"/>)</strong>{" "}
                        (obstacle <InlineMath math="m"/>개 기준). 위반된 line마다 그 이전 모든
                        line과의 유계 1차원 부분문제가 걸린다. 비슷한 매끄러움을 내려면 필요한
                        VO/RVO의 표본 수까지 감안하면 대개 더 싸다.</li>
                    <li><strong>속도를 반환하지 못하는 경우가 없다.</strong> 3D fallback은 드문
                        예외 처리가 아니라 hot-path 계약이다. 모든 half-plane을 동시에 만족할 수
                        없을 때도 포기하는 대신 총 침투를 최소화해 항상 유한한 점을 낸다.</li>
                    <li><strong>VO/RVO는 갖지 못한 가정 위의 증명된 보장.</strong> van den Berg
                        등은 동일한 알고리즘을 같은 파라미터로 돌리고 서로를 정확히 인지하는
                        임의 수의 agent에 대해 ORCA가 충돌 없음을 증명한다. RVO의 2체 전용
                        논증보다 실제로 한 단계 위지만, 딱 그 가정 안에서만 성립한다(behavior가
                        섞인 함대, 센싱 잡음, 다르게 반응하는 agent 하나는 모두 이 밖이다).</li>
                    <li><strong>여전히 자기 속도공간 안에서는 홀로노믹이다.</strong> half-plane
                        기하도 선형계획도 둘 다 원판 안 어떤 속도로든 즉시 명령할 수 있는
                        agent를 전제로 추론한다. 실제 차동 구동 로봇에 이게 어떤 대가인지는
                        아래 절에서 다룬다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Two separate half-plane batches — neighbors under one time horizon, static cells
                    under a shorter one — feed a single linear program, with the 3D fallback wired
                    in for the infeasible case.
                </p>}
                ko={<p>
                    두 개의 half-plane 묶음, 곧 하나의 time horizon을 쓰는 이웃과 더 짧은
                    horizon을 쓰는 정적 셀이 하나의 선형계획으로 들어간다. infeasible한 경우를
                    위한 3D fallback도 함께 연결돼 있다.
                </p>}
            />
            <Pseudocode code={`statics ← occupied_cells_within(neighbor_dist) as velocity-0 obstacles     # 1
v_pref ← toward(goal), capped at max_speed, tapered near goal              # 2
v_self ← (v · cos(theta), v · sin(theta))                                  # 3
planes ← [orca_half_plane(o.pos − pos, v_self − o.velocity,                # 4
                           v_self, r_self + o.r, time_horizon, dt)
          for o in neighbors if dist(o.pos, pos) < neighbor_dist + o.r]
planes += [orca_half_plane(o.pos − pos, v_self − o.velocity,               # 5
                            v_self, r_self + o.r, time_horizon_obst, dt)
           for o in statics if dist(o.pos, pos) < neighbor_dist + o.r]
ok, v_new, fail ← linear_program_2d(planes, v_pref, max_speed)              # 6
if not ok:                                                                  # 7
    v_new ← linear_program_3d(planes, fail, v_pref, max_speed)              # 8
return velocity_to_command(v_new, theta, max_omega, heading_gain)           # 9`}/>
            <T
                en={<ol>
                    <li>Static obstacles are the same velocity-0 folding as VO/RVO — occupied cells
                        within sensing range, row/col ascending.</li>
                    <li>Same goal-seeking preferred velocity as VO/RVO.</li>
                    <li>Same nonholonomic reading of the robot's own velocity as RVO's{" "}
                        <InlineMath math="v_{\text{self}}"/> — projected from forward speed and
                        heading, not a free 2D value.</li>
                    <li>One half-plane per moving neighbor, at{" "}
                        <InlineMath math="\text{time\_horizon}"/> — this is exactly the geometry
                        derived above, with the reciprocal split already built into where the plane
                        sits.</li>
                    <li>One half-plane per static obstacle cell, at a separately configured
                        (typically shorter) <InlineMath math="\text{time\_horizon\_obst}"/> — a
                        wall's urgency is not governed by the same lookahead as another moving
                        agent's.</li>
                    <li>Both batches feed one 2D linear program together — neighbors and statics are
                        not treated as two separate passes once their half-planes exist.</li>
                    <li><strong>The trap:</strong> checking <InlineMath math="\text{ok}"/> is not
                        optional — the 2D program reports exactly which line it failed on
                        (<InlineMath math="\text{fail}"/>), and treating a failed 2D solve as success
                        would silently hand back a half-planes-satisfying-nothing garbage velocity
                        instead of the deliberately-computed minimum-penetration one.</li>
                    <li>The 3D fallback resumes from the failing line, minimizing total penetration
                        across every remaining constraint — this is the call that guarantees a
                        velocity always comes back, however crowded the scene.</li>
                    <li>Same differential-drive projection as VO/RVO's final step — and the exact
                        gap that projection introduces is the subject of the next section.</li>
                </ol>}
                ko={<ol>
                    <li>정적 obstacle은 VO/RVO와 같은 속도-0 접기다. 감지 범위 안 점유 셀,
                        row/col 오름차순.</li>
                    <li>VO/RVO와 같은 goal 지향 선호 속도.</li>
                    <li>RVO의 <InlineMath math="v_{\text{self}}"/>와 같은 비홀로노믹 해석.
                        자유로운 2D 값이 아니라 전진 속력과 heading에서 사영한다.</li>
                    <li>움직이는 이웃마다 <InlineMath math="\text{time\_horizon}"/>에서
                        half-plane 하나. 위에서 유도한 기하 그대로이며, 상호 분담이 이미 평면이
                        놓이는 위치에 박혀 있다.</li>
                    <li>정적 obstacle 셀마다 별도로 설정된(대개 더 짧은){" "}
                        <InlineMath math="\text{time\_horizon\_obst}"/>에서 half-plane 하나.
                        벽의 긴급도는 움직이는 agent와 같은 lookahead로 다룰 게 아니다.</li>
                    <li>두 묶음 모두 하나의 2D 선형계획으로 함께 들어간다. half-plane이 만들어진
                        다음에는 이웃과 정적을 두 번의 별도 단계로 나누지 않는다.</li>
                    <li><strong>함정.</strong> <InlineMath math="\text{ok}"/> 확인은 선택이
                        아니다. 2D 풀이는 정확히 어느 line에서 실패했는지(<InlineMath math="\text{fail}"/>)를
                        보고하며, 실패한 2D 풀이를 성공으로 취급하면 일부러 계산한 최소 침투
                        속도 대신 아무것도 만족하지 않는 쓰레기 속도를 그대로 돌려주게 된다.</li>
                    <li>3D fallback은 실패한 line부터 다시 시작해 남은 모든 제약에 걸친 총 침투를
                        최소화한다. 장면이 아무리 붐벼도 속도가 항상 돌아오게 보장하는 지점이다.</li>
                    <li>VO/RVO의 마지막 단계와 같은 차동 구동 투영. 그 투영이 만드는 정확한 간극이
                        다음 절의 주제다.</li>
                </ol>}
            />
            <Proof title={t("Why the LP always returns something (never raises)", "LP가 항상 무언가를 반환하는 이유 (절대 raise하지 않는다)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> The 2D program processes half-planes one at a
                            time, keeping a running candidate <InlineMath math="v_i"/> that satisfies
                            every line seen so far. When line <InlineMath math="i"/> is violated, it
                            re-optimizes along that line's own boundary — a 1D sub-problem over an
                            interval <InlineMath math="[t_{\text{left}}, t_{\text{right}}]"/> cut by
                            the max-speed circle and every earlier line.
                        </p>
                        <BlockMath math="t_{\text{left}} > t_{\text{right}} \implies \text{line } i \text{ infeasible with lines } 0..i-1"/>
                        <Terms items={[
                            ["t_{\\text{left}},\\ t_{\\text{right}}", "the current feasible interval along line i's own boundary, after intersecting with every earlier constraint"],
                            ["i", "the index of the half-plane currently being enforced"],
                        ]}/>
                        <p>
                            An empty interval here is exactly a report — <InlineMath math="(\text{false}, v_i)"/>{" "}
                            — not a thrown exception. The 2D program propagates that failure straight
                            up as <InlineMath math="\text{fail\_index} = i"/>, never touching lines
                            <InlineMath math="i{+}1 .. m{-}1"/> at all.
                        </p>
                        <p>
                            <strong>The 3D fallback.</strong> Starting over from line{" "}
                            <InlineMath math="i"/>, it tracks a running <InlineMath math="\text{distance}"/>{" "}
                            (worst violation so far) instead of an all-or-nothing feasibility flag,
                            and re-optimizes each newly-violated line's own 1D sub-problem projected
                            onto every prior line — itself never required to be feasible, because
                            distance can only be improved or left unchanged, never required to reach
                            zero:
                        </p>
                        <BlockMath math="\text{distance}_{i} = -\big(v - p_i\big) \cdot n_i \ \ge 0 \text{ is not required for every } i"/>
                        <Terms items={[
                            ["\\text{distance}_i", "signed penetration depth into half-plane i at the current running result — the quantity being minimized, not zeroed"],
                            ["p_i,\\ n_i", "the point/normal defining half-plane i"],
                        ]}/>
                        <p>
                            Even the innermost 1D sub-problem here can itself report failure (the
                            direction-optimizing variant, over an already-projected line set) — and
                            the fallback's contract is to treat that as "no further improvement
                            available; keep the previous result" rather than propagate it any
                            further. There is no path through either program that requires raising:
                            every branch ends in a Point. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>설정.</strong> 2D 풀이는 half-plane을 하나씩 처리하며, 지금까지
                            본 모든 line을 만족하는 후보 <InlineMath math="v_i"/>를 유지한다. line{" "}
                            <InlineMath math="i"/>가 위반되면 그 line 자신의 경계를 따라
                            재최적화한다. max-speed 원과 이전의 모든 line으로 잘린 구간{" "}
                            <InlineMath math="[t_{\text{left}}, t_{\text{right}}]"/> 위의
                            1차원 부분문제다.
                        </p>
                        <BlockMath math="t_{\text{left}} > t_{\text{right}} \implies \text{line } i \text{는 line } 0..i-1\text{과 infeasible}"/>
                        <Terms items={[
                            ["t_{\\text{left}},\\ t_{\\text{right}}", "이전의 모든 제약과 교차한 뒤, line i 자신의 경계 위에서 현재 feasible한 구간"],
                            ["i", "지금 적용 중인 half-plane의 인덱스"],
                        ]}/>
                        <p>
                            여기서 빈 구간은 정확히 하나의 보고다 <InlineMath math="(\text{false}, v_i)"/>.
                            예외를 던지는 게 아니다. 2D 풀이는 그 실패를{" "}
                            <InlineMath math="\text{fail\_index} = i"/>로 그대로 위로 전파할 뿐,
                            line <InlineMath math="i{+}1 .. m{-}1"/>은 아예 건드리지 않는다.
                        </p>
                        <p>
                            <strong>3D fallback.</strong> line <InlineMath math="i"/>부터 다시
                            시작해, all-or-nothing feasibility 플래그 대신 지금까지의{" "}
                            <InlineMath math="\text{distance}"/>(최악 위반량)를 추적하고, 새로
                            위반된 line마다 그 자신의 1차원 부분문제를 이전 모든 line에 사영해
                            재최적화한다. 이 자체는 feasible일 필요가 없다. distance는 개선되거나
                            그대로 유지될 뿐, 0에 도달해야 할 필요가 없기 때문이다:
                        </p>
                        <BlockMath math="\text{distance}_{i} = -\big(v - p_i\big) \cdot n_i \ \ge 0 \text{이 모든 } i\text{에 대해 요구되지 않는다}"/>
                        <Terms items={[
                            ["\\text{distance}_i", "현재 결과가 half-plane i에 침투한 부호 있는 깊이. 0으로 만드는 게 아니라 최소화 대상"],
                            ["p_i,\\ n_i", "half-plane i를 정의하는 점/법선"],
                        ]}/>
                        <p>
                            여기서 가장 안쪽의 1차원 부분문제(이미 사영된 line 집합 위의
                            direction-optimizing 변형)조차 실패를 보고할 수 있는데, fallback의
                            계약은 이를 "더 이상 개선할 게 없으니 이전 결과를 유지한다"로
                            취급하는 것이지 더 위로 전파하는 게 아니다. 두 풀이 어디에도 raise를
                            요구하는 경로가 없다. 모든 분기가 Point로 끝난다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("The Holonomic Assumption", "홀로노믹 가정의 한계")}</h2>
            <T
                en={<p>
                    Every step above reasons about a robot that can be commanded to any velocity in
                    the max-speed disc, starting next tick, with no cost to changing direction. A
                    differential-drive robot cannot do that — it can only translate along its
                    current heading, and turning takes time governed by{" "}
                    <InlineMath math="\text{max\_omega}"/>. The bridge from ORCA's exact velocity{" "}
                    <InlineMath math="v_{\text{new}}"/> to an actual <InlineMath math="(v, \omega)"/>{" "}
                    command is the same heading-command law VO and RVO already use, and it is exactly
                    where the LP's one real advantage over sampling — an exact answer — gets
                    partially spent back:
                </p>}
                ko={<p>
                    위 모든 단계는 다음 tick부터 방향을 바꾸는 데 아무 비용 없이 max-speed 원판
                    안 어떤 속도로든 명령할 수 있는 로봇을 전제로 추론한다. 차동 구동 로봇은
                    그럴 수 없다. 현재 heading 방향으로만 이동할 수 있고, 회전에는{" "}
                    <InlineMath math="\text{max\_omega}"/>가 정하는 시간이 걸린다. ORCA의 정확한
                    속도 <InlineMath math="v_{\text{new}}"/>에서 실제{" "}
                    <InlineMath math="(v, \omega)"/> 명령으로 가는 다리는 VO/RVO가 이미 쓰는
                    조향 법칙과 같고, 바로 이 지점에서 LP의 진짜 이점, 곧 정확한 답이라는 것이
                    일부 도로 소모된다:
                </p>}
            />
            <BlockMath math="v_{\text{actual}} = \lVert v_{\text{new}} \rVert \max(0, \cos\theta_{\text{err}})\, (\cos\theta,\ \sin\theta), \qquad \theta_{\text{err}} = \text{desired} - \theta"/>
            <T
                en={<Terms items={[
                    ["v_{\\text{actual}}", "the robot's real instantaneous velocity vector this tick — along its current heading θ, not along v_new's direction"],
                    ["v_{\\text{new}}", "ORCA's exact solved velocity — assumed instantly achievable in the derivation above"],
                    ["\\theta", "the robot's current heading, before this tick's turn is applied"],
                    ["\\theta_{\\text{err}}", "the heading error between where v_new points and where the robot is actually facing"],
                ]}/>}
                ko={<Terms items={[
                    ["v_{\\text{actual}}", "이번 tick 로봇의 실제 순간 속도 벡터. v_new의 방향이 아니라 현재 heading θ 방향이다"],
                    ["v_{\\text{new}}", "ORCA가 정확히 풀어낸 속도. 위 유도에서는 즉시 달성 가능하다고 가정된다"],
                    ["\\theta", "이번 tick의 회전이 적용되기 전, 로봇의 현재 heading"],
                    ["\\theta_{\\text{err}}", "v_new가 가리키는 방향과 로봇이 실제로 향한 방향 사이의 heading 오차"],
                ]}/>}
            />
            <T
                en={<p>
                    Whenever <InlineMath math="\theta_{\text{err}}"/> is large — a sudden coalition
                    of half-planes can demand a sharp turn on a single tick — the robot's real
                    velocity this tick is nowhere near <InlineMath math="v_{\text{new}}"/>: the
                    <InlineMath math="\cos"/> gate cuts its speed down, and near
                    <InlineMath math="\theta_{\text{err}} = \pm\pi/2"/> it very nearly stops and
                    turns in place instead of translating along the LP's solution at all. The
                    exact velocity ORCA solved for is realized only gradually, over however many
                    ticks it takes <InlineMath math="\theta"/> to catch up — during which every
                    other agent's half-planes were built assuming this robot's velocity was already
                    the holonomic answer, not the slower, wrong-angle one it is actually executing.
                    This is exactly why ORCA is the standard reactive layer for holonomic platforms
                    (quadrotors, omnidirectional bases, or the point-agents of a crowd simulation) —
                    and why fleets of car-like or differential-drive robots typically treat an
                    ORCA-style velocity as a reference for a separate tracking controller, or hand
                    the harder kinematic questions up to a planner that reasons about full paths
                    ahead of time instead of one velocity per tick.
                </p>}
                ko={<p>
                    <InlineMath math="\theta_{\text{err}}"/>가 클 때마다(half-plane들이 갑자기
                    한꺼번에 몰려 한 tick에 급격한 회전을 요구할 수 있다) 이번 tick 로봇의 실제
                    속도는 <InlineMath math="v_{\text{new}}"/>와 전혀 다르다. <InlineMath math="\cos"/>{" "}
                    게이트가 속도를 깎아내리고, <InlineMath math="\theta_{\text{err}} = \pm\pi/2"/>{" "}
                    근처에서는 LP의 해를 따라 이동하는 대신 거의 제자리에서 멈춰 회전만 한다.
                    ORCA가 풀어낸 정확한 속도는 <InlineMath math="\theta"/>가 따라잡는 데 걸리는
                    몇 tick에 걸쳐서만 점진적으로 실현된다. 그 사이 다른 모든 agent의 half-plane은
                    이 로봇의 속도가 이미 그 홀로노믹 정답이라고 가정한 채 만들어져 있지, 실제로
                    실행 중인 더 느리고 방향이 틀린 속도를 반영하지 않는다. ORCA가 홀로노믹
                    플랫폼(쿼드로터, 전방향 베이스, 또는 군중 시뮬레이션의 점 agent)의 표준
                    반응층인 이유가 바로 이것이고, 차량형이나 차동 구동 로봇 함대가 보통 ORCA풍
                    속도를 별도의 추종 제어기를 위한 참조값으로 다루거나, 더 어려운 운동학적
                    질문을 tick당 속도 하나가 아니라 전체 경로를 미리 추론하는 planner에게 넘기는
                    이유이기도 하다.
                </p>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    Both presets reuse VO and RVO's scenarios. On head-on, watch the ego inset draw
                    a single half-plane line instead of a sampled wedge, and the chosen velocity land
                    exactly on it. Circle swap resolves the same way it did for RVO, without a
                    reciprocity slider to tune — the split is already built into the plane.
                </p>}
                ko={<p>
                    두 프리셋 모두 VO·RVO의 시나리오를 재사용한다. head-on에서는 ego inset에
                    표본화된 쐐기 대신 half-plane 선 하나가 그려지고, 선택된 속도가 정확히 그
                    위에 놓이는 모습을 보라. circle swap은 RVO에서와 같은 방식으로 풀리는데,
                    튜닝할 reciprocity 슬라이더 없이도 그렇다. 분담이 이미 평면 자체에 박혀
                    있기 때문이다.
                </p>}
            />
            <OrcaSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    <InlineMath math="\text{orca.py}"/>/<InlineMath math="\text{orca.cpp}"/> build
                    the two half-plane batches and wire the 2D/3D linear program together — the
                    half-plane construction and both solvers live in the same shared module as
                    VO and RVO's cone machinery.
                </p>}
                ko={<p>
                    <InlineMath math="\text{orca.py}"/>/<InlineMath math="\text{orca.cpp}"/>는 두
                    half-plane 묶음을 만들고 2D/3D 선형계획을 이어 붙인다. half-plane 구성과 두
                    solver 모두 VO·RVO의 원뿔 기반과 같은 공유 모듈에 있다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/velocity/orca.py",
                                code: orcaPy,
                                href: `${REPO}/python/navigation/local_planning/velocity/orca.py`,
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
                                name: "cpp/include/navigation/local_planning/velocity/orca.hpp",
                                code: orcaHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/velocity/orca.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/velocity/orca.cpp",
                                code: orcaCpp,
                                href: `${REPO}/cpp/src/local_planning/velocity/orca.cpp`,
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
                    "The half-plane construction and the 2D/3D linear program, embedded from the repository sources",
                    "half-plane 구성과 2D/3D 선형계획. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    J. van den Berg, S. J. Guy, M. Lin, D. Manocha,{" "}
                    <a href="https://doi.org/10.1007/978-3-642-19457-3_1" target="_blank" rel="noopener noreferrer">
                        <em>Reciprocal n-Body Collision Avoidance</em>
                    </a>, Robotics Research (ISRR 2009), Springer Tracts in Advanced Robotics, vol. 70, pp. 3–19, 2011.
                </li>
            </ol>
        </>
    )
}

export default Orca

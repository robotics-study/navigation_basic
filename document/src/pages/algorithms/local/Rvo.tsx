import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import RvoSandbox from "../../../components/panels/local/rvo/RvoSandbox";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import rvoPy from "../../../../../python/navigation/local_planning/velocity/rvo.py?raw";
import velocityObstaclePy from "../../../../../python/navigation/local_planning/velocity/_velocity_obstacle.py?raw";
import rvoHpp from "../../../../../cpp/include/navigation/local_planning/velocity/rvo.hpp?raw";
import rvoCpp from "../../../../../cpp/src/local_planning/velocity/rvo.cpp?raw";
import velocityObstacleHpp from "../../../../../cpp/include/navigation/local_planning/velocity/velocity_obstacle.hpp?raw";
import velocityObstacleCpp from "../../../../../cpp/src/local_planning/velocity/velocity_obstacle.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Rvo = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    VO's cone puts its apex at the obstacle's own velocity, extrapolated forward
                    unchanged — a fine model for a wall, and a fiction for another VO-driven agent
                    that is, at that very instant, avoiding <em>this</em> robot right back. Two such
                    agents on a symmetric head-on course fall into a swerve-then-relapse loop
                    forever, because neither cone ever accounts for the fact that the other's
                    velocity was itself a reaction. Jur van den Berg, Ming Lin and Dinesh Manocha's
                    2008 Reciprocal Velocity Obstacle keeps every piece of VO's geometry — the same
                    truncated cone, the same candidate scan — and changes exactly one thing: where
                    the apex sits.
                </p>}
                ko={<p>
                    VO의 원뿔은 apex를 obstacle 자신의 속도, 즉 바뀌지 않고 앞으로 그대로 외삽한
                    값에 둔다. 벽에는 좋은 모델이지만, 바로 그 순간에 <em>이</em> 로봇을 마찬가지로
                    피하고 있는 다른 VO 기반 agent에는 허구다. 그런 agent 둘을 대칭적인 정면
                    마주침 코스에 두면 회피와 재발을 영원히 반복하는 고리에 빠지는데, 어느 쪽
                    원뿔도 상대의 속도 자체가 반응이었다는 사실을 반영하지 못하기 때문이다.
                    Jur van den Berg, Ming Lin, Dinesh Manocha의 2008년 Reciprocal Velocity
                    Obstacle(RVO)은 VO의 기하는 전부 그대로 둔다. 같은 truncated cone, 같은 후보
                    스캔이다. 딱 한 가지, apex의 위치만 바꾼다.
                </p>}
            />

            <h2>{t("Splitting the Avoidance in Half", "회피 책임을 절반씩 나눈다")}</h2>
            <T
                en={<p>
                    Instead of anchoring the cone at the obstacle's velocity alone, RVO shifts the
                    apex toward the midpoint of both agents' current velocities:
                </p>}
                ko={<p>
                    원뿔을 obstacle의 속도 하나에만 고정하는 대신, RVO는 apex를 두 agent 현재
                    속도의 중점 쪽으로 옮긴다:
                </p>}
            />
            <BlockMath math="\text{apex} = (1 - \rho)\, v_{\text{other}} + \rho\, v_{\text{self}}"/>
            <T
                en={<Terms items={[
                    ["\\text{apex}", "the RVO cone's apex — everything else about the cone (axis, half-angle, truncation) is unchanged from VO"],
                    ["v_{\\text{other}}", "the neighboring agent's (or static obstacle's) current velocity — VO's own apex, recovered at \\rho = 0"],
                    ["v_{\\text{self}}", "this robot's own current velocity, projected from its heading and speed — new to RVO"],
                    ["\\rho", <>reciprocity — new to RVO. 0 recovers plain VO exactly (apex = <InlineMath math="v_{\\text{other}}"/>);
                        1 collapses the apex onto <InlineMath math="v_{\\text{self}}"/>; van den Berg et al.'s original formulation
                        fixes it at 0.5, splitting the avoidance burden evenly</>],
                ]}/>}
                ko={<Terms items={[
                    ["\\text{apex}", "RVO 원뿔의 apex. 원뿔의 다른 모든 것(축, 반각, truncation)은 VO에서 바뀌지 않는다"],
                    ["v_{\\text{other}}", "이웃 agent(또는 정적 obstacle)의 현재 속도. \\rho = 0에서 VO 자신의 apex가 그대로 복원된다"],
                    ["v_{\\text{self}}", "이 로봇 자신의 현재 속도. heading과 속력에서 사영한 값. RVO에서 새로 추가된 항"],
                    ["\\rho", <>reciprocity. RVO에서 새로 추가된 항. 0이면 정확히 순수 VO(apex ={" "}
                        <InlineMath math="v_{\\text{other}}"/>)가 복원되고, 1이면 apex가{" "}
                        <InlineMath math="v_{\\text{self}}"/>에 붕괴한다. van den Berg 등의 원 논문은
                        0.5로 고정해 회피 부담을 절반씩 나눈다</>],
                ]}/>}
            />
            <T
                en={<p>
                    At <InlineMath math="\rho = 0.5"/> the apex sits at the average of both
                    velocities rather than at either one alone — each agent's cone now already
                    reflects that <em>it too</em> is expected to move, not just the other side.
                    That single shift is enough to turn a mutual fiction into a mutual acknowledgment,
                    and it is the entire difference between this page and the last: same cone
                    shape, same candidate scan, same cost function — only the anchor point changes.
                </p>}
                ko={<p>
                    <InlineMath math="\rho = 0.5"/>에서 apex는 둘 중 하나가 아니라 두 속도의
                    평균에 놓인다. 이제 각 agent의 원뿔은 상대만이 아니라 <em>자기 자신도</em>{" "}
                    움직일 것으로 기대된다는 사실을 이미 반영하고 있다. 그 한 번의 이동만으로
                    상호 허구가 상호 인정으로 바뀐다. 그리고 이것이 이 페이지와 앞 페이지의
                    전부다. 같은 원뿔 모양, 같은 후보 스캔, 같은 비용 함수. 오직 anchor
                    지점만 바뀐다.
                </p>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Identical cost to VO: <InlineMath math="O(m \cdot n_s)"/>.</strong>{" "}
                        Shifting the apex is <InlineMath math="O(1)"/> extra work per obstacle; the
                        candidate grid, scan, and penalty cost are untouched.</li>
                    <li><strong><InlineMath math="v_{\text{self}}"/> is a nonholonomic
                        approximation.</strong> This implementation reads it off the robot's current
                        forward speed and heading (<InlineMath math="v\cos\theta,\ v\sin\theta"/>),
                        not a freely-chosen 2D velocity — a differential-drive robot cannot actually
                        hold an arbitrary lateral velocity, so the midpoint apex is itself only an
                        approximation of what the original (holonomic-agent) paper assumes.</li>
                    <li><strong>Fixes the two-body symmetric case cleanly.</strong> The proof below
                        shows the head-on oscillation from VO's page has no analog here at{" "}
                        <InlineMath math="\rho=0.5"/>: the apex sits at a point that does not itself
                        shift depending on which of the two mirrored evasive velocities either agent
                        happens to be running.</li>
                    <li><strong>Still heuristic beyond two agents.</strong> The reciprocal-apex
                        argument is built and verified for exactly one other agent at a time; nothing
                        in the rule coordinates <em>which</em> way several agents should split around
                        each other when more than one cone binds at once — see the next section.</li>
                    <li><strong>No stronger guarantee than VO's.</strong> RVO is still a finite
                        sampled grid scored by distance-to-preferred plus a penalty — no formal
                        completeness or collision-free proof, just a fix to one specific failure
                        mode.</li>
                </ul>}
                ko={<ul>
                    <li><strong>VO와 동일한 비용 <InlineMath math="O(m \cdot n_s)"/>.</strong>{" "}
                        apex를 옮기는 건 obstacle당 <InlineMath math="O(1)"/> 추가 작업뿐이고,
                        후보 격자·스캔·페널티 비용은 그대로다.</li>
                    <li><strong><InlineMath math="v_{\text{self}}"/>는 비홀로노믹 근사다.</strong>{" "}
                        이 구현은 이를 로봇의 현재 전진 속력과 heading에서 읽는다
                        (<InlineMath math="v\cos\theta,\ v\sin\theta"/>). 자유롭게 고를 수 있는
                        2D 속도가 아니다. 차동 구동 로봇은 실제로 임의의 횡방향 속도를 가질 수
                        없으므로, 중점 apex 자체가 원 논문(홀로노믹 agent를 가정)의 근사일
                        뿐이다.</li>
                    <li><strong>2체 대칭 케이스는 깔끔하게 고친다.</strong> 아래 증명은{" "}
                        <InlineMath math="\rho=0.5"/>에서 VO 페이지의 head-on 진동에 대응하는
                        현상이 없음을 보인다. apex가, 둘 중 어느 거울상 회피 속도를 실행 중이든
                        그에 따라 움직이지 않는 지점에 놓이기 때문이다.</li>
                    <li><strong>둘을 넘어서면 여전히 휴리스틱이다.</strong> reciprocal apex
                        논증은 한 번에 상대 agent 하나만을 두고 구성되고 검증된다. 원뿔이 여러 개
                        동시에 걸릴 때 여러 agent가 <em>어느 쪽으로</em> 서로 갈라져야 하는지
                        조율하는 것은 규칙 어디에도 없다. 다음 절을 보라.</li>
                    <li><strong>VO보다 강한 보장은 없다.</strong> RVO도 여전히 선호 속도까지의
                        거리 + 페널티로 채점되는 유한 표본 격자다. 형식적 완전성이나 충돌 없음
                        증명은 없다. 딱 한 가지 실패 모드에 대한 수정일 뿐이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Line for line, this is VO's algorithm — the pseudocode differs in exactly one
                    place: how the apex passed into each cone is computed.
                </p>}
                ko={<p>
                    한 줄 한 줄이 VO의 알고리즘이다. pseudocode는 딱 한 곳, 각 원뿔에 넘기는
                    apex를 계산하는 방식에서만 다르다.
                </p>}
            />
            <Pseudocode code={`statics ← occupied_cells_within(neighbor_dist) as velocity-0 obstacles     # 1
obstacles ← neighbors + statics                                            # 2
v_pref ← toward(goal), capped at max_speed, tapered near goal              # 3
v_self ← (v · cos(theta), v · sin(theta))                                  # 4
cones ← [truncated_vo_cone(o.pos − pos, r_self + o.r,                      # 5
                            rvo_apex(v_self, o.velocity, reciprocity), tau)
         for o in obstacles if dist(o.pos, pos) < neighbor_dist + o.r]
candidates ← [v_pref] + polar_grid(max_speed, speed_samples, angle_samples) # 6
best ← v_pref, best_cost ← 0                                               # 7
for v in candidates:                                                        # 8
    violated ← any(in_velocity_obstacle(v, c) for c in cones)               # 9
    cost ← dist(v, v_pref) + (PENALTY if violated else 0)                  # 10
    if cost < best_cost: best, best_cost ← v, cost                         # 11
return velocity_to_command(best, theta, max_omega, heading_gain)           # 12`}/>
            <T
                en={<ol>
                    <li>Static obstacles fold in as velocity-0 bodies, exactly as on VO's page.</li>
                    <li>Neighbors and statics are concatenated before scanning — the apex shift below
                        applies uniformly to both (a wall's own velocity is always zero, so its apex
                        collapses toward <InlineMath math="\rho \cdot v_{\text{self}}"/> alone).</li>
                    <li>The preferred velocity is identical to VO's — goal-seeking, tapering near
                        the goal.</li>
                    <li><strong>The one new line.</strong> The robot's own current velocity is
                        recovered from its forward speed and heading, not stored as a free 2D
                        vector — the nonholonomic approximation flagged above.</li>
                    <li>Each cone's apex is now <InlineMath math="\text{rvo\_apex}(v_{\text{self}},
                        o.\text{velocity}, \rho)"/> instead of <InlineMath math="o.\text{velocity}"/>{" "}
                        alone — every other part of the cone (axis, half-angle, truncation by{" "}
                        <InlineMath math="\tau"/>) is untouched from VO's derivation.</li>
                    <li>The candidate grid is bit-identical to VO's: <InlineMath math="v_{\text{pref}}"/>{" "}
                        first, then the same deterministic polar sweep.</li>
                    <li>Same zero-cost initialization as VO — an unobstructed tick still never has
                        to check a single cone to know it has already won.</li>
                    <li>Same scan, same strict tie-break rule.</li>
                    <li>Same cost function: distance to <InlineMath math="v_{\text{pref}}"/>, plus
                        the fixed penalty for violating any cone.</li>
                    <li>Same guaranteed win for any truly admissible candidate over one that merely
                        minimizes penetration.</li>
                    <li>Same differential-drive projection as VO's final step.</li>
                </ol>}
                ko={<ol>
                    <li>정적 obstacle은 VO 페이지와 똑같이 속도 0인 몸체로 접힌다.</li>
                    <li>이웃과 정적 obstacle을 이어 붙인 뒤 스캔한다. 아래 apex 이동은 둘 모두에
                        똑같이 적용된다(벽 자신의 속도는 항상 0이라, 벽의 apex는{" "}
                        <InlineMath math="\rho \cdot v_{\text{self}}"/> 하나로 붕괴한다).</li>
                    <li>선호 속도는 VO와 동일하다. goal을 향하고, goal 근처에서 줄어든다.</li>
                    <li><strong>새로 추가된 한 줄.</strong> 로봇 자신의 현재 속도를 자유로운
                        2D 벡터로 저장해 두는 대신 전진 속력과 heading에서 되살린다. 위에서 짚은
                        비홀로노믹 근사다.</li>
                    <li>각 원뿔의 apex는 이제 <InlineMath math="o.\text{velocity}"/> 하나가
                        아니라 <InlineMath math="\text{rvo\_apex}(v_{\text{self}},
                        o.\text{velocity}, \rho)"/>다. 원뿔의 나머지(축, 반각,{" "}
                        <InlineMath math="\tau"/>에 의한 truncation)는 VO의 유도에서 전혀
                        바뀌지 않았다.</li>
                    <li>후보 격자는 VO와 bit-identical하다. <InlineMath math="v_{\text{pref}}"/>가
                        먼저, 그다음 같은 결정적 극좌표 스캔이다.</li>
                    <li>VO와 같은 비용 0 초기화. 막힌 게 없는 tick은 여전히 원뿔을 하나도
                        검사하지 않고도 이미 이겼다는 걸 안다.</li>
                    <li>같은 스캔, 같은 strict 동률 처리 규칙.</li>
                    <li>같은 비용 함수. <InlineMath math="v_{\text{pref}}"/>까지의 거리에 원뿔
                        위반 시 고정 페널티.</li>
                    <li>침투를 최소화할 뿐인 후보보다 진짜 admissible한 후보가 항상 이기는 것도
                        동일하다.</li>
                    <li>VO의 마지막 단계와 같은 차동 구동 투영.</li>
                </ol>}
            />
            <Proof title={t("Why reciprocity 0.5 breaks the two-body oscillation", "reciprocity 0.5가 2체 진동을 끊는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Same symmetric two-agent course as VO's proof:
                            A and B mirror images through the midpoint, both running{" "}
                            <InlineMath math="\rho = 0.5"/>. Suppose a symmetric fixed point exists —
                            some velocity <InlineMath math="v^*"/> such that A commits
                            to <InlineMath math="v^*"/> and B, by the mirror symmetry, commits
                            to <InlineMath math="-v^*"/>, tick after tick.
                        </p>
                        <BlockMath math="\text{apex}_A = 0.5\, v_A + 0.5\, v_B = 0.5\, v^* + 0.5\,(-v^*) = 0"/>
                        <Terms items={[
                            ["\\text{apex}_A", "A's cone apex at this fixed point"],
                            ["v_A,\\ v_B", "A's and B's own current velocities — both equal to the fixed point's v* and -v* respectively"],
                        ]}/>
                        <p>
                            The same computation gives <InlineMath math="\text{apex}_B = 0"/> as
                            well — at <InlineMath math="\rho = 0.5"/>, both apexes land at the
                            velocity-space origin regardless of what <InlineMath math="v^*"/>{" "}
                            actually is, because the two terms are exact opposites and cancel. This
                            is the property VO's apex never has: there,{" "}
                            <InlineMath math="\text{apex}_A = v_B = -v^*"/> depends explicitly on
                            whichever evasive velocity B is currently running.
                        </p>
                        <p>
                            <strong>Consequence.</strong> Both cones are now centered at the same
                            fixed point (the origin) regardless of which mirrored
                            pair <InlineMath math="(v^*, -v^*)"/> is running — the cone A sees does
                            not shift depending on B's choice, and vice versa. With
                            relative position mirrored as well (<InlineMath math="\text{rel}_A =
                            -\text{rel}_B"/>), the two cones are themselves exact mirror images
                            (same half-angle <InlineMath math="\phi"/>, opposite axis):
                        </p>
                        <BlockMath math="v^* \notin \text{cone}_A \iff -v^* \notin \text{cone}_B"/>
                        <p>
                            So once A finds an admissible <InlineMath math="v^*"/> clearing its own
                            (now apex-stable) cone, B's mirrored choice <InlineMath math="-v^*"/>{" "}
                            clears its cone automatically, by the same geometry — and neither cone
                            moves out from under that choice at the next tick, because the apex
                            never depended on <InlineMath math="v^*"/> in the first place. The
                            mirrored pair is a genuine fixed point, not a state that gets undone one
                            tick later the way VO's straight-line relapse was. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>설정.</strong> VO 증명과 같은 대칭 2-agent 코스. A와 B는
                            중점을 기준으로 거울상이고 둘 다 <InlineMath math="\rho = 0.5"/>를
                            쓴다. 대칭 고정점이 존재한다고 하자. 곧 A는 어떤 속도{" "}
                            <InlineMath math="v^*"/>에 정착하고, 거울 대칭에 의해 B는{" "}
                            <InlineMath math="-v^*"/>에 tick마다 계속 정착한다고 하자.
                        </p>
                        <BlockMath math="\text{apex}_A = 0.5\, v_A + 0.5\, v_B = 0.5\, v^* + 0.5\,(-v^*) = 0"/>
                        <Terms items={[
                            ["\\text{apex}_A", "이 고정점에서 A의 원뿔 apex"],
                            ["v_A,\\ v_B", "A와 B 자신의 현재 속도. 고정점의 v*와 -v*와 각각 같다"],
                        ]}/>
                        <p>
                            같은 계산으로 <InlineMath math="\text{apex}_B = 0"/>도 나온다.{" "}
                            <InlineMath math="\rho = 0.5"/>에서는 <InlineMath math="v^*"/>가
                            실제로 무엇이든 상관없이 두 apex 모두 속도공간 원점에 놓인다. 두 항이
                            정확히 반대라 상쇄되기 때문이다. 이것이 VO의 apex는 결코 갖지 못하는
                            성질이다. VO에서는 <InlineMath math="\text{apex}_A = v_B = -v^*"/>가
                            B가 지금 실행 중인 회피 속도가 정확히 무엇인지에 명시적으로 좌우된다.
                        </p>
                        <p>
                            <strong>결론.</strong> 어느 거울상 쌍 <InlineMath math="(v^*, -v^*)"/>가
                            실행 중이든 상관없이 두 원뿔 모두 같은 고정점(원점)에 중심을 둔다. A가
                            보는 원뿔은 B의 선택에 따라 움직이지 않고, 그 반대도 마찬가지다.
                            상대 위치도 거울상이므로(<InlineMath math="\text{rel}_A =
                            -\text{rel}_B"/>) 두 원뿔 자체도 정확히 거울상이다(같은 반각{" "}
                            <InlineMath math="\phi"/>, 반대 축):
                        </p>
                        <BlockMath math="v^* \notin \text{cone}_A \iff -v^* \notin \text{cone}_B"/>
                        <p>
                            그래서 A가 자기 원뿔(이제 apex가 안정된)을 벗어나는 admissible{" "}
                            <InlineMath math="v^*"/>를 찾으면, B의 거울상 선택{" "}
                            <InlineMath math="-v^*"/>도 같은 기하에 의해 자동으로 자기 원뿔을
                            벗어난다. 그리고 apex가 애초에 <InlineMath math="v^*"/>에 좌우되지
                            않았으므로 다음 tick에도 어느 원뿔도 그 선택 밑에서 움직이지 않는다.
                            거울상 쌍은 VO의 직선 재발처럼 한 tick 뒤에 무효화되는 상태가 아니라
                            진짜 고정점이다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>{t("The Crowded Deadlock", "밀집 환경의 교착")}</h2>
            <T
                en={<p>
                    The proof above is explicitly a two-body argument — it needs exactly one other
                    agent's velocity to average against, and the apex-cancellation trick only works
                    because there is exactly one term to cancel against. Add more agents around a
                    shared crossing point and the cones multiply, but the reciprocal apex still only
                    ever splits the responsibility between <em>this</em> robot and one obstacle at a
                    time — nothing coordinates which of several simultaneously-binding cones the
                    candidate scan should escape through first, or in which consistent rotational
                    sense a whole group should slip past each other.
                </p>}
                ko={<p>
                    위 증명은 명시적으로 2체 논증이다. 평균을 낼 다른 agent의 속도가 정확히
                    하나 있어야 하고, apex가 상쇄되는 트릭도 상쇄할 항이 정확히 하나이기 때문에
                    성립한다. 공유 교차점 주변에 agent를 더 두면 원뿔은 늘어나지만, reciprocal
                    apex는 여전히 <em>이</em> 로봇과 obstacle 하나 사이의 책임만 매번 나눌 뿐이다.
                    동시에 여러 원뿔이 걸릴 때 후보 스캔이 어느 원뿔을 먼저 빠져나가야 하는지,
                    또는 그룹 전체가 일관된 회전 방향으로 서로를 비켜 가야 하는지는 아무것도
                    조율해 주지 않는다.
                </p>}
            />
            <T
                en={<p>
                    Push the symmetry further than the two-body case: put several agents at the
                    vertices of a regular polygon, each heading for the antipodal point straight
                    through the shared center. Every agent's candidate scan is now blocked by{" "}
                    <InlineMath math="n-1"/> cones at once instead of one, and rotational symmetry
                    means every agent sees the identical geometry rotated — whichever side one
                    picks to swerve toward, its neighbors pick the same relative side by the same
                    symmetry argument that made the two-body proof work, except now that leaves
                    every agent's chosen escape blocked by the next agent's identical escape rather
                    than clearing it. The group can freeze in a mutual near-standstill rather than
                    finding the rotational shuffle that would let everyone through — a scenario the
                    scripted circle-swap preset below is deliberately built to avoid (its lane
                    offsets break the exact rotational symmetry that causes the freeze), precisely
                    because a literal antipodal swap does not resolve cleanly under RVO.
                </p>}
                ko={<p>
                    대칭을 2체 케이스보다 더 밀어붙여 보자. agent 여럿을 정다각형의 꼭짓점에
                    두고, 각자 공유 중심을 정확히 관통해 대척점을 향하게 한다. 이제 모든
                    agent의 후보 스캔은 하나가 아니라 <InlineMath math="n-1"/>개의 원뿔에
                    동시에 막힌다. 회전 대칭이라는 것은 모든 agent가 회전된 동일한 기하를
                    본다는 뜻이고, 어느 쪽으로 회피하든 이웃도 2체 증명을 성립시켰던 것과 같은
                    대칭 논증으로 같은 상대 쪽을 고른다. 다만 이번엔 그것이 서로를 벗어나게
                    해주는 게 아니라, 각 agent가 고른 회피가 바로 다음 agent의 똑같은 회피에
                    막히는 결과를 낳는다. 그룹은 모두가 빠져나갈 회전형 셔플을 찾는 대신 상호
                    거의 정지 상태로 얼어붙을 수 있다. 아래 scripted circle-swap 프리셋이
                    의도적으로 이 상황을 피해 구성된 이유이기도 하다(레인 오프셋이 얼어붙음을
                    일으키는 정확한 회전 대칭을 깬다). 말 그대로의 대척점 스왑은 RVO에서
                    깔끔하게 풀리지 않기 때문이다.
                </p>}
            />

            <h2>Demo</h2>
            <T
                en={<p>
                    Head-on is the same near-symmetric course VO oscillated on — drag reciprocity
                    down toward 0 and watch the oscillation return, or leave it at 0.5 and watch both
                    agents settle onto a single consistent swerve with a far more comfortable margin.
                    Circle swap runs four agents through a shared crossing region with its rotational
                    symmetry deliberately broken (a per-lane offset), so all four resolve without the
                    deadlock described above.
                </p>}
                ko={<p>
                    head-on은 VO가 진동했던 그 거의 대칭인 코스와 같다. reciprocity를 0 쪽으로
                    내리면 진동이 되살아나고, 0.5로 두면 두 agent 모두 훨씬 여유 있는 마진으로
                    일관된 회피 하나에 정착하는 모습을 볼 수 있다. circle swap은 회전 대칭을
                    의도적으로 깬(레인별 오프셋) 공유 교차 영역을 네 agent가 통과하는 모습이라,
                    위에서 설명한 교착 없이 넷 모두 풀린다.
                </p>}
            />
            <RvoSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    <InlineMath math="\text{rvo.py}"/>/<InlineMath math="\text{rvo.cpp}"/> add
                    exactly the reciprocity parameter and the apex-shift call — everything else
                    (cone geometry, candidate grid, penalty scan) is the shared module from VO's
                    page, unchanged.
                </p>}
                ko={<p>
                    <InlineMath math="\text{rvo.py}"/>/<InlineMath math="\text{rvo.cpp}"/>는
                    reciprocity 파라미터와 apex 이동 호출만 추가한다. 나머지(원뿔 기하, 후보
                    격자, 페널티 스캔)는 VO 페이지의 공유 모듈 그대로다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/velocity/rvo.py",
                                code: rvoPy,
                                href: `${REPO}/python/navigation/local_planning/velocity/rvo.py`,
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
                                name: "cpp/include/navigation/local_planning/velocity/rvo.hpp",
                                code: rvoHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/velocity/rvo.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/velocity/rvo.cpp",
                                code: rvoCpp,
                                href: `${REPO}/cpp/src/local_planning/velocity/rvo.cpp`,
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
                    "The reciprocal apex shift on top of VO's shared cone/candidate machinery, embedded from the repository sources",
                    "VO의 공유 원뿔/후보 기반 위에 얹은 reciprocal apex 이동. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    J. van den Berg, M. Lin, D. Manocha,{" "}
                    <a href="https://doi.org/10.1109/ROBOT.2008.4543489" target="_blank" rel="noopener noreferrer">
                        <em>Reciprocal Velocity Obstacles for Real-Time Multi-Agent Navigation</em>
                    </a>, Proceedings of IEEE International Conference on Robotics and Automation (ICRA), 2008.
                </li>
            </ol>
        </>
    )
}

export default Rvo

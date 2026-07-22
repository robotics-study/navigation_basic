import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import DwaSandbox from "../../../components/panels/local/dwa/DwaSandbox";
import LocalVelocityWindow from "../../../components/panels/intro/LocalVelocityWindow";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import dwaPy from "../../../../../python/navigation/local_planning/reactive/dwa.py?raw";
import dwaHpp from "../../../../../cpp/include/navigation/local_planning/reactive/dwa.hpp?raw";
import dwaCpp from "../../../../../cpp/src/local_planning/reactive/dwa.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식적 전개는 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Dwa = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Every planner discussed so far searches for a path first and worries about how to
                    drive it second. The Dynamic Window Approach skips the path entirely. At every control
                    step it asks a narrower question directly: of the commands this robot's motors can
                    actually deliver next cycle, which one drives it best toward the goal without running
                    into anything? Dieter Fox, Wolfram Burgard and Sebastian Thrun introduced the method in
                    1997 for indoor mobile robots that needed to react to people and obstacles in real time,
                    and a direct descendant of their formulation still ships as a default local planner in
                    ROS navigation stacks today.
                </p>}
                ko={<p>
                    지금까지 다룬 planner는 모두 먼저 경로를 찾고, 그 경로를 어떻게 몰고 갈지는 나중
                    문제로 미뤘다. Dynamic Window Approach(DWA)는 경로 자체를 아예 건너뛴다. 매 제어
                    스텝마다 더 좁은 질문을 바로 던진다. 로봇의 모터가 다음 주기에 실제로 낼 수 있는
                    명령들 중, 아무것도 들이받지 않으면서 목표에 가장 잘 다가가는 명령은 무엇인가.
                    Dieter Fox, Wolfram Burgard, Sebastian Thrun이 1997년 실내 이동 로봇이 사람과
                    장애물에 실시간으로 반응해야 하는 문제를 풀기 위해 이 방법을 내놓았고, 그 정식화의
                    직계 후손이 지금도 ROS navigation 스택의 기본 local planner로 쓰이고 있다.
                </p>}
            />

            <h2>{t("Sampling the Velocity Space", "속도 공간을 샘플링한다")}</h2>
            <T
                en={<>
                    <p>
                        The shift in perspective is the whole idea: instead of searching over robot
                        positions, DWA searches over robot <strong>commands</strong>. A differential-drive
                        robot's command is a pair <InlineMath math="(v, \omega)"/> — forward speed and turn
                        rate — and at any instant only a small region of that plane is worth considering at
                        all. R. Simmons' Curvature-Velocity Method (1996) got there first: it scored
                        circular arcs directly in a velocity-like space rather than a Cartesian one, which
                        is the conceptual leap DWA inherits. What Fox, Burgard and Thrun added on top is the
                        piece that makes the search tractable and physically honest — bound the search to
                        exactly the box the motors can reach.
                    </p>
                    <p>
                        Two limits carve out that box. The robot's top speed and turn rate bound the
                        <strong> admissible velocity space</strong> outright. Its acceleration limits bound
                        something tighter: starting from the current <InlineMath math="(v, \omega)"/>, one
                        control cycle of length <InlineMath math="\Delta t"/> can only reach velocities
                        within <InlineMath math="\dot v_b \Delta t"/> and <InlineMath math="\dot\omega_b
                        \Delta t"/> of where the robot already is. That reachable rectangle is the{" "}
                        <strong>dynamic window</strong> — small, centered on the current velocity, and
                        moving with it tick by tick. DWA only ever samples inside the intersection of the
                        two: the dynamic window clipped to the admissible space.
                    </p>
                </>}
                ko={<>
                    <p>
                        관점의 전환이 이 알고리즘의 전부다. 로봇의 위치를 탐색하는 대신 로봇의{" "}
                        <strong>명령</strong>을 탐색한다. 차동 구동 로봇의 명령은 전진 속도와 회전율의
                        쌍 <InlineMath math="(v, \omega)"/>이고, 어느 순간이든 그 평면에서 고려할 가치가
                        있는 영역은 아주 작은 일부뿐이다. R. Simmons의 Curvature-Velocity Method(1996)가
                        먼저 이 길을 텄다. 데카르트 공간이 아니라 속도류 공간에서 곧바로 원호를
                        채점한다는 발상 자체가 DWA가 물려받은 개념적 도약이다. 그 위에 Fox, Burgard,
                        Thrun이 얹은 것은 탐색을 실제로 다룰 만하게, 그리고 물리적으로 정직하게 만드는
                        조각이다. 탐색 범위를 모터가 실제로 도달 가능한 상자로 정확히 제한하는 것이다.
                    </p>
                    <p>
                        그 상자는 두 종류의 한계로 정해진다. 로봇의 최고 속도와 최대 회전율은{" "}
                        <strong>admissible 속도 공간</strong> 자체를 정한다. 가속 한계는 그보다 더 좁은
                        영역을 정한다. 현재 <InlineMath math="(v, \omega)"/>에서 시작해 길이{" "}
                        <InlineMath math="\Delta t"/>인 제어 주기 한 번으로는 지금 속도로부터{" "}
                        <InlineMath math="\dot v_b \Delta t"/>, <InlineMath math="\dot\omega_b \Delta t"/>{" "}
                        이내의 속도까지만 도달할 수 있다. 그 도달 가능한 사각형이{" "}
                        <strong>dynamic window</strong>다. 작고, 현재 속도를 중심으로 하고, tick마다 함께
                        움직인다. DWA는 이 두 영역의 교집합, 곧 dynamic window를 admissible 공간으로
                        잘라낸 부분 안에서만 표본을 뽑는다.
                    </p>
                </>}
            />
            <LocalVelocityWindow/>
            <T
                en={<p>
                    Every command sampled from that intersection is rolled out for a short horizon and
                    scored against a single objective that trades off three things at once: does it point
                    the robot at the goal, does it keep distance from obstacles, and is it fast. Fixed
                    weights combine them:
                </p>}
                ko={<p>
                    그 교집합에서 뽑힌 후보 명령은 각각 짧은 horizon만큼 굴려본 뒤 세 가지를 동시에
                    저울질하는 하나의 목적함수로 채점된다. 로봇을 목표 쪽으로 향하게 하는가, 장애물과
                    거리를 두는가, 그리고 빠른가. 고정된 가중치로 이 셋을 합친다:
                </p>}
            />
            <BlockMath math="G(v, \omega) = \alpha \cdot H(v, \omega) + \beta \cdot C(v, \omega) + \gamma \cdot V(v, \omega)"/>
            <Terms items={[
                ["G", <>후보 <InlineMath math="(v, \omega)"/>의 총점 — admissible 후보 중 이 값이 가장
                    큰 것을 명령으로 낸다</>],
                ["H", "목표 방위 정합도 — 롤아웃 종점에서 목표를 바라보는 방향에 가까울수록 1에 가깝다"],
                ["C", "정규화된 clearance — 롤아웃을 따라 가장 가까웠던 장애물까지의 거리(상한 클램프)"],
                ["V", "정규화된 속도 — 빠를수록 점수가 높아, 제자리에서 맴도는 대신 전진을 선호한다"],
                ["\\alpha,\\ \\beta,\\ \\gamma", "세 항의 가중치 — 튜닝 손잡이는 이 셋뿐이다"],
            ]}/>
            <T
                en={<p>
                    Every candidate is scored on its own, with no reference to the rest of the batch — the
                    original paper smooths and normalizes across the whole candidate set each tick, but this
                    implementation fixes the normalization instead, so scoring one candidate never depends
                    on what else happened to be sampled that tick. That trade keeps the result deterministic
                    and exactly reproducible run to run.
                </p>}
                ko={<p>
                    각 후보는 그 후보 하나만으로 채점되고, 같은 tick의 다른 후보와는 무관하다. 원 논문은
                    매 tick 후보 집합 전체에 걸쳐 정규화·스무딩을 하지만, 이 구현은 정규화를 고정값으로
                    바꿔 한 후보의 점수가 그 tick에 어떤 다른 후보가 뽑혔는지에 좌우되지 않게 했다. 그
                    대가로 결과가 결정적이고, 실행마다 정확히 재현된다.
                </p>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Cost: <InlineMath math="O(n_v \cdot n_\omega \cdot n_k)"/> per
                            tick.</strong> Every one of the <InlineMath math="n_v \cdot n_\omega"/> sampled
                            commands gets rolled out at <InlineMath math="n_k"/> points along its arc and
                            checked against the obstacle map at each — a few hundred pose evaluations per
                            control cycle with typical sample counts, redone from scratch every tick (nothing
                            carries over, unlike pure pursuit's forward-only path scan).</li>
                        <li><strong>Purely local, on purpose.</strong> The rollout horizon is a couple of
                            seconds at most, so DWA has no notion of the obstacle it cannot yet see around
                            the corner or the dead end three meters ahead. It is not trying to be a
                            planner — it is a fast, reactive layer that a real stack pairs with one.</li>
                        <li><strong>Local minima are a real failure mode, not an edge case.</strong> Drive
                            it into a U-shaped dead end and every sampled command can eventually fail the
                            admissibility bound at once — no candidate can promise to stop before the walls
                            closing in on three sides. The honest outcome is the robot braking to a stop, not
                            a crash and not a magical escape.</li>
                        <li><strong>No formal optimality.</strong> <InlineMath math="G"/> is maximized over
                            a finite sample, not solved in closed form, and the weights
                            <InlineMath math="\alpha, \beta, \gamma"/> are tuned by hand, not derived —
                            "good enough, safely, every tick" is the design goal, not global efficiency.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>비용: tick당 <InlineMath math="O(n_v \cdot n_\omega \cdot n_k)"/>.</strong>{" "}
                            표본으로 뽑힌 <InlineMath math="n_v \cdot n_\omega"/>개 명령 각각을{" "}
                            <InlineMath math="n_k"/>개 지점으로 굴려보고 그때마다 장애물 지도와
                            대조한다. 일반적인 표본 수라면 제어 주기당 pose 평가가 수백 회 정도이고,
                            이걸 매 tick 처음부터 다시 한다(pure pursuit의 전진 전용 스캔처럼 이어지는
                            상태가 없다).</li>
                        <li><strong>철저히 국소적이다. 의도적으로.</strong> 롤아웃 horizon은 길어야 몇
                            초라, DWA는 모퉁이 너머에 아직 보이지 않는 장애물이나 3m 앞의 막다른 길
                            같은 것을 전혀 모른다. planner가 되려는 것이 아니라, 실제 스택에서 planner와
                            짝을 이루는 빠르고 반응적인 층이다.</li>
                        <li><strong>local minima는 예외 상황이 아니라 실제 실패 모드다.</strong> U자
                            막다른 길로 몰아넣으면 표본으로 뽑힌 모든 명령이 결국 한꺼번에 admissible
                            부등식을 만족하지 못하게 된다. 삼면에서 좁혀오는 벽 앞에서 멈출 수 있다고
                            보장하는 후보가 하나도 남지 않는 것이다. 이때 정직한 결과는 로봇이 그대로
                            정지하는 것이지, 충돌도 마법 같은 탈출도 아니다.</li>
                        <li><strong>형식적 최적성은 없다.</strong> <InlineMath math="G"/>는 닫힌 형태로
                            풀리는 게 아니라 유한 표본 위에서 최대화되고, 가중치{" "}
                            <InlineMath math="\alpha, \beta, \gamma"/>는 유도가 아니라 손으로 튜닝된다.
                            설계 목표는 전역 효율이 아니라 "매 tick 안전하게, 충분히 괜찮은" 명령이다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Nothing persists between ticks except the velocity the robot is already executing — the
                    dynamic window is centered on that, recomputed from scratch, and the candidate that wins
                    this tick has no memory of the one that won last tick.
                </p>}
                ko={<p>
                    tick 사이에 남는 것은 로봇이 지금 실행 중인 속도 하나뿐이다. dynamic window는 그
                    속도를 중심으로 매번 처음부터 다시 계산되고, 이번 tick에서 이긴 후보는 지난 tick에서
                    이긴 후보를 전혀 기억하지 못한다.
                </p>}
            />
            <Pseudocode code={`v_max_eff ← v_max * min(1, remaining_to_goal / slow_radius)      # 1
window_v ← [v_a - v_dot_b*dt, v_a + v_dot_b*dt] ∩ [v_min, v_max_eff]  # 2
window_omega ← [omega_a - omega_dot_b*dt, omega_a + omega_dot_b*dt] ∩ [-omega_max, omega_max]  # 3
best ← none                                                       # 4
for v in uniform_grid(window_v, n_v):                              # 5
    for omega in uniform_grid(window_omega, n_omega):              # 6
        rollout ← arc(pose, v, omega, sim_time, n_k)                # 7
        if collides(rollout): continue                              # 8
        clearance ← min_k dist_to_nearest(rollout_k) - footprint_r  # 9
        if not admissible(v, omega, clearance): continue             # 10
        G ← alpha*heading(rollout) + beta*C(clearance) + gamma*(v/v_max)  # 11
        if best is none or G > best.G: best ← (v, omega, G)          # 12
if best is none:                                                    # 13
    return decelerate(v_a, omega_a)                                  # 14
return (best.v, best.omega)                                          # 15`}/>
            <T
                en={<ol>
                    <li>Cap the top of the search box before anything else: the effective max speed ramps
                        down linearly inside <InlineMath math="\text{slow\_radius}"/> of the goal, so the
                        window itself shrinks near the end of the run instead of the robot arriving fast and
                        having to brake hard right at the goal.</li>
                    <li>Build the velocity half of the window: the current translational speed plus or minus
                        one tick of acceleration, clipped to the robot's absolute speed limits (and the
                        capped top from step 1).</li>
                    <li>Build the turn-rate half the same way, clipped to the absolute turn-rate limit.</li>
                    <li>Walk the window on a fixed, deterministic grid — outer loop over
                        <InlineMath math="v"/>, inner loop over <InlineMath math="\omega"/> — never randomly.
                        Fixing the traversal order is what makes tie-breaking (and the whole search)
                        reproducible.</li>
                    <li>For each sampled command, predict where holding it for the rollout horizon would
                        take the robot: a constant-curvature arc, evaluated in closed form at a handful of
                        points along it rather than integrated step by step.</li>
                    <li>Throw the candidate out immediately if the robot's footprint would touch an obstacle
                        at any sampled point along that arc — it never gets a score at all.</li>
                    <li>For everything that survives, measure clearance: the smallest distance from any
                        rollout point to the nearest obstacle, minus the footprint radius.</li>
                    <li>Check admissibility: could the robot still brake to a stop, at its acceleration
                        limits, before covering that clearance? This is the actual safety gate — a fast
                        candidate with a wide-open arc ahead passes easily, while a fast candidate pointed at
                        a nearby wall gets rejected even though it never technically collides.</li>
                    <li>Score everything that is both collision-free and admissible against
                        <InlineMath math="G"/>, and keep a running best.</li>
                    <li><strong>The trap:</strong> if nothing in the window is ever admissible — every
                        direction is blocked too close to brake in time — there is no candidate to fall back
                        on, scored or not. The only honest move is to brake at the acceleration limits and
                        try again next tick with a slower, wider window. Skipping this case and executing an
                        unscored guess instead is exactly the bug that would turn a should-be-STALLED episode
                        into a collision.</li>
                </ol>}
                ko={<ol>
                    <li>다른 무엇보다 먼저 탐색 상자의 위쪽 한계부터 낮춰 둔다. 목표로부터{" "}
                        <InlineMath math="\text{slow\_radius}"/> 이내에서는 유효 최고 속도가 선형으로
                        줄어들어, window 자체가 종반에 미리 좁아진다. 로봇이 빠른 속도로 도착해 목표
                        바로 앞에서 급브레이크를 밟는 대신이다.</li>
                    <li>window의 속도 쪽 절반을 만든다. 현재 병진 속도에서 한 tick 가속만큼 위아래로,
                        그리고 로봇의 절대 속도 한계(와 1단계에서 낮춘 상한)로 잘라낸다.</li>
                    <li>회전율 쪽 절반도 같은 방식으로 만들되 절대 회전율 한계로 자른다.</li>
                    <li>window를 고정된 결정적 격자로 훑는다. 바깥 루프는 <InlineMath math="v"/>, 안쪽
                        루프는 <InlineMath math="\omega"/>. 절대 난수로 뽑지 않는다. 순회 순서를
                        고정하는 것이 동률 처리를(그리고 탐색 전체를) 재현 가능하게 만든다.</li>
                    <li>표본으로 뽑힌 명령마다, 그 명령을 롤아웃 horizon 동안 유지했을 때 로봇이 어디로
                        가는지 예측한다. 일정 곡률의 원호이며, 한 스텝씩 적분하는 대신 그 위 몇 개
                        지점에서 closed-form으로 바로 계산한다.</li>
                    <li>그 원호를 따라 표본으로 찍은 어느 지점에서든 로봇의 footprint가 장애물에 닿으면
                        그 후보는 즉시 버린다. 점수를 아예 받지 못한다.</li>
                    <li>살아남은 후보마다 clearance를 잰다. 롤아웃 지점 중 최근접 장애물까지의 최소
                        거리에서 footprint 반경을 뺀 값이다.</li>
                    <li>admissibility를 확인한다. 로봇이 가속 한계로 그 clearance를 다 쓰기 전에 멈출 수
                        있는가. 이것이 실질적인 안전 게이트다. 앞이 뻥 뚫린 빠른 후보는 쉽게 통과하지만,
                        기술적으로는 충돌하지 않더라도 가까운 벽을 향한 빠른 후보는 여기서 걸러진다.</li>
                    <li>충돌도 없고 admissible한 후보 전부를 <InlineMath math="G"/>로 채점하며 지금까지의
                        최선을 갱신해 둔다.</li>
                    <li><strong>함정.</strong> window 안 어떤 후보도 끝내 admissible하지 않다면(모든
                        방향이 제때 멈추기엔 너무 가깝게 막혀 있다면) 점수와 무관하게 대신 쓸 후보 자체가
                        없다. 유일하게 정직한 선택은 가속 한계로 감속하고 다음 tick에 더 느리고 넓어진
                        window로 다시 시도하는 것이다. 이 경우를 빠뜨리고 채점되지 않은 후보를 그냥
                        실행해버리면, 원래 STALLED로 끝나야 할 상황이 충돌로 바뀌는 정확히 그 버그가
                        된다.</li>
                </ol>}
            />

            <h2>{t("The Admissible Velocity Bound", "Admissible 속도의 유도")}</h2>
            <T
                en={<p>
                    Where does the admissibility check in step 8 actually come from? It is a plain
                    stopping-distance argument, applied once to the forward speed and once to the turn rate.
                </p>}
                ko={<p>
                    8단계의 admissibility 판정은 대체 어디서 나온 식일까? 정지거리 논증 하나를 전진
                    속도와 회전율에 각각 한 번씩 적용한 것뿐이다.
                </p>}
            />
            <Proof title={t("Derivation (stopping-distance bound)", "유도 (정지거리 부등식)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Suppose the robot commits to braking at its maximum
                            deceleration <InlineMath math="\dot v_b"/> starting right now, from speed{" "}
                            <InlineMath math="v"/>. Under constant deceleration, speed decays linearly with
                            time until it reaches zero — this is exactly a constant-acceleration motion
                            problem, so the distance covered before stopping follows from
                            <InlineMath math="v^2 = v_0^2 - 2\dot v_b d"/> with final speed
                            zero:
                        </p>
                        <BlockMath math="d_{\text{stop}} = \frac{v^2}{2\dot v_b}"/>
                        <Terms items={[
                            ["d_{\\text{stop}}", "최대 감속으로 정지할 때까지 이동하는 거리"],
                            ["v", "제동을 시작하는 현재 전진 속도"],
                            ["\\dot v_b", "선가속 한계의 크기 — 낼 수 있는 최대 감속률"],
                        ]}/>
                        <p>
                            <strong>Requirement.</strong> For the candidate to be safe, this stopping
                            distance must not exceed the clearance actually available — the distance from
                            the rollout to the nearest obstacle, <InlineMath math="\text{dist}"/>:
                        </p>
                        <BlockMath math="\frac{v^2}{2\dot v_b} \le \text{dist}"/>
                        <Terms items={[
                            ["\\text{dist}", "롤아웃을 따라 표본화한 최소 clearance — 최근접 장애물까지 남은 여유"],
                            ["v,\\ \\dot v_b", "위와 동일: 현재 속도와 선가속 한계"],
                        ]}/>
                        <p>
                            Multiply through by <InlineMath math="2\dot v_b"/> and take the positive square
                            root (both sides are non-negative by construction):
                        </p>
                        <BlockMath math="v \le \sqrt{2 \cdot \text{dist} \cdot \dot v_b}"/>
                        <Terms items={[
                            ["v", "이 부등식을 만족해야 admissible한 전진 속도"],
                            ["\\text{dist},\\ \\dot v_b", "위와 동일"],
                        ]}/>
                        <p>
                            The identical argument applied to the turn rate, with angular deceleration limit
                            <InlineMath math="\dot\omega_b"/> in place of <InlineMath math="\dot v_b"/>,
                            gives the second half of the check:
                        </p>
                        <BlockMath math="|\omega| \le \sqrt{2 \cdot \text{dist} \cdot \dot\omega_b}"/>
                        <Terms items={[
                            ["\\omega", "이 부등식을 만족해야 admissible한 회전율"],
                            ["\\dot\\omega_b", "각가속 한계의 크기 — 낼 수 있는 최대 각감속률"],
                            ["\\text{dist}", "위와 동일 — 두 부등식이 같은 clearance를 공유한다"],
                        ]}/>
                        <p>
                            A candidate passes admissibility only if both hold at once. Note what this does
                            <em>not</em> claim: it is not a bound on the true distance to the nearest
                            obstacle along the candidate's actual curvature (the paper's original quantity),
                            only on the minimum clearance sampled along a finite rollout — a conservative
                            stand-in, not the exact value.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 로봇이 지금 속도 <InlineMath math="v"/>에서 바로 최대
                            감속 <InlineMath math="\dot v_b"/>로 제동을 건다고 하자. 등감속 운동에서
                            속도는 시간에 따라 선형으로 줄어 0에 이른다. 이는 정확히 등가속도 운동
                            문제이므로, 정지할 때까지 이동하는 거리는 나중 속도가 0인{" "}
                            <InlineMath math="v^2 = v_0^2 - 2\dot v_b d"/>에서 바로 나온다:
                        </p>
                        <BlockMath math="d_{\text{stop}} = \frac{v^2}{2\dot v_b}"/>
                        <Terms items={[
                            ["d_{\\text{stop}}", "최대 감속으로 정지할 때까지 이동하는 거리"],
                            ["v", "제동을 시작하는 현재 전진 속도"],
                            ["\\dot v_b", "선가속 한계의 크기. 낼 수 있는 최대 감속률"],
                        ]}/>
                        <p>
                            <strong>요구 조건.</strong> 후보가 안전하려면 이 정지거리가 실제로 남은
                            여유, 곧 롤아웃에서 최근접 장애물까지의 거리 <InlineMath math="\text{dist}"/>를
                            넘지 않아야 한다:
                        </p>
                        <BlockMath math="\frac{v^2}{2\dot v_b} \le \text{dist}"/>
                        <Terms items={[
                            ["\\text{dist}", "롤아웃을 따라 표본화한 최소 clearance. 최근접 장애물까지 남은 여유"],
                            ["v,\\ \\dot v_b", "위와 동일: 현재 속도와 선가속 한계"],
                        ]}/>
                        <p>
                            양변에 <InlineMath math="2\dot v_b"/>를 곱하고 (양변 모두 애초에 음수가 아니므로)
                            양의 제곱근을 취한다:
                        </p>
                        <BlockMath math="v \le \sqrt{2 \cdot \text{dist} \cdot \dot v_b}"/>
                        <Terms items={[
                            ["v", "이 부등식을 만족해야 admissible한 전진 속도"],
                            ["\\text{dist},\\ \\dot v_b", "위와 동일"],
                        ]}/>
                        <p>
                            회전율에도 같은 논증을 그대로 적용하되 <InlineMath math="\dot v_b"/> 자리에
                            각가속 한계 <InlineMath math="\dot\omega_b"/>를 넣으면 판정의 나머지 절반이
                            나온다:
                        </p>
                        <BlockMath math="|\omega| \le \sqrt{2 \cdot \text{dist} \cdot \dot\omega_b}"/>
                        <Terms items={[
                            ["\\omega", "이 부등식을 만족해야 admissible한 회전율"],
                            ["\\dot\\omega_b", "각가속 한계의 크기. 낼 수 있는 최대 각감속률"],
                            ["\\text{dist}", "위와 동일. 두 부등식이 같은 clearance를 공유한다"],
                        ]}/>
                        <p>
                            후보는 두 부등식을 동시에 만족해야만 admissible을 통과한다. 이 부등식이{" "}
                            <em>주장하지 않는</em> 것도 분명히 해 두자. 후보의 실제 곡률을 따라 최근접
                            장애물까지의 참거리(원 논문이 쓰는 값)를 보장하는 것이 아니라, 유한한
                            롤아웃을 따라 표본화한 최소 clearance에 대해서만 성립한다. 보수적인
                            대체값이지, 정확한 값이 아니다.
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs DWA live in your browser. Every sampled arc this tick is drawn as
                    a thin line fanning out from the robot, colored by whether it survived collision and
                    admissibility checks; the winning arc is drawn thick and bright. Switch to the sudden
                    wall preset and watch the fan shrink and shift as the robot closes in, or push it into
                    the dead end and watch every arc lose admissibility on the same tick.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 DWA를 라이브로 실행한다. 이번 tick이 표본으로 뽑은 모든
                    원호가 로봇에서 뻗어 나가는 가는 선으로 그려지고, 충돌·admissibility 판정 통과 여부에
                    따라 색이 달라진다. 이긴 원호만 굵고 밝게 그려진다. 급정지 벽 프리셋으로 바꿔 로봇이
                    다가갈수록 부채꼴이 줄고 옮겨가는 모습을 보거나, 막다른 길로 몰아 모든 원호가 같은
                    tick에 한꺼번에 admissible을 잃는 모습을 보라.
                </p>}
            />
            <DwaSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below follows the algorithm above near-literally: the window
                    intersection, the deterministic v-outer/omega-inner grid, the closed-form arc rollout,
                    and the fixed-normalization objective. The code is the actual repository source, not an
                    excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 거의 그대로 옮긴 것이다. window 교집합, 결정적인 v-바깥/
                    omega-안쪽 격자, closed-form 원호 롤아웃, 고정 정규화 목적함수까지 그대로다. 아래
                    코드는 발췌가 아니라 저장소의 실제 소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/reactive/dwa.py",
                                code: dwaPy,
                                href: `${REPO}/python/navigation/local_planning/reactive/dwa.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/reactive/dwa.hpp",
                                code: dwaHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/reactive/dwa.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/reactive/dwa.cpp",
                                code: dwaCpp,
                                href: `${REPO}/cpp/src/local_planning/reactive/dwa.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The window intersection, rollout, and fixed-normalization scoring, embedded from the repository sources",
                    "window 교집합, 롤아웃, 고정 정규화 채점. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    D. Fox, W. Burgard, S. Thrun,{" "}
                    <a href="https://doi.org/10.1109/100.580977" target="_blank" rel="noopener noreferrer">
                        <em>The Dynamic Window Approach to Collision Avoidance</em>
                    </a>, IEEE Robotics & Automation Magazine, vol. 4, no. 1, pp. 23–33, 1997.
                </li>
                <li>
                    R. Simmons,{" "}
                    <a href="https://doi.org/10.1109/ROBOT.1996.511077" target="_blank" rel="noopener noreferrer">
                        <em>The Curvature-Velocity Method for Local Obstacle Avoidance</em>
                    </a>, Proceedings of IEEE International Conference on Robotics and Automation (ICRA), 1996.
                </li>
            </ol>
        </>
    )
}

export default Dwa

import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import Pseudocode from "../../../components/Pseudocode";
import CodeTabs from "../../../components/CodeTabs";
import ForceVectorsFigure from "../../../components/panels/local/potential_fields/ForceVectors";
import UTrapEquilibrium from "../../../components/panels/local/potential_fields/UTrapFigure";
import PotentialFieldsSandbox from "../../../components/panels/local/potential_fields/PotentialFieldsSandbox";
import potentialFieldsPy from "../../../../../python/navigation/local_planning/reactive/potential_fields.py?raw";
import potentialFieldsCpp from "../../../../../cpp/src/local_planning/reactive/potential_fields.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 다른 알고리즘 페이지와 같은 패턴(본문은 직관, 형식 논증은 원할 때만).
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const PotentialFields = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Potential Fields (Khatib, 1986) treats the robot as a particle in an
                    artificial force field: the goal pulls, every nearby obstacle pushes, and
                    at each control tick the robot simply drives in the direction of the sum.
                    There is no search, no tree, no plan to follow — just a vector, recomputed
                    from scratch every 100 ms from whatever the robot currently sees. That
                    makes it the cheapest reactive controller in this section, and, as the
                    demo below shows honestly, the most easily fooled by a well-placed wall.
                </p>}
                ko={<p>
                    Potential Fields(Khatib, 1986)는 로봇을 인공적인 힘의 장 속 입자로
                    다룬다. 목표는 당기고, 주변 장애물은 저마다 민다. 매 제어 tick마다
                    로봇은 그 합의 방향으로 그저 달린다. 탐색도, 트리도, 따라갈 계획도
                    없다. 100 ms마다 로봇이 지금 보는 것만으로 처음부터 다시 계산되는
                    벡터 하나뿐이다. 그래서 이 section에서 가장 값싼 반응형 조향이 되고,
                    아래 demo가 정직하게 보여주듯 벽 하나만 잘 놓여도 가장 쉽게 속는다.
                </p>}
            />

            <h2>{t("The Goal Pulls, the Walls Push", "목표가 끌고, 벽이 민다")}</h2>
            <T
                en={<p>
                    The robot's state is just its position <InlineMath math="p=(x,y)"/> (heading
                    only decides which way it is already facing, not the force). Two forces sum
                    to one resultant every tick — an attractive pull toward the goal, and a
                    repulsive push from every obstacle cell within a fixed influence radius.
                </p>}
                ko={<p>
                    로봇의 상태는 위치 <InlineMath math="p=(x,y)"/>뿐이다(heading은 힘이
                    아니라 로봇이 지금 어디를 보고 있는지만 정한다). 매 tick 두 힘을 더해
                    합력 하나를 만든다. 목표로의 인력과, 고정된 영향 반경 안 모든 장애물
                    셀에서 오는 반발이다.
                </p>}
            />
            <BlockMath math="F_{\text{att}} = k_{\text{att}} \,(g - p)"/>
            <Terms items={[
                ["F_{\\text{att}}", t("the attractive force vector", "인력 벡터")],
                ["k_{\\text{att}}", t("attractive gain — a fixed constant, no normalization by distance",
                    "인력 게인. 거리 정규화 없는 고정 상수")],
                ["g", t("the goal position (x, y)", "목표 위치 (x, y)")],
                ["p", t("the robot's current position (x, y)", "로봇의 현재 위치 (x, y)")],
            ]}/>
            <T
                en={<p>
                    Linear in distance, so it never vanishes and never explodes: far from the
                    goal it pulls hard, close to the goal it eases off on its own. The repulsive
                    term is FIRAS (Force Induced by an Artificial Repulsive potential Surface,
                    Khatib 1986) — the only force that is aware obstacles exist at all:
                </p>}
                ko={<p>
                    거리에 선형이라 사라지지도, 터지지도 않는다. 목표에서 멀면 세게 당기고
                    가까우면 스스로 누그러진다. 반발항은 FIRAS(Force Induced by an Artificial
                    Repulsive potential Surface, Khatib 1986)다. 장애물의 존재를 아는 유일한
                    힘이다:
                </p>}
            />
            <BlockMath math="F_{\text{rep}} = \sum_{o\,\in\,\mathcal{O}(p,\rho_0)} k_{\text{rep}} \left(\frac{1}{d} - \frac{1}{\rho_0}\right) \frac{1}{d^2} \, \frac{p - o}{d}, \qquad d = \max\!\bigl(\lVert p - o \rVert,\, d_{\min}\bigr)"/>
            <Terms items={[
                ["F_{\\text{rep}}", t("the summed repulsive force vector", "합산된 반발 벡터")],
                ["\\mathcal{O}(p,\\rho_0)", t(
                    "every occupied (or out-of-bounds) cell center within radius \\rho_0 of p",
                    "p에서 반경 ρ₀ 안의 모든 점유(또는 경계 밖) 셀 중심")],
                ["o", t("one such obstacle cell center", "그 안의 장애물 셀 중심 하나")],
                ["k_{\\text{rep}}", t("repulsive gain", "반발 게인")],
                ["\\rho_0", t("influence radius — obstacles farther than this are ignored entirely",
                    "영향 반경. 이보다 먼 장애물은 아예 무시된다")],
                ["d", t("distance from the robot to o, floored at d_{\\min}", "로봇에서 o까지 거리, d_min 아래로 클램프")],
                ["d_{\\min}", t("the footprint radius — the closest the robot's center can physically get to an obstacle center",
                    "footprint 반경. 로봇 중심이 장애물 중심에 물리적으로 다가갈 수 있는 최소 거리")],
            ]}/>
            <T
                en={<>
                    <p>
                        Each term in the sum points away from its obstacle (direction{" "}
                        <InlineMath math="(p-o)/d"/>), scaled by a magnitude that blows up as{" "}
                        <InlineMath math="d \to d_{\min}"/> and vanishes smoothly as{" "}
                        <InlineMath math="d \to \rho_0"/> — the{" "}
                        <InlineMath math="1/\rho_0"/> term is exactly what makes obstacles fade
                        out continuously instead of switching on and off with a jump the moment
                        they enter or leave range.
                    </p>
                    <p>
                        The two sum to a single resultant <InlineMath math="F = F_{\text{att}} + F_{\text{rep}}"/>, and the robot steers toward it — no more, no less.
                    </p>
                </>}
                ko={<>
                    <p>
                        합의 각 항은 그 장애물에서 멀어지는 방향(<InlineMath math="(p-o)/d"/>)을
                        향하고, 크기는 <InlineMath math="d \to d_{\min}"/>일 때 발산하고{" "}
                        <InlineMath math="d \to \rho_0"/>일 때 매끄럽게 0으로 사라진다.{" "}
                        <InlineMath math="1/\rho_0"/> 항이 바로 이것을 만든다. 장애물이 영향
                        반경에 들고 나는 순간 힘이 뚝 끊기지 않고 연속적으로 사라진다.
                    </p>
                    <p>
                        둘을 더해 합력 <InlineMath math="F = F_{\text{att}} + F_{\text{rep}}"/>{" "}
                        하나를 만들고, 로봇은 그쪽으로 조향한다. 그게 전부다.
                    </p>
                </>}
            />
            <ForceVectorsFigure/>

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Reactive and memoryless</strong>: every tick recomputes{" "}
                        <InlineMath math="F"/> from the current pose and the static goal alone —
                        nothing carries over between ticks, so <code>reset()</code> stays the
                        base no-op.</li>
                    <li><strong>No completeness guarantee</strong>: unlike the search and
                        sampling planners on the earlier pages, Potential Fields offers no
                        probabilistic or resolution completeness. It can converge to a resultant
                        of zero strictly short of the goal — a local minimum of the artificial
                        potential — and the simulator's <code>STALLED</code> outcome reports that
                        honestly instead of hiding it as a timeout.</li>
                    <li><strong>Cost per tick</strong>: one bounded-box scan for{" "}
                        <InlineMath math="\mathcal{O}(p,\rho_0)"/> (obstacles within{" "}
                        <InlineMath math="\rho_0"/>) plus <InlineMath math="O(1)"/> vector
                        arithmetic per obstacle found — no queue, no tree, no memory that grows
                        with the episode.</li>
                    <li><strong>Speed tracks confidence</strong>: effective speed is{" "}
                        <InlineMath math="\min(v_{\max}, k_v\lVert F \rVert)"/>, so as the
                        resultant shrinks near a minimum the robot slows on its own — the stall
                        is a stop, not an oscillation at full throttle.</li>
                    <li><strong>Gain ratio decides doorways</strong>: <InlineMath math="k_{\text{rep}}/k_{\text{att}}"/>{" "}
                        and <InlineMath math="\rho_0"/> jointly set how much wall it takes to
                        outweigh a distant goal — tuned too repulsive, the robot refuses any
                        corridor narrower than roughly <InlineMath math="2\rho_0"/>.</li>
                </ul>}
                ko={<ul>
                    <li><strong>반응형, 무기억</strong>: 매 tick은 현재 pose와 고정된 목표만으로{" "}
                        <InlineMath math="F"/>를 처음부터 다시 계산한다. tick 사이에 넘어가는
                        상태가 없어 <code>reset()</code>은 기본 no-op으로 남는다.</li>
                    <li><strong>완전성 보장 없음</strong>: 앞 페이지들의 탐색·sampling
                        planner와 달리 확률적/해상도 완전성이 없다. 목표에 못 미친 지점에서
                        합력이 정확히 0으로 수렴할 수 있다(인공 potential의 local minimum).
                        시뮬레이터의 <code>STALLED</code>는 이를 timeout으로 숨기지 않고
                        정직하게 보고한다.</li>
                    <li><strong>tick당 비용</strong>: <InlineMath math="\rho_0"/> 안 장애물{" "}
                        <InlineMath math="\mathcal{O}(p,\rho_0)"/>을 위한 bounded-box 스캔 한
                        번과, 찾은 장애물마다 <InlineMath math="O(1)"/> 벡터 연산. queue도
                        tree도 없고, episode가 길어져도 늘지 않는 메모리.</li>
                    <li><strong>속도가 확신을 따라간다</strong>: 유효 속도는{" "}
                        <InlineMath math="\min(v_{\max}, k_v\lVert F \rVert)"/>라, minimum
                        근처에서 합력이 줄면 로봇도 스스로 느려진다. 정체는 전속력 진동이
                        아니라 정지다.</li>
                    <li><strong>게인 비율이 통로를 가른다</strong>: <InlineMath math="k_{\text{rep}}/k_{\text{att}}"/>와{" "}
                        <InlineMath math="\rho_0"/>이 함께, 먼 목표를 얼마나 벽이 이겨야
                        하는지를 정한다. 너무 반발 쪽으로 잡으면 대략{" "}
                        <InlineMath math="2\rho_0"/>보다 좁은 통로는 아예 거부한다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One force sum, one heading conversion, every tick. No memory to update, no
                    candidates to score:
                </p>}
                ko={<p>
                    매 tick 힘 합산 한 번, heading 변환 한 번. 갱신할 상태도, 채점할 후보도
                    없다:
                </p>}
            />
            <Pseudocode code={`p, theta ← state.pose;  g ← task.goal                                # 1
F_att ← k_att * (g - p)                                              # 2
F_rep ← (0, 0)
for o in occupied_within(p, rho_0):                                  # 3
    d ← max(||p - o||, d_min)
    if d >= rho_0: continue                                          # 4
    F_rep += k_rep * (1/d - 1/rho_0) * (1/d^2) * (p - o) / d
F ← F_att + F_rep
emit force_computed(p, F_att, F_rep, F)                               # 5
theta_d ← atan2(F.y, F.x)
v ← min(max_speed, k_v * ||F||)                                       # 6
return heading_command(wrap_to_pi(theta_d - theta), k_omega, v, max_omega)  # 7`}/>
            <T
                en={<ol>
                    <li>Read the current pose and the fixed goal. There is no path and no
                        history — these two facts are the entire input.</li>
                    <li>The attractive pull is computed once, unconditionally, before any
                        obstacle is even looked at.</li>
                    <li><code>occupied_within</code> is the only obstacle read the whole tick
                        makes — a single bounded-box scan over cells within{" "}
                        <InlineMath math="\rho_0"/>. Its row-major enumeration order fixes the
                        order this sum accumulates in.</li>
                    <li>Obstacles beyond <InlineMath math="\rho_0"/> (found by the box scan but
                        outside the true circular radius) are skipped rather than contributing
                        a tiny nonzero force — this is what keeps the field continuous at the
                        boundary rather than merely small.</li>
                    <li><code>force_computed</code> is emitted before the velocity command is
                        even built, so a replay's force arrows show exactly what decided this
                        tick's heading, not a value reconstructed after the fact.</li>
                    <li>Effective speed is proportional to the resultant's magnitude, by
                        design — not a separate safety clamp. This is what turns a local
                        minimum into an honest stop instead of a full-speed oscillation.</li>
                    <li><code>heading_command</code> turn-rate-clamps and cosine-gates forward
                        speed, so a resultant pointing behind the robot produces rotation in
                        place, never a wide arc or a reverse drive.</li>
                </ol>}
                ko={<ol>
                    <li>현재 pose와 고정된 목표를 읽는다. 경로도 이력도 없다. 이 둘이 입력의
                        전부다.</li>
                    <li>인력은 장애물을 보기도 전에, 조건 없이 한 번 계산된다.</li>
                    <li><code>occupied_within</code>이 이 tick의 유일한 장애물 읽기다.{" "}
                        <InlineMath math="\rho_0"/> 안 셀에 대한 bounded-box 스캔 한 번. 그
                        row-major 열거 순서가 이 합이 누적되는 순서를 고정한다.</li>
                    <li><InlineMath math="\rho_0"/> 밖 장애물(box 스캔엔 걸렸지만 실제 원형
                        반경 밖인 것)은 작은 값이라도 더하지 않고 건너뛴다. 이것이 경계에서
                        힘을 작게 만드는 게 아니라 아예 연속으로 만든다.</li>
                    <li><code>force_computed</code>는 속도 명령을 만들기도 전에 방출된다.
                        replay의 force 화살표는 사후에 재구성한 값이 아니라 이 tick의
                        heading을 실제로 정한 값 그대로다.</li>
                    <li>유효 속도는 별도 안전장치가 아니라 설계상 합력 크기에 비례한다.
                        이것이 local minimum을 전속력 진동이 아니라 정직한 정지로 만든다.</li>
                    <li><code>heading_command</code>는 회전율을 clamp하고 전진 속도를
                        cosine으로 게이팅한다. 합력이 로봇 뒤를 향하면 크게 도는 대신 제자리
                        회전을 낸다.</li>
                </ol>}
            />

            <h2>{t("The Local Minimum Trap", "Local minimum 함정")}</h2>
            <T
                en={<p>
                    A local minimum is any point where the resultant force is zero but the
                    robot has not reached the goal — the attractive pull and the summed
                    repulsion exactly cancel, and since effective speed is proportional to{" "}
                    <InlineMath math="\lVert F \rVert"/>, the robot stops there for good. A
                    U-shaped obstacle with the goal behind its closed end is the textbook
                    trigger: attraction points straight through the wall, and the wall pushes
                    straight back.
                </p>}
                ko={<p>
                    local minimum은 합력이 0이지만 로봇이 목표에 닿지 못한 지점이다. 인력과
                    합산된 반발이 정확히 상쇄되고, 유효 속도가 <InlineMath math="\lVert F \rVert"/>에
                    비례하므로 로봇은 거기서 영영 멈춘다. 목표가 닫힌 끝 너머에 있는 U자
                    장애물이 교과서적인 유발 조건이다. 인력은 벽을 똑바로 관통해 향하고,
                    벽은 그것을 똑바로 되받아 민다.
                </p>}
            />
            <Proof title={t("Proof: the symmetric U-trap has a zero-force equilibrium",
                "증명: 대칭 U-trap에는 합력 0인 평형점이 있다")}>
                <>
                        <p>
                            <strong>Setup.</strong> Place the robot at the origin with the goal
                            on the <InlineMath math="+x"/> axis, and two obstacles mirrored
                            across that axis — the front wall of a U-shaped trap and its two
                            symmetric side walls, exactly as in <code>pf_trap01</code>. Let{" "}
                            <InlineMath math="\Delta = g_x - p_x > 0"/> be the remaining distance
                            to the goal, and let each side obstacle sit at offset{" "}
                            <InlineMath math="(d_x, \pm d_y)"/> from the robot, at equal distance{" "}
                            <InlineMath math="d = \sqrt{d_x^2+d_y^2}"/>.
                        </p>
                        <BlockMath math="F_{\text{att}} = (k_{\text{att}}\Delta,\ 0), \qquad F_{\text{rep}} = \left(-2\varphi(d)\,\frac{d_x}{d},\ 0\right), \qquad \varphi(d) = k_{\text{rep}}\left(\frac{1}{d}-\frac{1}{\rho_0}\right)\frac{1}{d^2}"/>
                        <Terms items={[
                            ["F_{\\text{att}}", t("the attractive force at the robot's position", "로봇 위치에서의 인력")],
                            ["F_{\\text{rep}}", t("the summed repulsive force from the two mirrored side obstacles",
                                "좌우 대칭 장애물 둘에서 오는 합산 반발")],
                            ["\\Delta", t("remaining distance to the goal along the axis", "축을 따른 목표까지 남은 거리")],
                            ["k_{\\text{att}},\\,k_{\\text{rep}}", t("attractive / repulsive gains", "인력 / 반발 게인")],
                            ["d_x,\\,d_y", t("longitudinal / lateral offset from the robot to each side obstacle",
                                "로봇에서 각 측면 장애물까지의 축 방향 / 옆 방향 오프셋")],
                            ["d", t("distance to each side obstacle, \\sqrt{d_x^2+d_y^2}", "각 측면 장애물까지 거리")],
                            ["\\rho_0", t("influence radius", "영향 반경")],
                            ["\\varphi(d)", t(
                                "the new term: the FIRAS magnitude a single obstacle at distance d contributes",
                                "새로 추가된 항: 거리 d인 장애물 하나가 만드는 FIRAS 크기")],
                        ]}/>
                        <p>
                            {t(
                                "The lateral (y) components of the two side forces are equal and opposite by the mirror symmetry, so they cancel exactly — only the longitudinal component survives, opposing the goal's pull.",
                                "두 측면 힘의 옆(y) 성분은 거울 대칭으로 크기가 같고 부호가 반대라 정확히 상쇄된다. 축 방향 성분만 남아 목표의 인력과 맞선다.",
                            )}
                        </p>
                        <p>
                            {t(
                                "φ is strictly decreasing on (d_min, ρ₀) — both 1/d − 1/ρ₀ and 1/d² shrink as d grows — and it brackets k_att·Δ whenever the trap is tight enough relative to the gains:",
                                "φ는 (d_min, ρ₀) 위에서 단조 감소한다. d가 커질수록 1/d − 1/ρ₀와 1/d² 모두 줄어들기 때문이다. 함정이 게인 대비 충분히 좁으면 φ가 k_att·Δ를 사이에 둔다:",
                            )}
                        </p>
                        <BlockMath math="0 = \varphi(\rho_0) \;<\; k_{\text{att}}\,\Delta \;<\; \varphi(d_{\min}) = k_{\text{rep}}\left(\frac{1}{d_{\min}}-\frac{1}{\rho_0}\right)\frac{1}{d_{\min}^2}"/>
                        <Terms items={[
                            ["\\varphi(\\rho_0)", t("φ evaluated at d = ρ₀ — zero, since the FIRAS term vanishes at the influence boundary",
                                "d = ρ₀에서의 φ. 영향 경계에서 FIRAS 항이 사라지므로 0")],
                            ["k_{\\text{att}}\\,\\Delta", t("the (constant) magnitude of the attractive pull",
                                "인력 크기(상수)")],
                            ["\\varphi(d_{\\min})", t("φ evaluated at the contact clamp — its largest value, achieved as the robot nears the wall",
                                "접촉 클램프에서의 φ. 로봇이 벽에 가까워질 때 도달하는 최댓값")],
                            ["d_{\\min}", t("the footprint radius, the closest d can get before clamping",
                                "footprint 반경, 클램프 전 d가 도달할 수 있는 최솟값")],
                        ]}/>
                        <p>
                            {t(
                                "φ is continuous, so by the intermediate value theorem some d* strictly between d_min and ρ₀ satisfies φ(d*) = k_att·Δ. At that separation, F_rep exactly cancels F_att and the lateral components are already zero — the resultant vanishes at a point strictly short of the goal.",
                                "φ는 연속이므로 중간값 정리에 의해 d_min과 ρ₀ 사이 어떤 d*에서 φ(d*) = k_att·Δ가 성립한다. 그 거리에서 F_rep는 F_att와 정확히 상쇄되고 옆 성분은 이미 0이므로, 합력은 목표에 못 미친 지점에서 사라진다.",
                            )}
                            {" "}<InlineMath math="\blacksquare"/>
                        </p>
                        <p>
                            {t(
                                "This is not a bug to patch — it is the price of a controller with no memory of where it has already been. Koren & Borenstein (1991) catalog this and other pathologies (oscillation in narrow corridors, cycling between closely spaced obstacles) as the standard critique of purely reactive potential fields; VFH's histogram, on a later page, is one direct response.",
                                "고쳐야 할 버그가 아니다. 이미 지나온 곳을 기억하지 않는 조향기가 치르는 대가다. Koren와 Borenstein(1991)은 이것과 다른 병리(좁은 통로에서의 진동, 근접한 장애물 사이의 순환)를 순수 반응형 potential field에 대한 표준 비판으로 정리했다. 이후 페이지의 VFH 히스토그램이 이에 대한 직접적 응답 중 하나다.",
                            )}
                        </p>
                    </>
            </Proof>
            <UTrapEquilibrium/>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs the same closed-loop tick as the repository's Python demo,
                    in the browser. On <code>pf_trap01</code> the default gains reach exactly the
                    trap this page just proved exists — the resultant hits zero and the
                    simulator reports <code>STALLED</code>, not a silent hang. Switch to the
                    clutter field and the same controller, unmodified, threads between six
                    scattered blocks and reaches the goal: nothing about Potential Fields is
                    broken, the environment just decides which one it gets.
                </p>}
                ko={<p>
                    sandbox는 저장소 python demo와 같은 폐루프 tick을 브라우저에서 그대로
                    돌린다. <code>pf_trap01</code>에서는 기본 게인이 정확히 이 페이지가 방금
                    증명한 함정에 빠진다. 합력이 0에 닿고, 시뮬레이터는 조용한 멈춤이 아니라{" "}
                    <code>STALLED</code>를 보고한다. 산개 지형으로 바꾸면 같은 조향기가 수정
                    없이 흩어진 블록 여섯 개 사이를 지나 목표에 닿는다. Potential Fields
                    자체는 고장 나지 않았다. 어느 쪽을 겪을지는 환경이 정한다.
                </p>}
            />
            <PotentialFieldsSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The planner owns nothing but its seven gains and a per-tick loop over{" "}
                    <code>occupied_within</code> — there is no per-episode state to allocate, so
                    both language versions are short. Both are embedded in full below.
                </p>}
                ko={<p>
                    planner가 갖는 것은 게인 일곱 개와 <code>occupied_within</code> 위의
                    tick당 루프뿐이다. episode 단위로 할당할 상태가 없어 두 언어 버전 모두
                    짧다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/local_planning/reactive/potential_fields.py",
                            code: potentialFieldsPy,
                            href: `${REPO}/python/navigation/local_planning/reactive/potential_fields.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/local_planning/reactive/potential_fields.cpp",
                            code: potentialFieldsCpp,
                            href: `${REPO}/cpp/src/local_planning/reactive/potential_fields.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete Potential Fields implementation, embedded from the repository sources",
                    "Potential Fields 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    O. Khatib,{" "}
                    <a href="https://doi.org/10.1177/027836498600500106" target="_blank"
                       rel="noopener noreferrer">
                        <em>Real-Time Obstacle Avoidance for Manipulators and Mobile Robots</em>
                    </a>,
                    The International Journal of Robotics Research, 1986.
                </li>
                <li>
                    Y. Koren, J. Borenstein,{" "}
                    <a href="https://doi.org/10.1109/ROBOT.1991.131810" target="_blank"
                       rel="noopener noreferrer">
                        <em>Potential Field Methods and Their Inherent Limitations for Mobile
                            Robot Navigation</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation, 1991.
                </li>
            </ol>
        </>
    )
}

export default PotentialFields

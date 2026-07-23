import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import RegulatedPurePursuitSandbox from "../../../components/panels/local/regulated_pure_pursuit/RegulatedPurePursuitSandbox";
import RegulationCurveFigure from "../../../components/panels/local/regulated_pure_pursuit/RegulationCurveFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import regulatedPurePursuitPy from "../../../../../python/navigation/local_planning/tracking/regulated_pure_pursuit.py?raw";
import regulatedPurePursuitHpp from "../../../../../cpp/include/navigation/local_planning/tracking/regulated_pure_pursuit.hpp?raw";
import regulatedPurePursuitCpp from "../../../../../cpp/src/local_planning/tracking/regulated_pure_pursuit.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식적 전개는 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const RegulatedPurePursuit = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Regulated Pure Pursuit keeps Pure Pursuit's one-arc-through-a-point geometry but stops
                    pretending the world is empty. It scales how far ahead it looks with how fast it is
                    moving, caps its speed the tighter a corner gets, caps it again the closer an obstacle
                    gets, and checks the arc it is about to drive before committing to it. Macenski, Singh,
                    Martin and Gines described the method in a 2023 <em>Autonomous Robots</em> paper, and it
                    now ships as one of Nav2's built-in path-tracking controllers, among the most widely
                    used — the thing that actually drives the robot once a global planner has handed it a
                    path.
                </p>}
                ko={<p>
                    Regulated Pure Pursuit는 Pure Pursuit의 "점 하나를 지나는 원호 하나"라는 기하는 그대로
                    두되, 세상이 텅 비어 있는 척은 그만둔다. 속도가 빠를수록 더 멀리 내다보고, 코너가
                    조여질수록 속도를 낮추고, 장애물이 가까워질수록 다시 낮추며, 명령할 원호를 실행하기
                    전에 미리 걸어보고 검사한다. Macenski, Singh, Martin, Gines가 2023년 <em>Autonomous
                    Robots</em> 논문에서 이 방법을 정리했고, 지금은 Nav2에 내장된 경로 추종 controller 중
                    가장 널리 쓰이는 것들 중 하나다. global planner가 경로를 건네준 뒤 실제로 로봇을
                    움직이는 바로 그 부분이다.
                </p>}
            />

            <h2>{t("Pure Pursuit Meets the Real World", "Pure Pursuit의 실전 한계 손보기")}</h2>
            <T
                en={<>
                    <p>
                        Plain Pure Pursuit has three blind spots. Its lookahead distance is fixed, so a
                        setting tuned for cruising speed makes the robot chase a point too close when it is
                        going fast and too far when it is crawling. It has no notion of how sharp a corner
                        is, so it commands the same arc through a hairpin as through a gentle bend and lets
                        the resulting turn rate do whatever it does. And it does not look at the map at all
                        — a path that grazes a wall is driven exactly as confidently as a path down an open
                        corridor.
                    </p>
                    <p>
                        The fix for the first blind spot came before Regulated Pure Pursuit did. Campbell
                        (2007) scaled the lookahead distance with the vehicle's own speed for the DARPA
                        Urban Challenge — faster driving, farther lookahead — an idea usually called
                        Adaptive Pure Pursuit. Macenski, Singh, Martin and Gines (2023) folded that in and
                        added two more regulations of their own: a speed cap that tightens with commanded
                        curvature, and a speed cap that tightens with obstacle proximity, plus a short
                        lookahead collision check before the command ever reaches the motors. The result is
                        Regulated Pure Pursuit — still geometric, still cheap, but no longer indifferent to
                        speed, curvature, or the map. It is why Nav2 ships it as one of its built-in
                        controllers in place of plain Pure Pursuit, and why it is one of the most widely
                        used path-tracking plugins in practice.
                    </p>
                </>}
                ko={<>
                    <p>
                        Pure Pursuit 그대로에는 세 군데 사각지대가 있다. lookahead 거리가 고정이라, 순항
                        속도에 맞춰 튜닝한 값은 로봇이 빠를 땐 너무 가까운 점을 쫓게 하고 느릴 땐 너무 먼
                        점을 쫓게 한다. 코너가 얼마나 급한지에 대한 개념이 아예 없어서, 급커브든 완만한
                        커브든 같은 원호를 명령하고 그 결과 회전율이 어떻게 나오든 내버려 둔다. 그리고
                        지도를 아예 보지 않는다. 벽을 스치는 경로도 뻥 뚫린 통로를 지나는 경로와 똑같이
                        자신 있게 그대로 밟는다.
                    </p>
                    <p>
                        첫 번째 사각지대의 해법은 Regulated Pure Pursuit보다 먼저 나왔다. Campbell(2007)이
                        DARPA Urban Challenge를 위해 lookahead 거리를 차량 자신의 속도에 비례시켰다. 빠르게
                        달릴수록 더 멀리 본다는 뜻으로, 보통 Adaptive Pure Pursuit라 부른다. Macenski,
                        Singh, Martin, Gines(2023)는 이를 받아들이고 자기 나름의 규제 두 가지를 더
                        얹었다. 명령 곡률이 커질수록 조여지는 속도 상한과, 장애물이 가까워질수록 조여지는
                        속도 상한, 그리고 명령이 모터에 닿기 전 짧은 lookahead 충돌 검사까지. 그 결과가
                        Regulated Pure Pursuit다. 여전히 기하적이고 여전히 값싸지만, 더는 속도나 곡률이나
                        지도에 무심하지 않다. Nav2가 이걸 plain Pure Pursuit 대신 내장 controller로 채택해
                        가장 널리 쓰는 이유이기도 하다.
                    </p>
                </>}
            />

            <h2>{t("The Three Regulations", "세 가지 규제")}</h2>
            <T
                en={<>
                    <p>
                        First, the lookahead distance itself becomes a function of current speed instead of
                        a constant:
                    </p>
                    <BlockMath math="L_d = \operatorname{clamp}\big(t_l \cdot v,\; L_{\min},\; L_{\max}\big)"/>
                    <Terms items={[
                        ["L_d", "adaptive lookahead distance used this tick (meters)"],
                        ["t_l", "lookahead time constant — how many seconds ahead, at the current speed, the robot aims (seconds)"],
                        ["v", "robot's current forward speed (m/s)"],
                        ["L_{\\min},\\ L_{\\max}", "hard floor and ceiling on L_d, so a stopped robot still aims somewhere and a fast one never chases a point absurdly far away (meters)"],
                    ]}/>
                    <p>
                        Second, the commanded turning radius caps the speed. A tight turn — small radius —
                        gets a lower speed ceiling than a gentle one:
                    </p>
                    <BlockMath math="v_{\text{curv}} = \begin{cases} v_{\max} \cdot \dfrac{r}{r_{\min}} & r < r_{\min} \\[4pt] v_{\max} & r \ge r_{\min} \end{cases}"/>
                    <Terms items={[
                        ["v_{\\text{curv}}", "speed ceiling imposed by the curvature regulation (m/s)"],
                        ["r", "radius of the arc pure pursuit's geometry just commanded — the reciprocal of its curvature (meters)"],
                        ["r_{\\min}", "radius below which the ceiling starts tightening (meters) — the one tuning knob this regulation exposes"],
                        ["v_{\\max}", "the robot's top speed, and the ceiling's value once r \\ge r_{\\min} (m/s)"],
                    ]}/>
                    <p>
                        Third, distance to the nearest obstacle caps the speed the same way — closer
                        obstacle, lower ceiling. The paper's own proximity heuristic (Eq. 6) is already
                        distance-based: it scales the ceiling by the ratio of obstacle distance to a
                        proximity threshold, and costmap cost is only how the Nav2 plugin happens to read
                        that distance off its map. This implementation applies the same ratio directly to a
                        Euclidean clearance — distance to the nearest occupied cell, minus the footprint
                        radius — and omits only the paper's extra tunable gain constant:
                    </p>
                    <BlockMath math="v_{\text{prox}} = \begin{cases} v_{\max} \cdot \dfrac{\max(d,\, 0)}{d_{\text{prox}}} & d < d_{\text{prox}} \\[4pt] v_{\max} & d \ge d_{\text{prox}} \end{cases}"/>
                    <Terms items={[
                        ["v_{\\text{prox}}", "speed ceiling imposed by the proximity regulation (m/s)"],
                        ["d", "clearance — distance from the robot's center to the nearest obstacle, minus its own footprint radius (meters)"],
                        ["d_{\\text{prox}}", "clearance below which the ceiling starts tightening (meters)"],
                        ["v_{\\max}", "same top speed as above (m/s)"],
                    ]}/>
                    <p>
                        The three ceilings — this one, the curvature one, and the ordinary goal-approach
                        slowdown pure pursuit already had — are combined by taking whichever is lowest, so
                        the tightest constraint always wins. The paper instead applies the curvature and
                        proximity regulations sequentially, each one scaling the speed the previous one
                        already reduced, so its combination compounds multiplicatively rather than taking a
                        minimum; this implementation's minimum-of-ceilings approach is therefore slightly
                        less conservative when both regulations bind at the same time. Below, both
                        regulation ramps are drawn on the same speed axis: same shape, different thresholds,
                        each independently reaching <InlineMath math="v_{\max}"/> once its own hazard
                        clears.
                    </p>
                    <RegulationCurveFigure/>
                </>}
                ko={<>
                    <p>
                        먼저 lookahead 거리 자체가 상수가 아니라 현재 속도의 함수가 된다:
                    </p>
                    <BlockMath math="L_d = \operatorname{clamp}\big(t_l \cdot v,\; L_{\min},\; L_{\max}\big)"/>
                    <Terms items={[
                        ["L_d", "이번 tick에 쓰는 adaptive lookahead 거리 (meters)"],
                        ["t_l", "lookahead 시간 상수. 현재 속도로 몇 초 앞을 겨냥할지 (seconds)"],
                        ["v", "로봇의 현재 전진 속도 (m/s)"],
                        ["L_{\\min},\\ L_{\\max}", "L_d의 하한과 상한. 정지 상태에서도 어딘가를 겨냥하게 하고, 빠른 로봇이 터무니없이 먼 점을 쫓지 않게 한다 (meters)"],
                    ]}/>
                    <p>
                        두 번째로, 명령된 회전 반경이 속도 상한을 정한다. 급한 회전, 즉 반경이 작을수록
                        완만한 회전보다 낮은 속도 상한을 받는다:
                    </p>
                    <BlockMath math="v_{\text{curv}} = \begin{cases} v_{\max} \cdot \dfrac{r}{r_{\min}} & r < r_{\min} \\[4pt] v_{\max} & r \ge r_{\min} \end{cases}"/>
                    <Terms items={[
                        ["v_{\\text{curv}}", "곡률 규제가 매기는 속도 상한 (m/s)"],
                        ["r", "pure pursuit 기하가 방금 명령한 원호의 반경. 곡률의 역수 (meters)"],
                        ["r_{\\min}", "이보다 반경이 작아지면 상한이 조여지기 시작하는 값 (meters). 이 규제가 노출하는 유일한 튜닝 값"],
                        ["v_{\\max}", "로봇의 최고 속도이자, r \\ge r_{\\min}일 때 상한의 값 (m/s)"],
                    ]}/>
                    <p>
                        세 번째로, 가장 가까운 장애물까지의 거리도 같은 방식으로 속도 상한을 정한다.
                        장애물이 가까울수록 상한이 낮아진다. 논문 자체의 근접 휴리스틱(Eq. 6)도 이미
                        거리 기반이다. 장애물까지의 거리와 근접 문턱의 비율로 상한을 조절하며, costmap
                        비용은 Nav2 플러그인이 그 거리를 지도에서 읽어내는 방법일 뿐이다. 이 구현은 같은
                        비율을 유클리드 여유 거리(가장 가까운 점유 셀까지의 거리에서 footprint 반경을
                        뺀 값)에 직접 적용하고, 논문의 추가 튜닝 상수만 생략한다:
                    </p>
                    <BlockMath math="v_{\text{prox}} = \begin{cases} v_{\max} \cdot \dfrac{\max(d,\, 0)}{d_{\text{prox}}} & d < d_{\text{prox}} \\[4pt] v_{\max} & d \ge d_{\text{prox}} \end{cases}"/>
                    <Terms items={[
                        ["v_{\\text{prox}}", "근접 규제가 매기는 속도 상한 (m/s)"],
                        ["d", "여유 거리. 로봇 중심에서 가장 가까운 장애물까지의 거리에서 자신의 footprint 반경을 뺀 값 (meters)"],
                        ["d_{\\text{prox}}", "이보다 여유가 작아지면 상한이 조여지기 시작하는 값 (meters)"],
                        ["v_{\\max}", "위와 같은 최고 속도 (m/s)"],
                    ]}/>
                    <p>
                        이 규제와 곡률 규제, 그리고 pure pursuit이 원래 갖고 있던 goal 근접 감속까지 세
                        상한은 그중 가장 낮은 값을 취하는 방식으로 결합된다(가장 빡빡한 제약이 항상
                        이긴다). 논문은 곡률 규제와 근접 규제를 순차적으로 적용해서, 앞 규제가 이미
                        줄여놓은 속도에 다음 규제를 다시 곱하는 방식으로 결합한다. 즉 최솟값이 아니라
                        곱셈으로 누적되는 셈이다. 이 구현의 최솟값 방식은 두 규제가 동시에 걸릴 때
                        논문보다 약간 덜 보수적이다. 아래는 두 규제 램프를 같은 속도 축 위에 겹쳐 그린
                        것이다. 모양은 같고 문턱만 다르며, 각자 자신의 위험 요인이 해소되면 독립적으로{" "}
                        <InlineMath math="v_{\max}"/>에 도달한다.
                    </p>
                    <RegulationCurveFigure/>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Still <InlineMath math="O(1)"/>-ish per tick, plus a short arc walk.</strong>{" "}
                            The progress-index search is the same forward-only scan as plain Pure Pursuit.
                            On top of it, the lookahead collision check walks the commanded arc in fixed
                            steps out to the target, so each tick costs a handful of extra{" "}
                            <InlineMath math="\text{is\_collision}"/> queries — proportional to{" "}
                            <InlineMath math="L_d"/> divided by the step size, not to path length or map
                            size.</li>
                        <li><strong>Gains speed-awareness the plain algorithm never had.</strong> A robot
                            running Regulated Pure Pursuit brakes into corners and near obstacles on its own,
                            without a separate velocity planner bolted on top.</li>
                        <li><strong>Still reactive, not a planner.</strong> The lookahead collision check only
                            looks as far as the current arc reaches — a few tenths of a meter to a couple of
                            meters, depending on speed. It can stop the robot from driving straight into
                            something, but it cannot route around it; that job still belongs to whatever
                            produced the reference path.</li>
                        <li><strong>No formal convergence proof, same as Pure Pursuit.</strong> The
                            regulations are heuristics validated empirically (Macenski et al., 2023, §4), not
                            theorems about tracking error — the derivation below only justifies the shape of
                            the curvature regulation, not a stability guarantee for the whole controller.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>여전히 tick당 <InlineMath math="O(1)"/>에 가깝고, 짧은 원호 검사가
                            더해진다.</strong> progress index 탐색은 plain Pure Pursuit과 똑같은 전진
                            전용 스캔이다. 그 위에 lookahead 충돌 검사가 명령된 원호를 목표점까지 고정
                            간격으로 걸어보므로, tick마다 <InlineMath math="\text{is\_collision}"/> 질의가
                            몇 번 더 드는데 그 횟수는 경로 길이나 맵 크기가 아니라{" "}
                            <InlineMath math="L_d"/>를 스텝 크기로 나눈 값에 비례한다.</li>
                        <li><strong>기본 알고리즘엔 없던 속도 인지력을 얻는다.</strong> Regulated Pure
                            Pursuit을 도는 로봇은 별도의 속도 planner를 덧붙이지 않아도 스스로 코너와
                            장애물 앞에서 감속한다.</li>
                        <li><strong>여전히 반응형이지 planner가 아니다.</strong> lookahead 충돌 검사는
                            현재 원호가 닿는 범위, 속도에 따라 수십 cm에서 1~2 m 정도만 내다본다. 뭔가로
                            직진하는 것은 막을 수 있어도 그것을 돌아가지는 못한다. 그 일은 여전히 참조
                            경로를 만든 쪽의 몫이다.</li>
                        <li><strong>Pure Pursuit과 마찬가지로 형식적 수렴 증명은 없다.</strong> 세 규제는
                            정리가 아니라 경험적으로 검증된 휴리스틱이다(Macenski et al., 2023, §4). 아래
                            유도도 곡률 규제식의 모양만 정당화할 뿐, controller 전체의 안정성을 증명하지
                            않는다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Every tick recomputes the lookahead point exactly as plain Pure Pursuit does, then runs
                    it through the three regulations before checking the resulting arc for collisions.
                </p>}
                ko={<p>
                    매 tick은 plain Pure Pursuit과 똑같이 lookahead 점을 다시 계산한 다음, 세 규제를
                    거치고, 마지막으로 그 원호의 충돌 여부를 검사한다.
                </p>}
            />
            <Pseudocode code={`L_d ← clamp(t_l * v, L_min, L_max)                              # 1
i ← advance_forward(path, i, robot_xy)                          # 2
target ← lookahead_point(path, i, robot_xy, L_d)
alpha ← wrap(atan2(target_y - y, target_x - x) - theta)         # 3
kappa ← 2 sin(alpha) / L_d
v_goal ← v_max * min(1, remaining_to_goal / slow_radius)        # 4
v_curv ← v_max * min(1, (1/|kappa|) / r_min)  if kappa != 0 else v_max
v_prox ← v_max * clamp(clearance / d_prox, 0, 1)
v ← max(min(v_goal, v_curv, v_prox), v_regulated_min)           # min of ceilings; paper compounds curv & prox multiplicatively instead
arc_len ← L_d              if alpha near 0 or near +-pi         # 5
       ← L_d * alpha / sin(alpha)   otherwise
for s in step, 2*step, ... up to arc_len:
    if is_collision(pose_on_arc(kappa, s)):
        return (0, 0)                                            # stop outright
omega ← clamp(kappa * v, -omega_max, omega_max)                  # 6
if omega was clamped:
    v ← omega / kappa
return (v, omega)`}/>
            <T
                en={<ol>
                    <li>Compute the adaptive lookahead distance from the robot's current speed, clamped to
                        a floor and ceiling.</li>
                    <li>Find the lookahead point with the same forward-only progress search and
                        circle-path intersection as plain Pure Pursuit.</li>
                    <li>Derive the commanded curvature from the bearing to that point — identical geometry,
                        just using the adaptive <InlineMath math="L_d"/> instead of a fixed one.</li>
                    <li>Take the speed as the minimum of the three ceilings (goal-approach, curvature,
                        proximity), floored at a small regulated minimum so none of the three can drive it
                        all the way to zero on their own — that floor does not apply to the collision stop
                        in the next step, which must still be able to reach exactly zero.</li>
                    <li>Walk the commanded arc forward in fixed steps and check each point for collision.
                        This is the step easiest to get wrong: skip it, or apply the regulated-minimum floor
                        to it, and the robot can still be commanded straight into an obstacle it was
                        perfectly capable of seeing. If any step collides, return a stop command immediately
                        and skip the rest of the tick — no partial credit for a mostly-safe arc.</li>
                    <li>If nothing blocked the arc, clamp the turn rate to what the robot can physically do
                        and, if the clamp changed it, recompute <InlineMath math="v"/> from the clamped{" "}
                        <InlineMath math="\omega"/> and the original <InlineMath math="\kappa"/> — the same
                        curvature-preserving recompute plain Pure Pursuit uses, for the same reason.</li>
                </ol>}
                ko={<ol>
                    <li>로봇의 현재 속도로부터 adaptive lookahead 거리를 계산하고, 하한과 상한으로
                        clamp한다.</li>
                    <li>plain Pure Pursuit과 같은 전진 전용 progress 탐색과 원-경로 교차로 lookahead
                        점을 찾는다.</li>
                    <li>그 점까지의 방위로부터 명령 곡률을 유도한다. 기하는 동일하고, 고정 <InlineMath math="L_d"/>{" "}
                        대신 adaptive <InlineMath math="L_d"/>를 쓸 뿐이다.</li>
                    <li>속도는 세 상한(goal 접근·곡률·근접) 중 최솟값을 취하고, 작은 규제 최소값으로
                        바닥을 둬 셋 중 어느 것도 혼자서 속도를 0까지 끌어내리지 못하게 한다. 다만 이
                        바닥은 다음 단계의 충돌 정지에는 적용하지 않는다. 그 정지는 정확히 0까지 내려갈
                        수 있어야 하기 때문이다.</li>
                    <li>명령된 원호를 고정 간격으로 앞서 걸으며 각 지점의 충돌 여부를 검사한다. 가장
                        틀리기 쉬운 단계다. 이 단계를 건너뛰거나 여기에 규제 최소값 바닥을 적용하면,
                        로봇이 충분히 볼 수 있었던 장애물로 그대로 명령될 수 있다. 한 지점이라도
                        충돌하면 즉시 정지 명령을 반환하고 tick의 나머지를 건너뛴다. 대부분 안전한
                        원호라고 봐주지 않는다.</li>
                    <li>원호가 막히지 않았다면 회전율을 로봇이 물리적으로 낼 수 있는 값으로 클램프하고,
                        클램프가 값을 바꿨다면 클램프된 <InlineMath math="\omega"/>와 원래의{" "}
                        <InlineMath math="\kappa"/>로 <InlineMath math="v"/>를 다시 계산한다. plain
                        Pure Pursuit이 같은 이유로 쓰는 것과 같은 곡률-보존 재계산이다.</li>
                </ol>}
            />

            <h2>{t("Where the Curvature Bound Comes From", "곡률 규제식의 유도")}</h2>
            <T
                en={<p>
                    Why should speed fall off proportionally to radius at all, rather than in some other
                    shape? A short bound on turn rate makes the proportional form fall out on its own.
                </p>}
                ko={<p>
                    속도가 왜 다른 모양이 아니라 하필 반경에 비례해 떨어져야 할까? 회전율에 대한 짧은
                    부등식 하나로 비례 형태가 저절로 나온다.
                </p>}
            />
            <Proof title={t("Derivation (why radius-proportional speed makes sense)", "유도 (왜 반경 비례 속도가 타당한가)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> The robot's angular velocity and its commanded speed are
                            tied together through the curvature it is driving:
                        </p>
                        <BlockMath math="\omega = \kappa \cdot v"/>
                        <Terms items={[
                            ["\\omega", "the robot's angular velocity, executing the commanded arc (rad/s)"],
                            ["\\kappa", "commanded curvature — the reciprocal of the arc's radius, with sign (1/m)"],
                            ["v", "commanded forward speed (m/s)"],
                        ]}/>
                        <p>
                            The robot cannot exceed its own turn-rate limit, whatever speed it is
                            going:
                        </p>
                        <BlockMath math="|\omega| \le \omega_{\max}"/>
                        <Terms items={[
                            ["\\omega_{\\max}", "the robot's physical turn-rate limit (rad/s)"],
                        ]}/>
                        <p>
                            Substituting <InlineMath math="\omega = \kappa v"/> and writing the radius as{" "}
                            <InlineMath math="r = 1/|\kappa|"/> turns the turn-rate limit into a speed limit:
                        </p>
                        <BlockMath math="|\kappa|\, v \le \omega_{\max} \quad\Longrightarrow\quad v \le \omega_{\max} \cdot r"/>
                        <Terms items={[
                            ["r", "radius of the commanded arc — the reciprocal of |\\kappa| (meters)"],
                            ["v,\\ \\omega_{\\max}", "as above: commanded speed and the turn-rate limit"],
                        ]}/>
                        <p>
                            That is a hard ceiling: <InlineMath math="v"/> proportional to <InlineMath math="r"/>{" "}
                            is exactly what keeping <InlineMath math="\omega"/> within its limit demands, for
                            any radius at all. The curvature regulation does not wait for this ceiling to
                            bind and then clip <InlineMath math="\omega"/> — clipping alone would leave{" "}
                            <InlineMath math="v"/> too high for the clipped <InlineMath math="\omega"/>, and
                            the executed arc would undershoot the intended turn. Instead it scales{" "}
                            <InlineMath math="v"/> down proportionally to <InlineMath math="r"/> starting
                            from a radius <InlineMath math="r_{\min}"/> chosen with some margin above where
                            the hard ceiling would actually bind, so the robot slows into a tight corner
                            smoothly instead of hitting the turn-rate limit and being clamped at the last
                            moment.
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 로봇의 각속도와 명령 속도는 지금 그리고 있는 곡률을
                            통해 묶여 있다:
                        </p>
                        <BlockMath math="\omega = \kappa \cdot v"/>
                        <Terms items={[
                            ["\\omega", "명령된 원호를 실행할 때의 로봇 각속도 (rad/s)"],
                            ["\\kappa", "명령 곡률. 원호 반경의 역수, 부호 있음 (1/m)"],
                            ["v", "명령 전진 속도 (m/s)"],
                        ]}/>
                        <p>
                            로봇은 어떤 속도로 달리든 자신의 회전율 한계를 넘을 수 없다:
                        </p>
                        <BlockMath math="|\omega| \le \omega_{\max}"/>
                        <Terms items={[
                            ["\\omega_{\\max}", "로봇의 물리적 회전율 한계 (rad/s)"],
                        ]}/>
                        <p>
                            <InlineMath math="\omega = \kappa v"/>를 대입하고 반경을{" "}
                            <InlineMath math="r = 1/|\kappa|"/>로 쓰면, 회전율 한계가 속도 한계로 바뀐다:
                        </p>
                        <BlockMath math="|\kappa|\, v \le \omega_{\max} \quad\Longrightarrow\quad v \le \omega_{\max} \cdot r"/>
                        <Terms items={[
                            ["r", "명령된 원호의 반경. |\\kappa|의 역수 (meters)"],
                            ["v,\\ \\omega_{\\max}", "위와 동일: 명령 속도와 회전율 한계"],
                        ]}/>
                        <p>
                            이것이 확고한 상한이다. <InlineMath math="r"/>에 비례하는 <InlineMath math="v"/>는
                            어떤 반경에서든 <InlineMath math="\omega"/>를 한계 안에 묶어두기 위해 정확히
                            요구되는 형태다. 곡률 규제는 이 상한이 실제로 걸릴 때까지 기다렸다가{" "}
                            <InlineMath math="\omega"/>를 잘라내지 않는다. 자르기만 하면 잘린{" "}
                            <InlineMath math="\omega"/>에 비해 <InlineMath math="v"/>가 너무 높게 남아
                            실행되는 원호가 의도한 회전에 못 미치게(undershoot) 된다. 대신 상한이 실제로
                            걸리는 지점보다 어느 정도 여유를 둔 반경 <InlineMath math="r_{\min}"/>부터{" "}
                            <InlineMath math="v"/>를 <InlineMath math="r"/>에 비례해 미리 줄여, 로봇이
                            급코너로 들어갈 때 회전율 한계에 부딪혀 마지막 순간에 클램프되는 대신 매끄럽게
                            감속하게 한다.
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs Regulated Pure Pursuit live in your browser. Raise{" "}
                    <InlineMath math="r_{\min}"/> or <InlineMath math="d_{\text{prox}}"/> and watch the
                    robot brake earlier into the hairpin turns or the obstacle-hugging path. The plain
                    pursuit preset drops both thresholds to where they never bind, running the same paths
                    with the two speed regulations disabled for comparison — it still keeps the adaptive
                    lookahead and the lookahead collision check, both of which plain Pure Pursuit lacks.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 Regulated Pure Pursuit을 라이브로 실행한다.{" "}
                    <InlineMath math="r_{\min}"/>이나 <InlineMath math="d_{\text{prox}}"/>를 올리며 급코너나
                    장애물을 스치는 경로에서 더 일찍 감속하는 모습을 보라. plain pursuit 프리셋은 두 문턱을
                    결코 걸리지 않을 정도로 낮춰, 비교를 위해 두 속도 규제만 끈 채로 같은 경로를 몬다.
                    adaptive lookahead과 lookahead 충돌 검사는 그대로 켜져 있는데, 이는 plain Pure
                    Pursuit에는 없는 기능이다.
                </p>}
            />
            <RegulatedPurePursuitSandbox/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below computes the three regulations and the lookahead collision
                    check in the order derived above, then falls back to plain pursuit's curvature-preserving
                    clamp. The code is the actual repository source, not an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위에서 유도한 순서 그대로 세 규제와 lookahead 충돌 검사를 계산한 뒤, plain
                    pursuit의 곡률-보존 클램프로 마무리한다. 아래 코드는 발췌가 아니라 저장소의 실제
                    소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/tracking/regulated_pure_pursuit.py",
                                code: regulatedPurePursuitPy,
                                href: `${REPO}/python/navigation/local_planning/tracking/regulated_pure_pursuit.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/tracking/regulated_pure_pursuit.hpp",
                                code: regulatedPurePursuitHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/tracking/regulated_pure_pursuit.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/tracking/regulated_pure_pursuit.cpp",
                                code: regulatedPurePursuitCpp,
                                href: `${REPO}/cpp/src/local_planning/tracking/regulated_pure_pursuit.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The three regulations, the lookahead collision check, and the curvature-preserving clamp, embedded from the repository sources",
                    "세 규제, lookahead 충돌 검사, 곡률-보존 클램프. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. Macenski, S. Singh, F. Martin, J. Gines,{" "}
                    <a href="https://doi.org/10.1007/s10514-023-10097-6" target="_blank" rel="noopener noreferrer">
                        <em>Regulated Pure Pursuit for Robot Path Tracking</em>
                    </a>, Autonomous Robots, vol. 47, 2023.
                </li>
                <li>
                    R. C. Coulter,{" "}
                    <a href="https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Implementation of the Pure Pursuit Path Tracking Algorithm</em>
                    </a>,
                    Carnegie Mellon University Robotics Institute, Technical Report CMU-RI-TR-92-01, 1992.
                </li>
                <li>
                    S. F. Campbell,{" "}
                    <a href="https://hdl.handle.net/1721.1/42301" target="_blank" rel="noopener noreferrer">
                        <em>Steering Control of an Autonomous Ground Vehicle with Application to the DARPA
                            Urban Challenge</em>
                    </a>, Master's Thesis, Massachusetts Institute of Technology, 2007.
                </li>
            </ol>
        </>
    )
}

export default RegulatedPurePursuit

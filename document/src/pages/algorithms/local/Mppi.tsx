import {ReactNode} from "react";
import {T, useTr} from "../../../libs/i18n";
import Terms from "../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../components/math/Tex";
import MppiSandbox from "../../../components/panels/local/mppi/MppiSandbox";
import MppiTemperatureDemo from "../../../components/panels/local/mppi/MppiTemperatureDemo";
import MppiRolloutFigure from "../../../components/panels/local/mppi/MppiRolloutFigure";
import CodeTabs from "../../../components/CodeTabs";
import Pseudocode from "../../../components/Pseudocode";
import mppiPy from "../../../../../python/navigation/local_planning/predictive/mppi.py?raw";
import mppiHpp from "../../../../../cpp/include/navigation/local_planning/predictive/mppi.hpp?raw";
import mppiCpp from "../../../../../cpp/src/local_planning/predictive/mppi.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 유도 블록. 다른 알고리즘 페이지와 같은 패턴(본문은 직관, 형식적 전개는 원할 때만 편다).
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Mppi = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    MPC solved its receding-horizon problem by descending a gradient, which means it can only
                    ever walk downhill from where it stands. Model Predictive Path Integral control keeps the
                    exact same problem — the same control sequence, the same motion model, the same cost — and
                    changes only how it optimizes. Instead of a gradient, it scatters a cloud of{" "}
                    <strong>K noisy copies</strong> of the control sequence, rolls each one out, and moves the
                    nominal sequence toward the cheap ones by a softmax-weighted average. No derivative is ever
                    taken, so the cost can be as jagged or discontinuous as it likes. It is the controller
                    behind the widely used <code>mppi_controller</code> in ROS 2 Nav2, tracing back to Williams
                    and colleagues' information-theoretic formulation.
                </p>}
                ko={<p>
                    MPC는 receding-horizon 문제를 gradient를 내려가며 풀었고, 그래서 선 자리에서
                    내리막으로만 걸을 수 있었다. Model Predictive Path Integral control은 똑같은 문제,
                    즉 같은 제어열, 같은 운동 모델, 같은 비용을 그대로 두고 최적화 방식만 바꾼다.
                    gradient 대신 제어열의 <strong>노이즈 복사본 K개</strong>를 구름처럼 흩뿌려 각각
                    굴린 뒤, 싼 것들 쪽으로 공칭 제어열을 softmax 가중 평균만큼 옮긴다. 미분을 전혀
                    취하지 않으므로 비용이 아무리 들쭉날쭉하거나 불연속이어도 괜찮다. ROS 2 Nav2의
                    널리 쓰이는 <code>mppi_controller</code>가 바로 이것이고, Williams와 동료들의
                    정보이론 정식화로 거슬러 올라간다.
                </p>}
            />

            <h2>{t("From Gradients to Samples", "gradient에서 표본으로")}</h2>
            <T
                en={<p>
                    The problem does not change at all. The decision variable is still the control sequence, the
                    prediction is still the unicycle rollout, and the objective is still the same three-term
                    cost <InlineMath math="J(U)"/> from the MPC page. What changes is that instead of asking for
                    a gradient of <InlineMath math="J"/>, MPPI draws samples. Each sample perturbs the current
                    nominal sequence with Gaussian noise on every control step:
                </p>}
                ko={<p>
                    문제는 전혀 바뀌지 않는다. 결정 변수는 여전히 제어열이고, 예측은 여전히 unicycle
                    rollout이며, 목적함수는 여전히 MPC 페이지의 세 항짜리 비용{" "}
                    <InlineMath math="J(U)"/> 그대로다. 바뀌는 것은 <InlineMath math="J"/>의 gradient를
                    묻는 대신 MPPI가 표본을 뽑는다는 점이다. 각 표본은 현재 공칭 제어열의 매 제어
                    스텝에 Gauss 노이즈를 얹어 섭동한다:
                </p>}
            />
            <BlockMath math="V_k = \hat U + \varepsilon_k, \qquad \varepsilon_k[j] \sim \mathcal{N}(0, \Sigma),\quad \Sigma = \operatorname{diag}(\sigma_v^2, \sigma_\omega^2),\quad k = 1, \ldots, K"/>
            <T
                en={<Terms items={[
                    ["\\hat U", "the current nominal control sequence [\\hat u_0, \\ldots, \\hat u_{H-1}], warm-started from last tick"],
                    ["V_k", "the k-th sampled control sequence — the nominal plus noise, then clamped into the box"],
                    ["\\varepsilon_k", "the noise added to sample k, one draw \\varepsilon_k[j] per control step j"],
                    ["\\Sigma", "the noise covariance, diagonal so v and \\omega are perturbed independently"],
                    ["\\sigma_v,\\ \\sigma_\\omega", "the standard deviations of the velocity and turn-rate noise"],
                    ["K", "the number of sampled control sequences drawn each tick"],
                    ["H", "the horizon length — the number of control steps in each sequence"],
                ]}/>}
                ko={<Terms items={[
                    ["\\hat U", "현재 공칭 제어열 [\\hat u_0, \\ldots, \\hat u_{H-1}]. 지난 tick에서 warm-start"],
                    ["V_k", "k번째 표본 제어열. 공칭에 노이즈를 더한 뒤 box로 clamp한 것"],
                    ["\\varepsilon_k", "표본 k에 더한 노이즈. 제어 스텝 j마다 한 번 뽑은 \\varepsilon_k[j]"],
                    ["\\Sigma", "노이즈 공분산. 대각이라 v와 \\omega가 독립으로 섭동된다"],
                    ["\\sigma_v,\\ \\sigma_\\omega", "속도·회전율 노이즈의 표준편차"],
                    ["K", "매 tick 뽑는 표본 제어열의 수"],
                    ["H", "horizon 길이. 각 제어열의 제어 스텝 수"],
                ]}/>}
            />
            <T
                en={<p>
                    This is DWA's idea grown a horizon. DWA sampled a grid of single-step{" "}
                    <InlineMath math="(v, \omega)"/> commands and picked the best; MPPI samples whole{" "}
                    <InlineMath math="H"/>-step sequences from a Gaussian and blends them. The 1-step lattice
                    becomes an <InlineMath math="H"/>-dimensional cloud, and the argmax becomes a weighted
                    average.
                </p>}
                ko={<p>
                    이것은 DWA의 발상을 horizon만큼 키운 것이다. DWA는 한 스텝{" "}
                    <InlineMath math="(v, \omega)"/> 명령을 격자로 뽑아 최선을 골랐다. MPPI는{" "}
                    <InlineMath math="H"/>-스텝 제어열 전체를 Gauss 분포에서 뽑아 섞는다. 한 스텝 격자가{" "}
                    <InlineMath math="H"/>차원 구름이 되고, argmax가 가중 평균이 된다.
                </p>}
            />

            <h2>{t("The Path-Integral Weights", "경로적분 가중치")}</h2>
            <T
                en={<p>
                    Each sample is scored by the same cost the MPC page defined, rolling{" "}
                    <InlineMath math="V_k"/> out through the unicycle model and summing the goal, obstacle, and
                    control terms. Call that scalar <InlineMath math="S_k = J(V_k)"/>. The samples are then
                    combined by an exponential, cost-weighted average:
                </p>}
                ko={<p>
                    각 표본은 MPC 페이지가 정의한 바로 그 비용으로 채점된다. <InlineMath math="V_k"/>를
                    unicycle 모델로 굴려 goal, obstacle, control 항을 합산한다. 그 스칼라를{" "}
                    <InlineMath math="S_k = J(V_k)"/>라 하자. 표본들은 지수 비용 가중 평균으로 합쳐진다:
                </p>}
            />
            <BlockMath math="w_k = \frac{\exp\!\big(-\tfrac{1}{\lambda}(S_k - \beta)\big)}{\sum_{j=1}^{K} \exp\!\big(-\tfrac{1}{\lambda}(S_j - \beta)\big)}, \qquad \hat U \leftarrow \hat U + \sum_{k=1}^{K} w_k\, \varepsilon_k"/>
            <T
                en={<Terms items={[
                    ["S_k", <>the cost of sample k, the same three-term <InlineMath math="J(U)"/> from the MPC page evaluated on <InlineMath math="V_k"/>: goal-distance plus obstacle hinge plus control effort, summed over the horizon</>],
                    ["\\beta", "the minimum cost \\min_k S_k, subtracted only for numerical stability — it cancels in the ratio"],
                    ["\\lambda", "the temperature; small \\lambda sharpens the weights onto the cheapest sample, large \\lambda flattens them toward uniform"],
                    ["w_k", "the softmax importance weight of sample k, normalized so \\sum_k w_k = 1"],
                    ["\\varepsilon_k", "the raw noise of sample k — the update moves \\hat U along the weighted average of the noise, not of the clamped samples"],
                    ["\\hat U", "the nominal sequence being updated, then box-projected before the first control is executed"],
                ]}/>}
                ko={<Terms items={[
                    ["S_k", <>표본 k의 비용. MPC 페이지의 세 항짜리 <InlineMath math="J(U)"/>를 <InlineMath math="V_k"/>에 대해 평가한 것으로, goal 거리와 obstacle hinge와 control 노력을 horizon에 걸쳐 합산한다</>],
                    ["\\beta", "최소 비용 \\min_k S_k. 수치 안정만을 위해 빼며 비율에서 상쇄된다"],
                    ["\\lambda", "온도. 작으면 가중치가 최소비용 표본에 날카로워지고, 크면 균일 쪽으로 평평해진다"],
                    ["w_k", "표본 k의 softmax importance weight. \\sum_k w_k = 1로 정규화된다"],
                    ["\\varepsilon_k", "표본 k의 raw 노이즈. 갱신은 clamp된 표본이 아니라 노이즈의 가중 평균만큼 \\hat U를 옮긴다"],
                    ["\\hat U", "갱신되는 공칭 제어열. 첫 제어를 실행하기 전에 box로 투영한다"],
                ]}/>}
            />
            <T
                en={<p>
                    The temperature <InlineMath math="\lambda"/> is the whole character of the controller. As{" "}
                    <InlineMath math="\lambda \to 0"/> the exponential turns into a hard argmin and the update
                    chases the single cheapest sample. As <InlineMath math="\lambda \to \infty"/> every weight
                    approaches <InlineMath math="1/K"/> and the update becomes a plain average that has forgotten
                    the cost entirely. Everything good about MPPI, and its sharpest failure, lives between those
                    two limits.
                </p>}
                ko={<p>
                    온도 <InlineMath math="\lambda"/>가 이 컨트롤러의 성격 전부다.{" "}
                    <InlineMath math="\lambda \to 0"/>이면 지수가 딱딱한 argmin이 되어 갱신이 최소비용
                    표본 하나를 쫓는다. <InlineMath math="\lambda \to \infty"/>이면 모든 가중치가{" "}
                    <InlineMath math="1/K"/>로 다가가 갱신이 비용을 완전히 잊은 단순 평균이 된다. MPPI의
                    좋은 점 전부와 가장 날카로운 실패가 이 두 극한 사이에 산다.
                </p>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Cost: <InlineMath math="O(K \cdot H)"/> per tick.</strong> Each of the{" "}
                            <InlineMath math="K"/> samples rolls out and scores an <InlineMath math="O(H)"/>{" "}
                            trajectory, and there is no inner iteration loop — one pass of sampling replaces
                            MPC's <InlineMath math="\text{iterations}"/> gradient passes. The rollouts are fully
                            independent, which is exactly why MPPI is a natural fit for GPUs.</li>
                        <li><strong>Derivative-free.</strong> Nothing is ever differentiated, so the cost can
                            include hard, discontinuous, or non-smooth terms that a gradient method could not
                            handle. The obstacle penalty could be a step function and MPPI would not notice.</li>
                        <li><strong>Escapes local optima MPC cannot.</strong> Because the samples explore a
                            whole neighborhood at once rather than following one downhill direction, MPPI can
                            step over the kind of nonconvex trap that froze MPC's gradient — as long as some
                            sample lands on the far side of the barrier and scores well.</li>
                        <li><strong>Sensitive to <InlineMath math="K"/>, <InlineMath math="\sigma"/>, and{" "}
                            <InlineMath math="\lambda"/> together.</strong> Too few samples or too little noise
                            and the cloud never covers a good escape; too much noise and the average is jittery;
                            wrong temperature and the weights either collapse onto one sample or ignore cost
                            entirely. Warm-starting the nominal sequence from the previous tick keeps successive
                            plans consistent instead of re-sampling from scratch each time.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>비용: tick당 <InlineMath math="O(K \cdot H)"/>.</strong>{" "}
                            <InlineMath math="K"/>개 표본 각각이 <InlineMath math="O(H)"/> 궤적을 굴려
                            채점하고, 내부 반복 루프가 없다. 표본 한 pass가 MPC의{" "}
                            <InlineMath math="\text{iterations}"/> gradient pass를 대신한다. rollout이 완전히
                            독립이라 MPPI가 GPU에 자연히 맞는 이유다.</li>
                        <li><strong>미분이 필요 없다.</strong> 아무것도 미분하지 않으므로, gradient 방법이
                            다룰 수 없는 딱딱하거나 불연속이거나 매끄럽지 않은 항을 비용에 넣을 수 있다.
                            장애물 페널티가 계단 함수여도 MPPI는 알아채지 못한다.</li>
                        <li><strong>MPC가 못 벗어나는 국소 최적을 벗어난다.</strong> 표본이 한 내리막
                            방향을 따르는 대신 이웃 전체를 한 번에 탐색하므로, MPC의 gradient를 얼린 비볼록
                            함정을 넘어설 수 있다. 어떤 표본이 장벽 반대편에 떨어져 좋은 점수를 받기만
                            하면 된다.</li>
                        <li><strong><InlineMath math="K"/>, <InlineMath math="\sigma"/>,{" "}
                            <InlineMath math="\lambda"/>에 함께 민감하다.</strong> 표본이 너무 적거나
                            노이즈가 너무 작으면 구름이 좋은 탈출로를 덮지 못하고, 노이즈가 너무 크면
                            평균이 떨리며, 온도가 틀리면 가중치가 표본 하나로 붕괴하거나 비용을 아예
                            무시한다. 공칭 제어열을 지난 tick에서 warm-start하면 매번 처음부터 다시 뽑는
                            대신 연속한 계획이 일관되게 유지된다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Every tick warm-starts the nominal sequence, draws <InlineMath math="K"/> Gaussian-perturbed
                    samples, scores each with the shared cost, forms softmax weights, and updates the nominal
                    sequence by the weighted average of the noise before executing only its first control.
                </p>}
                ko={<p>
                    매 tick은 공칭 제어열을 warm-start하고, <InlineMath math="K"/>개 Gauss 섭동 표본을
                    뽑아 각각 공유 비용으로 채점하고, softmax 가중치를 만든 뒤, 노이즈의 가중 평균으로
                    공칭 제어열을 갱신하고, 첫 제어만 실행한다.
                </p>}
            />
            <Pseudocode code={`Û ← [û_1, ..., û_{H-1}, û_{H-1}]        # 1  warm start: left-shift, duplicate last
for k in 1..K:                          # 2  sample K perturbed sequences
    for j in 0..H-1:
        eps_k[j] ← (gauss()·sigma_v, gauss()·sigma_omega)   # 3  Box-Muller noise
        V_k[j]   ← clamp(Û_j + eps_k[j], box)
    S_k ← J(rollout(x_0, V_k))           # 4  same cost as MPC
beta ← min_k S_k                         # 5  numerical-stability baseline
w_k  ← exp(-(S_k - beta)/lambda);  w_k ← w_k / Σ_j w_j       # 6  softmax weights
for j in 0..H-1:                         # 7  weighted-average update
    Û_j ← clamp(Û_j + Σ_k w_k·eps_k[j], box)
v0 ← clamp(Û_0.v, state.v ± a_max·h)     # 8  accel-limit the executed command
return (clamp(v0,0,v_max), clampsym(Û_0.omega, omega_max))  # 9  execute only û_0`}/>
            <T
                en={<ol>
                    <li>Warm-start the nominal sequence the same way MPC does: drop the executed control, shift
                        left, duplicate the last — or seed <InlineMath math="H"/> zero controls on a cold start.</li>
                    <li>Draw <InlineMath math="K"/> samples. Every draw happens unconditionally, outside any
                        trace guard, because the samples <em>are</em> the trajectory — turning tracing on or off
                        must never change what the robot does.</li>
                    <li>Generate the Gaussian noise with Box–Muller over a reproducible uniform stream, not a
                        library's default normal generator. This is the reproducibility pitfall: numpy's ziggurat
                        normal is fast but not bit-reproducible across the Python and browser mirrors, so both
                        sides draw two uniforms and transform them the same way, giving identical noise from the
                        same seed.</li>
                    <li>Roll each sample out and score it with the exact same cost <InlineMath math="J"/> the
                        MPC page used — this shared objective is the whole point of putting the two on facing
                        pages.</li>
                    <li><strong>The overflow pitfall:</strong> subtract{" "}
                        <InlineMath math="\beta = \min_k S_k"/> before exponentiating. Costs can be hundreds or
                        thousands; <InlineMath math="\exp(-S_k/\lambda)"/> on a raw cost of a few thousand
                        underflows every weight to zero and the normalizer becomes <InlineMath math="0/0"/>.
                        Shifting so the cheapest sample sits at <InlineMath math="\exp(0) = 1"/> keeps the
                        normalizer at least one, and the shift cancels exactly in the ratio.</li>
                    <li>Form the normalized softmax weights.</li>
                    <li>Update the nominal sequence by the weighted average of the <em>raw noise</em>, then
                        box-project. Accumulation runs step-outer, sample-inner in a fixed order, because
                        floating-point summation is not associative and the cross-language mirror must fold in
                        the same sequence.</li>
                    <li>Acceleration-limit the executed speed against the velocity the simulator reports, then
                        box-clamp — the same feasibility guard MPC applies to its own first control.</li>
                    <li>Execute only <InlineMath math="\hat u_0"/>. As in MPC, the rest of the sequence exists
                        solely to shape that first control and is warm-started away on the next tick.</li>
                </ol>}
                ko={<ol>
                    <li>공칭 제어열을 MPC와 같은 방식으로 warm-start한다. 실행된 제어를 버리고 왼쪽으로
                        시프트한 뒤 마지막을 복제한다. cold start면 zero 제어 <InlineMath math="H"/>개를
                        시드한다.</li>
                    <li><InlineMath math="K"/>개 표본을 뽑는다. 모든 draw는 어떤 trace 가드 밖에서 무조건
                        일어난다. 표본이 곧 궤적이기 때문이다. trace를 켜고 끄는 것이 로봇의 동작을 바꿔선
                        안 된다.</li>
                    <li>Gauss 노이즈를 라이브러리 기본 정규 생성기가 아니라 재현 가능한 균등 스트림 위
                        Box–Muller로 만든다. 이것이 재현성 함정이다. numpy의 ziggurat 정규는 빠르지만
                        Python과 브라우저 미러 사이에서 bit-재현되지 않는다. 그래서 양쪽이 균등 난수 둘을
                        뽑아 같은 방식으로 변환해, 같은 seed에서 같은 노이즈를 낸다.</li>
                    <li>각 표본을 굴려 MPC 페이지가 쓴 바로 그 비용 <InlineMath math="J"/>로 채점한다. 이
                        공유 목적함수가 두 방식을 마주 보는 페이지에 둔 이유 전부다.</li>
                    <li><strong>overflow 함정.</strong> 지수를 취하기 전에{" "}
                        <InlineMath math="\beta = \min_k S_k"/>를 뺀다. 비용은 수백에서 수천일 수 있고,
                        raw 비용 수천에 <InlineMath math="\exp(-S_k/\lambda)"/>를 취하면 모든 가중치가 0으로
                        underflow해 정규화항이 <InlineMath math="0/0"/>이 된다. 최소비용 표본이{" "}
                        <InlineMath math="\exp(0) = 1"/>에 오도록 옮기면 정규화항이 최소 1이 되고, 그 shift는
                        비율에서 정확히 상쇄된다.</li>
                    <li>정규화된 softmax 가중치를 만든다.</li>
                    <li>공칭 제어열을 <em>raw 노이즈</em>의 가중 평균만큼 갱신한 뒤 box로 투영한다. 누적은
                        스텝 외측, 표본 내측으로 고정 순서로 돈다. 부동소수 합이 결합법칙을 따르지 않아 언어
                        간 미러가 같은 순서로 접어야 하기 때문이다.</li>
                    <li>실행 속도를 시뮬레이터가 넘긴 속도에 대해 가속 clamp한 뒤 box-clamp한다. MPC가
                        자신의 첫 제어에 적용하는 것과 같은 실현 가능성 가드다.</li>
                    <li>오직 <InlineMath math="\hat u_0"/>만 실행한다. MPC처럼 제어열의 나머지는 그 첫
                        제어를 빚는 데만 있고 다음 tick에 warm-start되어 밀려난다.</li>
                </ol>}
            />

            <Proof title={t(
                "Derivation (the information-theoretic weight)",
                "유도 (정보이론 가중치)",
            )}>
                <T
                    en={<>
                        <p><strong>Assumptions.</strong></p>
                        <ul>
                            <li>Sampling from the nominal sequence induces a base distribution{" "}
                                <InlineMath math="p(V)"/> over control sequences (Gaussian, centered at{" "}
                                <InlineMath math="\hat U"/>).</li>
                            <li>We seek the distribution <InlineMath math="q(V)"/> that minimizes the expected
                                cost regularized by how far it strays from <InlineMath math="p"/>:</li>
                        </ul>
                        <BlockMath math="q^\star = \arg\min_q\ \mathbb{E}_q[S(V)] + \lambda\, D_{\mathrm{KL}}(q \,\Vert\, p)"/>
                        <Terms items={[
                            ["q(V)", "the distribution over control sequences we are solving for"],
                            ["S(V)", "the rollout cost of sequence V (the same J evaluated on V)"],
                            ["D_{\\mathrm{KL}}(q \\Vert p)", "the KL divergence penalizing q for straying from the sampling base p"],
                            ["\\lambda", "the temperature weighting that free-energy trade-off"],
                        ]}/>
                        <p>This free-energy objective has a known closed-form minimizer (the Gibbs
                            distribution):</p>
                        <BlockMath math="q^\star(V) = \frac{1}{Z}\, p(V)\, \exp\!\Big(-\tfrac{1}{\lambda} S(V)\Big), \qquad Z = \int p(V)\, \exp\!\Big(-\tfrac{1}{\lambda} S(V)\Big)\, dV"/>
                        <Terms items={[
                            ["q^\\star(V)", "the optimal reweighting of the base distribution p"],
                            ["Z", "the normalizing constant (partition function)"],
                        ]}/>
                        <p>The optimal control is the mean of <InlineMath math="q^\star"/>. Estimating that mean
                            by <InlineMath math="K"/> Monte-Carlo samples <InlineMath math="V_k \sim p"/> makes
                            the <InlineMath math="p(V)"/> factor implicit in the draw, so the importance weight of
                            each sample is just its exponential cost, normalized:</p>
                        <BlockMath math="w_k = \frac{\exp(-S_k/\lambda)}{\sum_j \exp(-S_j/\lambda)}, \qquad \mathbb{E}_{q^\star}[V] \approx \sum_k w_k\, V_k"/>
                        <Terms items={[
                            ["w_k", "the Monte-Carlo importance weight of sample k — exactly the softmax weight above"],
                            ["V_k", "sample k, drawn from the base distribution p"],
                        ]}/>
                        <p>Subtracting <InlineMath math="\beta = \min_k S_k"/> from every exponent multiplies
                            numerator and denominator by <InlineMath math="\exp(\beta/\lambda)"/> and leaves{" "}
                            <InlineMath math="w_k"/> unchanged, which is why the baseline is free to take for
                            numerical stability. The softmax weighted average is therefore a Monte-Carlo estimate
                            of the optimal control under the free-energy objective.</p>
                    </>}
                    ko={<>
                        <p><strong>가정.</strong></p>
                        <ul>
                            <li>공칭 제어열에서 표본을 뽑는 것은 제어열 위의 base 분포{" "}
                                <InlineMath math="p(V)"/>를 만든다(<InlineMath math="\hat U"/>를 중심으로 한
                                Gauss).</li>
                            <li>기대 비용을, <InlineMath math="p"/>에서 멀어진 정도로 정규화해 최소화하는
                                분포 <InlineMath math="q(V)"/>를 찾는다:</li>
                        </ul>
                        <BlockMath math="q^\star = \arg\min_q\ \mathbb{E}_q[S(V)] + \lambda\, D_{\mathrm{KL}}(q \,\Vert\, p)"/>
                        <Terms items={[
                            ["q(V)", "우리가 구하는 제어열 위의 분포"],
                            ["S(V)", "제어열 V의 rollout 비용(같은 J를 V에 평가한 것)"],
                            ["D_{\\mathrm{KL}}(q \\Vert p)", "q가 표본 base p에서 벗어난 것을 벌하는 KL divergence"],
                            ["\\lambda", "그 자유에너지 절충을 조절하는 온도"],
                        ]}/>
                        <p>이 자유에너지 목적함수는 알려진 닫힌형 최소해(Gibbs 분포)를 갖는다:</p>
                        <BlockMath math="q^\star(V) = \frac{1}{Z}\, p(V)\, \exp\!\Big(-\tfrac{1}{\lambda} S(V)\Big), \qquad Z = \int p(V)\, \exp\!\Big(-\tfrac{1}{\lambda} S(V)\Big)\, dV"/>
                        <Terms items={[
                            ["q^\\star(V)", "base 분포 p의 최적 재가중"],
                            ["Z", "정규화 상수(분배 함수)"],
                        ]}/>
                        <p>최적 제어는 <InlineMath math="q^\star"/>의 평균이다. 그 평균을{" "}
                            <InlineMath math="K"/>개 몬테카를로 표본 <InlineMath math="V_k \sim p"/>로
                            추정하면 <InlineMath math="p(V)"/> 인자가 뽑기 안에 암묵적으로 들어가므로, 각
                            표본의 importance weight는 정규화된 지수 비용 그대로다:</p>
                        <BlockMath math="w_k = \frac{\exp(-S_k/\lambda)}{\sum_j \exp(-S_j/\lambda)}, \qquad \mathbb{E}_{q^\star}[V] \approx \sum_k w_k\, V_k"/>
                        <Terms items={[
                            ["w_k", "표본 k의 몬테카를로 importance weight. 위 softmax 가중치와 정확히 같다"],
                            ["V_k", "base 분포 p에서 뽑은 표본 k"],
                        ]}/>
                        <p>모든 지수에서 <InlineMath math="\beta = \min_k S_k"/>를 빼면 분자와 분모에{" "}
                            <InlineMath math="\exp(\beta/\lambda)"/>가 곱해져 <InlineMath math="w_k"/>는
                            그대로다. 그래서 baseline을 수치 안정을 위해 마음껏 빼도 된다. 따라서 softmax
                            가중 평균은 자유에너지 목적함수 아래 최적 제어의 몬테카를로 추정이다.</p>
                    </>}
                />
            </Proof>

            <h2>{t("The Temperature Knife-Edge", "온도의 외줄타기")}</h2>
            <T
                en={<p>
                    MPPI's characteristic failure is the temperature itself. A useful diagnostic is the{" "}
                    <strong>effective sample size</strong>{" "}
                    <InlineMath math="N_{\text{eff}} = (\sum_k w_k)^2 / \sum_k w_k^2"/>, which counts how many
                    samples actually carry weight. Push <InlineMath math="\lambda"/> too low and{" "}
                    <InlineMath math="N_{\text{eff}}"/> collapses toward 1: the update is a single high-variance
                    sample, and the plan jitters with the noise. Push <InlineMath math="\lambda"/> too high and{" "}
                    <InlineMath math="N_{\text{eff}}"/> climbs toward <InlineMath math="K"/>, but the weights
                    stop distinguishing good sequences from bad and the robot drifts off, cost-blind. A workable{" "}
                    <InlineMath math="\lambda"/> lives on a narrow ridge between collapse and drift, and the demo
                    below lets you walk right up to both edges.
                </p>}
                ko={<p>
                    MPPI의 특징적 실패는 온도 자체다. 유용한 진단은 <strong>유효 표본수</strong>{" "}
                    <InlineMath math="N_{\text{eff}} = (\sum_k w_k)^2 / \sum_k w_k^2"/>로, 실제로 가중치를
                    지는 표본이 몇 개인지 센다. <InlineMath math="\lambda"/>를 너무 낮추면{" "}
                    <InlineMath math="N_{\text{eff}}"/>가 1로 붕괴한다. 갱신이 고분산 표본 하나가 되어
                    계획이 노이즈와 함께 떨린다. <InlineMath math="\lambda"/>를 너무 높이면{" "}
                    <InlineMath math="N_{\text{eff}}"/>가 <InlineMath math="K"/>로 오르지만, 가중치가 좋은
                    제어열과 나쁜 제어열을 구분하지 못해 로봇이 비용을 못 보고 표류한다. 쓸 만한{" "}
                    <InlineMath math="\lambda"/>는 붕괴와 표류 사이 좁은 능선에 살고, 아래 데모로 양쪽
                    가장자리까지 걸어가 볼 수 있다.
                </p>}
            />
            <MppiTemperatureDemo/>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs MPPI live in your browser. The faint fan is the{" "}
                    <InlineMath math="K"/> rollouts sampled this tick, and the blue chain is their softmax
                    weighted average — the sequence actually executed, one control at a time. Raise{" "}
                    <InlineMath math="\lambda"/> and watch the plan lose its grip on the cost, or widen{" "}
                    <InlineMath math="\sigma_v"/> and watch the fan spread.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 MPPI를 라이브로 실행한다. 옅은 부채꼴은 이번 tick 뽑은{" "}
                    <InlineMath math="K"/>개 rollout이고, 파란 사슬은 그것의 softmax 가중 평균, 즉 한
                    번에 한 제어씩 실제로 실행되는 제어열이다. <InlineMath math="\lambda"/>를 올려 계획이
                    비용을 놓치는 모습을 보거나, <InlineMath math="\sigma_v"/>를 넓혀 부채꼴이 퍼지는
                    모습을 보라.
                </p>}
            />
            <MppiSandbox/>
            <MppiRolloutFigure/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The implementation below follows the algorithm above line for line: the warm-start shift,
                    the Box–Muller sampling over a reproducible uniform stream, the baseline-shifted softmax, the
                    weighted-average update, and the acceleration-limited first control. It shares the rollout
                    and cost with MPC through the family's <code>_rollout</code> module. The code is the actual
                    repository source, not an excerpt.
                </p>}
                ko={<p>
                    아래 구현은 위 알고리즘을 그대로 옮긴 것이다. warm-start 시프트, 재현 가능한 균등
                    스트림 위 Box–Muller 표본, baseline을 뺀 softmax, 가중 평균 갱신, 가속 제한된 첫
                    제어까지 그대로다. rollout과 비용은 패밀리의 <code>_rollout</code> 모듈을 통해 MPC와
                    공유한다. 아래 코드는 발췌가 아니라 저장소의 실제 소스다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/local_planning/predictive/mppi.py",
                                code: mppiPy,
                                href: `${REPO}/python/navigation/local_planning/predictive/mppi.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/include/navigation/local_planning/predictive/mppi.hpp",
                                code: mppiHpp,
                                href: `${REPO}/cpp/include/navigation/local_planning/predictive/mppi.hpp`,
                            },
                            {
                                name: "cpp/src/local_planning/predictive/mppi.cpp",
                                code: mppiCpp,
                                href: `${REPO}/cpp/src/local_planning/predictive/mppi.cpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "Box–Muller sampling over a reproducible uniform stream, baseline-shifted softmax weights, and the weighted-average update, embedded from the repository sources",
                    "재현 가능한 균등 스트림 위 Box–Muller 표본, baseline을 뺀 softmax 가중치, 가중 평균 갱신. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    G. Williams, A. Aldrich, E. A. Theodorou,{" "}
                    <a href="https://doi.org/10.1109/ICRA.2016.7487277" target="_blank" rel="noopener noreferrer">
                        <em>Aggressive driving with model predictive path integral control</em>
                    </a>, 2016 IEEE International Conference on Robotics and Automation (ICRA), pp. 1433–1440.
                </li>
                <li>
                    G. Williams, N. Wagener, B. Goldfain, P. Drews, J. M. Rehg, B. Boots, E. A. Theodorou,{" "}
                    <a href="https://doi.org/10.1109/ICRA.2017.7989202" target="_blank" rel="noopener noreferrer">
                        <em>Information theoretic MPC for model-based reinforcement learning</em>
                    </a>, 2017 IEEE International Conference on Robotics and Automation (ICRA), pp. 1714–1721.
                </li>
                <li>
                    G. Williams, P. Drews, B. Goldfain, J. M. Rehg, E. A. Theodorou,{" "}
                    <a href="https://doi.org/10.1109/TRO.2018.2865891" target="_blank" rel="noopener noreferrer">
                        <em>Information-Theoretic Model Predictive Control: Theory and Applications to Autonomous Driving</em>
                    </a>, IEEE Transactions on Robotics, vol. 34, no. 6, pp. 1603–1622, 2018.
                </li>
            </ol>
        </>
    )
}

export default Mppi

import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import AbitStarSandbox from "../../../../components/panels/global/abit_star/AbitStarSandbox";
import abitStarPy from "../../../../../../python/navigation/global_planning/sampling/abit_star.py?raw";
import abitStarCpp from "../../../../../../cpp/src/global_planning/sampling/abit_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const AbitStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    BIT* is patient. It processes the batch's edges in strict order of an
                    admissible cost estimate, so the very first solution it reports is already
                    the best the current samples allow — but reaching it can mean grinding
                    through a long queue of edges that barely bend toward the goal. ABIT*
                    (Strub &amp; Gammell, 2020) keeps BIT*'s machinery and adds two dials.
                    It <em>inflates</em> the heuristic so early batches rush greedily to a first
                    solution, and it <em>truncates</em> each batch once no remaining edge can
                    meaningfully improve the incumbent. Both dials relax to 1 across the batches,
                    so the last batch is plain BIT* again and the optimum is never given up.
                </p>}
                ko={<p>
                    BIT*는 참을성이 많다. 배치의 간선을 admissible 비용 추정 순으로 엄격히
                    처리하므로 처음 내놓는 해가 이미 현재 표본이 허락하는 최선이다. 그러나
                    거기 닿기까지 goal 쪽으로 거의 휘지 않는 간선의 긴 줄을 갈아 넣어야 할 수
                    있다. ABIT*(Strub &amp; Gammell, 2020)는 BIT*의 장치를 그대로 두고 조절
                    다이얼 둘을 더한다. heuristic을 <em>부풀려</em> 초반 배치가 탐욕적으로 첫
                    해로 달려가게 하고, 남은 어떤 간선도 현직 해를 의미 있게 못 줄이면 그 배치를
                    <em>조기 종료</em>한다. 두 다이얼은 배치를 거치며 1로 풀려, 마지막 배치는
                    다시 순수 BIT*가 되고 최적은 끝내 포기되지 않는다.
                </p>}
            />

            <h2>{t("Rushing the Batch, Then Tightening", "배치를 서둘러 훑고, 조인다")}</h2>
            <T
                en={<>
                    <p>
                        BIT* orders its edge queue by a lower bound on the solution that would
                        run through each edge: the tree cost-to-come of the edge's start,
                        <InlineMath math="g_T(v)"/>, plus the edge length, plus an admissible
                        estimate <InlineMath math="\hat h(x)"/> of the cost from the far end to
                        the goal. Admissible means <em>never an overestimate</em>, which is what
                        forces BIT* to clear every cheap-looking edge before it trusts a
                        solution.
                    </p>
                    <p>
                        The <strong>inflation</strong> dial breaks that patience on purpose.
                        ABIT* multiplies the cost-to-go term by a factor{" "}
                        <InlineMath math="\varepsilon_{\text{infl}} \ge 1"/> in the queue key
                        only, so edges that point straight at the goal jump to the front. This is
                        the weighted-A*/ARA* trick (Likhachev, Gordon &amp; Thrun, 2003) lifted
                        onto BIT*'s implicit graph: an early batch reaches a first solution after
                        far fewer edge processings, at the price of a solution that may be up to
                        a bounded factor above optimal.
                    </p>
                    <p>
                        The <strong>truncation</strong> dial stops the wasted tail. Once a batch
                        holds an incumbent of cost <InlineMath math="c_{\text{best}}"/>, any edge
                        whose admissible estimate already exceeds{" "}
                        <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/> can only
                        shave the last sliver of cost, and shaving it still costs a full lazy
                        collision check. ABIT* simply stops the batch there. Both dials decay
                        across batches, <InlineMath math="\varepsilon_{\text{infl}}"/> toward its
                        final value and <InlineMath math="\varepsilon_{\text{trunc}}"/> toward 1,
                        so the search starts greedy and cheap and ends admissible and exact.
                    </p>
                </>}
                ko={<>
                    <p>
                        BIT*는 간선 큐를 각 간선을 경유하는 해의 하한으로 정렬한다. 간선 시작점의
                        트리 cost-to-come <InlineMath math="g_T(v)"/>에 간선 길이를 더하고, 먼
                        끝에서 goal까지의 비용을 admissible하게 추정한{" "}
                        <InlineMath math="\hat h(x)"/>를 더한 값이다. admissible이란{" "}
                        <em>과대평가하지 않는다</em>는 뜻이고, 그래서 BIT*는 싸 보이는 간선을
                        모두 치운 뒤에야 해를 믿는다.
                    </p>
                    <p>
                        <strong>inflation</strong>다이얼은 그 참을성을 일부러 깬다. ABIT*는 큐
                        키에서만 cost-to-go 항에 계수{" "}
                        <InlineMath math="\varepsilon_{\text{infl}} \ge 1"/>을 곱해, goal을 곧장
                        가리키는 간선을 앞으로 끌어올린다. weighted-A*/ARA*의 수법(Likhachev,
                        Gordon &amp; Thrun, 2003)을 BIT*의 암시적 그래프 위로 옮긴 것이다. 초반
                        배치는 훨씬 적은 간선 처리로 첫 해에 닿고, 그 대가로 해가 최적보다 정해진
                        배수까지 높을 수 있다.
                    </p>
                    <p>
                        <strong>truncation</strong>다이얼은 낭비되는 꼬리를 자른다. 한 배치가
                        비용 <InlineMath math="c_{\text{best}}"/>의 현직 해를 쥐면, admissible
                        추정이 이미 <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/>를
                        넘는 간선은 마지막 비용 조각만 깎을 수 있는데, 그마저 온전한 lazy 충돌
                        검사 한 번이 든다. ABIT*는 거기서 배치를 멈춘다. 두 다이얼은 배치를 거치며
                        감소해 <InlineMath math="\varepsilon_{\text{infl}}"/>은 최종값으로,{" "}
                        <InlineMath math="\varepsilon_{\text{trunc}}"/>은 1로 향한다. 탐색은 탐욕적
                        이고 값싸게 시작해 admissible하고 정확하게 끝난다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Almost-sure asymptotic optimality</strong>: the acceptance gates
                        stay un-inflated and the last batch runs at{" "}
                        <InlineMath math="\varepsilon_{\text{infl}} = \varepsilon_{\text{trunc}} = 1"/>,
                        so ABIT* reduces to BIT* and inherits its convergence to the optimum as{" "}
                        <InlineMath math="n \to \infty"/>.</li>
                    <li><strong>Anytime with a bounded gap</strong>: an incumbent reported under
                        inflation <InlineMath math="\varepsilon_{\text{infl}}"/> is within that
                        factor of the current-batch optimum, and the gap closes as the schedule
                        relaxes.</li>
                    <li><strong>Faster first solution</strong>: the inflated key reaches a first
                        feasible path after far fewer edge processings than BIT*'s admissible
                        order — the anytime win.</li>
                    <li><strong>Fewer collision checks per batch</strong>: truncation skips the
                        lazy checks on edges that cannot improve the incumbent past{" "}
                        <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/>, the
                        edge count the sandbox measures.</li>
                    <li><strong>Informed sampling</strong>: like BIT* and Informed RRT*, once a
                        solution exists new samples fall only in the start–goal ellipse of cost{" "}
                        <InlineMath math="c_{\text{best}}"/>, and samples that can no longer
                        improve it are pruned before each batch.</li>
                    <li><strong>Cost</strong>: the ordering and repair are BIT*'s heap-driven
                        search over the <InlineMath math="\Theta(n \log n)"/> radius-graph edges;
                        this repository's naive radius graph is <InlineMath math="O(n^2)"/>{" "}
                        distance checks to build per batch.</li>
                </ul>}
                ko={<ul>
                    <li><strong>거의 확실한 점근 최적성</strong>: 채택 게이트는 un-inflated로
                        남고 마지막 배치가{" "}
                        <InlineMath math="\varepsilon_{\text{infl}} = \varepsilon_{\text{trunc}} = 1"/>
                        에서 돌아 ABIT*가 BIT*로 환원되므로,{" "}
                        <InlineMath math="n \to \infty"/>에서 최적 수렴을 물려받는다.</li>
                    <li><strong>격차가 한정된 anytime</strong>: inflation{" "}
                        <InlineMath math="\varepsilon_{\text{infl}}"/> 아래 내놓은 현직 해는 현재
                        배치 최적의 그 배수 안에 들고, 스케줄이 풀리며 격차가 좁혀진다.</li>
                    <li><strong>더 빠른 첫 해</strong>: 부풀린 키는 BIT*의 admissible 순서보다
                        훨씬 적은 간선 처리로 첫 실행 가능 경로에 닿는다. anytime 이점이다.</li>
                    <li><strong>배치당 충돌 검사가 적다</strong>: truncation은 현직 해를{" "}
                        <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/> 아래로
                        못 줄이는 간선의 lazy 검사를 건너뛴다. sandbox가 재는 간선 수다.</li>
                    <li><strong>Informed 표본</strong>: BIT*·Informed RRT*처럼, 해가 생기면 새
                        표본이 비용 <InlineMath math="c_{\text{best}}"/>의 start–goal 타원 안에만
                        떨어지고, 더는 개선할 수 없는 표본은 배치마다 미리 쳐낸다.</li>
                    <li><strong>비용</strong>: 정렬과 수리는{" "}
                        <InlineMath math="\Theta(n \log n)"/>개의 반경 그래프 간선 위 BIT*의 heap
                        탐색이다. 이 저장소의 순진한 반경 그래프는 배치마다{" "}
                        <InlineMath math="O(n^2)"/> 거리 계산으로 짓는다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    Each batch draws informed samples, rebuilds the radius graph, and runs
                    BIT*'s vertex/edge queues — but every queue key inflates its cost-to-go term
                    by <InlineMath math="\varepsilon_{\text{infl}}"/>, and the edge loop stops
                    early once the best admissible edge can no longer beat{" "}
                    <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/>. The
                    enqueue and accept tests below use the <em>un-inflated</em> estimate, so
                    inflation reorders work without ever admitting a worse edge.
                </p>}
                ko={<p>
                    배치마다 informed 표본을 뽑고 반경 그래프를 다시 지어 BIT*의 vertex/edge
                    큐를 돌린다. 다만 모든 큐 키는 cost-to-go 항을{" "}
                    <InlineMath math="\varepsilon_{\text{infl}}"/>로 부풀리고, 간선 루프는 최선
                    admissible 간선이 <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/>
                    를 더는 못 이기면 조기 종료한다. 아래 enqueue·채택 검사는{" "}
                    <em>un-inflated</em> 추정을 쓰므로, inflation은 더 나쁜 간선을 들이지 않고
                    일의 순서만 바꾼다.
                </p>}
            />
            <Pseudocode code={`for batch = 0 … B−1:
    ε_infl ← schedule(batch → ε_final);  ε_trunc ← schedule(batch → 1)   # 1
    prune samples with ĝ(x) + ĥ(x) ≥ c_best;  draw a new informed batch  # 2
    r ← γ · (log|V| / |V|)^(1/d);  rebuild radius graph
    Q_v ← tree vertices, keyed g_T(v) + ε_infl · ĥ(v)                    # 3
    loop:
        while best Q_v key ≤ best Q_e key: expand vertex → enqueue edges  # 4
        (v, x) ← pop Q_e                                                  # 5
        if g_T(v) + ‖v−x‖ + ĥ(x) ≥ c_best / ε_trunc: break               # 6  truncation
        if g_T(v) + ‖v−x‖ < g_T(x) and motion v→x collision-free:        # 7
            connect/rewire x under v;  g_T(x) ← g_T(v) + ‖v−x‖
            if x reaches goal and g_T(goal) < c_best: c_best ← g_T(goal)  # 8`}/>
            <T
                en={<ol>
                    <li>Both dials are set for this batch by a linear schedule:{" "}
                        <InlineMath math="\varepsilon_{\text{infl}}"/> decays to its final value
                        and <InlineMath math="\varepsilon_{\text{trunc}}"/> to 1, so the last
                        batch is admissible and untruncated.</li>
                    <li>Samples that even the un-inflated bound can no longer route below{" "}
                        <InlineMath math="c_{\text{best}}"/> are discarded, then a fresh batch is
                        drawn from the informed ellipse. The prune bound stays un-inflated so a
                        later, less greedy batch can still use a sample this one skipped.</li>
                    <li>The vertex queue is seeded with the tree, each keyed by its inflated
                        estimate. Inflation lives only in the key, never in the acceptance test
                        at step 7.</li>
                    <li>BIT*'s interleave: expand vertices into candidate edges as long as the
                        cheapest queued vertex could still beat the cheapest queued edge, then
                        process that edge. Inflation makes both orders lean toward the goal.</li>
                    <li>Edges leave the queue in inflated-key order, so goal-directed edges are
                        tried first — the source of the fast first solution.</li>
                    <li>The truncation test, on the <em>admissible</em> estimate. Once the best
                        remaining edge cannot pull the incumbent below{" "}
                        <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/>, the
                        batch ends and its remaining lazy collision checks are never paid.</li>
                    <li>The acceptance gate is un-inflated and strict: connect or rewire only
                        when the edge genuinely lowers <InlineMath math="x"/>'s cost-to-come and
                        its single lazy collision check passes.</li>
                    <li>Reaching the goal cheaper lowers the incumbent, which tightens both the
                        prune bound and the truncation threshold for the rest of the run.</li>
                </ol>}
                ko={<ol>
                    <li>두 다이얼은 이 배치에서 선형 스케줄로 정해진다.{" "}
                        <InlineMath math="\varepsilon_{\text{infl}}"/>은 최종값으로,{" "}
                        <InlineMath math="\varepsilon_{\text{trunc}}"/>은 1로 감소해 마지막 배치는
                        admissible하고 truncation이 없다.</li>
                    <li>un-inflated 하한으로도 <InlineMath math="c_{\text{best}}"/> 아래로 못
                        보내는 표본을 버린 뒤, informed 타원에서 새 배치를 뽑는다. prune 경계가
                        un-inflated로 남아, 덜 탐욕적인 뒤 배치가 이번에 건너뛴 표본을 아직 쓸 수
                        있다.</li>
                    <li>정점 큐는 트리로 채워지고 각자 부풀린 추정을 키로 갖는다. inflation은 키
                        에만 있고 7번 채택 검사에는 없다.</li>
                    <li>BIT*의 교차 처리다. 큐의 가장 싼 정점이 가장 싼 간선을 아직 이길 수 있는
                        동안 정점을 후보 간선으로 확장하고, 그다음 그 간선을 처리한다. inflation이
                        두 순서를 모두 goal 쪽으로 기울인다.</li>
                    <li>간선은 부풀린 키 순으로 큐를 떠나므로 goal 지향 간선이 먼저 시도된다.
                        빠른 첫 해의 원천이다.</li>
                    <li><em>admissible</em> 추정 위의 truncation 검사다. 남은 최선 간선이 현직
                        해를 <InlineMath math="c_{\text{best}} / \varepsilon_{\text{trunc}}"/> 아래로
                        못 끌면 배치가 끝나고, 남은 lazy 충돌 검사는 치르지 않는다.</li>
                    <li>채택 게이트는 un-inflated하고 엄격하다.{" "}
                        <InlineMath math="x"/>의 cost-to-come을 실제로 낮추고 그 단 한 번의 lazy
                        충돌 검사를 통과할 때만 잇거나 rewire한다.</li>
                    <li>goal에 더 싸게 닿으면 현직 해가 낮아지고, 남은 실행 동안 prune 경계와
                        truncation 임계가 함께 조여진다.</li>
                </ol>}
            />
            <T
                en={<p>
                    Written out, the two keys and the stop test are the whole difference from
                    BIT*. The queue key inflates the cost-to-go term,
                </p>}
                ko={<p>
                    풀어 쓰면 두 키와 정지 검사가 BIT*와의 차이 전부다. 큐 키는 cost-to-go 항을
                    부풀린다.
                </p>}
            />
            <BlockMath math="\operatorname{key}(v, x) = g_T(v) + \|v - x\| + \varepsilon_{\text{infl}}\,\hat h(x),"/>
            <Terms items={[
                ["g_T(v)", <T en={<>tree cost-to-come of the edge's start vertex <InlineMath math="v"/>: the length of its current tree path back to the start</>} ko={<>간선 시작 정점 <InlineMath math="v"/>의 트리 cost-to-come. 시작점까지 현재 트리 경로의 길이다</>}/>],
                ["\\|v - x\\|", <T en={<>the Euclidean length of the candidate edge from <InlineMath math="v"/> to the far end <InlineMath math="x"/></>} ko={<><InlineMath math="v"/>에서 먼 끝 <InlineMath math="x"/>까지 후보 간선의 유클리드 길이</>}/>],
                ["\\hat h(x)", <T en={<>admissible cost-to-go: the straight-line distance from <InlineMath math="x"/> to the goal, never an overestimate</>} ko={<>admissible cost-to-go. <InlineMath math="x"/>에서 goal까지 직선 거리이며 과대평가하지 않는다</>}/>],
                ["\\varepsilon_{\\text{infl}}", <T en={<><strong>the new term</strong>: an inflation factor <InlineMath math="\ge 1"/> on the cost-to-go, applied in the key only, that decays to its final value across batches</>} ko={<><strong>새로 추가된 항</strong>. cost-to-go에 곱하는 <InlineMath math="\ge 1"/>의 팽창 계수로 키에만 적용되며 배치를 거쳐 최종값으로 감소한다</>}/>],
            ]}/>
            <T
                en={<p>and the batch stops as soon as the best admissible edge estimate clears
                    the truncated incumbent,</p>}
                ko={<p>그리고 배치는 최선 admissible 간선 추정이 truncation된 현직 해를 넘는
                    즉시 멈춘다.</p>}
            />
            <BlockMath math="g_T(v) + \|v - x\| + \hat h(x) \;\ge\; \frac{c_{\text{best}}}{\varepsilon_{\text{trunc}}} \;\Longrightarrow\; \text{stop the batch.}"/>
            <Terms items={[
                ["g_T(v)", <T en={<>tree cost-to-come of the edge's start vertex <InlineMath math="v"/></>} ko={<>간선 시작 정점 <InlineMath math="v"/>의 트리 cost-to-come</>}/>],
                ["\\|v - x\\|", <T en={<>Euclidean length of the candidate edge to <InlineMath math="x"/></>} ko={<><InlineMath math="x"/>까지 후보 간선의 유클리드 길이</>}/>],
                ["\\hat h(x)", <T en={<>admissible cost-to-go from <InlineMath math="x"/> to the goal (the key here is <em>un-inflated</em>, so the stop test never discards an edge that could still improve the optimum)</>} ko={<><InlineMath math="x"/>에서 goal까지 admissible cost-to-go. 여기 키는 <em>un-inflated</em>라 정지 검사가 아직 최적을 개선할 간선을 버리지 않는다</>}/>],
                ["c_{\\text{best}}", <T en={<>cost of the current incumbent solution, or <InlineMath math="\infty"/> before the first path</>} ko={<>현직 해의 비용. 첫 경로 이전엔 <InlineMath math="\infty"/>다</>}/>],
                ["\\varepsilon_{\\text{trunc}}", <T en={<><strong>the new term</strong>: a truncation factor <InlineMath math="\ge 1"/> that decays to 1 across batches; at 1 the bound is exactly <InlineMath math="c_{\text{best}}"/> and the batch runs to BIT*'s stopping point</>} ko={<><strong>새로 추가된 항</strong>. 배치를 거쳐 1로 감소하는 <InlineMath math="\ge 1"/>의 절단 계수다. 1이면 경계가 정확히 <InlineMath math="c_{\text{best}}"/>이고 배치는 BIT*의 정지점까지 돈다</>}/>],
            ]}/>
            <Proof title={t("Why inflation never costs optimality", "inflation이 최적성을 해치지 않는 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Assumptions.</strong> An edge{" "}
                            <InlineMath math="(v, x)"/> is accepted only when its un-inflated
                            cost-to-come is a strict improvement and its lazy collision check
                            passes,
                        </p>
                        <BlockMath math="g_T(v) + \|v - x\| \;<\; g_T(x),"/>
                        <Terms items={[
                            ["g_T(v)", <T en={<>tree cost-to-come of the accepted edge's start</>} ko={<>채택 간선 시작점의 트리 cost-to-come</>}/>],
                            ["\\|v - x\\|", <T en={<>Euclidean length of the accepted edge</>} ko={<>채택 간선의 유클리드 길이</>}/>],
                            ["g_T(x)", <T en={<>the far end's current cost-to-come, before this edge (<InlineMath math="\infty"/> if unconnected)</>} ko={<>이 간선 이전 먼 끝의 현재 cost-to-come. 미연결이면 <InlineMath math="\infty"/></>}/>],
                        ]}/>
                        <p>
                            and the inflation factor{" "}
                            <InlineMath math="\varepsilon_{\text{infl}}"/> appears only in the
                            queue key, never in this test. Let{" "}
                            <InlineMath math="g_T(x)"/> and <InlineMath math="g_T'(x)"/> be the
                            cost-to-come before and after acceptance.
                        </p>
                        <p>Then every acceptance strictly lowers a cost-to-come,</p>
                        <BlockMath math="g_T'(x) \;=\; g_T(v) + \|v - x\| \;<\; g_T(x),"/>
                        <Terms items={[
                            ["g_T'(x)", <T en={<><strong>the new term</strong>: <InlineMath math="x"/>'s cost-to-come immediately after this edge is accepted</>} ko={<><strong>새로 추가된 항</strong>. 이 간선이 채택된 직후 <InlineMath math="x"/>의 cost-to-come</>}/>],
                            ["g_T(x)", <T en={<>its cost-to-come just before, which the acceptance strictly beats</>} ko={<>바로 이전의 cost-to-come. 채택이 이를 엄격히 이긴다</>}/>],
                            ["g_T(v) + \\|v - x\\|", <T en={<>the route through the accepted parent <InlineMath math="v"/></>} ko={<>채택된 부모 <InlineMath math="v"/>를 경유하는 경로</>}/>],
                        ]}/>
                        <p>
                            so when <InlineMath math="x"/> is the goal the incumbent{" "}
                            <InlineMath math="c_{\text{best}} = g_T(\text{goal})"/> is
                            nonincreasing. Inflation changed only the <em>order</em> in which
                            edges were tried, not which edges pass this gate.
                        </p>
                        <p>
                            <strong>Conclusion.</strong> No accepted edge can raise a cost or the
                            incumbent, whatever <InlineMath math="\varepsilon_{\text{infl}}"/>{" "}
                            was. On the final batch both dials are 1, so the key equals the
                            admissible key and the truncation bound equals{" "}
                            <InlineMath math="c_{\text{best}}"/> — the queue order and stopping
                            point are exactly BIT*'s. ABIT* therefore returns a solution no worse
                            than BIT* on the same samples and keeps BIT*'s almost-sure
                            convergence to the optimum as <InlineMath math="n \to \infty"/>.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 간선 <InlineMath math="(v, x)"/>는 un-inflated
                            cost-to-come이 엄격히 개선되고 lazy 충돌 검사를 통과할 때만 채택된다.
                        </p>
                        <BlockMath math="g_T(v) + \|v - x\| \;<\; g_T(x),"/>
                        <Terms items={[
                            ["g_T(v)", <T en={<>tree cost-to-come of the accepted edge's start</>} ko={<>채택 간선 시작점의 트리 cost-to-come</>}/>],
                            ["\\|v - x\\|", <T en={<>Euclidean length of the accepted edge</>} ko={<>채택 간선의 유클리드 길이</>}/>],
                            ["g_T(x)", <T en={<>the far end's current cost-to-come, before this edge (<InlineMath math="\infty"/> if unconnected)</>} ko={<>이 간선 이전 먼 끝의 현재 cost-to-come. 미연결이면 <InlineMath math="\infty"/></>}/>],
                        ]}/>
                        <p>
                            그리고 팽창 계수 <InlineMath math="\varepsilon_{\text{infl}}"/>은 큐
                            키에만 나타나고 이 검사에는 없다. 채택 전후의 cost-to-come을{" "}
                            <InlineMath math="g_T(x)"/>, <InlineMath math="g_T'(x)"/>라 하자.
                        </p>
                        <p>그러면 채택은 언제나 cost-to-come을 엄격히 낮춘다.</p>
                        <BlockMath math="g_T'(x) \;=\; g_T(v) + \|v - x\| \;<\; g_T(x),"/>
                        <Terms items={[
                            ["g_T'(x)", <T en={<><strong>the new term</strong>: <InlineMath math="x"/>'s cost-to-come immediately after this edge is accepted</>} ko={<><strong>새로 추가된 항</strong>. 이 간선이 채택된 직후 <InlineMath math="x"/>의 cost-to-come</>}/>],
                            ["g_T(x)", <T en={<>its cost-to-come just before, which the acceptance strictly beats</>} ko={<>바로 이전의 cost-to-come. 채택이 이를 엄격히 이긴다</>}/>],
                            ["g_T(v) + \\|v - x\\|", <T en={<>the route through the accepted parent <InlineMath math="v"/></>} ko={<>채택된 부모 <InlineMath math="v"/>를 경유하는 경로</>}/>],
                        ]}/>
                        <p>
                            따라서 <InlineMath math="x"/>가 goal이면 현직 해{" "}
                            <InlineMath math="c_{\text{best}} = g_T(\text{goal})"/>는 비증가한다.
                            inflation은 간선을 시도하는 <em>순서</em>만 바꿨을 뿐, 이 게이트를
                            통과하는 간선을 바꾸지 않았다.
                        </p>
                        <p>
                            <strong>결론.</strong>{" "}
                            <InlineMath math="\varepsilon_{\text{infl}}"/>이 무엇이었든 채택된
                            간선은 어떤 비용이나 현직 해도 올릴 수 없다. 마지막 배치에서 두
                            다이얼이 모두 1이므로 키는 admissible 키와 같고 truncation 경계는{" "}
                            <InlineMath math="c_{\text{best}}"/>와 같아, 큐 순서와 정지점이 정확히
                            BIT*와 같다. 그러므로 ABIT*는 같은 표본에서 BIT*보다 나쁘지 않은 해를
                            돌려주고, <InlineMath math="n \to \infty"/>에서 BIT*의 거의 확실한
                            최적 수렴을 유지한다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs ABIT* and an un-inflated BIT* baseline on the{" "}
                    <em>same samples</em>. Both land within a hair of the same cost, but watch
                    the edge counts: ABIT*'s inflated sweep and truncation build a fraction of
                    the edges — each edge being one lazy collision check — and the gap widens as
                    you add batches. The replay below is the repository demo on the benchmark
                    maps, where inflation decays across fifteen batches to the admissible final
                    one.
                </p>}
                ko={<p>
                    sandbox는 <em>같은 표본</em> 위에서 ABIT*와 un-inflated BIT* 기준선을 돌린다.
                    둘 다 거의 같은 비용에 닿지만 간선 수를 보라. ABIT*의 부풀린 훑기와 truncation
                    은 간선을 몇 분의 일만 세우고, 간선 하나가 lazy 충돌 검사 한 번이다. 배치를
                    늘리면 격차가 벌어진다. 아래 replay는 벤치마크 맵 위의 저장소 demo로, inflation
                    이 열다섯 배치에 걸쳐 admissible한 마지막 배치로 감소한다.
                </p>}
            />
            <AbitStarSandbox/>
            <TraceReplay algo="abit_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's ABIT* demo: informed batches drawn into the ellipse, an inflated goal-directed sweep to an early solution, then batches that tighten toward the admissible optimum",
                "저장소 ABIT* demo의 실제 trace. informed 배치가 타원 안으로 뿌려지고, 부풀린 goal 지향 훑기가 이른 해에 닿은 뒤, 배치들이 admissible 최적으로 조여 간다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The informed sampling, radius graph, and BIT* queues are shared with the
                    batch planners; ABIT* adds the per-batch schedule, the inflated keys, and the
                    truncation test. Embedded below in full.
                </p>}
                ko={<p>
                    informed 표본, 반경 그래프, BIT* 큐는 batch planner들과 공유한다. ABIT*는
                    배치별 스케줄과 부풀린 키, truncation 검사를 더한다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/abit_star.py",
                            code: abitStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/abit_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/abit_star.cpp",
                            code: abitStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/abit_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete ABIT* implementation, embedded from the repository sources",
                    "ABIT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    M. P. Strub, J. D. Gammell,{" "}
                    <a href="https://doi.org/10.1109/ICRA40945.2020.9196580" target="_blank"
                       rel="noopener noreferrer">
                        <em>Advanced BIT* (ABIT*): Sampling-Based Planning with Advanced
                            Graph-Search Techniques</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2020.
                </li>
                <li>
                    J. D. Gammell, S. S. Srinivasa, T. D. Barfoot,{" "}
                    <a href="https://doi.org/10.1109/ICRA.2015.7139620" target="_blank"
                       rel="noopener noreferrer">
                        <em>Batch Informed Trees (BIT*): Sampling-based Optimal Planning via the
                            Heuristically Guided Search of Implicit Random Geometric Graphs</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2015.
                </li>
                <li>
                    M. Likhachev, G. Gordon, S. Thrun,{" "}
                    <a href="https://proceedings.neurips.cc/paper/2003/hash/ee8fe9093fbbb687bef15a38facc44d2-Abstract.html"
                       target="_blank" rel="noopener noreferrer">
                        <em>ARA*: Anytime A* with Provable Bounds on Sub-Optimality</em>
                    </a>,
                    Advances in Neural Information Processing Systems (NeurIPS), 2003.
                </li>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank"
                       rel="noopener noreferrer">
                        <em>Sampling-based Algorithms for Optimal Motion Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2011.
                </li>
            </ol>
        </>
    )
}

export default AbitStar

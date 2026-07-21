import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import BitStarSandbox from "../../../../components/panels/global/bit_star/BitStarSandbox";
import bitStarPy from "../../../../../../python/navigation/global_planning/sampling/bit_star.py?raw";
import bitStarCpp from "../../../../../../cpp/src/global_planning/sampling/bit_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const BitStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Informed RRT* stops wasting samples once a path exists; FMT* stops
                    rewiring by marching one batch in cost order. BIT* (Gammell, Srinivasa
                    &amp; Barfoot, 2015) fuses both and adds a third idea: order the search by{" "}
                    <em>edges</em>, not vertices. It draws a batch of informed samples, treats
                    them as an implicit random geometric graph, and expands that graph
                    best-first with a priority queue keyed on the estimated solution cost
                    through each candidate edge — A* run over the batch — collision-checking an
                    edge only when it is dequeued. Each new batch tightens the incumbent, so the
                    answer improves the longer it runs.
                </p>}
                ko={<p>
                    Informed RRT*는 경로가 생기면 표본 낭비를 멈추고, FMT*는 한 배치를 cost
                    순으로 행진해 rewire를 없앤다. BIT*(Gammell, Srinivasa &amp; Barfoot, 2015)는
                    둘을 합치고 세 번째 아이디어를 더한다. 탐색을 정점이 아니라{" "}
                    <em>간선</em> 순으로 정렬하는 것이다. informed 표본 한 배치를 뽑아 암묵적인
                    random geometric graph로 보고, 후보 간선을 지나는 추정 해 비용을 키로 하는
                    priority queue로 그 그래프를 best-first 확장한다. 배치 위에서 A*를 돌리는
                    셈이고, 간선은 큐에서 꺼내는 순간에만 충돌 검사한다. 배치가 하나씩 쌓일 때마다
                    현직 해가 조여지므로, 오래 돌릴수록 답이 좋아진다.
                </p>}
            />

            <h2>{t("From Informed Samples to an Edge Queue", "informed 표본에서 간선 큐로")}</h2>
            <T
                en={<>
                    <p>
                        BIT* keeps three inherited pieces intact. From PRM*/FMT* it takes the{" "}
                        <em>batch</em> of samples and the shrinking connection radius{" "}
                        <InlineMath math="r_n"/> that make an implicit graph asymptotically
                        optimal. From Informed RRT* it takes the <em>ellipse</em>: once a
                        solution of cost <InlineMath math="c_{\text{best}}"/> exists, every later
                        sample is drawn only from the set of states that could still beat it.
                        What is new is <em>how the graph is searched</em>.
                    </p>
                    <p>
                        PRM* wires every feasible neighbor pair, then searches the dense roadmap.
                        FMT* marches vertices outward in cost-to-come order. BIT* instead keeps a{" "}
                        <strong>queue of edges</strong>, each scored by the cost of the whole
                        solution it would complete,
                    </p>
                    <BlockMath math="\text{key}(v, x) \;=\; g_t(v) \;+\; \hat{c}(v, x) \;+\; \hat{h}(x)."/>
                    <Terms items={[
                        ["g_t(v)", <>cost-to-come of tree vertex <InlineMath math="v"/>: the length of its tree path back to the start</>],
                        ["\\hat{c}(v, x)", <>the edge-cost estimate <InlineMath math="\|v - x\|"/>, the straight-line distance from <InlineMath math="v"/> to the unconnected sample <InlineMath math="x"/></>],
                        ["\\hat{h}(x)", <>the cost-to-go estimate <InlineMath math="\|x - \text{goal}\|"/>, the straight-line distance from <InlineMath math="x"/> to the goal</>],
                        ["\\text{key}(v, x)", <>the whole key, <strong>the new term</strong>: an admissible estimate of the best solution that routes through edge <InlineMath math="(v, x)"/></>],
                    ]}/>
                    <p>
                        This is exactly the A* <InlineMath math="f = g + h"/> value, but attached
                        to an edge rather than a node. Popping edges in increasing key order
                        expands the graph toward the goal, so BIT* touches a thin cost-ordered
                        band of edges instead of PRM*'s dense wiring. And because the key is a
                        lower bound, the first edge that connects the goal at{" "}
                        <InlineMath math="\text{key} < c_{\text{best}}"/> and survives its
                        collision check is the batch's best improvement.
                    </p>
                    <p>
                        The <strong>lazy</strong> check is the same bet FMT* makes: an edge is
                        collision-checked only when it reaches the front of the queue, not when
                        it is enqueued. Most edges never get there, because the goal is connected
                        before the queue drains — so the expensive collision checks are spent
                        only on edges cheap enough to matter.
                    </p>
                </>}
                ko={<>
                    <p>
                        BIT*는 물려받은 세 조각을 그대로 둔다. PRM*/FMT*에서는 암묵적 그래프를
                        점근 최적으로 만드는 표본 <em>배치</em>와 줄어드는 연결 반경{" "}
                        <InlineMath math="r_n"/>을 가져온다. Informed RRT*에서는 <em>타원</em>을
                        가져온다. 비용 <InlineMath math="c_{\text{best}}"/>인 해가 하나 생기면,
                        이후 표본은 그것을 아직 이길 수 있는 상태 집합 안에서만 뽑는다. 새로운
                        것은 <em>그래프를 어떻게 탐색하는가</em>다.
                    </p>
                    <p>
                        PRM*는 실행 가능한 이웃 쌍을 모두 잇고 빽빽한 roadmap을 탐색한다. FMT*는
                        정점을 cost-to-come 순으로 바깥으로 행진시킨다. BIT*는 대신 각 간선이
                        완성할 해 전체의 비용으로 점수 매긴 <strong>간선 큐</strong>를 유지한다.
                    </p>
                    <BlockMath math="\text{key}(v, x) \;=\; g_t(v) \;+\; \hat{c}(v, x) \;+\; \hat{h}(x)."/>
                    <Terms items={[
                        ["g_t(v)", <>트리 정점 <InlineMath math="v"/>의 cost-to-come. 시작점까지 트리 경로의 길이다</>],
                        ["\\hat{c}(v, x)", <>간선 비용 추정 <InlineMath math="\|v - x\|"/>. <InlineMath math="v"/>에서 아직 연결되지 않은 표본 <InlineMath math="x"/>까지의 직선 거리다</>],
                        ["\\hat{h}(x)", <>cost-to-go 추정 <InlineMath math="\|x - \text{goal}\|"/>. <InlineMath math="x"/>에서 goal까지의 직선 거리다</>],
                        ["\\text{key}(v, x)", <>키 전체. <strong>새로 추가된 항</strong>이며, 간선 <InlineMath math="(v, x)"/>를 지나는 최선 해의 admissible 추정이다</>],
                    ]}/>
                    <p>
                        이것은 정확히 A*의 <InlineMath math="f = g + h"/> 값인데, 노드가 아니라
                        간선에 붙는다. 키가 작은 간선부터 꺼내면 그래프가 goal 쪽으로 확장되므로,
                        BIT*는 PRM*의 빽빽한 배선 대신 cost 순으로 정렬된 얇은 간선 띠만 건드린다.
                        그리고 키가 하한이므로, <InlineMath math="\text{key} < c_{\text{best}}"/>에서
                        goal을 잇고 충돌 검사를 통과하는 첫 간선이 그 배치의 최선 개선이다.
                    </p>
                    <p>
                        <strong>lazy</strong> 검사는 FMT*가 거는 것과 같은 내기다. 간선은 큐에 넣을
                        때가 아니라 큐 맨 앞에 닿는 순간에만 충돌 검사한다. 대부분의 간선은 거기까지
                        가지 못한다. 큐가 마르기 전에 goal이 연결되기 때문이다. 비싼 충돌 검사는
                        중요할 만큼 싼 간선에만 쓰인다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Almost-surely asymptotically optimal</strong>: the shrinking
                        radius <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/> and informed
                        sampling give the same convergence guarantee as PRM*/Informed RRT* as{" "}
                        <InlineMath math="n \to \infty"/>.</li>
                    <li><strong>Anytime</strong>: the returned cost is non-increasing across
                        batches. Each batch only tightens <InlineMath math="c_{\text{best}}"/>,
                        so stopping after any batch yields a valid path no worse than the last —
                        the property the sandbox and the proof below make precise.</li>
                    <li><strong>Ordered like A*</strong>: the edge-queue key is{" "}
                        <InlineMath math="g + h"/> on the implicit graph, so the search advances
                        toward the goal and spends collision checks on a cost-ordered band of
                        edges instead of every feasible pair — far more sample-efficient than
                        Informed RRT*'s one steered extension per sample.</li>
                    <li><strong>Lazy suboptimality within a batch</strong>: like FMT*, an edge
                        is trusted until dequeued, so a single batch need not return that batch
                        graph's exact optimum. Successive batches close the gap, and the limit is
                        optimal.</li>
                    <li><strong>Cost, paper version</strong>: BIT* is designed to <em>reuse</em>
                        the graph, queues, and collision results across batches, and to prune the
                        tree — that incremental bookkeeping is what makes it fast on large
                        problems.</li>
                    <li><strong>Cost, this repository</strong>: for clarity the implementation
                        rebuilds the radius graph (<InlineMath math="O(n^2)"/> distance checks)
                        and both queues from scratch every batch, and prunes only disconnected
                        samples outside the ellipse — never tree vertices. The search order and
                        returned costs match the algorithm; the per-batch work does not.</li>
                </ul>}
                ko={<ul>
                    <li><strong>거의 확실히 점근 최적</strong>: 줄어드는 반경{" "}
                        <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/>과 informed 표본이{" "}
                        <InlineMath math="n \to \infty"/>에서 PRM*/Informed RRT*와 같은 수렴
                        보장을 준다.</li>
                    <li><strong>Anytime</strong>: 반환 비용이 배치를 거치며 비증가한다. 각 배치는{" "}
                        <InlineMath math="c_{\text{best}}"/>를 조이기만 하므로, 어느 배치 뒤에
                        멈춰도 직전보다 나쁘지 않은 유효 경로를 얻는다. sandbox와 아래 증명이
                        엄밀히 하는 성질이다.</li>
                    <li><strong>A*처럼 정렬</strong>: 간선 큐의 키가 암묵적 그래프 위의{" "}
                        <InlineMath math="g + h"/>라, 탐색이 goal 쪽으로 나아가며 실행 가능한 모든
                        쌍이 아니라 cost 순으로 정렬된 간선 띠에 충돌 검사를 쓴다. 표본마다 한 번
                        steer하는 Informed RRT*보다 훨씬 표본 효율적이다.</li>
                    <li><strong>배치 내 lazy 준최적</strong>: FMT*처럼 간선은 꺼낼 때까지 믿으므로,
                        한 배치가 그 배치 그래프의 정확한 최적을 반환하지는 않는다. 이어지는 배치가
                        격차를 좁히고, 극한은 최적이다.</li>
                    <li><strong>비용, 논문 버전</strong>: BIT*는 그래프와 큐와 충돌 검사 결과를
                        배치 사이에 <em>재사용</em>하고 트리를 가지치기하도록 설계됐다. 그
                        incremental한 관리가 큰 문제에서 BIT*를 빠르게 만든다.</li>
                    <li><strong>비용, 이 저장소</strong>: 명료함을 위해 구현은 배치마다 반경
                        그래프(<InlineMath math="O(n^2)"/> 거리 계산)와 두 큐를 처음부터 다시
                        세우고, 타원 밖으로 연결이 끊긴 표본만 쳐낸다. 트리 정점은 쳐내지 않는다.
                        탐색 순서와 반환 비용은 알고리즘과 같고, 배치당 작업량만 다르다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    The state is the growing sample array with per-sample cost-to-come, parent,
                    and an <em>in-tree</em> flag, plus the incumbent cost{" "}
                    <InlineMath math="c_{\text{best}}"/>. Each batch runs two queues over a
                    freshly built radius graph: a <em>vertex</em> queue{" "}
                    <InlineMath math="Q_V"/> of tree vertices to expand, keyed{" "}
                    <InlineMath math="g_t(v) + \hat{h}(v)"/>, and an <em>edge</em> queue{" "}
                    <InlineMath math="Q_E"/> of candidate connections, keyed as above. The two
                    are interleaved so that edges are always dequeued in true key order.
                </p>}
                ko={<p>
                    상태는 표본마다 cost-to-come, parent, <em>in-tree</em> 플래그를 갖는 커지는
                    표본 배열과 현직 비용 <InlineMath math="c_{\text{best}}"/>다. 각 배치는 새로
                    세운 반경 그래프 위에서 두 큐를 돌린다. 확장할 트리 정점의{" "}
                    <em>정점</em> 큐 <InlineMath math="Q_V"/>는{" "}
                    <InlineMath math="g_t(v) + \hat{h}(v)"/>를 키로, 후보 연결의{" "}
                    <em>간선</em> 큐 <InlineMath math="Q_E"/>는 위 키를 쓴다. 둘을 번갈아 처리해
                    간선이 항상 참 키 순서로 나오게 한다.
                </p>}
            />
            <Pseudocode code={`c_best ← ∞
for each batch:
    prune samples with ĝ(x) + ĥ(x) ≥ c_best          # 1
    add batch of informed samples (ellipse if c_best < ∞)   # 2
    r ← γ · (log|V| / |V|)^(1/d);  build radius graph  # 3
    Q_V ← tree vertices keyed g_t(v)+ĥ(v);  Q_E ← ∅
    loop:
        while best Q_V key ≤ best Q_E key:            # 4
            pop v; for near x: push edge (v,x) to Q_E # 5
        if Q_E empty: end batch
        (v,x) ← pop min-key edge from Q_E
        if key(v,x) ≥ c_best: end batch               # 6
        if g_t(v)+‖v−x‖ < g_t(x) and motion v→x free: # 7
            connect or rewire x under v; propagate    # 8
            push x to Q_V
            if g_t(goal) < c_best:                     # 9
                c_best ← g_t(goal)`}/>
            <T
                en={<ol>
                    <li>Prune first: any still-unconnected sample whose optimistic
                        start-to-goal cost <InlineMath math="\hat{g}(x) + \hat{h}(x)"/> already
                        meets the incumbent cannot improve it, so it is dropped before the batch
                        grows.</li>
                    <li>Draw the batch. Before any solution the samples are uniform; once{" "}
                        <InlineMath math="c_{\text{best}} < \infty"/> they come from the informed
                        ellipse with foci start and goal and transverse diameter{" "}
                        <InlineMath math="c_{\text{best}}"/> (Gammell et al., 2014), so every new
                        point lies where the incumbent can still improve.</li>
                    <li>The connection radius is recomputed from the current sample count — the
                        PRM* formula — and the radius graph over all samples is rebuilt for this
                        batch.</li>
                    <li>The interleave rule. A tree vertex whose key{" "}
                        <InlineMath math="g_t(v) + \hat{h}(v)"/> is no larger than the best queued
                        edge might, once expanded, produce an even cheaper edge; expanding it
                        first is what guarantees edges leave <InlineMath math="Q_E"/> in true
                        increasing key order.</li>
                    <li>Expanding a vertex enqueues its outgoing candidate edges: connections to
                        nearby unconnected samples, and rewirings of nearby tree vertices that{" "}
                        <InlineMath math="v"/> could reach more cheaply. Both are enqueued only if
                        their key can still beat <InlineMath math="c_{\text{best}}"/>.</li>
                    <li>Batch termination. Because edges leave in key order, once the best
                        remaining edge cannot beat the incumbent, nothing left can — end the batch
                        and draw the next.</li>
                    <li>The lazy collision check, deferred to dequeue time. The edge is taken only
                        if it strictly lowers <InlineMath math="x"/>'s cost-to-come <em>and</em>
                        the motion is free; a collision simply discards the edge, and{" "}
                        <InlineMath math="x"/> waits for a later one.</li>
                    <li>Accepting the edge connects a new sample or reroutes an existing vertex
                        under <InlineMath math="v"/>; the cost change is pushed down{" "}
                        <InlineMath math="x"/>'s subtree so every descendant's{" "}
                        <InlineMath math="g_t"/> stays exact, and <InlineMath math="x"/> re-enters{" "}
                        <InlineMath math="Q_V"/> so it can expand in turn.</li>
                    <li>Whenever an accepted edge lowers the goal's cost-to-come, the incumbent
                        drops. This is the only place <InlineMath math="c_{\text{best}}"/> moves,
                        and it only moves down.</li>
                </ol>}
                ko={<ol>
                    <li>먼저 가지치기한다. 아직 연결되지 않은 표본 중 낙관적 시작–목표 비용{" "}
                        <InlineMath math="\hat{g}(x) + \hat{h}(x)"/>이 이미 현직 해에 닿는 것은
                        개선할 수 없으므로, 배치가 커지기 전에 버린다.</li>
                    <li>배치를 뽑는다. 해가 없을 땐 표본이 균일하고, 일단{" "}
                        <InlineMath math="c_{\text{best}} < \infty"/>가 되면 시작과 goal을 초점,{" "}
                        <InlineMath math="c_{\text{best}}"/>를 횡축 지름으로 하는 informed 타원에서
                        나온다(Gammell et al., 2014). 새 점은 모두 현직 해가 아직 개선될 수 있는
                        곳에 떨어진다.</li>
                    <li>연결 반경을 현재 표본 수로 다시 계산하고(PRM* 공식), 모든 표본 위 반경
                        그래프를 이번 배치용으로 다시 세운다.</li>
                    <li>번갈기 규칙이다. 키 <InlineMath math="g_t(v) + \hat{h}(v)"/>가 큐의 최선
                        간선보다 크지 않은 트리 정점은, 확장하면 더 싼 간선을 낼 수도 있다. 그것을
                        먼저 확장하는 것이 간선이 <InlineMath math="Q_E"/>에서 참 증가 키 순서로
                        나오도록 보장한다.</li>
                    <li>정점을 확장하면 그 나가는 후보 간선이 큐에 들어간다. 가까운 미연결 표본으로의
                        연결과, <InlineMath math="v"/>를 거치면 더 싸게 닿는 가까운 트리 정점의
                        rewiring이다. 둘 다 키가 아직 <InlineMath math="c_{\text{best}}"/>를 이길 수
                        있을 때만 넣는다.</li>
                    <li>배치 종료. 간선이 키 순으로 나오므로, 남은 최선 간선이 현직 해를 못 이기면
                        남은 어떤 것도 못 이긴다. 배치를 끝내고 다음을 뽑는다.</li>
                    <li>lazy 충돌 검사는 꺼내는 시점으로 미룬다. 간선은{" "}
                        <InlineMath math="x"/>의 cost-to-come을 엄격히 낮추고 <em>그리고</em> 이동이
                        비어 있을 때만 채택된다. 막히면 그 간선을 버리고{" "}
                        <InlineMath math="x"/>는 뒤의 간선을 기다린다.</li>
                    <li>간선 채택은 새 표본을 연결하거나 기존 정점을{" "}
                        <InlineMath math="v"/> 아래로 다시 잇는다. 바뀐 비용을{" "}
                        <InlineMath math="x"/>의 부분트리로 밀어 모든 자손의{" "}
                        <InlineMath math="g_t"/>를 정확히 유지하고, <InlineMath math="x"/>는 다시{" "}
                        <InlineMath math="Q_V"/>에 들어가 차례로 확장된다.</li>
                    <li>채택된 간선이 goal의 cost-to-come을 낮출 때마다 현직 해가 내려간다.{" "}
                        <InlineMath math="c_{\text{best}}"/>가 움직이는 유일한 지점이고, 내려가기만
                        한다.</li>
                </ol>}
            />
            <Proof title={t("Why BIT* is anytime: the incumbent never rises", "BIT*가 anytime인 이유: 현직 해는 오르지 않는다")}>
                <T
                    en={<>
                        <p>
                            <strong>Assumptions.</strong> The incumbent{" "}
                            <InlineMath math="c_{\text{best}}"/> starts at{" "}
                            <InlineMath math="\infty"/> and is written in exactly one place
                            (step 9), only when an accepted edge makes{" "}
                            <InlineMath math="g_t(\text{goal})"/> strictly smaller. An edge is
                            accepted (step 7) only when it strictly lowers a vertex's{" "}
                            <InlineMath math="g_t"/> and its motion is collision-free, and each
                            batch only <em>adds</em> samples and edges — no tree edge carrying the
                            incumbent is ever removed.
                        </p>
                        <p>
                            Let <InlineMath math="c_k"/> be the incumbent at the end of batch{" "}
                            <InlineMath math="k"/>. Any write during batch{" "}
                            <InlineMath math="k+1"/> replaces it by some{" "}
                            <InlineMath math="g_t(\text{goal})"/> that passed the step-9 test, so
                        </p>
                        <BlockMath math="c_{k+1} \;=\; \min\bigl(c_k,\; \min_j g_t^{(j)}(\text{goal})\bigr) \;\le\; c_k,"/>
                        <Terms items={[
                            ["c_k", <>the incumbent cost at the end of batch <InlineMath math="k"/></>],
                            ["c_{k+1}", <>the incumbent at the end of the next batch</>],
                            ["g_t^{(j)}(\\text{goal})", <>the goal's tree cost-to-come at the <InlineMath math="j"/>-th acceptance in batch <InlineMath math="k+1"/>, <strong>the new term</strong>, each written only when below the current incumbent</>],
                        ]}/>
                        <p>
                            where the outer <InlineMath math="\min"/> is over every acceptance in
                            the batch and the pruning of step 1 removes only samples with{" "}
                            <InlineMath math="\hat{g}(x) + \hat{h}(x) \ge c_{\text{best}}"/>, none
                            of which can lie on a cheaper path than the incumbent.
                        </p>
                        <p>
                            <strong>Conclusion.</strong> The sequence{" "}
                            <InlineMath math="c_0 \ge c_1 \ge c_2 \ge \cdots"/> is
                            non-increasing, so halting after any batch returns a path no worse
                            than every earlier one. That is precisely what makes BIT* anytime, and
                            with the shrinking radius the sequence converges to the optimum almost
                            surely. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 현직 해{" "}
                            <InlineMath math="c_{\text{best}}"/>는{" "}
                            <InlineMath math="\infty"/>에서 시작해 단 한 곳(스텝 9)에서만, 채택된
                            간선이 <InlineMath math="g_t(\text{goal})"/>을 엄격히 줄일 때만
                            갱신된다. 간선은(스텝 7) 어떤 정점의{" "}
                            <InlineMath math="g_t"/>를 엄격히 낮추고 이동이 충돌 없을 때만
                            채택되며, 각 배치는 표본과 간선을 <em>더하기만</em> 한다. 현직 해를
                            나르는 트리 간선은 제거되지 않는다.
                        </p>
                        <p>
                            배치 <InlineMath math="k"/> 끝의 현직 해를{" "}
                            <InlineMath math="c_k"/>라 하자. 배치{" "}
                            <InlineMath math="k+1"/> 동안의 어떤 갱신도 스텝 9 검사를 통과한{" "}
                            <InlineMath math="g_t(\text{goal})"/>으로 바꾸므로,
                        </p>
                        <BlockMath math="c_{k+1} \;=\; \min\bigl(c_k,\; \min_j g_t^{(j)}(\text{goal})\bigr) \;\le\; c_k"/>
                        <Terms items={[
                            ["c_k", <>배치 <InlineMath math="k"/> 끝의 현직 비용</>],
                            ["c_{k+1}", <>다음 배치 끝의 현직 비용</>],
                            ["g_t^{(j)}(\\text{goal})", <>배치 <InlineMath math="k+1"/>의 <InlineMath math="j"/>번째 채택 시점 goal의 트리 cost-to-come. <strong>새로 추가된 항</strong>이며, 현직 해보다 낮을 때만 기록된다</>],
                        ]}/>
                        <p>
                            이고, 바깥 <InlineMath math="\min"/>은 배치 내 모든 채택에 대한 것이다.
                            스텝 1의 가지치기는{" "}
                            <InlineMath math="\hat{g}(x) + \hat{h}(x) \ge c_{\text{best}}"/>인
                            표본만 지우는데, 그중 어느 것도 현직 해보다 싼 경로 위에 있을 수 없다.
                        </p>
                        <p>
                            <strong>결론.</strong> 수열{" "}
                            <InlineMath math="c_0 \ge c_1 \ge c_2 \ge \cdots"/>은 비증가이므로,
                            어느 배치 뒤에 멈춰도 앞선 모든 것보다 나쁘지 않은 경로를 반환한다.
                            이것이 BIT*를 anytime으로 만들고, 줄어드는 반경과 함께 이 수열은 거의
                            확실히 최적으로 수렴한다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs BIT* around a central block and lets you add batches one at
                    a time. The first batch already returns a bent path; each further batch draws
                    a fresh informed batch and the edge queue tightens the incumbent toward the
                    taut corner route. Alongside, Informed RRT* runs on the same sample budget —
                    and often has not found a path yet, because BIT*'s edge queue reaches the goal
                    with far fewer samples. The replay below is the repository demo on the
                    benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 가운데 블록을 도는 BIT*를 돌리고, 배치를 하나씩 더할 수 있게 한다.
                    첫 배치가 이미 꺾인 경로를 반환하고, 배치를 더할 때마다 새 informed 배치가
                    뽑히며 간선 큐가 현직 해를 모서리에 밀착한 팽팽한 경로로 조인다. 옆에서는 같은
                    표본 예산의 Informed RRT*가 함께 도는데, 아직 경로를 못 찾은 경우가 많다. BIT*의
                    간선 큐가 훨씬 적은 표본으로 goal에 닿기 때문이다. 아래 replay는 벤치마크 맵 위의
                    저장소 demo다.
                </p>}
            />
            <BitStarSandbox/>
            <TraceReplay algo="bit_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's BIT* demo: informed batches drawn, then a cost-ordered edge queue that lazily connects each accepted edge and tightens the goal cost batch by batch",
                "저장소 BIT* demo의 실제 trace. informed 배치를 뽑은 뒤, cost 순 간선 큐가 채택된 간선을 lazy하게 잇고 배치마다 goal 비용을 조인다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The informed ellipse and the shrinking-radius graph are shared with the batch
                    planners; BIT* itself is the two-queue loop with its single lazy collision
                    check per dequeued edge. Embedded below in full.
                </p>}
                ko={<p>
                    informed 타원과 줄어드는 반경 그래프는 batch planner들과 공유한다. BIT* 자체는
                    꺼낸 간선마다 단 한 번 lazy 충돌 검사를 하는 두 큐 루프다. 전체를 아래에 embed
                    했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/bit_star.py",
                            code: bitStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/bit_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/bit_star.cpp",
                            code: bitStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/bit_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete BIT* implementation, embedded from the repository sources",
                    "BIT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
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
                    J. D. Gammell, T. D. Barfoot, S. S. Srinivasa,{" "}
                    <a href="https://doi.org/10.1177/0278364919890396" target="_blank"
                       rel="noopener noreferrer">
                        <em>Batch Informed Trees (BIT*): Informed Asymptotically Optimal
                            Anytime Search</em>
                    </a>,
                    The International Journal of Robotics Research, 2020.
                </li>
                <li>
                    J. D. Gammell, S. S. Srinivasa, T. D. Barfoot,{" "}
                    <a href="https://doi.org/10.1109/IROS.2014.6942976" target="_blank"
                       rel="noopener noreferrer">
                        <em>Informed RRT*: Optimal Sampling-based Path Planning Focused via
                            Direct Sampling of an Admissible Ellipsoidal Heuristic</em>
                    </a>,
                    IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS), 2014.
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

export default BitStar

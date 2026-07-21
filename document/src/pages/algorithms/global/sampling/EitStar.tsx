import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import EitStarSandbox from "../../../../components/panels/global/eit_star/EitStarSandbox";
import eitStarPy from "../../../../../../python/navigation/global_planning/sampling/eit_star.py?raw";
import eitStarCpp from "../../../../../../cpp/src/global_planning/sampling/eit_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const EitStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    AIT* steers a forward search with a reverse cost-to-go heuristic that bends
                    around obstacles. But cost is not the only thing that makes a path expensive to
                    find: in high-dimensional or expensive-to-validate spaces the collision checker,
                    not the graph search, dominates runtime. EIT* (Strub &amp; Gammell, 2022) keeps
                    AIT*'s obstacle-aware cost heuristic and adds a <em>second</em> reverse search
                    that estimates the remaining <strong>validation effort</strong> — roughly, how
                    many collision-check segments the rest of the path will cost. Its forward search
                    orders candidates by cost first and effort second, so among routes of equal cost
                    it prefers the ones that are cheaper to check, surfacing feasible solutions
                    sooner.
                </p>}
                ko={<p>
                    AIT*는 장애물을 우회하는 역방향 cost-to-go heuristic으로 전방 탐색을 이끈다.
                    그런데 경로를 찾기 비싸게 만드는 것이 비용만은 아니다. 고차원이거나 검증이 비싼
                    공간에서는 그래프 탐색이 아니라 충돌 검사가 런타임을 지배한다. EIT*(Strub &amp;
                    Gammell, 2022)는 AIT*의 장애물 인지 cost heuristic을 그대로 두고, 남은 경로의
                    <strong>검증 노력</strong>, 곧 나머지 경로가 충돌 검사 segment 몇 개를 들일지를
                    추정하는 <em>두 번째</em> 역방향 탐색을 더한다. 전방 탐색은 후보를 비용으로 먼저,
                    노력으로 그 다음 정렬한다. 비용이 같은 경로들 사이에서 더 싸게 검사되는 쪽을
                    골라, 실행 가능한 해를 더 일찍 내놓는다.
                </p>}
            />

            <h2>{t("From Cost to Cost and Effort", "비용에서 비용과 노력으로")}</h2>
            <T
                en={<>
                    <p>
                        AIT* runs one reverse search over the sample graph: a Dijkstra from the goal
                        with edge weight equal to Euclidean distance, giving a cost-to-go{" "}
                        <InlineMath math="\hat h(x)"/> that already knows to route around walls. EIT*
                        keeps that search unchanged and runs a second one over the same graph, this
                        time with edge weight equal to <strong>effort</strong>: the number of
                        fixed-step sub-segments a discrete collision checker would test along that
                        edge. The result is an effort-to-go <InlineMath math="\hat e(x)"/> — a
                        cost-to-go in the currency of collision checks rather than metres.
                    </p>
                    <BlockMath math="e(u, v) \;=\; \max\!\left(1,\; \operatorname{round}\!\left(\frac{\lVert u - v \rVert}{\eta}\right)\right)"/>
                    <Terms items={[
                        ["e(u, v)", <><strong>the new term</strong>: the per-edge <strong>effort</strong> — how many <InlineMath math="\eta"/>-sized sub-segments a discrete validator checks along edge <InlineMath math="(u, v)"/></>],
                        ["\\lVert u - v \\rVert", <>Euclidean length of the edge between samples <InlineMath math="u"/> and <InlineMath math="v"/></>],
                        ["\\eta", <><strong>the new term</strong>: <code>step_size</code>, the discretization interval of the collision checker (metres per sub-segment)</>],
                        ["\\operatorname{round}(\\cdot)", <>nearest integer; the <InlineMath math="\max(1, \cdot)"/> floor keeps every real edge at a cost of at least one check</>],
                    ]}/>
                    <p>
                        Effort needs no new capability from the map: it reads only the sampling
                        distance the planner already uses. And because it counts sub-segments, it is
                        a proxy for the true cost of validating a motion, not for its length. A route
                        of many short hops costs one check per hop even where the hops are tiny, so
                        the fewest-effort route is not the same as the shortest route — it favours
                        fewer, longer edges.
                    </p>
                    <p>
                        The forward search consumes both heuristics through a lexicographic key. Cost
                        is primary, so EIT* returns exactly the cost AIT* would; effort is secondary,
                        breaking ties toward the candidate cheaper to collision-check.
                    </p>
                    <BlockMath math="k(v) \;=\; \big(\; g(v) + \hat h(v),\;\; \varepsilon(v) + \hat e(v) \;\big)"/>
                    <Terms items={[
                        ["k(v)", <>the priority key of vertex <InlineMath math="v"/> in the forward queue; keys compare <strong>lexicographically</strong> — cost first, effort only on an exact cost tie</>],
                        ["g(v)", <>cost-to-come: the cost of the current forward-tree path from the start to <InlineMath math="v"/></>],
                        ["\\hat h(v)", <>reverse cost-to-go at <InlineMath math="v"/>, from the distance Dijkstra — the same estimate AIT* uses</>],
                        ["\\varepsilon(v)", <><strong>the new term</strong>: effort-to-come, the accumulated collision-check segments of the forward-tree path to <InlineMath math="v"/></>],
                        ["\\hat e(v)", <><strong>the new term</strong>: reverse effort-to-go at <InlineMath math="v"/>, from the effort Dijkstra — the estimated checks remaining to the goal</>],
                    ]}/>
                    <p>
                        This repository keeps that defining behavior — two reverse searches, a
                        cost-primary/effort-secondary forward key — but simplifies the mechanics, the
                        same way it does for AIT*. Each batch recomputes both reverse searches from
                        scratch over all accumulated samples rather than repairing them incrementally
                        (LPA*-style) as the paper does, and the two heuristics come from two clean
                        independent Dijkstra passes rather than the paper's integrated joint search.
                        Reduced to a strict lexicographic tie-break over real-valued costs, effort
                        rarely changes the returned path on continuous samples; it is the mechanism,
                        not the paper's full validation-effort savings, that this page demonstrates.
                    </p>
                </>}
                ko={<>
                    <p>
                        AIT*는 표본 그래프 위에서 역방향 탐색을 한 번 돌린다. goal로부터 간선 가중치를
                        유클리드 거리로 둔 Dijkstra로, 벽을 이미 우회할 줄 아는 cost-to-go{" "}
                        <InlineMath math="\hat h(x)"/>를 얻는다. EIT*는 그 탐색을 그대로 두고, 같은
                        그래프 위에서 두 번째 탐색을 돌린다. 이번엔 간선 가중치가 <strong>effort</strong>다.
                        이산 충돌 검사기가 그 간선을 따라 검사할 고정 스텝 sub-segment의 수다. 그 결과가
                        effort-to-go <InlineMath math="\hat e(x)"/>다. meters가 아니라 충돌 검사라는
                        단위로 잰 cost-to-go다.
                    </p>
                    <BlockMath math="e(u, v) \;=\; \max\!\left(1,\; \operatorname{round}\!\left(\frac{\lVert u - v \rVert}{\eta}\right)\right)"/>
                    <Terms items={[
                        ["e(u, v)", <><strong>새로 추가된 항</strong>. 간선별 <strong>effort</strong>다. 이산 검증기가 간선 <InlineMath math="(u, v)"/>를 따라 검사하는 <InlineMath math="\eta"/> 크기 sub-segment의 수</>],
                        ["\\lVert u - v \\rVert", <>표본 <InlineMath math="u"/>와 <InlineMath math="v"/> 사이 간선의 유클리드 길이</>],
                        ["\\eta", <><strong>새로 추가된 항</strong>. <code>step_size</code>, 충돌 검사기의 이산화 간격(sub-segment당 meters)</>],
                        ["\\operatorname{round}(\\cdot)", <>가장 가까운 정수. <InlineMath math="\max(1, \cdot)"/> 하한이 모든 실제 간선을 최소 검사 한 번으로 유지한다</>],
                    ]}/>
                    <p>
                        effort는 맵에 새 capability를 요구하지 않는다. planner가 이미 쓰는 sampling
                        거리만 읽는다. 그리고 sub-segment를 세므로, motion을 검증하는 참 비용의 proxy이지
                        길이의 proxy가 아니다. 짧은 hop이 많은 경로는 hop이 아무리 작아도 hop마다 검사
                        하나가 든다. 그래서 effort 최소 경로는 최단 경로와 다르다. 더 적고 더 긴 간선을
                        선호한다.
                    </p>
                    <p>
                        전방 탐색은 사전순 key로 두 heuristic을 함께 소비한다. 비용이 1차라 EIT*는 AIT*가
                        낼 비용을 그대로 반환한다. effort는 2차라, 더 싸게 충돌 검사되는 후보로 tie를
                        가른다.
                    </p>
                    <BlockMath math="k(v) \;=\; \big(\; g(v) + \hat h(v),\;\; \varepsilon(v) + \hat e(v) \;\big)"/>
                    <Terms items={[
                        ["k(v)", <>전방 큐에서 정점 <InlineMath math="v"/>의 우선순위 key. key는 <strong>사전순</strong>으로 비교한다. 비용이 먼저, effort는 비용이 정확히 같을 때만</>],
                        ["g(v)", <>cost-to-come. 시작점에서 <InlineMath math="v"/>까지 현재 전방 트리 경로의 비용</>],
                        ["\\hat h(v)", <><InlineMath math="v"/>에서의 역방향 cost-to-go. 거리 Dijkstra에서 나온다. AIT*가 쓰는 것과 같은 추정</>],
                        ["\\varepsilon(v)", <><strong>새로 추가된 항</strong>. effort-to-come. 시작점에서 <InlineMath math="v"/>까지 전방 트리 경로의 누적 충돌 검사 segment 수</>],
                        ["\\hat e(v)", <><strong>새로 추가된 항</strong>. <InlineMath math="v"/>에서의 역방향 effort-to-go. effort Dijkstra에서 나온다. goal까지 남은 검사 추정치</>],
                    ]}/>
                    <p>
                        이 저장소는 그 정의적 동작, 곧 역방향 탐색 둘과 비용 1차·effort 2차 전방 key를
                        지키되 기계 부분을 단순화한다. AIT*와 같은 방식이다. 배치마다 두 역방향 탐색을
                        누적된 전체 표본 위에서 처음부터 다시 계산한다. 원 논문의 LPA* 증분 수리가 아니다.
                        두 heuristic도 통합된 joint 탐색이 아니라 독립적인 Dijkstra 두 번에서 나온다. 실수
                        비용 위 엄격한 사전순 tie-break로 줄어든 effort는 연속 표본에서 반환 경로를 바꾸는
                        일이 드물다. 이 페이지가 보이는 것은 논문의 완전한 검증 노력 절감이 아니라 그
                        메커니즘이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Almost-surely asymptotically optimal</strong>: the cost-primary key
                        keyed on the admissible <InlineMath math="\hat h"/> returns the graph optimum,
                        which converges to the true optimum as <InlineMath math="n \to \infty"/> with
                        the RGG radius <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/> above
                        threshold — the same guarantee class as AIT* and BIT*.</li>
                    <li><strong>Effort-ordered</strong>: among candidates of equal cost the forward
                        search prefers the cheaper-to-validate one, using the second reverse search's
                        effort-to-go as a tie-break — the property that names the algorithm.</li>
                    <li><strong>Obstacle-aware in both currencies</strong>: both reverse searches run
                        over the graph's real connectivity, so <InlineMath math="\hat h"/> and{" "}
                        <InlineMath math="\hat e"/> both bend around walls and exceed their
                        straight-line baselines wherever an obstacle blocks the direct line — the two
                        estimates the demo reads out.</li>
                    <li><strong>Adaptive</strong>: invalid edges found by the forward search persist
                        across batches and prune <em>both</em> reverse graphs, so cost-to-go and
                        effort-to-go both sharpen as the map is discovered.</li>
                    <li><strong>Anytime</strong> and <strong>probabilistically complete</strong>: each
                        batch can only improve the incumbent, and the informed ellipse focuses later
                        samples where they can still help.</li>
                    <li><strong>Cost</strong>: per batch, <em>two</em> reverse Dijkstra passes and one
                        forward search, each <InlineMath math="O(m \log n)"/> over the{" "}
                        <InlineMath math="m = \Theta(n \log n)"/> radius-graph edges; this repository's
                        naive radius graph is <InlineMath math="O(n^2)"/> distance checks to build, and
                        each batch recomputes all three searches rather than reusing the previous
                        batch's work.</li>
                </ul>}
                ko={<ul>
                    <li><strong>거의 확실히 점근 최적</strong>: admissible한{" "}
                        <InlineMath math="\hat h"/>로 잡은 비용 1차 key가 그래프 최적을 반환하고, RGG
                        반경 <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/>이 임계값을 넘으면{" "}
                        <InlineMath math="n \to \infty"/>에서 참 최적으로 수렴한다. AIT*, BIT*와 같은
                        보장 등급이다.</li>
                    <li><strong>effort 정렬</strong>: 비용이 같은 후보들 사이에서 전방 탐색은 두 번째
                        역방향 탐색의 effort-to-go를 tie-break로 써서 더 싸게 검증되는 쪽을 고른다.
                        알고리즘 이름이 가리키는 성질이다.</li>
                    <li><strong>두 단위 모두에서 장애물 인지</strong>: 두 역방향 탐색이 그래프의 실제
                        연결성 위에서 돌아, <InlineMath math="\hat h"/>와 <InlineMath math="\hat e"/>가
                        모두 벽을 우회하며, 장애물이 직선을 막는 곳에서는 각자의 직선 baseline을 웃돈다.
                        demo가 읽어 주는 두 추정이다.</li>
                    <li><strong>적응형</strong>: 전방 탐색이 발견한 무효 간선이 배치를 넘어 남아{" "}
                        <em>두</em> 역방향 그래프를 모두 가지치기하므로, cost-to-go와 effort-to-go가 함께
                        날카로워진다.</li>
                    <li><strong>anytime</strong>이며 <strong>확률적 완전</strong>이다. 배치마다 현직 해를
                        개선만 할 수 있고, informed 타원이 뒤 표본을 아직 도움이 되는 곳으로 모은다.</li>
                    <li><strong>비용</strong>: 배치마다 역방향 Dijkstra <em>두 번</em>과 전방 탐색 한 번이
                        각각 <InlineMath math="m = \Theta(n \log n)"/>개의 반경 그래프 간선 위에서{" "}
                        <InlineMath math="O(m \log n)"/>이다. 이 저장소의 순진한 반경 그래프 구축은{" "}
                        <InlineMath math="O(n^2)"/> 거리 계산이고, 배치마다 이전 배치의 일을 재사용하지
                        않고 세 탐색을 다시 계산한다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    State that persists across batches is the same small set as AIT*: the growing
                    sample array, the incumbent cost <InlineMath math="c_{\text{best}}"/> that sets
                    the informed ellipse, and the set of edges already found invalid. Each batch draws
                    samples, builds the radius graph, runs <em>two</em> reverse Dijkstra passes over
                    it — one on distance, one on effort — then runs the forward search whose queue is
                    ordered by the lexicographic (cost, effort) key and which validates edges lazily
                    as it relaxes them.
                </p>}
                ko={<p>
                    배치를 넘어 남는 상태는 AIT*와 같은 작은 집합이다. 자라는 표본 배열, informed 타원을
                    정하는 현직 해 비용 <InlineMath math="c_{\text{best}}"/>, 이미 무효로 밝혀진 간선
                    집합이다. 배치마다 표본을 뽑고, 반경 그래프를 세우고, 그 위에서 역방향 Dijkstra를{" "}
                    <em>두 번</em> 돌린다. 하나는 거리로, 하나는 effort로. 그런 뒤 사전순 (비용, effort)
                    key로 큐를 정렬하고 이완하는 간선을 lazy하게 검증하는 전방 탐색을 돌린다.
                </p>}
            />
            <Pseudocode code={`V ← [start, goal];  E_inv ← ∅;  c_best ← ∞
for each batch:
    V ← V + m samples (informed ellipse once c_best < ∞)   # 1
    r ← γ · (log |V| / |V|)^(1/d);  build RGG neighbors     # 2
    ĥ ← reverse Dijkstra from goal, edge weight ‖u−v‖      # 3
    ê ← reverse Dijkstra from goal, edge weight e(u,v)     # 4
    forward best-first, key (g + ĥ, ε + ê):                # 5
        pop v with the smallest key (cost, then effort)
        for each neighbor x of v:
            if motion v → x collides:                       # 6
                add (v, x) to E_inv;  continue
            if (g[v]+d, ε[v]+e) < (g[x], ε[x]):  relax x    # 7
            if x = goal and improved: c_best ← g[goal]      # 8
return path to goal from the last forward tree`}/>
            <T
                en={<ol>
                    <li>Grow the graph. Before a solution exists the samples are uniform; after, they
                        are drawn from the informed ellipse with foci start and goal and transverse
                        diameter <InlineMath math="c_{\text{best}}"/>, the same focusing Informed
                        RRT*, BIT*, and AIT* use.</li>
                    <li>The shrinking RGG radius, computed once per batch from the current sample
                        count — the same formula as PRM*, FMT*, BIT*, and AIT*.</li>
                    <li>The first reverse search: Dijkstra from the goal weighted by distance, over
                        the RGG minus the known-invalid edges, with no collision checks. This is
                        exactly AIT*'s heuristic <InlineMath math="\hat h"/>.</li>
                    <li>The second reverse search, EIT*'s addition: the same Dijkstra but weighted by
                        effort <InlineMath math="e(u, v)"/>, giving <InlineMath math="\hat e"/>, the
                        estimated collision-check segments remaining to the goal.</li>
                    <li>The forward search, keyed on the lexicographic pair. Cost decides the order;
                        effort decides only when two candidates have exactly equal cost.</li>
                    <li>The one lazy validation. The forward search collision-checks each edge it
                        would relax; an invalid edge is added to the persistent set, so the next
                        batch's <em>two</em> reverse searches both route around it.</li>
                    <li>Lexicographic relaxation: accept the edge when it lowers the cost, or ties the
                        cost and lowers the accumulated effort. This is the one place effort enters
                        the tree.</li>
                    <li>Reaching the goal with a cheaper cost updates the incumbent, which shrinks the
                        informed ellipse for the next batch's samples.</li>
                </ol>}
                ko={<ol>
                    <li>그래프를 키운다. 해가 생기기 전엔 표본이 균일하고, 생긴 뒤엔 시작·goal을 초점으로
                        하고 횡축 지름이 <InlineMath math="c_{\text{best}}"/>인 informed 타원에서 뽑는다.
                        Informed RRT*, BIT*, AIT*가 쓰는 것과 같은 집중이다.</li>
                    <li>줄어드는 RGG 반경이다. 배치마다 현재 표본 수로 한 번 계산한다. PRM*, FMT*, BIT*,
                        AIT*와 같은 공식이다.</li>
                    <li>첫 역방향 탐색이다. goal로부터 거리로 가중한 Dijkstra를, 알려진 무효 간선을 뺀 RGG
                        위에서, 충돌 검사 없이 돌린다. AIT*의 heuristic <InlineMath math="\hat h"/>와 정확히
                        같다.</li>
                    <li>두 번째 역방향 탐색이다. EIT*가 더한 것이다. 같은 Dijkstra를 effort{" "}
                        <InlineMath math="e(u, v)"/>로 가중해, goal까지 남은 충돌 검사 segment 추정치{" "}
                        <InlineMath math="\hat e"/>를 얻는다.</li>
                    <li>전방 탐색을 사전순 쌍으로 키를 잡는다. 비용이 순서를 정하고, effort는 두 후보의
                        비용이 정확히 같을 때만 정한다.</li>
                    <li>단 한 번의 lazy 검증이다. 전방 탐색은 이완하려는 간선마다 충돌 검사한다. 무효 간선은
                        영속 집합에 더해져, 다음 배치의 <em>두</em> 역방향 탐색이 모두 그것을 우회한다.</li>
                    <li>사전순 이완이다. 간선이 비용을 낮추거나, 비용은 같으면서 누적 effort를 낮출 때
                        채택한다. effort가 트리로 들어오는 유일한 지점이다.</li>
                    <li>더 싼 비용으로 goal에 닿으면 현직 해가 갱신되고, 다음 배치 표본을 위한 informed
                        타원이 줄어든다.</li>
                </ol>}
            />
            <Proof title={t(
                "Why adding effort keeps the cost optimal",
                "effort를 더해도 비용이 최적으로 남는 이유",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> The lexicographic key returns the same optimal
                            cost as a cost-only search over the collision-free sample graph; effort
                            only chooses among paths that already tie on cost.
                        </p>
                        <p>
                            <strong>Assumptions.</strong> The distance reverse search runs over a
                            graph that still contains every collision-free edge, so its estimate never
                            overestimates the true cost-to-go — the AIT* admissibility result,
                        </p>
                        <BlockMath math="\hat h(x) \;\le\; c^*(x)."/>
                        <Terms items={[
                            ["\\hat h(x)", <>reverse cost-to-go at <InlineMath math="x"/>: the shortest distance-weighted path cost to the goal in the reverse-search graph</>],
                            ["c^*(x)", <><strong>the true cost-to-go</strong>: the shortest collision-free path cost from <InlineMath math="x"/> to the goal over the samples</>],
                        ]}/>
                        <p>
                            <strong>Argument.</strong> The forward key compares lexicographically, so
                            its primary component is exactly the admissible key of a cost-only A*.
                            Relaxation accepts an edge only when
                        </p>
                        <BlockMath math="\big(g(v)+d,\; \varepsilon(v)+e\big) \;<\; \big(g(x),\; \varepsilon(x)\big),"/>
                        <Terms items={[
                            ["g(v),\\; g(x)", <>cost-to-come of <InlineMath math="v"/> and <InlineMath math="x"/> in the forward tree</>],
                            ["d", <>edge cost <InlineMath math="\lVert v - x \rVert"/></>],
                            ["\\varepsilon(v),\\; \\varepsilon(x)", <>effort-to-come of <InlineMath math="v"/> and <InlineMath math="x"/> (accumulated collision-check segments)</>],
                            ["e", <>edge effort <InlineMath math="e(v, x)"/>, the new tie-break term</>],
                        ]}/>
                        <p>
                            and the comparison is lexicographic, so it can only <em>lower or tie</em>{" "}
                            <InlineMath math="g(x)"/> — never raise the cost to buy lower effort:
                        </p>
                        <BlockMath math="g(v) + d \;\le\; g(x) \quad\text{whenever the edge is accepted.}"/>
                        <Terms items={[
                            ["g(v) + d", <>the cost-to-come <InlineMath math="x"/> would receive through <InlineMath math="v"/></>],
                            ["g(x)", <>the incumbent cost-to-come of <InlineMath math="x"/></>],
                        ]}/>
                        <p>
                            <strong>Conclusion.</strong> Every accepted relaxation is one a cost-only
                            search would also accept (strictly cheaper) or a pure tie (equal cost,
                            lower effort). So the final <InlineMath math="g"/> values, and the returned
                            cost, match the cost-optimal search over the collision-free graph — the
                            AIT* optimum. Effort never sacrifices cost; it only selects the
                            fewest-check representative among equal-cost paths.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 사전순 key는 충돌 없는 표본 그래프 위에서 비용만 보는
                            탐색과 같은 최적 비용을 반환한다. effort는 이미 비용이 같은 경로들 사이에서만
                            고른다.
                        </p>
                        <p>
                            <strong>가정.</strong> 거리 역방향 탐색은 충돌 없는 간선을 모두 담은 그래프
                            위에서 돌아, 그 추정이 참 cost-to-go를 절대 과대평가하지 않는다. AIT*의
                            admissibility 결과다.
                        </p>
                        <BlockMath math="\hat h(x) \;\le\; c^*(x)."/>
                        <Terms items={[
                            ["\\hat h(x)", <><InlineMath math="x"/>에서의 역방향 cost-to-go. 역방향 탐색 그래프에서 goal까지 거리 가중 최단 경로 비용이다</>],
                            ["c^*(x)", <><strong>참 cost-to-go</strong>. 표본 위 <InlineMath math="x"/>부터 goal까지 충돌 없는 최단 경로 비용이다</>],
                        ]}/>
                        <p>
                            <strong>논증.</strong> 전방 key는 사전순으로 비교하므로, 그 1차 성분이 비용만
                            보는 A*의 admissible key와 정확히 같다. 이완은 다음일 때만 간선을 채택한다.
                        </p>
                        <BlockMath math="\big(g(v)+d,\; \varepsilon(v)+e\big) \;<\; \big(g(x),\; \varepsilon(x)\big),"/>
                        <Terms items={[
                            ["g(v),\\; g(x)", <>전방 트리에서 <InlineMath math="v"/>와 <InlineMath math="x"/>의 cost-to-come</>],
                            ["d", <>간선 비용 <InlineMath math="\lVert v - x \rVert"/></>],
                            ["\\varepsilon(v),\\; \\varepsilon(x)", <><InlineMath math="v"/>와 <InlineMath math="x"/>의 effort-to-come(누적 충돌 검사 segment 수)</>],
                            ["e", <>간선 effort <InlineMath math="e(v, x)"/>. 새 tie-break 항이다</>],
                        ]}/>
                        <p>
                            비교가 사전순이므로 <InlineMath math="g(x)"/>를 <em>낮추거나 같게</em> 둘 수만
                            있다. effort를 낮추려고 비용을 올리는 일은 절대 없다.
                        </p>
                        <BlockMath math="g(v) + d \;\le\; g(x) \quad\text{(간선을 채택할 때마다)}."/>
                        <Terms items={[
                            ["g(v) + d", <><InlineMath math="v"/>를 거쳐 <InlineMath math="x"/>가 받을 cost-to-come</>],
                            ["g(x)", <><InlineMath math="x"/>의 현직 cost-to-come</>],
                        ]}/>
                        <p>
                            <strong>결론.</strong> 채택된 이완은 모두 비용만 보는 탐색도 채택할 것(엄격히
                            더 쌈)이거나 순수한 tie(비용 같음, effort 낮음)다. 그래서 최종{" "}
                            <InlineMath math="g"/> 값과 반환 비용이 충돌 없는 그래프 위 비용 최적 탐색,
                            곧 AIT* 최적과 일치한다. effort는 비용을 희생하지 않는다. 비용이 같은 경로들
                            사이에서 검사가 가장 적은 대표를 고를 뿐이다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs EIT* through a thick wall with a single gate at the top, with the
                    start and goal both low so the path must detour up and over. Watch the two reverse
                    heuristics read out at the start: cost-to-go <InlineMath math="\hat h"/> in metres
                    — the same estimate AIT* builds — and effort-to-go <InlineMath math="\hat e"/> in
                    collision-check segments, the second reverse search EIT* adds. Both route over the
                    gate, so both climb above the obstacle-blind straight-line count. Add batches and
                    the estimates sharpen as invalid edges prune both reverse graphs. The replay below
                    is the repository demo on the benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 위쪽에 문 하나만 난 두꺼운 벽에 EIT*를 돌린다. 시작점과 goal이 모두 아래에
                    있어 경로는 위로 크게 우회해야 한다. 시작점에서 읽히는 두 역방향 heuristic을 보라.
                    meters로 잰 cost-to-go <InlineMath math="\hat h"/>는 AIT*가 세우는 것과 같은 추정이고,
                    충돌 검사 segment로 잰 effort-to-go <InlineMath math="\hat e"/>는 EIT*가 더한 두 번째
                    역방향 탐색이다. 둘 다 선반 오른쪽 끝을 돌아, 장애물을 못 보는 직선 검사 수를 넘어 오른다.
                    배치를 늘리면 무효 간선이 두 역방향 그래프를 가지치기하면서 추정이 날카로워진다. 아래
                    replay는 벤치마크 맵 위의 저장소 demo다.
                </p>}
            />
            <EitStarSandbox/>
            <TraceReplay algo="eit_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's EIT* demo: each batch draws informed samples, two reverse searches build the cost-to-go and effort-to-go heuristics, and the forward search extends the tree while validating edges",
                "저장소 EIT* demo의 실제 trace. 배치마다 informed 표본을 뽑고, 두 역방향 탐색이 cost-to-go와 effort-to-go heuristic을 세우며, 전방 탐색이 간선을 검증하면서 트리를 넓힌다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The informed sampler, the shrinking radius, and the radius graph are shared with
                    the other batch planners; EIT* itself is the two reverse Dijkstra passes plus the
                    forward search with its lexicographic (cost, effort) key, lazy validation, and the
                    persistent invalid-edge set. Embedded below in full.
                </p>}
                ko={<p>
                    informed 표본기, 줄어드는 반경, 반경 그래프는 다른 batch planner들과 공유한다. EIT*
                    자체는 역방향 Dijkstra 두 번과, 사전순 (비용, effort) key·lazy 검증·영속 무효 간선
                    집합을 갖는 전방 탐색이다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/eit_star.py",
                            code: eitStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/eit_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/eit_star.cpp",
                            code: eitStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/eit_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete EIT* implementation, embedded from the repository sources",
                    "EIT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    M. P. Strub, J. D. Gammell,{" "}
                    <a href="https://doi.org/10.1177/02783649211069572" target="_blank"
                       rel="noopener noreferrer">
                        <em>AIT* and EIT*: Asymmetric bidirectional sampling-based path planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2022.
                </li>
                <li>
                    M. P. Strub, J. D. Gammell,{" "}
                    <a href="https://doi.org/10.1109/ICRA40945.2020.9197338" target="_blank"
                       rel="noopener noreferrer">
                        <em>Adaptively Informed Trees (AIT*): Fast Asymptotically Optimal Path Planning
                            through Adaptive Heuristics</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2020.
                </li>
                <li>
                    J. D. Gammell, T. D. Barfoot, S. S. Srinivasa,{" "}
                    <a href="https://doi.org/10.1177/0278364919890396" target="_blank"
                       rel="noopener noreferrer">
                        <em>Batch Informed Trees (BIT*): Informed asymptotically optimal anytime
                            search</em>
                    </a>,
                    The International Journal of Robotics Research, 2020.
                </li>
                <li>
                    J. D. Gammell, S. S. Srinivasa, T. D. Barfoot,{" "}
                    <a href="https://doi.org/10.1109/IROS.2014.6942976" target="_blank"
                       rel="noopener noreferrer">
                        <em>Informed RRT*: Optimal Sampling-based Path Planning Focused via Direct
                            Sampling of an Admissible Ellipsoidal Heuristic</em>
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

export default EitStar

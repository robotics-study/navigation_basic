import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import AitStarSandbox from "../../../../components/panels/global/ait_star/AitStarSandbox";
import aitStarPy from "../../../../../../python/navigation/global_planning/sampling/ait_star.py?raw";
import aitStarCpp from "../../../../../../cpp/src/global_planning/sampling/ait_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const AitStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    BIT* orders its edge queue by <InlineMath math="g + \hat h"/>, where{" "}
                    <InlineMath math="\hat h(x) = \lVert x - x_{\text{goal}} \rVert"/> is the
                    straight-line distance to the goal. That estimate is blind to obstacles: when
                    a wall sits between a state and the goal, it points straight through the wall
                    and pulls the search into it. AIT* (Strub &amp; Gammell, 2020) keeps
                    everything else about BIT* and swaps that heuristic for a cost-to-go computed
                    by a <em>reverse search over the same sample graph</em> the forward search is
                    exploring — an estimate that bends around obstacles. And because every edge
                    the forward search finds blocked is fed back into that reverse search, the
                    heuristic sharpens as the map is discovered.
                </p>}
                ko={<p>
                    BIT*는 간선 큐를 <InlineMath math="g + \hat h"/>로 정렬한다. 여기서{" "}
                    <InlineMath math="\hat h(x) = \lVert x - x_{\text{goal}} \rVert"/>는 goal까지의
                    직선거리다. 이 추정은 장애물을 못 본다. 상태와 goal 사이에 벽이 있으면 벽을
                    정면으로 관통하는 방향을 가리키며 탐색을 그 안으로 끌어당긴다. AIT*(Strub &amp;
                    Gammell, 2020)는 BIT*의 나머지를 그대로 두고 그 heuristic만 바꾼다. 전방 탐색이
                    돌고 있는 바로 그 표본 그래프 위에서 <em>역방향 탐색</em>으로 cost-to-go를
                    계산해, 장애물을 우회하는 추정을 만든다. 게다가 전방 탐색이 막혔다고 밝힌 간선은
                    모두 그 역방향 탐색으로 되먹여져, heuristic이 맵을 발견해 가며 날카로워진다.
                </p>}
            />

            <h2>{t("From a Straight Line to a Reverse Search", "직선에서 역방향 탐색으로")}</h2>
            <T
                en={<>
                    <p>
                        A heuristic guides A* well only when it is close to the true cost-to-go.
                        The straight-line estimate is admissible, so it never breaks optimality,
                        but near obstacles it is loose: it reports the through-the-wall distance
                        while the real path detours around. AIT* computes a tighter estimate by
                        running a <strong>lazy reverse search</strong> — Dijkstra from the goal
                        over the current random geometric graph (RGG), following the graph's real
                        connectivity instead of a straight line. The result{" "}
                        <InlineMath math="\hat h(x)"/> is a cost-to-go that already knows to go
                        around the wall.
                    </p>
                    <p>
                        The reverse search is <em>optimistic</em>: it does not collision-check its
                        edges. Validating them is the forward search's job. So the two searches
                        cooperate. The reverse search hands the forward search a problem-fit
                        heuristic. The forward search follows it, and as it relaxes each edge it
                        collision-checks that edge. Every edge found <strong>invalid</strong> is
                        recorded once and permanently excluded from the graph the reverse search
                        runs over — so on the next batch the reverse heuristic routes around the
                        newly discovered obstacle. This is the <em>adaptive</em> loop: when the
                        forward search hits a wall the heuristic itself is repaired, and the two
                        searches converge on the real geometry (Strub &amp; Gammell, 2020).
                    </p>
                    <p>
                        This repository keeps the defining behavior but simplifies the mechanics.
                        The full algorithm interleaves a single incremental reverse search
                        (LPA*-based) that repairs its tree event by event and reuses forward
                        costs across events. Here each batch instead recomputes the reverse
                        heuristic and the forward tree from scratch over all accumulated samples.
                        That drops the incrementality — an optimization of <em>how</em> the
                        searches are updated — while preserving what makes AIT* what it is: a
                        forward search steered by an obstacle-aware reverse heuristic that adapts
                        to discovered invalid edges.
                    </p>
                </>}
                ko={<>
                    <p>
                        heuristic은 참 cost-to-go에 가까울 때만 A*를 잘 이끈다. 직선거리 추정은
                        admissible이라 최적성을 깨지는 않지만, 장애물 근처에서는 헐겁다. 실제 경로는
                        돌아가는데 벽을 관통하는 거리를 보고한다. AIT*는 <strong>lazy 역방향
                        탐색</strong>으로 더 조인 추정을 만든다. 현재 random geometric graph(RGG)
                        위에서 goal로부터 Dijkstra를 돌려, 직선이 아니라 그래프의 실제 연결성을
                        따른다. 그 결과 <InlineMath math="\hat h(x)"/>는 벽을 돌아갈 줄 이미 아는
                        cost-to-go다.
                    </p>
                    <p>
                        역방향 탐색은 <em>낙관적</em>이다. 자기 간선을 충돌 검사하지 않는다. 검증은
                        전방 탐색의 몫이다. 그래서 두 탐색이 협력한다. 역방향 탐색이 전방 탐색에게
                        문제 맞춤 heuristic을 건네준다. 전방 탐색은 그것을 따라가며, 간선을 이완할
                        때마다 그 간선을 충돌 검사한다. <strong>무효</strong>로 밝혀진 간선은 한 번
                        기록되어 역방향 탐색이 도는 그래프에서 영구히 빠진다. 그래서 다음 배치에서
                        역방향 heuristic은 새로 발견된 장애물을 우회한다. 이것이 <em>적응형</em>
                        루프다. 전방 탐색이 벽에 부딪히면 heuristic 자체가 수리되고, 두 탐색이 실제
                        지형으로 수렴한다 (Strub &amp; Gammell, 2020).
                    </p>
                    <p>
                        이 저장소는 정의적 동작은 지키되 기계 부분을 단순화한다. 원 알고리즘은 트리를
                        이벤트 단위로 수리하고 전방 비용을 이벤트 사이에서 재사용하는 단일 증분 역방향
                        탐색(LPA* 기반)을 엮는다. 여기서는 배치마다 역방향 heuristic과 전방 트리를
                        누적된 전체 표본 위에서 처음부터 다시 계산한다. 이는 증분성, 곧 탐색을
                        <em>어떻게</em> 갱신하는지에 대한 최적화를 버리되, AIT*를 AIT*이게 하는 것은
                        지킨다. 발견된 무효 간선에 적응하는 장애물 인지 역방향 heuristic이 이끄는 전방
                        탐색이다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Almost-surely asymptotically optimal</strong>: with the RGG
                        radius <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/> above the
                        threshold, the returned cost converges to the optimum as{" "}
                        <InlineMath math="n \to \infty"/> — the same guarantee class as BIT*.</li>
                    <li><strong>Probabilistically complete</strong> and <strong>anytime</strong>:
                        each batch can only improve the incumbent, and the informed ellipse
                        focuses later samples where they can still help.</li>
                    <li><strong>Obstacle-aware heuristic</strong>: the reverse-graph estimate
                        bends around walls, so it is far tighter than the straight-line distance
                        wherever an obstacle blocks the direct line — the property the demo
                        measures.</li>
                    <li><strong>Admissible</strong>: the reverse search runs over a graph that
                        still contains every collision-free edge, so it never overestimates and
                        the forward search stays optimal on the validated graph.</li>
                    <li><strong>Adaptive</strong>: invalid edges persist across batches and prune
                        the reverse graph, so the heuristic rises toward the true cost-to-go as
                        the map is discovered.</li>
                    <li><strong>Cost</strong>: per batch, a reverse Dijkstra and a forward A*,
                        each <InlineMath math="O(m \log n)"/> over the{" "}
                        <InlineMath math="m = \Theta(n \log n)"/> radius-graph edges; this
                        repository's naive radius graph is <InlineMath math="O(n^2)"/> distance
                        checks to build, and each batch recomputes both searches rather than
                        reusing the previous batch's work.</li>
                </ul>}
                ko={<ul>
                    <li><strong>거의 확실히 점근 최적</strong>: RGG 반경{" "}
                        <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/>이 임계값을 넘으면 반환
                        비용이 <InlineMath math="n \to \infty"/>에서 최적으로 수렴한다. BIT*와 같은
                        보장 등급이다.</li>
                    <li><strong>확률적 완전</strong>이며 <strong>anytime</strong>이다. 배치마다
                        현직 해를 개선만 할 수 있고, informed 타원이 뒤 표본을 아직 도움이 되는
                        곳으로 모은다.</li>
                    <li><strong>장애물 인지 heuristic</strong>: 역방향 그래프 추정은 벽을 돌아가므로,
                        장애물이 직선을 막는 곳에서는 직선거리보다 훨씬 조인다. demo가 재는 성질이다.</li>
                    <li><strong>Admissible</strong>: 역방향 탐색은 충돌 없는 간선을 모두 포함한
                        그래프 위에서 돌아, 절대 과대평가하지 않는다. 전방 탐색이 검증된 그래프 위에서
                        최적을 유지한다.</li>
                    <li><strong>적응형</strong>: 무효 간선이 배치를 넘어 남아 역방향 그래프를
                        가지치기하므로, heuristic이 맵을 발견해 가며 참 cost-to-go로 올라간다.</li>
                    <li><strong>비용</strong>: 배치마다 역방향 Dijkstra와 전방 A*가 각각{" "}
                        <InlineMath math="m = \Theta(n \log n)"/>개의 반경 그래프 간선 위에서{" "}
                        <InlineMath math="O(m \log n)"/>이다. 이 저장소의 순진한 반경 그래프 구축은{" "}
                        <InlineMath math="O(n^2)"/> 거리 계산이고, 배치마다 이전 배치의 일을
                        재사용하지 않고 두 탐색을 다시 계산한다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    State that persists across batches is small: the growing sample array, the
                    incumbent cost <InlineMath math="c_{\text{best}}"/> (which sets the informed
                    ellipse), and the set of edges already found invalid. Each batch draws
                    samples, builds the radius graph, runs the reverse search to get the
                    heuristic, then runs the forward A* that both extends the tree and validates
                    edges. The forward search's discoveries feed the next batch's reverse search.
                </p>}
                ko={<p>
                    배치를 넘어 남는 상태는 작다. 자라는 표본 배열, 현직 해 비용{" "}
                    <InlineMath math="c_{\text{best}}"/>(informed 타원을 정한다), 이미 무효로 밝혀진
                    간선 집합이다. 배치마다 표본을 뽑고, 반경 그래프를 세우고, 역방향 탐색으로
                    heuristic을 얻은 뒤, 트리를 넓히며 간선을 검증하는 전방 A*를 돌린다. 전방 탐색의
                    발견이 다음 배치의 역방향 탐색으로 되먹여진다.
                </p>}
            />
            <Pseudocode code={`V ← [start, goal];  E_inv ← ∅;  c_best ← ∞
for each batch:
    V ← V + m samples (informed ellipse once c_best < ∞)   # 1
    r ← γ · (log |V| / |V|)^(1/d);  build RGG neighbors     # 2
    ĥ ← reverse Dijkstra from goal over (RGG minus E_inv)  # 3
    forward A* from start, key g[v] + ĥ[v]:                # 4
        pop cheapest v
        for each neighbor x of v:
            if (v, x) ∈ E_inv: continue                     # 5
            if motion v → x collides:                       # 6
                add (v, x) to E_inv;  continue
            relax g[x] through v
            if x = goal and improved: c_best ← g[goal]      # 7
return path to goal from the last forward tree`}/>
            <T
                en={<ol>
                    <li>Grow the graph. Before a solution exists the samples are uniform. After,
                        they are drawn from the informed ellipse with foci start and goal and
                        transverse diameter <InlineMath math="c_{\text{best}}"/>, the same
                        focusing Informed RRT* and BIT* use.</li>
                    <li>The shrinking RGG radius, computed once per batch from the current sample
                        count — the same formula as PRM*, FMT*, and BIT*.</li>
                    <li>The reverse search is the heuristic. Dijkstra from the goal, over the RGG
                        with every known-invalid edge removed, and with <em>no</em> collision
                        checks. <InlineMath math="\hat h(x)"/> is the graph cost-to-go, already
                        routed around whatever obstacles have been discovered so far.</li>
                    <li>The forward A*, keyed on <InlineMath math="g + \hat h"/>. Because{" "}
                        <InlineMath math="\hat h"/> is obstacle-aware, the ordering pulls the
                        search along the real detour rather than into the wall.</li>
                    <li>Edges already known invalid are skipped outright. This accumulated set is
                        the feedback that makes the heuristic adaptive.</li>
                    <li>The one lazy validation. The forward search collision-checks each edge it
                        would relax. An invalid edge is added to the persistent set, so the next
                        batch's reverse search routes around it — the repair step.</li>
                    <li>Reaching the goal with a cheaper cost updates the incumbent, which shrinks
                        the informed ellipse for the next batch's samples.</li>
                </ol>}
                ko={<ol>
                    <li>그래프를 키운다. 해가 생기기 전엔 표본이 균일하다. 생긴 뒤엔 시작·goal을
                        초점으로 하고 횡축 지름이 <InlineMath math="c_{\text{best}}"/>인 informed
                        타원에서 뽑는다. Informed RRT*와 BIT*가 쓰는 것과 같은 집중이다.</li>
                    <li>줄어드는 RGG 반경이다. 배치마다 현재 표본 수로 한 번 계산한다. PRM*, FMT*,
                        BIT*와 같은 공식이다.</li>
                    <li>역방향 탐색이 곧 heuristic이다. goal로부터의 Dijkstra를, 알려진 무효 간선을
                        모두 뺀 RGG 위에서, 충돌 검사 <em>없이</em> 돌린다.{" "}
                        <InlineMath math="\hat h(x)"/>는 지금까지 발견된 장애물을 이미 우회한 그래프
                        cost-to-go다.</li>
                    <li>전방 A*를 <InlineMath math="g + \hat h"/>로 키를 잡는다.{" "}
                        <InlineMath math="\hat h"/>가 장애물을 인지하므로, 정렬이 탐색을 벽 안이
                        아니라 실제 우회로를 따라 끌어당긴다.</li>
                    <li>이미 무효로 아는 간선은 곧장 건너뛴다. 이 누적 집합이 heuristic을 적응형으로
                        만드는 피드백이다.</li>
                    <li>단 한 번의 lazy 검증이다. 전방 탐색은 이완하려는 간선마다 충돌 검사한다. 무효
                        간선은 영속 집합에 더해져, 다음 배치의 역방향 탐색이 그것을 우회한다. 수리
                        단계다.</li>
                    <li>더 싼 비용으로 goal에 닿으면 현직 해가 갱신되고, 다음 배치 표본을 위한
                        informed 타원이 줄어든다.</li>
                </ol>}
            />
            <Proof title={t(
                "Why the reverse heuristic keeps the forward search optimal",
                "역방향 heuristic이 전방 탐색을 최적으로 유지하는 이유",
            )}>
                <T
                    en={<>
                        <p>
                            <strong>Claim.</strong> The reverse heuristic never overestimates the
                            true cost-to-go over the collision-free sample graph, so A* keyed on{" "}
                            <InlineMath math="g + \hat h"/> returns that graph's optimal cost.
                        </p>
                        <p>
                            <strong>Assumptions.</strong> Let <InlineMath math="G_{\text{free}}"/>{" "}
                            be the sample graph keeping only collision-free RGG edges, and{" "}
                            <InlineMath math="G_{\text{rev}}"/> the graph the reverse search runs
                            over: the RGG minus the edges already found invalid. A collision-free
                            edge is never found invalid, so it is never removed. Hence
                        </p>
                        <BlockMath math="\Pi_{\text{free}}(x) \subseteq \Pi_{\text{rev}}(x),"/>
                        <Terms items={[
                            ["\\Pi_{\\text{free}}(x)", <>set of paths from <InlineMath math="x"/> to the goal in <InlineMath math="G_{\text{free}}"/> (collision-free edges only)</>],
                            ["\\Pi_{\\text{rev}}(x)", <>set of paths from <InlineMath math="x"/> to the goal in <InlineMath math="G_{\text{rev}}"/>, <strong>the reverse-search graph</strong>, which still contains every not-yet-invalidated edge</>],
                        ]}/>
                        <p>
                            because removing only invalid edges cannot delete a collision-free
                            path. Minimizing a path cost over a superset can only lower the
                            minimum, so
                        </p>
                        <BlockMath math="\hat h(x) \;=\; \min_{\pi \in \Pi_{\text{rev}}(x)} \operatorname{cost}(\pi) \;\le\; \min_{\pi \in \Pi_{\text{free}}(x)} \operatorname{cost}(\pi) \;=\; c^*(x)."/>
                        <Terms items={[
                            ["\\hat h(x)", <>reverse-search estimate at <InlineMath math="x"/>: the shortest path cost in <InlineMath math="G_{\text{rev}}"/></>],
                            ["\\operatorname{cost}(\\pi)", <>summed Euclidean length of the edges of path <InlineMath math="\pi"/></>],
                            ["c^*(x)", <><strong>the true cost-to-go</strong>: the shortest collision-free path cost from <InlineMath math="x"/> to the goal over the samples</>],
                        ]}/>
                        <p>
                            <strong>Conclusion.</strong> <InlineMath math="\hat h \le c^*"/> is
                            exactly admissibility, so the forward A* returns the optimal cost over{" "}
                            <InlineMath math="G_{\text{free}}"/>. As batches discover more invalid
                            edges, <InlineMath math="G_{\text{rev}}"/> shrinks toward{" "}
                            <InlineMath math="G_{\text{free}}"/> and <InlineMath math="\hat h"/>{" "}
                            rises toward <InlineMath math="c^*"/>: tighter each batch, yet always
                            admissible. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>주장.</strong> 역방향 heuristic은 충돌 없는 표본 그래프 위의 참
                            cost-to-go를 절대 과대평가하지 않는다. 그래서{" "}
                            <InlineMath math="g + \hat h"/>로 키를 잡은 A*가 그 그래프의 최적 비용을
                            반환한다.
                        </p>
                        <p>
                            <strong>가정.</strong> <InlineMath math="G_{\text{free}}"/>를 충돌 없는
                            RGG 간선만 남긴 표본 그래프, <InlineMath math="G_{\text{rev}}"/>를 역방향
                            탐색이 도는 그래프, 곧 RGG에서 이미 무효로 밝혀진 간선을 뺀 것이라 하자.
                            충돌 없는 간선은 무효로 밝혀지지 않으므로 절대 제거되지 않는다. 따라서
                        </p>
                        <BlockMath math="\Pi_{\text{free}}(x) \subseteq \Pi_{\text{rev}}(x),"/>
                        <Terms items={[
                            ["\\Pi_{\\text{free}}(x)", <><InlineMath math="G_{\text{free}}"/>에서 <InlineMath math="x"/>부터 goal까지의 경로 집합 (충돌 없는 간선만)</>],
                            ["\\Pi_{\\text{rev}}(x)", <><InlineMath math="G_{\text{rev}}"/>에서 <InlineMath math="x"/>부터 goal까지의 경로 집합. <strong>역방향 탐색 그래프</strong>이며 아직 무효가 아닌 간선을 모두 담는다</>],
                        ]}/>
                        <p>
                            무효 간선만 지우는 것으로는 충돌 없는 경로를 없앨 수 없기 때문이다. 경로
                            비용을 상위집합 위에서 최소화하면 최소값은 낮아질 수만 있으므로
                        </p>
                        <BlockMath math="\hat h(x) \;=\; \min_{\pi \in \Pi_{\text{rev}}(x)} \operatorname{cost}(\pi) \;\le\; \min_{\pi \in \Pi_{\text{free}}(x)} \operatorname{cost}(\pi) \;=\; c^*(x)."/>
                        <Terms items={[
                            ["\\hat h(x)", <><InlineMath math="x"/>에서의 역방향 탐색 추정. <InlineMath math="G_{\text{rev}}"/> 위 최단 경로 비용이다</>],
                            ["\\operatorname{cost}(\\pi)", <>경로 <InlineMath math="\pi"/> 간선들의 유클리드 길이 합</>],
                            ["c^*(x)", <><strong>참 cost-to-go</strong>. 표본 위 <InlineMath math="x"/>부터 goal까지 충돌 없는 최단 경로 비용이다</>],
                        ]}/>
                        <p>
                            <strong>결론.</strong> <InlineMath math="\hat h \le c^*"/>가 곧
                            admissibility다. 그래서 전방 A*는 <InlineMath math="G_{\text{free}}"/>{" "}
                            위 최적 비용을 반환한다. 배치가 무효 간선을 더 발견할수록{" "}
                            <InlineMath math="G_{\text{rev}}"/>는 <InlineMath math="G_{\text{free}}"/>{" "}
                            쪽으로 줄고 <InlineMath math="\hat h"/>는 <InlineMath math="c^*"/>로
                            오른다. 배치마다 더 조이면서도 늘 admissible이다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs AIT* through a thick wall with a single gate at the top.
                    Watch the two cost-to-go estimates read out at the start: the straight-line
                    heuristic points through the wall and underestimates, while AIT*'s
                    reverse-graph heuristic routes over the gate and tracks the true optimal cost.
                    Add batches and the reverse estimate tightens onto the optimum. Erase the wall
                    and the two estimates meet — with no obstacle, the reverse search and the
                    straight line agree. The replay below is the repository demo on the benchmark
                    maps.
                </p>}
                ko={<p>
                    sandbox는 위쪽에 문 하나만 난 두꺼운 벽에 AIT*를 돌린다. 시작점에서 읽히는 두
                    cost-to-go 추정을 보라. 직선거리 heuristic은 벽을 관통해 비용을 낮게 잡지만,
                    AIT*의 역방향 그래프 heuristic은 문 위로 돌아 참 최적 비용을 따라간다. 배치를
                    늘리면 역방향 추정이 최적값에 조여든다. 벽을 지우면 두 추정이 만난다. 장애물이
                    없으면 역방향 탐색과 직선이 일치한다. 아래 replay는 벤치마크 맵 위의 저장소
                    demo다.
                </p>}
            />
            <AitStarSandbox/>
            <TraceReplay algo="ait_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's AIT* demo: each batch draws informed samples, a reverse search builds the obstacle-aware heuristic, and the forward search extends the tree while validating edges",
                "저장소 AIT* demo의 실제 trace. 배치마다 informed 표본을 뽑고, 역방향 탐색이 장애물 인지 heuristic을 세우며, 전방 탐색이 간선을 검증하면서 트리를 넓힌다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The informed sampler, the shrinking radius, and the radius graph are shared
                    with the other batch planners; AIT* itself is the reverse search plus the
                    forward A* with its lazy validation and the persistent invalid-edge set.
                    Embedded below in full.
                </p>}
                ko={<p>
                    informed 표본기, 줄어드는 반경, 반경 그래프는 다른 batch planner들과 공유한다.
                    AIT* 자체는 역방향 탐색과, lazy 검증 및 영속 무효 간선 집합을 갖는 전방 A*다.
                    전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/ait_star.py",
                            code: aitStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/ait_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/ait_star.cpp",
                            code: aitStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/ait_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete AIT* implementation, embedded from the repository sources",
                    "AIT* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    M. P. Strub, J. D. Gammell,{" "}
                    <a href="https://doi.org/10.1109/ICRA40945.2020.9197338" target="_blank"
                       rel="noopener noreferrer">
                        <em>Adaptively Informed Trees (AIT*): Fast Asymptotically Optimal Path
                            Planning through Adaptive Heuristics</em>
                    </a>,
                    IEEE International Conference on Robotics and Automation (ICRA), 2020.
                </li>
                <li>
                    M. P. Strub, J. D. Gammell,{" "}
                    <a href="https://doi.org/10.1177/02783649211069572" target="_blank"
                       rel="noopener noreferrer">
                        <em>AIT* and EIT*: Asymmetric bidirectional sampling-based path planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2022.
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

export default AitStar

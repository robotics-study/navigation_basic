import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import AStarSandbox from "../../../../components/panels/global/astar/AStarSandbox";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import CodeTabs from "../../../../components/CodeTabs";
import Pseudocode from "../../../../components/Pseudocode";
import astarPy from "../../../../../../python/navigation/global_planning/search/astar.py?raw";
import bestFirstPy from "../../../../../../python/navigation/global_planning/search/_bestfirst.py?raw";
import astarCpp from "../../../../../../cpp/src/global_planning/search/astar.cpp?raw";
import discreteSearchHpp from "../../../../../../cpp/include/navigation/global_planning/search/discrete_search.hpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 증명 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 증명은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Astar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    A* is the workhorse of pathfinding. Given a graph, a start, and a goal, it returns a
                    least-cost path — and it does so while looking at as little of the graph as it can
                    justify. Published in 1968 by Hart, Nilsson, and Raphael for the Shakey robot
                    project, it remains the default answer to "find me a shortest path" half a century
                    later, from game maps to robot occupancy grids.
                </p>}
                ko={<p>
                    A*는 pathfinding의 표준 도구다. 그래프와 시작·목표를 주면 최소 비용 경로를
                    찾아 주는데, 그 과정에서 탐색하는 노드 수를 최소한으로 억제한다. 1968년
                    Hart, Nilsson, Raphael이 Shakey 로봇 프로젝트에서 발표했고, 반세기가 지난
                    지금도 게임 맵부터 로봇 occupancy grid까지 "최단 경로를 찾아 달라"는 문제의
                    기본 답으로 남아 있다.
                </p>}
            />

            <h2>{t("From Dijkstra to A*", "Dijkstra에서 A*로")}</h2>
            <T
                en={<>
                    <p>
                        Start with Dijkstra's algorithm. It grows a frontier outward from the start,
                        always expanding the node whose cost-from-start <InlineMath math="g(n)"/> is
                        smallest. This is provably optimal — but blind. The frontier spreads as a
                        circular ripple in every direction, spending most of its effort on nodes that
                        take it <em>away</em> from the goal, because nothing in the ordering says where
                        the goal is.
                    </p>
                    <p>
                        A* fixes exactly this. Suppose that for every node we also had an
                        estimate <InlineMath math="h(n)"/> of the cost still remaining to the goal.
                        Then the interesting quantity is not "how far have I come" but "how expensive
                        would the whole path through this node be":
                    </p>
                    <BlockMath math="f(n) \;=\; \underbrace{g(n)}_{\text{cost from start}} \;+\; \underbrace{h(n)}_{\text{estimated cost to goal}}"/>
                    <p>
                        A* is Dijkstra with the frontier ordered by <InlineMath math="f"/> instead
                        of <InlineMath math="g"/>. The two extremes are illuminating. With{" "}
                        <InlineMath math="h \equiv 0"/> the ordering degenerates to Dijkstra's. With a
                        perfect estimate <InlineMath math="h = h^*"/> (the true remaining cost), every
                        node on an optimal path has the same <InlineMath math="f"/>, and the search
                        walks straight to the goal expanding almost nothing else. Real heuristics live
                        between these extremes: the better <InlineMath math="h"/> approximates{" "}
                        <InlineMath math="h^*"/> from below, the narrower the searched region.
                    </p>
                </>}
                ko={<>
                    <p>
                        Dijkstra 알고리즘에서 출발하자. Dijkstra는 시작점에서 frontier를 바깥으로
                        키워 가며, 항상 시작점부터의 비용 <InlineMath math="g(n)"/>이 가장 작은
                        노드를 확장한다. 최적성은 증명되지만, 맹목 탐색이다. frontier가 모든
                        방향으로 동심원처럼 퍼지면서, 목표에서 <em>멀어지는</em> 노드에도 똑같이
                        공을 들인다. 정렬 기준 어디에도 목표가 어디 있는지에 대한 정보가 없기
                        때문이다.
                    </p>
                    <p>
                        A*가 고치는 지점이 정확히 여기다. 각 노드에 대해 목표까지 남은 비용의
                        추정치 <InlineMath math="h(n)"/>을 하나 더 갖고 있다고 하자. 그러면 중요한
                        양은 "여기까지 얼마나 왔나"가 아니라 "이 노드를 지나는 전체 경로가 얼마나
                        비쌀 것인가"다:
                    </p>
                    <BlockMath math="f(n) \;=\; \underbrace{g(n)}_{\text{시작점부터의 비용}} \;+\; \underbrace{h(n)}_{\text{목표까지의 추정 비용}}"/>
                    <p>
                        A*는 frontier를 <InlineMath math="g"/> 대신 <InlineMath math="f"/>로
                        정렬한 Dijkstra 다. 양 극단을 보면 감이 온다. <InlineMath math="h \equiv 0"/>
                        이면 정렬이 Dijkstra로 퇴화한다. 완벽한 추정치 <InlineMath math="h = h^*"/>
                        (실제 남은 비용)라면 최적 경로 위의 모든 노드가 같은 <InlineMath math="f"/>
                        값을 가져, 탐색은 다른 곳을 거의 확장하지 않고 목표까지 곧장 걸어간다. 실전의
                        heuristic은 이 사이 어딘가에 있다: <InlineMath math="h"/>가 아래에서{" "}
                        <InlineMath math="h^*"/>에 가까울수록 탐색 영역이 좁아진다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<>
                    <ul>
                        <li><strong>Complete</strong> on finite graphs: if a path exists, A* finds one.</li>
                        <li><strong>Optimal</strong> with an admissible <InlineMath math="h"/>; cost at
                            most <InlineMath math="w \cdot C^*"/> with weight <InlineMath math="w > 1"/>.</li>
                        <li><strong>Optimally efficient</strong>: with a consistent heuristic, no other
                            optimal algorithm using the same heuristic information expands fewer nodes
                            up to tie-breaking (Dechter &amp; Pearl, 1985).</li>
                        <li><strong>Cost</strong>: worst-case time and memory are{" "}
                            <InlineMath math="O(b^d)"/> in branching factor <InlineMath math="b"/> and
                            solution depth <InlineMath math="d"/> — the heuristic shrinks the constant
                            enormously but not the asymptotics, and memory is usually what runs out
                            first. That limit is what motivates the iterative and incremental variants
                            (ARA*, D* Lite) covered in later pages.</li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li><strong>완전성</strong>: 유한 그래프에서 경로가 존재하면 반드시 찾는다.</li>
                        <li><strong>최적성</strong>: admissible <InlineMath math="h"/>에서 최적;
                            가중치 <InlineMath math="w > 1"/>에서는 비용이 최대{" "}
                            <InlineMath math="w \cdot C^*"/>.</li>
                        <li><strong>최적 효율성</strong>: consistent heuristic에서, 같은 heuristic
                            정보를 쓰는 어떤 최적 알고리즘도 tie-breaking 차이를 빼면 A*보다 적게
                            확장할 수 없다 (Dechter &amp; Pearl, 1985).</li>
                        <li><strong>비용</strong>: 최악의 경우 시간·메모리 모두 분기 계수{" "}
                            <InlineMath math="b"/>와 해 깊이 <InlineMath math="d"/>에 대해{" "}
                            <InlineMath math="O(b^d)"/> 다. heuristic은 상수를 크게 줄이지만 점근
                            차수는 못 줄이고, 보통 메모리가 먼저 바닥난다. 이 한계가 뒤 페이지에서
                            다루는 반복·증분 변형(ARA*, D* Lite)의 동기다.</li>
                    </ul>
                </>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    A* keeps two bookkeeping structures: an <strong>OPEN</strong> set, a priority
                    queue of discovered-but-unexpanded nodes keyed by <InlineMath math="f"/>, and
                    a <strong>CLOSED</strong> set of already-expanded nodes. Each node remembers
                    its best-known <InlineMath math="g"/> and its parent, from which the final path
                    is reconstructed.
                </p>}
                ko={<p>
                    A*는 두 가지 자료구조를 유지한다. 발견됐지만 아직 확장되지 않은 노드를{" "}
                    <InlineMath math="f"/> 순으로 담는 우선순위 큐 <strong>OPEN</strong>과,
                    이미 확장된 노드의 집합 <strong>CLOSED</strong> 다. 각 노드는 지금까지 알려진
                    최선의 <InlineMath math="g"/> 값과 부모를 기억하고, 최종 경로는 부모를
                    거슬러 올라가 복원한다.
                </p>}
            />
            <Pseudocode code={`OPEN ← min-heap keyed by f;  g[start] ← 0;  push (h(start), start)   # 1
while OPEN is not empty:
    n ← pop_min(OPEN);  move n to CLOSED                               # 2
    if n = goal:                                                       # 3
        return reconstruct(parent, goal)
    for each neighbor n' with edge cost c(n, n'):
        if g[n] + c(n, n') < g[n']:                                    # 4
            g[n'] ← g[n] + c(n, n');  parent[n'] ← n                   # 5
            f[n'] ← g[n'] + h(n')
            push (f[n'], n') into OPEN
return failure`}/>
            <T
                en={<ol>
                    <li>Put the start in OPEN. Its <InlineMath math="g"/> is 0, so its priority is
                        the pure estimate <InlineMath math="f = h(\text{start})"/>.</li>
                    <li>Pop the node with the smallest <InlineMath math="f"/> — the most promising
                        total route — and move it to CLOSED so it is never expanded again (safe
                        with a consistent <InlineMath math="h"/>: its <InlineMath math="g"/> is
                        already final).</li>
                    <li>Check the goal <em>at pop time</em>. Checking when the goal is generated
                        would return the first path found, not the cheapest — the classic way to
                        lose optimality.</li>
                    <li>Compare going through <InlineMath math="n"/> against the best known route
                        to each neighbor (relaxation). Nothing happens unless this is an
                        improvement.</li>
                    <li>Record the better <InlineMath math="g"/> and parent, recompute{" "}
                        <InlineMath math="f"/>, and push the neighbor again — the stale queue entry
                        is skipped when it surfaces.</li>
                </ol>}
                ko={<ol>
                    <li>시작 노드를 OPEN에 넣는다. <InlineMath math="g"/>가 0이라 우선순위는
                        순수 추정치 <InlineMath math="f = h(\text{start})"/> 다.</li>
                    <li><InlineMath math="f"/>가 가장 작은, 즉 전체 경로 전망이 가장 좋은 노드를
                        꺼내 CLOSED로 옮긴다. 다시는 확장되지 않는다 (consistent{" "}
                        <InlineMath math="h"/>에서는 이 시점의 <InlineMath math="g"/>가 이미
                        확정이라 안전하다).</li>
                    <li>goal 검사를 <em>pop 시점에</em> 한다. 목표가 생성될 때 검사하면 가장 싼
                        경로가 아니라 처음 찾은 경로를 반환한다. 최적성을 잃는 전형적 실수다.</li>
                    <li>각 이웃에 대해, <InlineMath math="n"/>을 거쳐 가는 길과 지금까지의 최선을
                        비교한다 (relaxation). 개선이 아니면 아무 일도 일어나지 않는다.</li>
                    <li>개선이면 더 나은 <InlineMath math="g"/>와 부모를 기록하고{" "}
                        <InlineMath math="f"/>를 다시 계산해 push 한다. 낡은 큐 항목은 나중에
                        올라올 때 건너뛴다.</li>
                </ol>}
            />
            <T
                en={<p>
                    One practical detail matters more than it looks: <strong>tie-breaking</strong>.
                    On a uniform grid many nodes share the same <InlineMath math="f"/>, and a naive
                    queue may expand entire plateaus of them. Preferring the node with the
                    larger <InlineMath math="g"/> (deeper along its path) among equal{" "}
                    <InlineMath math="f"/> breaks these plateaus and is what keeps A* visibly
                    beating Dijkstra in the demo below.
                </p>}
                ko={<p>
                    보기보다 중요한 실전 디테일이 하나 있다: <strong>tie-breaking</strong>. 균일
                    비용 grid에서는 많은 노드가 같은 <InlineMath math="f"/>를 가져, 순진한
                    큐는 그 동률 구간 전체를 확장해 버린다. <InlineMath math="f"/> 동률일 때{" "}
                    <InlineMath math="g"/>가 큰(경로를 따라 더 깊이 간) 노드를 우선하면 이
                    동률이 깨진다. 아래 demo에서 A*가 Dijkstra를 눈에 띄게 이기는 것도 이
                    덕분이다.
                </p>}
            />

            <h2>{t("Heuristics", "Heuristic")}</h2>
            <T
                en={<>
                    <p>
                        Everything about A*'s guarantees hinges on what we demand
                        of <InlineMath math="h"/>. Two properties matter:
                    </p>
                    <ul>
                        <li>
                            <strong>Admissible</strong>: <InlineMath math="h(n) \le h^*(n)"/> for every
                            node — the estimate never overestimates the true remaining cost. This is
                            what makes the first path A* returns optimal.
                        </li>
                        <li>
                            <strong>Consistent</strong> (monotone):{" "}
                            <InlineMath math="h(n) \le c(n, n') + h(n')"/> for every edge, a triangle
                            inequality. Consistency implies admissibility, and it additionally
                            guarantees that once a node is expanded its <InlineMath math="g"/> is
                            final — no node ever needs re-expansion.
                        </li>
                    </ul>
                    <p>
                        On grids the standard choices are all admissible for their motion model:
                        Manhattan distance for 4-connected motion, <em>octile</em> distance for
                        8-connected motion, and Euclidean distance as a safe (if looser) bound for
                        any-angle motion:
                    </p>
                    <BlockMath math="h_{\text{octile}}(n) = \max(\Delta r, \Delta c) + (\sqrt{2} - 1)\,\min(\Delta r, \Delta c)"/>
                    <p>
                        Deliberately <em>inflating</em> an admissible heuristic gives{" "}
                        <strong>weighted A*</strong> (Pohl, 1970): order the frontier
                        by <InlineMath math="f = g + w\,h"/> with <InlineMath math="w > 1"/>. The
                        search becomes greedier and expands fewer nodes, and although optimality is
                        lost, the damage is bounded: the returned path costs at
                        most <InlineMath math="w"/> times the optimum. This g–h dial — Dijkstra at one
                        end, greedy best-first at the other — is the single most useful knob in
                        practice, and it is the <code>heuristic_weight</code> parameter of this
                        repository's implementation.
                    </p>
                </>}
                ko={<>
                    <p>
                        A*의 모든 보장은 <InlineMath math="h"/>에 무엇을 요구하느냐에 달려 있다.
                        중요한 성질은 둘이다:
                    </p>
                    <ul>
                        <li>
                            <strong>Admissible</strong>: 모든 노드에서{" "}
                            <InlineMath math="h(n) \le h^*(n)"/>, 즉 추정치가 실제 남은 비용을 절대
                            과대평가하지 않는다. A*가 처음 반환하는 경로가 최적인 것은 이 성질
                            덕분이다.
                        </li>
                        <li>
                            <strong>Consistent</strong> (monotone): 모든 간선에서{" "}
                            <InlineMath math="h(n) \le c(n, n') + h(n')"/>, 즉 삼각 부등식이다.
                            consistency는 admissibility를 함의하고, 나아가 한번 확장된 노드의{" "}
                            <InlineMath math="g"/>가 최종값임을 보장한다. 어떤 노드도 재확장할
                            필요가 없다.
                        </li>
                    </ul>
                    <p>
                        grid 에서의 표준 선택지는 각자의 이동 모델에 대해 모두 admissible 하다:
                        4-connected 이동에는 Manhattan 거리, 8-connected 이동에는 <em>octile</em>{" "}
                        거리, any-angle 이동에는 (느슨하지만 안전한) Euclidean 거리:
                    </p>
                    <BlockMath math="h_{\text{octile}}(n) = \max(\Delta r, \Delta c) + (\sqrt{2} - 1)\,\min(\Delta r, \Delta c)"/>
                    <p>
                        admissible heuristic을 의도적으로 <em>부풀리면</em>{" "}
                        <strong>weighted A*</strong> (Pohl, 1970)가 된다: frontier 를{" "}
                        <InlineMath math="w > 1"/> 인 <InlineMath math="f = g + w\,h"/>로 정렬한다.
                        탐색은 더 탐욕적이 되어 확장 노드가 줄고, 최적성은 잃어도 손실에는 상한이 있다:
                        반환 경로의 비용이 최적의 <InlineMath math="w"/> 배를 넘지 않는다. Dijkstra
                        가 한쪽 끝, greedy best-first가 반대쪽 끝인 이 g–h 다이얼이 실전에서 가장
                        요긴한 조절 수단이며, 이 저장소 구현의 <code>heuristic_weight</code> parameter
                        가 바로 그것이다.
                    </p>
                </>}
            />

            <h2>{t("Why A* Is Optimal", "A*는 왜 최적인가")}</h2>
            <T
                en={<p>
                    With an admissible heuristic, the first time A* pops the goal from OPEN, the path
                    it has found is optimal. The intuition: any not-yet-expanded prefix of a better
                    path would carry an <InlineMath math="f"/> value no larger than that better path's
                    cost — so it would have been popped first. The formal statements follow; expand
                    them if you want the details.
                </p>}
                ko={<p>
                    admissible heuristic 이라면, A*가 목표를 OPEN에서 처음 꺼내는 순간 찾은 경로는
                    최적이다. 직관은 이렇다. 더 싼 경로가 있었다면 그 경로 위 어딘가의 노드가 그 경로
                    비용 이하의 <InlineMath math="f"/> 값으로 OPEN에 남아 있었을 테니, 목표보다
                    먼저 꺼내졌을 것이다. 형식적 서술은 아래에 접어 두었으니, 자세히 보고 싶으면
                    펼쳐 보라.
                </p>}
            />
            <Proof title={t("Theorem (optimality of A*)", "정리 (A*의 최적성)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> <InlineMath math="h"/> admissible, edge costs
                            positive. Suppose the goal pops with{" "}
                            <InlineMath math="g(\text{goal}) > C^*"/>.
                        </p>
                        <p>
                            Take an optimal path <InlineMath math="\sigma"/> — some node of it is
                            always in OPEN; let <InlineMath math="n"/> be the first. Everything on{" "}
                            <InlineMath math="\sigma"/> before <InlineMath math="n"/> is settled
                            optimally, so <InlineMath math="g(n) = g^*(n)"/> and:
                        </p>
                        <BlockMath math="f(n) = g^*(n) + h(n) \;\overset{\text{admissible}}{\le}\; g^*(n) + h^*(n) \;=\; C^* \;<\; g(\text{goal}) = f(\text{goal})"/>
                        <p>
                            <InlineMath math="\Rightarrow n"/> pops before the goal — contradiction.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> <InlineMath math="h"/>는 admissible, 간선 비용은
                            양수. 귀류법으로 목표가 <InlineMath math="g(\text{goal}) > C^*"/>로
                            꺼내졌다고 하자.
                        </p>
                        <p>
                            최적 경로 <InlineMath math="\sigma"/>를 잡으면 그 노드 중 하나는 항상
                            OPEN에 있다. 그중 첫 노드를 <InlineMath math="n"/>이라 하면{" "}
                            <InlineMath math="n"/> 앞은 전부 최적으로 settle 되어{" "}
                            <InlineMath math="g(n) = g^*(n)"/> 이고:
                        </p>
                        <BlockMath math="f(n) = g^*(n) + h(n) \;\overset{\text{admissible}}{\le}\; g^*(n) + h^*(n) \;=\; C^* \;<\; g(\text{goal}) = f(\text{goal})"/>
                        <p>
                            <InlineMath math="\Rightarrow n"/>이 목표보다 먼저 꺼내진다. 모순.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>
            <Proof title={t("Lemma (consistency ⇒ no re-expansion)", "보조정리 (consistency ⇒ 재확장 불필요)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Consistency:{" "}
                            <InlineMath math="h(n) \le c(n, n') + h(n')"/> on every edge. Then along
                            any expansion:
                        </p>
                        <BlockMath math="f(n') = g(n) + c(n, n') + h(n') \;\overset{\text{consistency}}{\ge}\; g(n) + h(n) = f(n)"/>
                        <p>
                            <InlineMath math="\Rightarrow f"/> never decreases along explored paths{" "}
                            <InlineMath math="\Rightarrow"/> expansion order is non-decreasing in{" "}
                            <InlineMath math="f"/> <InlineMath math="\Rightarrow"/> a node's first
                            expansion already carries its optimal <InlineMath math="g"/>. No
                            reopening; a plain CLOSED set suffices.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> consistency:{" "}
                            <InlineMath math="h(n) \le c(n, n') + h(n')"/>이 모든 간선에서 성립.
                            그러면 임의의 확장에서:
                        </p>
                        <BlockMath math="f(n') = g(n) + c(n, n') + h(n') \;\overset{\text{consistency}}{\ge}\; g(n) + h(n) = f(n)"/>
                        <p>
                            <InlineMath math="\Rightarrow f"/>는 탐색 경로를 따라 감소하지 않는다{" "}
                            <InlineMath math="\Rightarrow"/> 확장 순서는 <InlineMath math="f"/> 비감소{" "}
                            <InlineMath math="\Rightarrow"/> 노드의 첫 확장이 이미 최적{" "}
                            <InlineMath math="g"/>를 가진다. 재확장이 없고, 단순한 CLOSED 집합이면
                            충분하다. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox below runs A* live in your browser. Drag on the grid to draw or erase
                    walls, drag the two endpoints to move start and goal, and switch the weight{" "}
                    <InlineMath math="w"/>. Watch three things: how the searched region (indigo)
                    changes shape as <InlineMath math="w"/> grows; how <InlineMath math="w = 0"/>{" "}
                    floods symmetrically in all directions; and how greedy settings dive into the
                    U-shaped pocket and come out with a visibly longer path.
                </p>}
                ko={<p>
                    아래 sandbox는 브라우저에서 A*를 라이브로 실행한다. grid를 드래그해 벽을
                    그리거나 지우고, 두 끝점을 끌어 시작/목표를 옮기고, 가중치{" "}
                    <InlineMath math="w"/>를 바꿔 보라. 볼거리는 셋이다: <InlineMath math="w"/>가
                    커질수록 탐색 영역(남색)의 모양이 어떻게 변하는지, <InlineMath math="w = 0"/>{" "}
                    이 모든 방향으로 대칭으로 번지는 것, 그리고 greedy 설정이 U자 pocket 안으로
                    뛰어들었다가 눈에 띄게 더 긴 경로를 들고 나오는 것.
                </p>}
            />
            <AStarSandbox/>
            <T
                en={<p>
                    The player below is different in kind: it replays <em>recorded traces</em> emitted
                    by this repository's actual implementations running on the benchmark maps. Every algorithm in the repository emits the same JSON event
                    stream (<code>node_expanded</code>, <code>edge_added</code>,{" "}
                    <code>path_found</code>, …), so this one player can replay any of them — the
                    visualization layer never touches algorithm internals.
                </p>}
                ko={<p>
                    아래 플레이어는 종류가 다르다: 이 저장소의 실제 구현이 벤치마크 맵 위에서 실행되며 방출한 <em>기록된 trace</em>를 재생한다. 저장소의 모든
                    알고리즘이 같은 JSON 이벤트 스트림(<code>node_expanded</code>,{" "}
                    <code>edge_added</code>, <code>path_found</code>, …)을 방출하므로 이 플레이어
                    하나로 무엇이든 재생할 수 있다. 시각화 계층은 알고리즘 내부를 전혀 만지지
                    않는다.
                </p>}
            />
            <TraceReplay algo="astar" maps={["maze01", "open01"]} label={t(
                "Replaying a real trace emitted by the repository's A* demo — the same JSON event stream drives this player",
                "저장소의 A* demo가 실제로 방출한 trace 재생. 이 플레이어는 그 JSON 이벤트 스트림을 그대로 소비한다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The repository's A* is deliberately thin. Dijkstra and A* share one best-first
                    loop (lazy priority queue, edge relaxation, trace emission); A* is the subclass
                    that turns the heuristic on and supplies the weight <InlineMath math="w"/> from
                    its parameters. The code below is the actual source, not an excerpt.
                </p>}
                ko={<p>
                    이 저장소의 A* 구현은 의도적으로 얇다. Dijkstra와 A*는 하나의 best-first
                    루프(lazy priority queue, edge relaxation, trace 방출)를 공유하고, A*는
                    heuristic을 켜고 parameter에서 가중치 <InlineMath math="w"/>를 공급하는
                    subclass 다. 아래 코드는 발췌가 아니라 실제 소스 그대로다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/global_planning/search/astar.py",
                                code: astarPy,
                                href: `${REPO}/python/navigation/global_planning/search/astar.py`,
                            },
                            {
                                name: "python/navigation/global_planning/search/_bestfirst.py",
                                code: bestFirstPy,
                                href: `${REPO}/python/navigation/global_planning/search/_bestfirst.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [
                            {
                                name: "cpp/src/global_planning/search/astar.cpp",
                                code: astarCpp,
                                href: `${REPO}/cpp/src/global_planning/search/astar.cpp`,
                            },
                            {
                                name: "cpp/include/navigation/global_planning/search/discrete_search.hpp",
                                code: discreteSearchHpp,
                                href: `${REPO}/cpp/include/navigation/global_planning/search/discrete_search.hpp`,
                            },
                        ],
                    },
                ]}
                caption={t(
                    "The shared best-first core and the A* subclass, embedded from the repository sources",
                    "공유 best-first 코어와 A* subclass. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    P. E. Hart, N. J. Nilsson, B. Raphael,{" "}
                    <a href="https://doi.org/10.1109/TSSC.1968.300136" target="_blank"
                       rel="noopener noreferrer">
                        <em>A Formal Basis for the Heuristic Determination of Minimum Cost Paths</em>
                    </a>,
                    IEEE Transactions on Systems Science and Cybernetics, 1968.
                </li>
                <li>
                    I. Pohl,{" "}
                    <a href="https://doi.org/10.1016/0004-3702(70)90007-X" target="_blank"
                       rel="noopener noreferrer">
                        <em>Heuristic Search Viewed as Path Finding in a Graph</em>
                    </a>,
                    Artificial Intelligence, 1970.
                </li>
                <li>
                    R. Dechter, J. Pearl,{" "}
                    <a href="https://doi.org/10.1145/3828.3830" target="_blank"
                       rel="noopener noreferrer">
                        <em>Generalized Best-First Search Strategies and the Optimality of A*</em>
                    </a>,
                    Journal of the ACM, 1985.
                </li>
            </ol>
        </>
    )
}

export default Astar

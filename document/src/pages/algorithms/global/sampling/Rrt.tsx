import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import RrtSandbox from "../../../../components/panels/global/rrt/RrtSandbox";
import rrtPy from "../../../../../../python/navigation/global_planning/sampling/rrt.py?raw";
import samplingPy from "../../../../../../python/navigation/global_planning/sampling/_sampling.py?raw";
import rrtCpp from "../../../../../../cpp/src/global_planning/sampling/rrt.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const Rrt = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    PRM builds a city map and then asks for directions. RRT (LaValle, 1998)
                    is the opposite temperament: <em>grow</em> a tree out of the start,
                    one collision-checked step at a time, and stop the moment any branch
                    touches the goal. No roadmap, no separate query phase — just a structure
                    that rushes outward into unexplored space. It became the workhorse of
                    single-query and kinodynamic planning precisely because each extension is
                    one local, incremental, dynamics-friendly step.
                </p>}
                ko={<p>
                    PRM은 도시 지도를 만들어 놓고 길을 묻는다. RRT(LaValle, 1998)는 반대
                    기질이다. 시작점에서 트리를 <em>키운다</em>. 충돌 검사를 통과한 한
                    스텝씩, 어느 가지든 goal에 닿는 순간 멈춘다. roadmap도, 분리된 질의
                    단계도 없다. 미탐사 공간으로 돌진하는 구조물 하나가 있을 뿐이다. 확장
                    하나하나가 국소적·점진적이고 동역학과 잘 맞는 한 스텝이라서, RRT는
                    single-query와 kinodynamic planning의 주력이 되었다.
                </p>}
            />

            <h2>{t("Pulled by the Void", "빈 공간이 끌어당긴다")}</h2>
            <T
                en={<>
                    <p>
                        Each iteration draws a random state and extends the <em>nearest</em>{" "}
                        tree node one step <InlineMath math="\eta"/> toward it. The magic is
                        in what "nearest" implies: a node is chosen exactly when the sample
                        lands in its Voronoi cell, so
                    </p>
                    <BlockMath math="\Pr[\text{node } v \text{ is extended}] \;=\; \frac{\mu(\mathrm{Vor}(v))}{\mu(X)}"/>
                    <Terms items={[
                        ["v", "a node of the tree"],
                        ["\\mathrm{Vor}(v)", <>the Voronoi cell of <InlineMath math="v"/>: the region of states closer to <InlineMath math="v"/> than to any other tree node</>],
                        ["\\mu(\\cdot)", "area (volume) of a region"],
                        ["X", "the sampling domain (the whole state space)"],
                    ]}/>
                    <p>
                        Nodes on the frontier of the explored region own huge Voronoi cells,
                        so the tree is statistically <em>pulled toward the emptiness</em> —
                        that is the "rapidly-exploring" in the name. A small goal bias (draw
                        the goal itself with probability <InlineMath math="p"/>) adds just
                        enough greed to finish; the sandbox shows it cutting iteration counts
                        roughly in half on the bug trap.
                    </p>
                </>}
                ko={<>
                    <p>
                        매 반복은 무작위 상태를 하나 뽑고, 트리에서 <em>가장 가까운</em>{" "}
                        노드를 그쪽으로 한 스텝 <InlineMath math="\eta"/>만큼 늘인다. 마법은
                        "가장 가까운"이 함의하는 것에 있다. 어떤 노드가 뽑히는 것은 표본이
                        그 노드의 Voronoi 셀에 떨어졌을 때이므로
                    </p>
                    <BlockMath math="\Pr[\text{노드 } v \text{가 확장됨}] \;=\; \frac{\mu(\mathrm{Vor}(v))}{\mu(X)}"/>
                    <Terms items={[
                        ["v", "트리의 노드 하나"],
                        ["\\mathrm{Vor}(v)", <><InlineMath math="v"/>의 Voronoi 셀. 다른 어떤 트리 노드보다 <InlineMath math="v"/>에 가까운 상태들의 영역</>],
                        ["\\mu(\\cdot)", "영역의 넓이(부피)"],
                        ["X", "표본을 뽑는 정의역 (상태 공간 전체)"],
                    ]}/>
                    <p>
                        탐사된 영역의 가장자리 노드들이 거대한 Voronoi 셀을 차지하므로,
                        트리는 통계적으로 <em>빈 공간 쪽으로 끌려간다</em>. 이름의
                        "rapidly-exploring"이 그 뜻이다. 작은 goal bias(확률{" "}
                        <InlineMath math="p"/>로 goal 자체를 표본으로 뽑기)가 마무리에 딱
                        필요한 만큼의 탐욕을 더한다. sandbox에서 bug trap의 반복 수가
                        대략 절반으로 줄어드는 것이 보인다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Probabilistically complete</strong>: the tree's vertex
                        distribution converges toward the sampling distribution, so any
                        reachable region is eventually entered.</li>
                    <li><strong>Not optimal — not even asymptotically</strong>: Karaman &amp;
                        Frazzoli (2011) proved the first path RRT finds converges to a{" "}
                        <em>suboptimal</em> one almost surely. Jagged paths are structural,
                        not bad luck; RRT* exists to fix exactly this.</li>
                    <li><strong>Single-query</strong>: everything the tree builds is rooted
                        at this start; a new query means a new tree.</li>
                    <li><strong>Cost</strong>: with the naive linear nearest-neighbor used
                        here, iteration <InlineMath math="k"/> costs{" "}
                        <InlineMath math="O(k)"/> distances —{" "}
                        <InlineMath math="O(n^2)"/> total; k-d trees drop it to{" "}
                        <InlineMath math="O(n \log n)"/>.</li>
                </ul>}
                ko={<ul>
                    <li><strong>확률적 완전</strong>: 트리 정점의 분포가 표본 분포로
                        수렴하므로, 도달 가능한 영역에는 결국 들어간다.</li>
                    <li><strong>최적이 아니다. 점근적으로도 아니다</strong>: Karaman &amp;
                        Frazzoli(2011)가 RRT의 첫 경로가 <em>준최적</em> 경로로 거의 확실히
                        수렴함을 증명했다. 삐죽삐죽한 경로는 불운이 아니라 구조다. RRT* 이
                        정확히 이것을 고치러 나온다.</li>
                    <li><strong>Single-query</strong>: 트리가 쌓는 모든 것이 이 시작점에
                        뿌리내려 있다. 새 질의는 새 트리다.</li>
                    <li><strong>비용</strong>: 여기 쓰인 순진한 선형 nearest로는 반복{" "}
                        <InlineMath math="k"/>가 거리 계산 <InlineMath math="O(k)"/>회라
                        총 <InlineMath math="O(n^2)"/>이다. k-d tree를 쓰면{" "}
                        <InlineMath math="O(n \log n)"/>으로 내려간다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One loop, four verbs — sample, nearest, steer, check:
                </p>}
                ko={<p>
                    루프 하나에 동사 넷이다. sample, nearest, steer, check:
                </p>}
            />
            <Pseudocode code={`tree ← {start}
repeat up to max_iterations:
    q_rand ← goal with probability p, else uniform sample     # 1
    q_near ← nearest tree node to q_rand                      # 2
    q_new ← steer(q_near, q_rand, η)                          # 3
    if motion q_near → q_new is collision-free:
        add q_new to tree with parent q_near
        if ‖q_new − goal‖ ≤ tolerance and motion to goal is free:
            add goal;  return path along parent links         # 4`}/>
            <T
                en={<ol>
                    <li>Goal bias: a coin flip decides between greed (the goal itself) and
                        exploration (a uniform draw). Even 5% changes the character of the
                        search.</li>
                    <li>Nearest in Euclidean distance — the step that creates the Voronoi
                        pull, and the step worth accelerating with a spatial index.</li>
                    <li>Steer caps the extension at <InlineMath math="\eta"/>: samples far
                        away only choose the <em>direction</em>, keeping every new edge
                        short enough to collision-check cheaply.</li>
                    <li>Reaching the goal region ends the search immediately — first
                        feasible path wins, which is exactly why the result is not
                        optimal.</li>
                </ol>}
                ko={<ol>
                    <li>goal bias: 동전 하나가 탐욕(goal 자체)과 탐사(균일 표본)를 가른다.
                        5%만 줘도 탐색의 성격이 달라진다.</li>
                    <li>유클리드 거리 기준 nearest. Voronoi 끌림을 만드는 스텝이자, 공간
                        인덱스로 가속할 가치가 있는 스텝이다.</li>
                    <li>steer는 확장을 <InlineMath math="\eta"/>로 자른다. 먼 표본은{" "}
                        <em>방향</em>만 정하고, 새 간선은 늘 싸게 충돌 검사할 만큼 짧게
                        유지된다.</li>
                    <li>goal 반경에 닿으면 즉시 종료한다. 처음 찾은 실행 가능 경로가
                        이기는 규칙이고, 결과가 최적이 아닌 이유가 정확히 이것이다.</li>
                </ol>}
            />
            <Proof title={t("Why exploration is 'rapid' (Voronoi bias)", "탐사가 '빠른' 이유 (Voronoi bias)")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Uniform samples choose the node whose
                            Voronoi cell they land in. Early in the search the tree occupies
                            a small ball; every state outside it is closest to some frontier
                            node, so
                        </p>
                        <BlockMath math="\sum_{v \in \text{frontier}} \Pr[v \text{ extended}] \;\ge\; \frac{\mu(X \setminus \text{explored})}{\mu(X)} \;\approx\; 1"/>
                        <Terms items={[
                            ["\\text{frontier}", "tree nodes on the boundary of the explored region"],
                            ["X \\setminus \\text{explored}", "the as-yet-unvisited part of the space — early on, almost everything"],
                            ["\\mu(\\cdot)", "area (volume) of a region"],
                        ]}/>
                        <p>
                            So nearly every iteration pushes the boundary outward instead of
                            refining the interior; interior nodes have tiny Voronoi cells and
                            are almost never selected. The tree expands like a front, not
                            like a random walk. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> 균일 표본은 자신이 떨어진 Voronoi 셀의
                            노드를 고른다. 탐색 초기에 트리는 작은 공 안에 있고, 그 밖의
                            모든 상태는 어떤 가장자리 노드에 가장 가까우므로
                        </p>
                        <BlockMath math="\sum_{v \in \text{frontier}} \Pr[v \text{ 확장}] \;\ge\; \frac{\mu(X \setminus \text{explored})}{\mu(X)} \;\approx\; 1"/>
                        <Terms items={[
                            ["\\text{frontier}", "탐사 영역 경계에 있는 트리 노드들"],
                            ["X \\setminus \\text{explored}", "아직 방문하지 않은 공간. 초기에는 거의 전부다"],
                            ["\\mu(\\cdot)", "영역의 넓이(부피)"],
                        ]}/>
                        <p>
                            따라서 거의 모든 반복이 내부를 다듬는 대신 경계를 바깥으로
                            민다. 내부 노드는 Voronoi 셀이 작아 거의 뽑히지 않는다. 트리는
                            random walk이 아니라 전선처럼 퍼진다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox starts the tree inside a bug trap whose only exit faces away
                    from the goal. Pure exploration (bias 0) wanders out in a couple of
                    thousand iterations; 5% bias roughly halves that; crank η and the tree
                    strides instead of shuffling. The replay below is the repository demo on
                    the benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 유일한 출구가 goal 반대쪽인 bug trap 안에서 트리를 키운다.
                    순수 탐사(bias 0)는 이천 번쯤 헤매다 나오고, 5% bias는 그걸 대략
                    절반으로 줄인다. η를 키우면 트리가 종종걸음 대신 성큼성큼 걷는다.
                    아래 replay는 벤치마크 맵 위의 저장소 demo다.
                </p>}
            />
            <RrtSandbox/>
            <TraceReplay algo="rrt" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's RRT demo — the tree floods the free space and stops at the first branch that touches the goal",
                "저장소 RRT demo의 실제 trace. 트리가 free 공간을 채워 가다 goal에 닿는 첫 가지에서 멈춘다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The tree container (parallel arrays with subtree cost propagation, ready
                    for RRT*'s rewiring) lives in the shared sampling module; RRT itself is
                    the four-verb loop. Embedded below in full.
                </p>}
                ko={<p>
                    트리 컨테이너(부분 트리 비용 전파까지 갖춰 RRT*의 rewiring을 준비해
                    둔 병렬 배열)는 공유 sampling 모듈에 있다. RRT 자체는 동사 네 개짜리
                    루프다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [
                            {
                                name: "python/navigation/global_planning/sampling/rrt.py",
                                code: rrtPy,
                                href: `${REPO}/python/navigation/global_planning/sampling/rrt.py`,
                            },
                            {
                                name: "python/navigation/global_planning/sampling/_sampling.py",
                                code: samplingPy,
                                href: `${REPO}/python/navigation/global_planning/sampling/_sampling.py`,
                            },
                        ],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/rrt.cpp",
                            code: rrtCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/rrt.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete RRT implementation, embedded from the repository sources",
                    "RRT 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. M. LaValle,{" "}
                    <a href="https://msl.cs.illinois.edu/~lavalle/papers/Lav98c.pdf"
                       target="_blank" rel="noopener noreferrer">
                        <em>Rapidly-Exploring Random Trees: A New Tool for Path Planning</em>
                    </a>,
                    Technical Report TR 98-11, Iowa State University, 1998.
                </li>
                <li>
                    S. M. LaValle, J. J. Kuffner,{" "}
                    <a href="https://doi.org/10.1177/02783640122067453" target="_blank"
                       rel="noopener noreferrer">
                        <em>Randomized Kinodynamic Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2001.
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

export default Rrt

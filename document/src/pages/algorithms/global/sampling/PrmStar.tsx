import {ReactNode} from "react";
import {T, useTr} from "../../../../libs/i18n";
import Terms from "../../../../components/math/Terms";
import {BlockMath, InlineMath} from "../../../../components/math/Tex";
import Pseudocode from "../../../../components/Pseudocode";
import CodeTabs from "../../../../components/CodeTabs";
import TraceReplay from "../../../../components/panels/global/TraceReplay";
import PrmStarSandbox from "../../../../components/panels/global/prm_star/PrmStarSandbox";
import prmStarPy from "../../../../../../python/navigation/global_planning/sampling/prm_star.py?raw";
import prmStarCpp from "../../../../../../cpp/src/global_planning/sampling/prm_star.cpp?raw";

const REPO = "https://github.com/robotics-study/navigation/blob/main"

// 접이식 논증 블록 — 본문 흐름은 직관 중심으로 유지하고, 형식 논증은 원할 때만 편다.
const Proof = ({title, children}: {title: string; children: ReactNode}) => (
    <details className="border border-border rounded-xl px-4 py-3 my-4 bg-surface">
        <summary className="font-semibold cursor-pointer select-none">{title}</summary>
        <div className="pt-3">{children}</div>
    </details>
)

const PrmStar = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    PRM's fixed radius is a dilemma: too small and the roadmap shatters, too
                    large and every node drags in a crowd of useless edges — and neither
                    choice makes the answer converge to the <em>optimal</em> path. PRM*
                    (Karaman &amp; Frazzoli, 2011) resolves it with one formula: let the
                    connection radius shrink with the sample count, just slowly enough that
                    each node keeps <InlineMath math="\Theta(\log n)"/> neighbors. That
                    single change turns "probably finds a path" into "almost surely converges
                    to the shortest one".
                </p>}
                ko={<p>
                    PRM의 고정 반경은 딜레마다. 너무 작으면 roadmap이 부서지고, 너무 크면
                    노드마다 쓸모없는 간선 무리가 붙는다. 어느 쪽을 골라도 답이{" "}
                    <em>최적</em> 경로로 수렴하지는 않는다. PRM*(Karaman &amp; Frazzoli,
                    2011)은 공식 하나로 이 딜레마를 푼다. 연결 반경을 표본 수와 함께
                    줄이되, 노드마다 <InlineMath math="\Theta(\log n)"/>개의 이웃이 남을
                    만큼만 천천히 줄인다. 그 한 가지 변화가 "경로를 아마 찾는다"를 "최단
                    경로로 거의 확실히 수렴한다"로 바꾼다.
                </p>}
            />

            <h2>{t("The Radius That Knows n", "n을 아는 반경")}</h2>
            <T
                en={<>
                    <p>
                        Random geometric graph theory pins down the threshold. Connect every
                        pair of <InlineMath math="n"/> uniform samples within
                    </p>
                    <BlockMath math="r_n \;=\; \gamma \left(\frac{\log n}{n}\right)^{1/d}"/>
                    <Terms items={[
                        ["r_n", <><strong>the new ingredient</strong>: the connection radius, now a function of the sample count instead of a constant</>],
                        ["n", "number of roadmap nodes"],
                        ["d", <>dimension of the state space (here <InlineMath math="d = 2"/>, hence the square root)</>],
                        ["\\gamma", <>a constant that must exceed a space-dependent threshold <InlineMath math="\gamma^*"/> for the guarantees to hold</>],
                    ]}/>
                    <p>
                        Below this scale the graph falls apart into islands; above it, edges
                        pile up without improving the answer. Exactly at it, the expected
                        neighbor count grows like <InlineMath math="\log n"/> — enough
                        redundancy that near-optimal routes survive every finite sample set,
                        yet cheap enough that the roadmap stays sparse. In the sandbox the
                        radius readout shrinks (6.4 → 2.7 as <InlineMath math="n"/> goes
                        100 → 800) while the path cost creeps down toward the straight-line
                        bound.
                    </p>
                </>}
                ko={<>
                    <p>
                        random geometric graph 이론이 그 임계 규모를 정확히 짚는다.{" "}
                        <InlineMath math="n"/>개의 균일 표본에서
                    </p>
                    <BlockMath math="r_n \;=\; \gamma \left(\frac{\log n}{n}\right)^{1/d}"/>
                    <Terms items={[
                        ["r_n", <><strong>새로 추가된 재료</strong>: 연결 반경. 상수가 아니라 표본 수의 함수다</>],
                        ["n", "roadmap 노드 수"],
                        ["d", <>상태 공간의 차원 (여기서는 <InlineMath math="d = 2"/>라서 제곱근)</>],
                        ["\\gamma", <>공간에 따른 임계값 <InlineMath math="\gamma^*"/>보다 커야 보장이 성립하는 상수</>],
                    ]}/>
                    <p>
                        이 규모 아래에서는 그래프가 섬으로 부서지고, 위에서는 답을 개선하지
                        못하는 간선만 쌓인다. 정확히 이 규모에서 기대 이웃 수가{" "}
                        <InlineMath math="\log n"/>처럼 자란다. 유한한 표본마다 준최적
                        경로가 살아남을 만큼의 중복이면서, roadmap이 희소하게 유지될 만큼
                        싸다. sandbox의 반경 readout은 <InlineMath math="n"/>이 100 → 800으로
                        가는 동안 6.4 → 2.7로 줄고, 경로 비용은 직선 하한으로 기어 내려간다.
                    </p>
                </>}
            />

            <h2>{t("Properties and Complexity", "성질과 복잡도")}</h2>
            <T
                en={<ul>
                    <li><strong>Asymptotically optimal</strong>: with{" "}
                        <InlineMath math="\gamma"/> above the threshold, the returned cost
                        converges to the optimum almost surely as{" "}
                        <InlineMath math="n \to \infty"/> — the property PRM lacks at any
                        fixed radius.</li>
                    <li><strong>Probabilistically complete</strong>, like PRM — and in
                        practice <em>better</em> connected at small <InlineMath math="n"/>,
                        because <InlineMath math="r_n"/> starts generous and shrinks only as
                        samples densify.</li>
                    <li><strong>Still multi-query</strong>: nothing about sampling or
                        connection looks at the query.</li>
                    <li><strong>Cost</strong>: expected edges{" "}
                        <InlineMath math="\Theta(n \log n)"/> instead of PRM's{" "}
                        <InlineMath math="\Theta(n)"/>-per-radius-ball tradeoff; this
                        repository's naive all-pairs connect is{" "}
                        <InlineMath math="O(n^2)"/> distance checks either way.</li>
                </ul>}
                ko={<ul>
                    <li><strong>점근 최적</strong>: <InlineMath math="\gamma"/>가 임계값을
                        넘으면 반환 비용이 <InlineMath math="n \to \infty"/>에서 최적으로
                        거의 확실히 수렴한다. 어떤 고정 반경의 PRM에도 없는 성질이다.</li>
                    <li>PRM처럼 <strong>확률적 완전</strong>이고, 실전에서는 작은{" "}
                        <InlineMath math="n"/>에서 오히려 <em>더 잘</em> 이어진다.{" "}
                        <InlineMath math="r_n"/>이 넉넉하게 시작해 표본이 빽빽해질 때만
                        줄기 때문이다.</li>
                    <li><strong>여전히 multi-query</strong>: 표본과 연결 어디에도 질의가
                        들어가지 않는다.</li>
                    <li><strong>비용</strong>: 기대 간선 수가 PRM의 반경-공 트레이드오프
                        대신 <InlineMath math="\Theta(n \log n)"/>이다. 이 저장소의 순진한
                        전 쌍 연결은 어느 쪽이든 <InlineMath math="O(n^2)"/> 거리 계산이다.</li>
                </ul>}
            />

            <h2>{t("The Algorithm", "알고리즘")}</h2>
            <T
                en={<p>
                    One line changes relative to PRM — the radius is computed from the final
                    node count before the connection pass:
                </p>}
                ko={<p>
                    PRM에서 한 줄이 바뀐다. 연결 패스 전에 최종 노드 수로 반경을 계산한다:
                </p>}
            />
            <Pseudocode code={`nodes ← [start, goal] + n free samples          # same as PRM
r ← γ · (log |nodes| / |nodes|)^(1/d)           # 1  the PRM* radius
for each node v (in insertion order):
    for each earlier node u with ‖u − v‖ ≤ r:   # 2
        if straight motion u → v is collision-free:
            add undirected edge (u, v) with weight ‖u − v‖
return dijkstra(nodes, edges, start, goal)`}/>
            <T
                en={<ol>
                    <li>Computed once, from the count that includes start and goal. Batch
                        construction permits this; incremental planners (RRT*) recompute the
                        equivalent radius at every insertion.</li>
                    <li>Everything else — sampling, connection order, query — is PRM
                        verbatim, which is why the repository implements both over the same
                        roadmap plumbing.</li>
                </ol>}
                ko={<ol>
                    <li>시작·목표를 포함한 개수로 한 번만 계산한다. batch 구축이라 가능한
                        일이고, incremental 한 planner(RRT*)는 삽입마다 같은 반경을 다시
                        계산한다.</li>
                    <li>나머지 전부, 곧 표본·연결 순서·질의는 PRM 그대로다. 저장소가 두
                        알고리즘을 같은 roadmap 배관 위에 구현한 이유다.</li>
                </ol>}
            />
            <Proof title={t("Why log n neighbors is the threshold", "이웃 log n개가 임계인 이유")}>
                <T
                    en={<>
                        <p>
                            <strong>Setup.</strong> Cover the free space with disjoint balls
                            of radius <InlineMath math="r_n/2"/>; there are roughly{" "}
                            <InlineMath math="\mu(C_{\text{free}})/\mu(B_{r_n/2})"/> of them.
                            The expected number of samples per ball is
                        </p>
                        <BlockMath math="n \cdot \frac{\mu(B_{r_n/2})}{\mu(C_{\text{free}})} \;=\; \Theta\!\left(n \cdot r_n^{\,d}\right) \;=\; \Theta\!\left(\gamma^d \log n\right)"/>
                        <Terms items={[
                            ["B_{r_n/2}", <>a ball of half the connection radius: two samples in the same ball are surely within <InlineMath math="r_n"/></>],
                            ["\\mu(\\cdot)", "area (volume) of a region"],
                            ["n \\cdot r_n^{\\,d}", <>plugging in <InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/> makes the <InlineMath math="n"/> cancel, leaving <InlineMath math="\log n"/></>],
                        ]}/>
                        <p>
                            With <InlineMath math="\Theta(\log n)"/> expected samples per
                            ball, the probability that <em>some</em> ball among the{" "}
                            <InlineMath math="\Theta(n/\log n)"/> of them stays empty is
                            summable (coupon-collector asymptotics), so eventually every ball
                            along any path is occupied — connectivity and, with the
                            constant-factor slack in <InlineMath math="\gamma"/>, enough
                            nearby detours to realize near-optimal routes. Shrinking any
                            faster (say <InlineMath math="r_n \sim n^{-1/d}"/> without the
                            log) leaves balls empty forever. <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                    ko={<>
                        <p>
                            <strong>가정.</strong> free 공간을 반지름{" "}
                            <InlineMath math="r_n/2"/>짜리 서로소 공들로 덮으면 대략{" "}
                            <InlineMath math="\mu(C_{\text{free}})/\mu(B_{r_n/2})"/>개가
                            나온다. 공 하나당 기대 표본 수는
                        </p>
                        <BlockMath math="n \cdot \frac{\mu(B_{r_n/2})}{\mu(C_{\text{free}})} \;=\; \Theta\!\left(n \cdot r_n^{\,d}\right) \;=\; \Theta\!\left(\gamma^d \log n\right)"/>
                        <Terms items={[
                            ["B_{r_n/2}", <>연결 반경 절반짜리 공. 같은 공의 두 표본은 반드시 <InlineMath math="r_n"/> 안에 든다</>],
                            ["\\mu(\\cdot)", "영역의 넓이(부피)"],
                            ["n \\cdot r_n^{\\,d}", <><InlineMath math="r_n = \gamma(\log n / n)^{1/d}"/>를 대입하면 <InlineMath math="n"/>이 소거되고 <InlineMath math="\log n"/>만 남는다</>],
                        ]}/>
                        <p>
                            공마다 기대 표본이 <InlineMath math="\Theta(\log n)"/>개면,{" "}
                            <InlineMath math="\Theta(n/\log n)"/>개의 공 중 <em>어느</em>{" "}
                            하나라도 비어 있을 확률이 합산 가능해진다 (coupon collector
                            점근). 결국 어떤 경로를 따라가든 모든 공이 채워지고, 연결성과
                            함께 <InlineMath math="\gamma"/>의 상수 여유만큼의 우회로가
                            생겨 준최적 경로가 실현된다. 이보다 빨리 줄이면 (log 없이{" "}
                            <InlineMath math="r_n \sim n^{-1/d}"/>) 영원히 비는 공이 남는다.{" "}
                            <InlineMath math="\blacksquare"/>
                        </p>
                    </>}
                />
            </Proof>

            <h2>Demo</h2>
            <T
                en={<p>
                    The sandbox runs PRM* and fixed-radius PRM on the <em>same samples</em>.
                    At 100 samples PRM's r = 2.5 roadmap is usually in pieces while PRM*'s
                    generous early radius connects; push to 800 and both connect, but watch
                    the radius readout shrink while the PRM* cost edges toward the
                    straight-line bound. The replay below is the repository demo on the
                    benchmark maps.
                </p>}
                ko={<p>
                    sandbox는 <em>같은 표본</em> 위에서 PRM*과 고정 반경 PRM을 돌린다.
                    표본 100개에서는 PRM의 r = 2.5 roadmap이 대개 조각나 있는데, PRM*은
                    넉넉한 초기 반경으로 이어진다. 800개로 올리면 둘 다 이어지지만, 반경
                    readout이 줄어드는 동안 PRM* 비용이 직선 하한으로 다가가는 것을 보라.
                    아래 replay는 벤치마크 맵 위의 저장소 demo다.
                </p>}
            />
            <PrmStarSandbox/>
            <TraceReplay algo="prm_star" maps={["open01", "maze01"]} label={t(
                "Real traces from the repository's PRM* demo — same three phases as PRM, with the radius computed from the final sample count",
                "저장소 PRM* demo의 실제 trace. PRM과 같은 세 단계이고, 반경만 최종 표본 수로 계산된다",
            )}/>

            <h2>Implementation</h2>
            <T
                en={<p>
                    The roadmap plumbing is shared with PRM; PRM* is the sampling loop plus
                    the one-line radius formula. Embedded below in full.
                </p>}
                ko={<p>
                    roadmap 배관은 PRM과 공유한다. PRM*은 표본 루프에 한 줄짜리 반경 공식을
                    더한 것이다. 전체를 아래에 embed 했다.
                </p>}
            />
            <CodeTabs
                tabs={[
                    {
                        label: "python",
                        lang: "python",
                        files: [{
                            name: "python/navigation/global_planning/sampling/prm_star.py",
                            code: prmStarPy,
                            href: `${REPO}/python/navigation/global_planning/sampling/prm_star.py`,
                        }],
                    },
                    {
                        label: "c++",
                        lang: "cpp",
                        files: [{
                            name: "cpp/src/global_planning/sampling/prm_star.cpp",
                            code: prmStarCpp,
                            href: `${REPO}/cpp/src/global_planning/sampling/prm_star.cpp`,
                        }],
                    },
                ]}
                caption={t(
                    "The complete PRM* implementation, embedded from the repository sources",
                    "PRM* 전체 구현. 저장소 소스를 그대로 embed 한 것이다",
                )}
            />

            <h2>References</h2>
            <ol>
                <li>
                    S. Karaman, E. Frazzoli,{" "}
                    <a href="https://doi.org/10.1177/0278364911406761" target="_blank"
                       rel="noopener noreferrer">
                        <em>Sampling-based Algorithms for Optimal Motion Planning</em>
                    </a>,
                    The International Journal of Robotics Research, 2011.
                </li>
                <li>
                    L. E. Kavraki, P. Švestka, J.-C. Latombe, M. H. Overmars,{" "}
                    <a href="https://doi.org/10.1109/70.508439" target="_blank"
                       rel="noopener noreferrer">
                        <em>Probabilistic Roadmaps for Path Planning in High-Dimensional
                            Configuration Spaces</em>
                    </a>,
                    IEEE Transactions on Robotics and Automation, 1996.
                </li>
            </ol>
        </>
    )
}

export default PrmStar

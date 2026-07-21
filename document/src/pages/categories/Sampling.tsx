import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import RrtVsPrm from "../../components/panels/intro/RrtVsPrm";

const Sampling = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    Sampling-based planning gives up on covering the space and settles for{" "}
                    <em>probing</em> it: draw random states, keep the ones that are collision-free,
                    and wire them together until start and goal end up in the same connected
                    structure. It sounds too crude to work. It works astonishingly well.
                </p>}
                ko={<p>
                    Sampling 기반 planning은 공간을 덮는 것을 포기하고 <em>찔러 보는</em> 것으로
                    타협한다. 무작위 상태를 뽑고, 충돌 없는 것만 남기고, 시작과 목표가 같은 연결
                    구조 안에 들어올 때까지 이어 붙인다. 너무 조잡해서 안 될 것 같지만, 놀랍도록
                    잘 된다.
                </p>}
            />

            <h2>{t("Why Sample", "왜 sampling 인가")}</h2>
            <T
                en={<p>
                    Grids die of dimensionality. A 2D grid at sensible resolution is thousands of
                    cells; the same idea for a 7-DOF arm is <InlineMath math="10^{14}"/>+ — and
                    building it means collision-checking every cell, almost all of which the search
                    will never touch. Sampling inverts the deal: never enumerate the space, only
                    collision-check the states you actually draw and the short motions between them.
                    The cost scales with how much of the space the planner <em>needs to see</em>,
                    not with how big the space is.
                </p>}
                ko={<p>
                    격자는 차원이 늘면 버티지 못한다. 적당한 해상도의 2D 격자는 수천 셀이지만, 같은
                    아이디어를 7-DOF 팔에 쓰면 <InlineMath math="10^{14}"/> 개가 넘는다. 게다가
                    격자를 만드는 것 자체가 모든 셀의 충돌 검사를 뜻하는데, 그 대부분은 탐색이
                    건드리지도 않는다. sampling은 접근을 뒤집는다. 공간을 열거하지 않고, 실제로
                    뽑은 상태와 그 사이의 짧은 이동만 충돌 검사한다. 비용은 공간의 크기가 아니라
                    planner가 <em>봐야 하는</em> 양에 비례한다.
                </p>}
            />

            <h2>{t("The Building Blocks", "구성 요소")}</h2>
            <T
                en={<>
                    <p>
                        Every planner in this category is assembled from five primitives (the
                        repository's <code>SamplingSpace</code> capability):
                    </p>
                    <ul>
                        <li><code>sample()</code> — draw a random state.</li>
                        <li><code>is_state_valid(s)</code> — is this state collision-free?</li>
                        <li><code>is_motion_valid(a, b)</code> — is the straight motion between two
                            states collision-free?</li>
                        <li><code>distance(a, b)</code> — how far apart are two states?</li>
                        <li><code>steer(a, b, η)</code> — move from <InlineMath math="a"/> toward{" "}
                            <InlineMath math="b"/>, at most <InlineMath math="\eta"/> far.</li>
                    </ul>
                    <p>
                        Swap the space, keep the planner: the same RRT* code plans for a point robot
                        in 2D and, in principle, for an arm in 7D.
                    </p>
                </>}
                ko={<>
                    <p>
                        이 카테고리의 모든 planner는 다섯 개의 primitive(저장소의{" "}
                        <code>SamplingSpace</code> capability)로 조립된다:
                    </p>
                    <ul>
                        <li><code>sample()</code>: 무작위 상태를 뽑는다.</li>
                        <li><code>is_state_valid(s)</code>: 이 상태는 충돌이 없는가?</li>
                        <li><code>is_motion_valid(a, b)</code>: 두 상태 사이의 직선 이동은 충돌이
                            없는가?</li>
                        <li><code>distance(a, b)</code>: 두 상태는 얼마나 떨어져 있는가?</li>
                        <li><code>steer(a, b, η)</code>: <InlineMath math="a"/> 에서{" "}
                            <InlineMath math="b"/> 쪽으로 최대 <InlineMath math="\eta"/> 만큼
                            움직인다.</li>
                    </ul>
                    <p>
                        공간을 바꿔도 planner는 그대로다. 같은 RRT* 코드가 2D 점 로봇을 계획하고,
                        원리적으로는 7D 팔도 계획한다.
                    </p>
                </>}
            />

            <h2>{t("Trees and Roadmaps", "Tree와 Roadmap")}</h2>
            <T
                en={<ul>
                    <li>
                        <strong>Trees (RRT family)</strong> grow from the start: sample, find the
                        nearest tree node, steer toward the sample, add the edge if valid. The tree
                        naturally spreads into unexplored space (the Voronoi bias) — good for
                        single-query planning, and the only game in town once dynamics enter.
                    </li>
                    <li>
                        <strong>Roadmaps (PRM family)</strong> sample the whole space up front,
                        connect neighbors into a graph, then answer queries with graph search. The
                        construction cost is amortized over many queries — good for a robot that
                        lives in one environment.
                    </li>
                </ul>}
                ko={<ul>
                    <li>
                        <strong>Tree (RRT 계열)</strong>는 시작점에서 자란다. 샘플을 뽑고, 가장
                        가까운 트리 노드를 찾고, 샘플 쪽으로 steer 하고, 유효하면 간선을 더한다.
                        트리는 자연스럽게 미탐험 공간으로 퍼진다(Voronoi bias). 단일 질의에 좋고,
                        동역학이 들어오면 사실상 유일한 선택지다.
                    </li>
                    <li>
                        <strong>Roadmap (PRM 계열)</strong>은 공간 전체를 미리 샘플링해 이웃을
                        그래프로 잇고, 질의는 graph search로 답한다. 구축 비용이 여러 질의에
                        분할 상환된다. 한 환경에서 오래 사는 로봇에 좋다.
                    </li>
                </ul>}
            />

            <RrtVsPrm/>

            <h2>{t("Guarantees", "보장")}</h2>
            <T
                en={<p>
                    The guarantees are softer than graph search's, and the history of this category
                    is the history of tightening them. Basic RRT/PRM are{" "}
                    <em>probabilistically complete</em>: if a solution exists, the probability of
                    finding one approaches 1 as samples grow. RRT* (Karaman &amp; Frazzoli, 2011)
                    added <em>asymptotic optimality</em>: the returned path converges to the optimum
                    as samples grow, at the price of rewiring work per sample. Informed and batch
                    variants (Informed RRT*, BIT* and its successors) then made that convergence
                    fast by focusing samples where an improvement can possibly lie.
                </p>}
                ko={<p>
                    보장은 graph search보다 약하고, 이 카테고리의 역사는 그것을 조여 온 역사다.
                    기본 RRT/PRM은 <em>확률적 완전</em>이다. 해가 존재하면 샘플이 늘수록 찾을
                    확률이 1에 다가간다. RRT*(Karaman &amp; Frazzoli, 2011)는 <em>점근 최적성</em>
                    을 더했다. 샘플이 늘수록 반환 경로가 최적으로 수렴하며, 샘플마다의 rewiring
                    작업이 그 대가다. informed·batch 변형(Informed RRT*, BIT*와 후속들)은 개선이
                    있을 수 있는 곳에만 샘플을 집중시켜 그 수렴을 빠르게 만들었다.
                </p>}
            />

            <h2>{t("A Map of the Algorithms", "알고리즘 지도")}</h2>
            <T
                en={<ul>
                    <li><strong>Feasibility first</strong> — RRT (grow a tree), RRT-Connect (grow
                        two, from both ends), Fast-RRT (bias the growth), PRM (build a roadmap).</li>
                    <li><strong>Optimality</strong> — RRT* and PRM* (rewire / connect at the radius
                        that preserves asymptotic optimality).</li>
                    <li><strong>Informed &amp; batch</strong> — Informed RRT* (sample the ellipsoid
                        that can improve the solution), FMT* (marching over a batch), BIT*, ABIT*,
                        AIT*, EIT*, FCIT* (batches + heuristically ordered edge processing).</li>
                    <li><strong>Kinodynamic</strong> — Kinodynamic RRT*, LQR-RRT*, SST (plan
                        directly over dynamics, where steering exactly is hard or impossible).</li>
                </ul>}
                ko={<ul>
                    <li><strong>Feasibility 우선</strong>: RRT(트리 하나), RRT-Connect(양끝에서
                        둘), Fast-RRT(성장에 bias), PRM(roadmap 구축).</li>
                    <li><strong>최적성</strong>: RRT*와 PRM*(점근 최적성이 보존되는 반경으로
                        rewire/연결).</li>
                    <li><strong>Informed &amp; batch</strong>: Informed RRT*(해를 개선할 수 있는
                        타원체만 샘플링), FMT*(batch 단위 전파), BIT*, ABIT*, AIT*, EIT*,
                        FCIT*(batch와 heuristic 순서의 간선 처리).</li>
                    <li><strong>Kinodynamic</strong>: Kinodynamic RRT*, LQR-RRT*, SST(정확한 steer
                        가 어렵거나 불가능한 동역학 위에서 직접 계획).</li>
                </ul>}
            />
        </>
    )
}

export default Sampling

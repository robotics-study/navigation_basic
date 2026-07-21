import {T, useTr} from "../../libs/i18n";
import {InlineMath} from "../../components/math/Tex";
import SpaceTimeConflict from "../../components/panels/intro/SpaceTimeConflict";

const MultiAgent = () => {
    const t = useTr()
    return (
        <>
            <T
                en={<p>
                    One robot with a path is a solved problem. A warehouse with three hundred robots
                    sharing the same aisles is not: paths that are individually perfect can be
                    collectively impossible. Multi-agent planning is about finding paths
                    that <em>coexist</em>.
                </p>}
                ko={<p>
                    경로를 가진 로봇 한 대는 풀린 문제다. 같은 통로를 공유하는 로봇 삼백 대의
                    창고는 그렇지 않다: 각자로서는 완벽한 경로들이 모이면 불가능해질 수 있다.
                    Multi-agent planning은 <em>공존하는</em> 경로들을 찾는 일이다.
                </p>}
            />

            <h2>{t("The Problem", "문제 정의")}</h2>
            <T
                en={<p>
                    Multi-Agent Path Finding (MAPF): given one shared map
                    and <InlineMath math="k"/> agents, each with its own start and goal, find a
                    collision-free path <em>per agent</em> such that no two agents occupy the same
                    cell at the same time (vertex conflict) or swap cells across an edge (edge
                    conflict). Solution quality is measured by <strong>sum of costs</strong> (total
                    of all path lengths) or <strong>makespan</strong> (when the last agent
                    arrives).
                </p>}
                ko={<p>
                    Multi-Agent Path Finding (MAPF): 공유 지도 하나와 각자 시작·목표를 가진{" "}
                    <InlineMath math="k"/> 개의 agent가 주어졌을 때, 어떤 두 agent도 같은 시각에
                    같은 셀을 차지하지 않고(vertex conflict) 간선을 사이에 두고 자리를 맞바꾸지
                    않도록(edge conflict), <em>agent 별</em> 충돌 없는 경로를 찾는다. 해의 품질은{" "}
                    <strong>sum of costs</strong>(전체 경로 길이 합) 또는{" "}
                    <strong>makespan</strong>(마지막 agent의 도착 시각)으로 잰다.
                </p>}
            />

            <SpaceTimeConflict/>

            <h2>{t("Why It Is Hard", "왜 어려운가")}</h2>
            <T
                en={<p>
                    The honest search space is the <em>joint</em> space: the state is the tuple of
                    all agent positions, so its size is roughly <InlineMath math="|V|^k"/> — a
                    20×20 grid that A* clears in milliseconds becomes, for ten agents, a state space
                    of ~<InlineMath math="10^{26}"/>. Optimal MAPF is NP-hard. Every practical
                    algorithm is a strategy for <em>not</em> searching that joint space naively,
                    trading between solution quality, completeness, and how much coupling between
                    agents it is willing to consider.
                </p>}
                ko={<p>
                    탐색 공간을 곧이곧대로 잡으면 <em>joint</em> 공간이 된다. 상태가 모든 agent 위치의 튜플이라
                    크기가 대략 <InlineMath math="|V|^k"/>다. A*가 밀리초에 끝내는 20×20
                    격자도 agent 열 대면 ~<InlineMath math="10^{26}"/> 상태 공간이 된다. 최적 MAPF는
                    NP-hard다. 실용적인 알고리즘은 전부 이 joint 공간을 순진하게 탐색하지{" "}
                    <em>않기</em> 위한 전략이며, 해 품질·완전성·agent 간 결합을 얼마나 고려할지를
                    맞바꾼다.
                </p>}
            />

            <h2>{t("Decoupled, Coupled, and In Between", "Decoupled, Coupled, 그리고 그 사이")}</h2>
            <T
                en={<>
                    <ul>
                        <li>
                            <strong>Decoupled — Prioritized A*.</strong> Order the agents; each plans
                            with single-agent A* in space-time, treating earlier agents' paths as
                            moving obstacles. Fast and scalable, but incomplete: a bad priority order
                            can paint later agents into corners.
                        </li>
                        <li>
                            <strong>Coupled — Joint-space A*.</strong> Search the joint space
                            directly. Complete and optimal, and hopeless beyond a handful of agents —
                            it exists here mostly as the baseline that explains why everything else
                            exists.
                        </li>
                        <li>
                            <strong>In between — CBS.</strong> Conflict-Based Search plans each agent
                            independently (low level), detects conflicts, and branches on
                            constraints "agent <InlineMath math="i"/> may not be at{" "}
                            <InlineMath math="v"/> at time <InlineMath math="t"/>" (high level).
                            Optimal, yet it only ever couples agents that actually conflict — the
                            standard modern approach.
                        </li>
                    </ul>
                </>}
                ko={<>
                    <ul>
                        <li>
                            <strong>Decoupled (Prioritized A*).</strong> agent에 순서를 매기고, 각자
                            앞선 agent들의 경로를 움직이는 장애물로 취급하며 시공간 A*로
                            계획한다. 빠르고 확장성 있지만 불완전하다: 우선순위를 잘못 매기면 뒤의
                            agent를 구석에 가둘 수 있다.
                        </li>
                        <li>
                            <strong>Coupled (Joint-space A*).</strong> joint 공간을 직접 탐색한다.
                            완전하고 최적이지만 agent 몇 대만 넘어도 감당이 안 된다. 여기서는 다른
                            접근들이 왜 필요한지 보여 주는 baseline 역할에 그친다.
                        </li>
                        <li>
                            <strong>그 사이 (CBS).</strong> Conflict-Based Search는 agent를 각자
                            계획하고(low level), conflict를 찾아 "agent <InlineMath math="i"/>는
                            시각 <InlineMath math="t"/>에 <InlineMath math="v"/>에 있을 수 없다"는
                            제약으로 분기한다(high level). 최적이면서도 실제로 충돌한 agent만
                            결합한다. 현대의 표준 접근이다.
                        </li>
                    </ul>
                </>}
            />

            <h2>{t("What Is Coming", "구현 예정")}</h2>
            <T
                en={<p>
                    Planned for this section: <strong>Prioritized A*</strong>,{" "}
                    <strong>Joint-space A*</strong>, and <strong>CBS</strong>, in C++ and Python,
                    with traces that carry per-agent events (<code>agent</code> field,{" "}
                    <code>constraint_added</code>, <code>conflict_found</code>) so the demos can
                    replay multi-robot runs the same way the single-robot pages do.
                </p>}
                ko={<p>
                    이 섹션의 구현 예정: <strong>Prioritized A*</strong>,{" "}
                    <strong>Joint-space A*</strong>, <strong>CBS</strong>를 C++/Python으로
                    구현한다. trace는 agent 별 이벤트(<code>agent</code> 필드,{" "}
                    <code>constraint_added</code>, <code>conflict_found</code>)를 담아, 단일 로봇
                    페이지와 같은 방식으로 다중 로봇 실행을 재생한다.
                </p>}
            />
        </>
    )
}

export default MultiAgent

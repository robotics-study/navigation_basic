"""Shared plumbing for roadmap-based sampling planners (PRM family).

PRM and PRM* differ only in their connection radius policy; both build an
undirected roadmap over sampled free states and then answer a shortest-path
query with Dijkstra over that graph. The roadmap data structure and the query
live here; each algorithm owns its own radius policy in its own module
(mirrors the ``_sampling`` split for the tree-based family).
"""

from __future__ import annotations

import heapq

from navigation.core.capabilities import Capability, SamplingSpace
from navigation.core.planner import GlobalPlanner
from navigation.core.trace import TraceRecorder
from navigation.core.types import Point

from ._sampling import near_points


class Roadmap:
    """Undirected graph over world Points: node list + weighted adjacency."""

    def __init__(self) -> None:
        self.nodes: list[Point] = []
        self.adj: list[list[tuple[int, float]]] = []

    def __len__(self) -> int:
        return len(self.nodes)

    def add_node(self, p: Point) -> int:
        idx = len(self.nodes)
        self.nodes.append(p)
        self.adj.append([])
        return idx

    def add_edge(self, a: int, b: int, cost: float) -> None:
        self.adj[a].append((b, cost))
        self.adj[b].append((a, cost))


def connect(
    space: SamplingSpace[Point],
    roadmap: Roadmap,
    idx: int,
    radius: float,
    recorder: TraceRecorder | None,
) -> None:
    """Wire ``idx`` to every earlier node within ``radius`` via a valid motion.

    Only earlier nodes are considered so each undirected edge is added once as the
    roadmap grows incrementally.
    """
    node = roadmap.nodes[idx]
    for j in near_points(space, roadmap.nodes, range(idx), node, radius):
        if space.is_motion_valid(roadmap.nodes[j], node):
            cost = space.distance(roadmap.nodes[j], node)
            roadmap.add_edge(idx, j, cost)
            if recorder is not None:
                recorder.edge_added(node, roadmap.nodes[j], cost)


def dijkstra(
    roadmap: Roadmap,
    start_idx: int,
    goal_idx: int,
    recorder: TraceRecorder | None,
) -> tuple[list[Point], float, int]:
    """Shortest path start→goal over the roadmap. Returns (path, cost, expanded)."""
    dist = [float("inf")] * len(roadmap)
    parent = [-1] * len(roadmap)
    dist[start_idx] = 0.0
    pq: list[tuple[float, int]] = [(0.0, start_idx)]
    expanded = 0
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        expanded += 1
        if recorder is not None:
            recorder.node_expanded(roadmap.nodes[u], d)
        if u == goal_idx:
            break
        for v, w in roadmap.adj[u]:
            nd = d + w
            if nd < dist[v]:
                dist[v] = nd
                parent[v] = u
                heapq.heappush(pq, (nd, v))
    if dist[goal_idx] == float("inf"):
        return [], 0.0, expanded
    path: list[Point] = []
    node = goal_idx
    while node != -1:
        path.append(roadmap.nodes[node])
        node = parent[node]
    path.reverse()
    return path, dist[goal_idx], expanded


class _RoadmapPlanner(GlobalPlanner[Point, "SamplingSpace[Point]"]):
    """Base for PRM-family planners: capability declaration + finish emit."""

    def required_capabilities(self) -> set[Capability]:
        return {Capability.SAMPLING_SPACE}

    def _sample_free(
        self, space: SamplingSpace[Point], roadmap: Roadmap, num_samples: int,
        recorder: TraceRecorder | None,
    ) -> None:
        """Draw ``num_samples`` collision-free nodes into the roadmap."""
        drawn = 0
        # Cap attempts so a nearly-full map cannot loop forever chasing free states.
        for _ in range(num_samples * 20):
            if drawn >= num_samples:
                break
            q = space.sample()
            if not space.is_state_valid(q):
                continue
            roadmap.add_node(q)
            drawn += 1
            if recorder is not None:
                recorder.sample_drawn(q)

    def _finish(
        self,
        recorder: TraceRecorder | None,
        success: bool,
        cost: float,
        expanded: int,
        num_nodes: int,
        runtime: float,
    ) -> None:
        if recorder is not None:
            recorder.planning_finished(
                success,
                {
                    "runtime_sec": runtime,
                    "path_cost": cost,
                    "expanded_nodes": float(expanded),
                    "samples": float(num_nodes),
                    "tree_size": float(num_nodes),
                },
            )

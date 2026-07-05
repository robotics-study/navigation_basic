---
title: Map representations
layout: default
---

# Map representations

Algorithms require a **capability interface**, not a concrete map type. Each map type implements the
capabilities it can support via adapters, so one map can be attached to many algorithms. Adding a new
map type does not change any algorithm code (OCP). The canonical format definitions live in the
repository's `spec/map_formats.md` — the single source of truth that both the C++ and Python loaders
read directly.

Common rules: every map file is identified by a top-level `type` field
(`occupancy_grid | graph | topology | continuous`, independent of extension/directory). World
coordinates are meters as float `(x, y)`, angles in radians; grid indices are `(row, col)` ints with
`row 0 = top of the image`.

## Capability model

| capability | key methods | required by |
|---|---|---|
| `DiscreteSpace` | `neighbors(state)`, `heuristic(a, b)` | BFS, Dijkstra, A*, CBS low-level |
| `SamplingSpace` | `sample()`, `is_state_valid`, `is_motion_valid`, `distance`, `steer` | RRT family |
| `ObstacleQuery` | `is_collision(footprint, pose)`, `distance_to_nearest(p)` | DWA, VFH, MPC |

Map type × capability support matrix:

| Map type | DiscreteSpace | SamplingSpace | ObstacleQuery | Status |
|---|:---:|:---:|:---:|:---:|
| OccupancyGrid2D | O (4/8-connected) | O | O | ✅ implemented |
| GraphMap | O | X | X | planned |
| TopologyMap | O (semantic edge) | X | X | planned |
| ContinuousMap | O via grid adapter | O | O | planned |

A planner declares `required_capabilities()`, and the bench/demo runner checks `map.supports(cap)`
before running. An incompatible combination (e.g. `GraphMap` × RRT) is reported as "incompatible"
rather than raised as an error. See [Architecture](architecture.md) for the design rationale.

## OccupancyGrid2D — occupancy grid ✅

ROS `map_server` style: a yaml file plus a grayscale image (pgm/png). `0 (black) = occupied`,
`255 (white) = free`.

```yaml
type: occupancy_grid
image: maze01.pgm
resolution: 0.05           # meters / pixel
origin: [0.0, 0.0, 0.0]    # world pose of the bottom-left pixel [x, y, theta]
occupied_thresh: 0.65      # (1 - pixel/255) >= this → occupied
free_thresh: 0.196         # (1 - pixel/255) <= this → free (in-between = unknown = impassable)
```

- **Coordinate frames**: world `(x, y)` float (meters) ↔ grid `(row, col)` int. Only the map class
  performs the conversion.
- **DiscreteSpace**: 8-connected moves (orthogonal 1.0, diagonal √2 × resolution). A diagonal is
  allowed only when both shared orthogonal cells are free, preventing corner cutting. The heuristic
  is the octile distance, which is admissible for this move model.
- **SamplingSpace / ObstacleQuery**: samples continuous world coordinates (not cell centers) and
  checks collisions by interpolating segments at the resolution step.
- Because it provides all three capabilities, [every implemented algorithm](algorithms/index.md) runs
  on this map — the benchmark maps `maze01` (narrow-corridor maze) and `open01` (open field) are of
  this type.

## GraphMap — explicit weighted graph (planned)

A graph with explicit nodes and edges, like a roadmap or road network. States are node-id strings.

```yaml
type: graph
directed: false
nodes:
  - { id: n1, pos: [1.0, 2.0] }   # pos = world coords for visualization/heuristic
edges:
  - { from: n1, to: n2 }          # euclidean over pos if cost is omitted
  - { from: n2, to: n3, cost: 7.5 }
```

- Provides **DiscreteSpace** only (heuristic = euclidean over `pos`). No sampling/obstacle support →
  incompatible with RRT / local planners.

## TopologyMap — topological map (planned)

Centered on places and connectivity. Unlike a graph, nodes are semantic units (rooms, junctions) and
edges carry semantic labels.

```yaml
type: topology
places:
  - { id: kitchen, name: "Kitchen", pos: [2.0, 3.0] }   # pos optional (visualization)
  - { id: hallway, name: "Hallway" }
connections:
  - { from: kitchen, to: hallway, cost: 1.0, label: door }
  - { from: hallway, to: lobby, cost: 3.0, label: corridor }
```

- Undirected, `cost` required (no euclidean fallback since geometry may be absent). Provides
  **DiscreteSpace**; euclidean heuristic if `pos` exists on all nodes, otherwise 0 (degenerates to
  Dijkstra).

## ContinuousMap — geometric obstacles (planned)

A list of circle/rectangle/polygon obstacles, for sampling-based and local planners.

```yaml
type: continuous
bounds: { x: [0.0, 10.0], y: [0.0, 10.0] }
obstacles:
  - { shape: circle, center: [3.0, 4.0], radius: 0.5 }
  - { shape: rectangle, center: [7.0, 2.0], size: [2.0, 1.0], theta: 0.0 }
  - { shape: polygon, vertices: [[1.0, 1.0], [2.0, 1.0], [1.5, 2.0]] }
```

- Provides **SamplingSpace / ObstacleQuery**. **DiscreteSpace** is available only through a
  gridding adapter (with a resolution parameter).

## Scenario — problem definition

The concrete problem to solve on a map. A single-agent scenario uses `start`/`goal`; a multi-agent
one uses `agents` (exactly one of the two).

```yaml
map: ../grid/maze01.yaml      # relative to the scenario file
start: [0.5, 0.5]            # grid/continuous: world coords · graph/topology: node id
goal: [9.5, 9.5]
# --- multi-agent ---
# agents:
#   - { start: [0.5, 0.5], goal: [9.5, 9.5] }
#   - { start: [9.5, 0.5], goal: [0.5, 9.5] }
```

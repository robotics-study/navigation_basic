---
title: Architecture
layout: default
parent: English
nav_order: 2
---

[🇰🇷 한국어](../ko/architecture.md) | [🇬🇧 English](architecture.md)

# Architecture
{: .no_toc }

1. TOC
{:toc}

## Repository Layout

```
.
├── spec/                        # language-neutral contracts (single source of truth, override implementations)
│   ├── trace_schema.json        #   JSON Schema for step-by-step trace events
│   ├── param_schema.json        #   schema for algorithm parameter declarations (name/type/range/default)
│   └── map_formats.md           #   map file format definitions (grid / graph / topology / continuous)
├── maps/                        # shared benchmark maps + start/goal scenarios
├── configs/                     # per-algorithm parameter yaml (shared across languages)
├── cpp/                         # C++20 implementation (include/nav_study + src + demos + GoogleTest)
├── python/                      # Python implementation (nav_study package + demos + pytest)
└── tools/                       # viz (trace replayer) + bench (matrix runner) — Python
```

C++ and Python are independent trees that implement **the same design, each in its own idiomatic style**. Class/method concept names, parameter names, and trace events are identical, so reading one side makes the other readable too. Everything shared across languages (trace schema, param yaml, map data, scenarios) must live under `spec/`, `configs/`, and `maps/` and be loaded from both sides.

## Dependency Directions

```
demos ──▶ algorithms ──▶ core ◀── maps ◀── tools/viz·bench
                          ▲                     │
                          └── spec (format contracts) ─┘
```

- `core` depends only on the stdlib (+ numpy / Eigen). It knows nothing about algorithm or map modules.
- `maps` depends only on `core`.
- Algorithm modules depend only on the abstract interfaces in `core`. **Direct references to concrete map classes are forbidden**, as are dependencies between algorithm modules.
- `tools/viz` and `tools/bench` depend only on the spec formats and the `core`/`maps` loaders. Access to algorithm internals is forbidden — every piece of information the visualization needs must be emitted as trace events.
- `demos` is the top-level assembly layer: it only wires together algorithms + maps + configs. No logic allowed.

## Capability Model

Algorithms require **capability interfaces**, not concrete map types. Each map type implements the capabilities it can support via adapters, so a single map can be plugged into multiple algorithms for testing.

| capability | Key methods | Required by |
|---|---|---|
| `DiscreteSpace` | `neighbors(state) -> [(state, cost)]`, `heuristic(a, b)` | BFS, Dijkstra, A*, CBS low-level |
| `SamplingSpace` | `sample()`, `is_state_valid(s)`, `is_motion_valid(a, b)`, `distance(a, b)`, `steer(a, b, eta)` | RRT family |
| `ObstacleQuery` | `is_collision(footprint, pose)`, `distance_to_nearest(p)` | Local planners such as DWA, VFH, MPC |

Map type × capability support matrix:

| Map type | DiscreteSpace | SamplingSpace | ObstacleQuery |
|---|:---:|:---:|:---:|
| `OccupancyGrid2D` [^elfes] | O (4/8-connected) | O | O |
| `GraphMap` | O | X | X |
| `TopologyMap` | O (semantic edge cost) | X | X |
| `ContinuousMap` | X (O via a discretization adapter) | O | O |

Each planner declares `required_capabilities()`, and the bench runner / demos check compatibility with `map.supports(capability)` before execution. Incompatible combinations are not errors — they are recorded as "incompatible" in the report.

The `DiscreteSpace` adapter of `OccupancyGrid2D` provides 8-connected motion (orthogonal 1.0 / diagonal √2 × resolution), and diagonal moves are allowed only when both adjacent orthogonal cells are free (to prevent corner cutting). The heuristic is the octile distance, which is admissible under this motion model.

## Trace — the Contract for Step-by-Step Visualization

Algorithms emit their search progress through a `TraceRecorder`. The event list and fields are a language-neutral contract defined by `spec/trace_schema.json`.

| Event | Meaning | Emitted by |
|---|---|---|
| `planning_started` | Run started (snapshot of algorithm, map, params) | All |
| `node_expanded` | Node settled into the closed set | BFS, Dijkstra, A* |
| `edge_added` | Edge added to the tree/search graph | All |
| `sample_drawn` | Random sample drawn | RRT family |
| `rewire` | Parent of an existing node replaced (cost improvement) | RRT*, Fast-RRT |
| `candidate_evaluated` | Candidate evaluated (for local planners) | DWA etc. (planned) |
| `constraint_added` / `conflict_found` | Constraint/conflict (for multi-agent) | CBS (planned) |
| `path_found` | (Improved) path found | All |
| `planning_finished` | Finished + common metrics | All |

- Traces are stored as JSON Lines files and replayed on top of the map by `tools/viz/replay.py`. The C++ demos emit the same format, so **there is a single visualization codebase**.
- Trace emission is off by default (for performance measurement) and on for demos/visualization. When the recorder is null in the hot loop, it is zero-cost.
- State representation: continuous spaces use `[x, y(, theta)]` world coordinates (float), grids use `[row, col]` (int), and graph/topology maps use node id strings. The world ↔ grid conversion is the sole responsibility of the map classes.

## Parameter Abstraction

- Each algorithm declares its own `ParamSet`: name, type, default value, valid range. The declaration format follows `spec/param_schema.json`.
- Values are loaded from `configs/<category>/<algorithm>.yaml` and validated against the declaration at load time (out of range → error). Parameters are never buried in the code as magic numbers.
- The same yaml is read as-is by both C++ and Python.

## References

[^elfes]: Elfes, A. (1989). "Using occupancy grids for mobile robot perception and navigation." *Computer*, 22(6), 46–57. [doi:10.1109/2.30720](https://doi.org/10.1109/2.30720)

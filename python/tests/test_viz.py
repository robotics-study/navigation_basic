"""tools/viz/replay.py: GIF animates (>1 frame) and snapshots are written.

Behavior-focused: we assert on the produced files, not matplotlib internals.
replay lives outside the package (tools/), so it is loaded by file path.
"""

from __future__ import annotations

import importlib.util
import sys
from pathlib import Path
from types import ModuleType
from typing import Any

import numpy as np
from conftest import REPO_ROOT
from PIL import Image

from navigation.maps.loader import load_map


def _load_replay() -> ModuleType:
    path = REPO_ROOT / "tools" / "viz" / "replay.py"
    spec = importlib.util.spec_from_file_location("replay", path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    # Register before exec so the dataclass can resolve its own module (forward refs).
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


replay = _load_replay()


def _synthetic_events() -> list[dict[str, object]]:
    return [
        {"seq": 0, "event": "planning_started", "algorithm": "astar",
         "map": "maps/grid/open01.yaml", "params": {}},
        {"seq": 1, "event": "node_expanded", "state": [18, 1]},
        {"seq": 2, "event": "edge_added", "state": [17, 2], "parent": [18, 1]},
        {"seq": 3, "event": "node_expanded", "state": [17, 2]},
        {"seq": 4, "event": "edge_added", "state": [16, 3], "parent": [17, 2]},
        {"seq": 5, "event": "path_found", "path": [[18, 1], [17, 2], [16, 3]]},
        {"seq": 6, "event": "planning_finished", "success": True,
         "metrics": {"runtime_sec": 0.0, "path_cost": 2.8, "expanded_nodes": 2.0}},
    ]


def _scene() -> Any:
    # replay is loaded by file path (ModuleType), so its Scene class cannot be
    # referenced statically; Any is the honest type here.
    grid = load_map(REPO_ROOT / "maps" / "grid" / "open01.yaml")
    return replay.build_scene(_synthetic_events(), grid)


def test_build_scene_counts_ops_and_path() -> None:
    scene = _scene()
    assert scene.total_ops == 4  # 2 node_expanded + 2 edge_added
    assert len(scene.path) == 3
    assert scene.algorithm == "astar"


def test_gif_is_animated(tmp_path: Path) -> None:
    out = str(tmp_path / "replay.gif")
    frames = replay.save_gif(_scene(), out, fps=6, events_per_frame=1)
    assert frames > 1
    with Image.open(out) as im:
        assert getattr(im, "n_frames", 1) > 1


def test_snapshots_written_and_sortable(tmp_path: Path) -> None:
    out_dir = str(tmp_path / "snaps")
    paths = replay.save_snapshots(_scene(), out_dir, count=4)
    assert len(paths) == 5  # 4 mid-search + 1 final
    assert all(Path(p).stat().st_size > 0 for p in paths)
    assert [Path(p).name for p in paths] == [f"snapshot_{i:03d}.png" for i in range(5)]


def test_gif_frames_reveal_path_progressively() -> None:
    # Search frames draw no path; the epilogue reveals a growing prefix ending full.
    frames = replay._gif_frames(total=100, cutoffs=[50, 100], n_path_segments=10, fps=10)
    search = [f for f in frames if f[1] is False]
    assert search and all(f[2] is None for f in search)
    revealed = [f[2] for f in frames if f[1] is True and f[2] is not None]
    assert revealed[-1] == 10  # reveal reaches the full path
    assert any(0 < r < 10 for r in revealed)  # a partially-drawn path frame exists
    assert frames[-1] == (100, True, None)  # holds the completed path
    hold = [f for f in frames if f == (100, True, None)]
    assert len(hold) == 10 * replay._HOLD_SECONDS  # completion hold = fps * _HOLD_SECONDS


def test_expanded_marks_darken_with_search_time() -> None:
    # The synthetic scene expands node 0 first and node 2 last; the sequential
    # ramp must render the earlier expansion brighter than the later one.
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import PathCollection

    fig, ax = plt.subplots()
    replay._draw(ax, _scene(), 4, show_path=False)  # cutoff = full search
    expanded = [
        c
        for c in ax.collections
        if isinstance(c, PathCollection) and np.asarray(c.get_offsets()).shape[0] == 2
    ]
    assert expanded, "expanded scatter not found"
    fc = expanded[0].get_facecolor()

    def luminance(rgba: object) -> float:
        r, g, b = rgba[0], rgba[1], rgba[2]  # type: ignore[index]
        return 0.2126 * r + 0.7152 * g + 0.0722 * b

    assert luminance(fc[0]) > luminance(fc[-1])  # earlier expansion is lighter
    plt.close(fig)


def test_path_gradient_draws_only_revealed_prefix() -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection

    path = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0), (3.0, 0.0)]  # 3 segments

    fig, ax = plt.subplots()
    replay._draw_path_gradient(ax, path, 2)  # reveal 2 of 3
    lines = [c for c in ax.collections if isinstance(c, LineCollection)]
    markers = [c for c in ax.collections if not isinstance(c, LineCollection)]
    assert lines and all(len(c.get_segments()) == 2 for c in lines)  # halo + gradient prefix
    assert len(markers) == 1  # start marker only; goal withheld until complete
    plt.close(fig)

    fig, ax = plt.subplots()
    replay._draw_path_gradient(ax, path, None)  # full path
    lines = [c for c in ax.collections if isinstance(c, LineCollection)]
    markers = [c for c in ax.collections if not isinstance(c, LineCollection)]
    assert all(len(c.get_segments()) == 3 for c in lines)
    assert len(markers) == 2  # start + goal
    plt.close(fig)

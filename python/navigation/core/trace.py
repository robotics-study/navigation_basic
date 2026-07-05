"""Step-by-step trace recorder — the contract for visualization.

Mirrors the C++ `core/trace.hpp`. Emits JSON Lines per `spec/trace_schema.json`.
`seq` starts at 0 and increments per event; `t` is seconds since planning_started.
A null recorder must cost nothing on the hot path: callers guard every emit with
``if recorder is not None`` so a None recorder never runs any of this code.
"""

from __future__ import annotations

import json
import time
from collections.abc import Sequence
from types import TracebackType
from typing import TextIO

from .params import ParamValue

# A serialized state is a numeric tuple -> JSON array (Cell ints, Point floats).
State = Sequence[float]


class TraceRecorder:
    def __init__(self, out: TextIO, owns: bool = False) -> None:
        self._out = out
        self._owns = owns
        self._seq = 0
        self._t0 = time.monotonic()

    def __enter__(self) -> TraceRecorder:
        return self

    def __exit__(
        self,
        exc_type: type[BaseException] | None,
        exc: BaseException | None,
        tb: TracebackType | None,
    ) -> None:
        self.close()

    def close(self) -> None:
        if self._owns:
            self._out.close()

    def _emit(self, event: str, fields: dict[str, object]) -> None:
        record: dict[str, object] = {"seq": self._seq, "t": round(time.monotonic() - self._t0, 6)}
        record["event"] = event
        record.update(fields)
        self._out.write(json.dumps(record, separators=(",", ":")) + "\n")
        self._seq += 1

    # --- events -----------------------------------------------------------
    def planning_started(
        self, algorithm: str, map_path: str, params: dict[str, ParamValue]
    ) -> None:
        self._emit("planning_started", {"algorithm": algorithm, "map": map_path, "params": params})

    def node_expanded(self, state: State, cost: float | None = None) -> None:
        fields: dict[str, object] = {"state": list(state)}
        if cost is not None:
            fields["cost"] = cost
        self._emit("node_expanded", fields)

    def edge_added(self, state: State, parent: State, cost: float | None = None) -> None:
        fields: dict[str, object] = {"state": list(state), "parent": list(parent)}
        if cost is not None:
            fields["cost"] = cost
        self._emit("edge_added", fields)

    def sample_drawn(self, state: State) -> None:
        self._emit("sample_drawn", {"state": list(state)})

    def rewire(self, state: State, parent: State) -> None:
        self._emit("rewire", {"state": list(state), "parent": list(parent)})

    def candidate_evaluated(self, state: State, cost: float) -> None:
        self._emit("candidate_evaluated", {"state": list(state), "cost": cost})

    def robot_moved(self, state: State) -> None:
        # Dynamic replanning (D* Lite): the robot's new executed cell.
        self._emit("robot_moved", {"state": list(state)})

    def obstacle_revealed(self, state: State) -> None:
        # Dynamic replanning (D* Lite): a cell newly sensed as blocked.
        self._emit("obstacle_revealed", {"state": list(state)})

    def path_found(self, path: Sequence[State]) -> None:
        self._emit("path_found", {"path": [list(s) for s in path]})

    def planning_finished(self, success: bool, metrics: dict[str, float]) -> None:
        self._emit("planning_finished", {"success": success, "metrics": metrics})


def open_trace(path: str) -> TraceRecorder:
    """Open ``path`` for writing and return a recorder that owns the file."""
    return TraceRecorder(open(path, "w", encoding="utf-8"), owns=True)

"""Step-by-step trace recorder — the contract for visualization.

Mirrors the C++ `core/trace.hpp`. Emits JSON Lines per `spec/trace_schema.json`.
`seq` starts at 0 and increments per event; `t` is seconds since planning_started.
A null recorder must cost nothing on the hot path: callers guard every emit with
``if recorder is not None`` so a None recorder never runs any of this code.
"""

from __future__ import annotations

import json
import time
from collections.abc import Mapping, Sequence
from types import TracebackType
from typing import TextIO

from .params import ParamValue

# A serialized state is a numeric tuple -> JSON array (Cell ints, Point floats).
State = Sequence[float]
# Optional algorithm-specific extra info attached to an event (spec/trace_schema.json
# `data`). Numeric only, mirroring the C++ recorder's std::map<string, double>; viz may
# read it or ignore it. Never put info here that a common field already carries.
EventData = Mapping[str, float]


def _add_data(fields: dict[str, object], data: EventData | None) -> None:
    # Empty data carries no information, so it is elided to keep traces (and the
    # C++ recorder, which omits an empty map) byte-identical when unused.
    if data:
        fields["data"] = dict(data)


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
        self,
        algorithm: str,
        map_path: str,
        params: dict[str, ParamValue],
        scenario: str | None = None,
    ) -> None:
        fields: dict[str, object] = {"algorithm": algorithm, "map": map_path, "params": params}
        # scenario is local-demo only (replay recovers reference_path/goal from it); omit
        # rather than emit null so existing global-planning traces stay byte-identical.
        if scenario is not None:
            fields["scenario"] = scenario
        self._emit("planning_started", fields)

    def node_expanded(
        self, state: State, cost: float | None = None, data: EventData | None = None
    ) -> None:
        fields: dict[str, object] = {"state": list(state)}
        if cost is not None:
            fields["cost"] = cost
        _add_data(fields, data)
        self._emit("node_expanded", fields)

    def edge_added(
        self, state: State, parent: State, cost: float | None = None, data: EventData | None = None
    ) -> None:
        fields: dict[str, object] = {"state": list(state), "parent": list(parent)}
        if cost is not None:
            fields["cost"] = cost
        _add_data(fields, data)
        self._emit("edge_added", fields)

    def sample_drawn(self, state: State, data: EventData | None = None) -> None:
        fields: dict[str, object] = {"state": list(state)}
        _add_data(fields, data)
        self._emit("sample_drawn", fields)

    def rewire(self, state: State, parent: State, data: EventData | None = None) -> None:
        fields: dict[str, object] = {"state": list(state), "parent": list(parent)}
        _add_data(fields, data)
        self._emit("rewire", fields)

    def candidate_evaluated(
        self,
        state: State,
        cost: float,
        data: EventData | None = None,
        rollout: Sequence[State] | None = None,
    ) -> None:
        # rollout (spec/trace_schema.json): predicted trajectory polyline for
        # rollout-scoring planners (DWA). A top-level array like `bins` because
        # the numeric-only `data` map cannot carry arrays; omitted when absent
        # so existing traces stay byte-identical.
        fields: dict[str, object] = {"state": list(state), "cost": cost}
        _add_data(fields, data)
        if rollout is not None:
            fields["rollout"] = [list(p) for p in rollout]
        self._emit("candidate_evaluated", fields)

    def robot_moved(self, state: State, data: EventData | None = None) -> None:
        # Dynamic replanning (D* Lite): the robot's new executed cell. Also reused by
        # the local-planning simulator for every closed-loop tick's executed pose,
        # where `data` optionally carries the (v, omega) command that produced it.
        fields: dict[str, object] = {"state": list(state)}
        _add_data(fields, data)
        self._emit("robot_moved", fields)

    def force_computed(self, state: State, data: EventData | None = None) -> None:
        # Potential Fields (Khatib 1986): attractive/repulsive/resultant force
        # components at the current pose, one event per control tick.
        fields: dict[str, object] = {"state": list(state)}
        _add_data(fields, data)
        self._emit("force_computed", fields)

    def histogram_updated(
        self, state: State, bins: Sequence[float], data: EventData | None = None
    ) -> None:
        # VFH (Borenstein & Koren 1991): polar obstacle-density histogram. `bins` is
        # an array, which the numeric-only `data` map cannot carry, so it is a
        # dedicated top-level field.
        fields: dict[str, object] = {"state": list(state), "bins": list(bins)}
        _add_data(fields, data)
        self._emit("histogram_updated", fields)

    def band_updated(self, band: Sequence[Sequence[float]], data: EventData | None = None) -> None:
        # Elastic Bands / TEB (spec/trace_schema.json `band`): the deformed
        # bubble-chain or timed-pose-chain state, one event per control tick. No
        # `state` field -- unlike candidate_evaluated/histogram_updated, the band
        # itself *is* the event's subject, not an annotation on a robot pose.
        fields: dict[str, object] = {"band": [list(b) for b in band]}
        _add_data(fields, data)
        self._emit("band_updated", fields)

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

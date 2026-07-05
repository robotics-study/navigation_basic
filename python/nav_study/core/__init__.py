"""Core abstractions: state types, capabilities, params, trace, planner base.

Depends only on stdlib + numpy. Knows nothing about concrete maps or algorithms.
"""

from .capabilities import Capability, DiscreteSpace, MapBase, SamplingSpace
from .params import ParamDecl, ParamError, ParamSet, ParamValue
from .planner import DiscretePlanner, GlobalPlanner, SamplingPlanner
from .trace import TraceRecorder, open_trace
from .types import Cell, PlanResult, PlanStats, Point, StateT

__all__ = [
    "Capability",
    "DiscreteSpace",
    "SamplingSpace",
    "MapBase",
    "ParamDecl",
    "ParamError",
    "ParamSet",
    "ParamValue",
    "GlobalPlanner",
    "DiscretePlanner",
    "SamplingPlanner",
    "TraceRecorder",
    "open_trace",
    "Cell",
    "Point",
    "PlanResult",
    "PlanStats",
    "StateT",
]

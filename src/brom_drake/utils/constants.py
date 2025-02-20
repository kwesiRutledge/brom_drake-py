import networkx as nx
import numpy as np
from pydrake.systems.framework import LeafSystem, Diagram
from pydrake.systems.primitives import VectorLogSink
from typing import Tuple, Union

Performer = Union[LeafSystem, Diagram]

# A MotionPlan is either a networkx DiGraph or a numpy array
# The MotionPlan is either a graph that CONTAINS the plan (DiGraph) or
# a path in the configuration space (numpy array)
MotionPlan = Union[nx.DiGraph, np.ndarray]

MotionPlanningResult = Union[
    Tuple[MotionPlan, int],
    MotionPlan,
]

SupportedLogger = Union[VectorLogSink]
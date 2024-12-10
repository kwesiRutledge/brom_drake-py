import networkx as nx
import numpy as np
from pydrake.systems.framework import LeafSystem, Diagram
from typing import Union

Performer = Union[LeafSystem, Diagram]
MotionPlan = Union[nx.DiGraph, np.ndarray]
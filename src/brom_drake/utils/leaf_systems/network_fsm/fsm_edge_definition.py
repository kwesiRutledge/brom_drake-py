from dataclasses import dataclass
from enum import IntEnum
import networkx as nx
import numpy as np
from typing import List, Union

# Internal Imports
from brom_drake.utils.leaf_systems.network_fsm.fsm_transition_condition import FSMTransitionCondition

class FSMEdgeDefinition:
    """
    Description
    """
    def __init__(
        self,
        conditions: List[FSMTransitionCondition],
        src: int,
        dst: int,
    ):
        # Setup

        # Input Checking
        assert len(conditions) > 0, "At least one condition must be provided."

        # Assign Attributes
        self.conditions = conditions
        self.src = src
        self.dst = dst


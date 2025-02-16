from dataclasses import dataclass
from enum import IntEnum
import numpy as np
from typing import List, Union

class FSMTransitionConditionType(IntEnum):
    """
    Enum to represent the different types of conditions that can be used to
    transition between states in a finite state machine.
    """
    kEqual = 0
    kGreaterThanEqual = 1
    kLessThanEqual = 2
    kAfterThisManySeconds = 3

class FSMTransitionCondition:
    """
    Dataclass to represent a single condition that can be used to transition
    between states in a finite state machine.
    """
    def __init__(
        self,
        input_port_name: Union[str, None],
        condition_type: FSMTransitionConditionType,
        condition_value: Union[bool, float, np.ndarray]
    ):
        """
        Description
        -----------
        Constructor for the FSMTransitionCondition class.
        """
        # Setup

        # Input Checking
        assert condition_type in FSMTransitionConditionType, "Invalid condition type provided."

        # Assign Attributes
        self.input_port_name = input_port_name
        self.type = condition_type
        self.condition_value = condition_value

        # Check that the condition contains the input_port_name IF the condition type
        # is not kAfterThisManySeconds
        if self.type != FSMTransitionConditionType.kAfterThisManySeconds:
            assert self.input_port_name is not None, \
                f"Input port name must be provided for condition type \"{self.type}\"."
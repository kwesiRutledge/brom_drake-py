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
        condition_type: FSMTransitionConditionType,
        condition_value: Union[bool, float, np.ndarray],
        input_port_name: Union[str, None] = None,
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
            
        # Check that the condition value is a positive float IF the condition type is
        # kAfterThisManySeconds
        if self.type == FSMTransitionConditionType.kAfterThisManySeconds:
            assert isinstance(self.condition_value, float), \
                f"Condition value must be a float for condition type \"{self.type}\"."
            assert self.condition_value > 0.0, \
                f"Condition value must be a positive float for condition type \"{self.type}\"."
            
    def evaluate_comparison(self, input_port_value: Union[bool, float, np.ndarray]):
        """
        Description
        -----------
        This function evaluates the condition based on the input port value.
        """
        # Setup
        input_x = input_port_value
        if (type(input_x) == int) or (type(input_x) == float):
            input_x = np.ndarray([input_x])


        # Input Checking
        if self.type == FSMTransitionConditionType.kAfterThisManySeconds:
            raise NotImplementedError(
                f"Condition type \"kAfterThisManySeconds\" can not yet be evaluated yet implemented."
            )

        # Evaluate the condition for array types
        if type(input_x) == np.ndarray:
            if self.type == FSMTransitionConditionType.kEqual:
                return np.all(input_x == self.condition_value)
            elif self.type == FSMTransitionConditionType.kGreaterThanEqual:
                return np.all(input_x >= self.condition_value)
            elif self.type == FSMTransitionConditionType.kLessThanEqual:
                return np.all(input_x <= self.condition_value)
            else:
                raise NotImplementedError("Condition type not yet implemented.")
        else:
            # Fallback for all other types of comparisons
            if self.type == FSMTransitionConditionType.kEqual:
                return input_port_value == self.condition_value
            elif self.type == FSMTransitionConditionType.kGreaterThanEqual:
                return input_port_value >= self.condition_value
            elif self.type == FSMTransitionConditionType.kLessThanEqual:
                return input_port_value <= self.condition_value
            else:
                raise NotImplementedError("Condition type not yet implemented.")
            
    def requires_input_port(self):
        """
        Description
        -----------
        This function returns whether or not the condition requires an input port.
        """
        return self.type != FSMTransitionConditionType.kAfterThisManySeconds
from dataclasses import dataclass
import numpy as np
from typing import Union

@dataclass
class FSMOutputDefinition:
    """
    Dataclass to represent the output of a state in a finite state machine.
    """
    output_port_name: str
    output_port_value: Union[bool, float, np.ndarray]

    #TODO(kwesi): Eventually it would be cool to add support for
    #             output_port_value to be a function that can be called.
    #             This would allow for more dynamic outputs and can be
    #             checked with the "callable" method.

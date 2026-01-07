"""
DiagramTarget.py
Description:

    Creates a DiagramTarget class that can be used to monitor the state of a Diagram.
"""
from typing import NamedTuple, List, Union

class DiagramTarget(NamedTuple):
    """
    *Description*
    
    The target on a Diagram to record data from. 
    The object should define both:
    
    1. The LeafSystem's name, and
    2. The ports to monitor (if None, all ports are monitored).

    *Parameters*
    
    name: str
        Name of the LeafSystem to monitor

    ports: Union[None, List[int]]
        List of port indices to monitor. If None, all ports are monitored

    *Usage*

    .. code-block:: python

        from brom_drake.DiagramTarget import DiagramTarget

        # Creating a DiagramTarget to monitor all ports of a system named "my_system"
        target_all_ports = DiagramTarget(name="my_system")

        # Creating a DiagramTarget to monitor specific ports of a system named "another_system"
        target_specific_ports = DiagramTarget(name="another_system", ports=[0, 2, 4])

    """
    name: str #: Name of the LeafSystem to monitor
    ports: Union[None, List[int]] = None #: List of port indices to monitor. If None, all ports are monitored

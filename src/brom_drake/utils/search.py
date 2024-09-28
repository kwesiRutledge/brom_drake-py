from pydrake.systems.framework import DiagramBuilder
from typing import List

# Internal Imports
from .constants import Performer

def find_all_systems_with_output_port(
    builder: DiagramBuilder,
    target_port_name: str,
) -> List[Performer]:
    """
    Description
    -----------
    This method looks through the builder and finds all systems that have
    an output port with the given name.
    :param builder:
    :param target_port_name:
    :return:
    """
    # Setup
    potential_performers = []

    # Algorithm
    for system in builder.GetSystems():
        try:
            output_port = system.GetOutputPort(target_port_name)
            potential_performers.append(system)
        finally:
            continue

    return potential_performers

def find_all_systems_with_input_port(
    builder: DiagramBuilder,
    target_port_name: str,
) -> List[Performer]:
    """
    Description
    -----------
    This method looks through the builder and finds all systems that have
    an input port with the given name.
    :param builder:
    :param target_port_name:
    :return:
    """
    # Setup
    potential_performers = []

    # Algorithm
    for system in builder.GetSystems():
        try:
            input_port = system.GetInputPort(target_port_name)
            potential_performers.append(system)
        finally:
            continue

    return potential_performers
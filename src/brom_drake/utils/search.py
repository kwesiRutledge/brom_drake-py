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
    an output port with the given name. Systems without a matching port
    are skipped.

    :param builder: Builder whose registered systems will be searched.
    :type builder: DiagramBuilder
    :param target_port_name: Name of the output port to look for.
    :type target_port_name: str
    :return: Every system in the builder that has an output port named
        ``target_port_name``. The list is empty if there are none.
    :rtype: List[Performer]
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
    an input port with the given name. Systems without a matching port
    are skipped.

    :param builder: Builder whose registered systems will be searched.
    :type builder: DiagramBuilder
    :param target_port_name: Name of the input port to look for.
    :type target_port_name: str
    :return: Every system in the builder that has an input port named
        ``target_port_name``. The list is empty if there are none.
    :rtype: List[Performer]
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


def find_all_systems_with_name(
    builder: DiagramBuilder,
    target_system_name: str,
) -> List[Performer]:
    """
    Description
    -----------
    This method looks through the builder and finds all systems that have
    a name (as returned by ``get_name()``) equal to the given name.

    :param builder: Builder whose registered systems will be searched.
    :type builder: DiagramBuilder
    :param target_system_name: Target name of the system that we would like to find.
    :type target_system_name: str
    :return: Every system in the builder whose name matches exactly. The list
        is empty if there are none.
    :rtype: List[Performer]
    """
    # Setup
    potential_performers = []

    # Algorithm
    for system in builder.GetSystems():
        if system.get_name() == target_system_name:
            potential_performers.append(system)

    return potential_performers

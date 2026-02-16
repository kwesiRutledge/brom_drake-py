"""
errors.py
Description:

    Contains the custom errors for the DiagramWatcher module.
"""

from typing import List

from brom_drake.watchers.diagram_watcher import constants
from brom_drake.watchers.diagram_target import DiagramTarget


class PortIsNotFoundInDiagramError(ValueError):
    """
    **Description**

    Raised when the DiagramWatcher is attempting to add a port to its tracking but does not find it in the Diagram.
    """

    def __init__(
        self,
        target: DiagramTarget,
        port_reference: int | str,
        port_names: List[str] = None,
    ):
        self.target = target
        self.port_reference = port_reference
        self.message = f'Port "{port_reference}" on system with name {target.name} was not found in the Diagram.\n'
        if port_names is not None:
            self.message += f"Available ports are:\n"
            for port_index, port_name in enumerate(port_names):
                self.message += f"\t- {port_index}: {port_name}\n"

        super().__init__(self.message)


class SystemIsNotFoundInDiagramError(ValueError):
    """
    **Description**

    Raised when the DiagramWatcher is attempting to add a system to its tracking but does not find it in the Diagram.
    """

    def __init__(self, target: DiagramTarget, system_names: List[str] = None):
        self.target = target
        self.message = (
            f'System with name "{target.name}" was not found in the Diagram.\n'
        )
        if system_names is not None:
            self.message += f"Available systems are:\n"
            for system_index, system_name in enumerate(system_names):
                self.message += f"\t- {system_index}: {system_name}\n"

        super().__init__(self.message)


class PortIsNotBeingWatchedError(ValueError):
    """
    **Description**

    Raised when someone requests a port's information from the DiagramWatcher and that port is not being watched.
    """

    def __init__(
        self,
        target: DiagramTarget,
        port_reference: int | str,
        port_names: List[str] = None,
    ):
        self.target = target
        self.message = f'Port "{port_reference}" on system with name {target.name} is not being watched by the DiagramWatcher.\n'
        if port_names is not None:
            self.message += f"Available ports are:\n"
            for port_index, port_name in enumerate(port_names):
                self.message += f"\t- {port_index}: {port_name}\n"

        super().__init__(self.message)


class SystemIsNotBeingWatchedError(ValueError):
    """
    **Description**

    Raised when someone requests a system's information from the DiagramWatcher and that system is not being watched.
    """

    def __init__(self, target: DiagramTarget, system_names: List[str] = None):
        self.target = target
        self.message = f'System with name "{target.name}" is not being watched by the DiagramWatcher.\n'
        if system_names is not None:
            self.message += f"Available systems are:\n"
            for system_index, system_name in enumerate(system_names):
                self.message += f"\t- {system_index}: {system_name}\n"

        self.message += f'If system with name "{target.name}" does exist, it may be one of the ineligible types:\n '
        self.message += f"\t{constants.INELIGIBLE_SYSTEM_TYPES}.\n"

        super().__init__(self.message)

"""
errors.py
Description:

    Contains the custom errors for the DiagramWatcher module.
"""
from typing import List

from brom_drake.watchers.diagram_watcher import constants
from brom_drake.watchers.diagram_target import DiagramTarget


class UnrecognizedTargetError(ValueError):
    """
    *Description*

    Raised when a system is not recognized by the DiagramWatcher.
    """
    def __init__(self, target: DiagramTarget, system_names: List[str] = None):
        self.target = target
        self.message = f"Target with name {target.name} is not recognized by the DiagramWatcher.\n"
        if system_names is not None:
            self.message += f"(Available systems are {system_names})\n"

        self.message += f"If system with name {target.name} does exist, it may be one of the ineligible types:\n "
        self.message += f"\t{constants.INELIGIBLE_SYSTEM_TYPES}.\n"

        super().__init__(self.message)


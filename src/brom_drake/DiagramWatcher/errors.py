"""
errors.py
Description:

    Contains the custom errors for the DiagramWatcher module.
"""
from typing import List

from .constants import INELIGIBLE_SYSTEM_TYPES
from brom_drake import DiagramTarget


class UnrecognizedTargetError(ValueError):
    """
    UnrecognizedSystemError
    Description:

        Raised when a system is not recognized by the DiagramWatcher.
    """
    def __init__(self, target: DiagramTarget, system_names: List[str] = None):
        self.target = target
        self.message = f"Target with name {target.name} is not recognized by the DiagramWatcher.\n"
        if system_names is not None:
            self.message += f"(Available systems are {system_names})\n"

        super().__init__(self.message)


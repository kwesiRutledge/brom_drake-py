"""
gripper_target.py
Description:

    This file defines the GripperTarget enum and associated values.
"""

from enum import Enum


class GripperTarget(Enum):
    kPosition = 1
    kVelocity = 2
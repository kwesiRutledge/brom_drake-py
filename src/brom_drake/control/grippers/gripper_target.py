"""
gripper_target.py
Description:

    This file defines the GripperTarget enum and associated values.
"""

from enum import IntEnum


class GripperTarget(IntEnum):
    kPosition = 1
    kVelocity = 2
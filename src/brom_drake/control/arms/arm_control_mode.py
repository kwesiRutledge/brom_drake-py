"""
The file defining the ArmControlMode enum.
"""

from enum import IntEnum

class ArmControlMode(IntEnum):
    """
    An enum describing the multiple control types that can be used for an
    arm in Brom.

    Options are:
        kJoint or
        kEndEffector

    Usage:

        arm_control_mode = ArmControlMode.kJoint
    """
    kJoint = 0
    kEndEffector = 1
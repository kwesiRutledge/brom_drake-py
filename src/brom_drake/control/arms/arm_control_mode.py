"""
The file defining the ArmControlMode enum.
"""

from enum import IntEnum

class ArmControlMode(IntEnum):
    """
    *Description*

    An enum describing the multiple control types that can be used for an
    arm in Brom.

    Options are:
        kJoint or
        kEndEffector

    *Usage*
    
    .. code:: python

        from brom_drake.control.arms.arm_control_mode import ArmControlMode
        # or
        # from brom_drake.all import ArmControlMode
    
        # Selecting the joint control mode
        arm_control_mode = ArmControlMode.kJoint
    """
    kJoint = 0
    kEndEffector = 1
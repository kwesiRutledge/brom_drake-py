"""
joint_target.py
Description:

    This file defines the JointTarget enum.
    This class is used to control different manipulators and helps us
    define the type of target that we want to control.
"""
from enum import IntEnum

class JointTarget(IntEnum):
    """
    *Description*
    The type of target being given for the control of a robot's joints.

    *Usage*

    .. code-block:: python

        from brom_drake.control.arms.joint_target import JointTarget
        # or
        # from brom_drake.all import JointTarget

        # Selecting the position target type
        joint_target_type = JointTarget.kPosition
    """
    kPosition = 1
    kVelocity = 2
    kTorque = 3
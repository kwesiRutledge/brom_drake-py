"""
end_effector_target.py
Description:

    An enum useful for defining the type of target we're giving to a given manipulator.
"""

from enum import Enum

class EndEffectorTarget(Enum):
    """
    *Description*
    The type of target being given for the control of a robot's end effector.

    *Usage*

    .. code-block:: python
    
        from brom_drake.control.arms.end_effector_target import EndEffectorTarget
        # or
        # from brom_drake.all import EndEffectorTarget

        # Selecting the pose target type
        ee_target_type = EndEffectorTarget.kPose
    """
    kPose = 1   # Pose (position and orientation) expressed as a 7-element vector
                # of the End Effector w.r.t. the base of the arm.
    kTwist = 2
    kWrench = 3

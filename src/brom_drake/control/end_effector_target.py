"""
end_effector_target.py
Description:

    An enum useful for defining the type of target we're giving to a given manipulator.
"""

from enum import Enum


class EndEffectorTarget(Enum):
    kPose = 1   # Pose (position and orientation) expressed as a 7-element vector
                # of the End Effector w.r.t. the base of the arm.
    kTwist = 2
    kWrench = 3

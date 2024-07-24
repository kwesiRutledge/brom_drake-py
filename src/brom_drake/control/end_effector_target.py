"""
end_effector_target.py
Description:

    An enum useful for defining the type of target we're giving to a given manipulator.
"""

from enum import Enum


class EndEffectorTarget(Enum):
    kPose = 1
    kTwist = 2
    kWrench = 3

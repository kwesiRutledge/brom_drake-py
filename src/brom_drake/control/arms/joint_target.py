"""
joint_target.py
Description:

    This file defines the JointTarget enum.
    This class is used to control different manipulators and helps us
    define the type of target that we want to control.
"""
from enum import IntEnum

class JointTarget(IntEnum):
    kPosition = 1
    kVelocity = 2
    kTorque = 3
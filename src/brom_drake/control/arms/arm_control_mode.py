from enum import IntEnum


class ArmControlMode(IntEnum):
    """
    Description:
        An enumeration of the different control modes that the arm can be in.
    """
    kJoint = 0
    kEndEffector = 1
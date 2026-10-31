from enum import IntEnum


class GraspSelection1Phase(IntEnum):
    """
    *Description*

    An enum describing the different phases of the GraspSelection1
    with puppeteer wrist production.

    Options are:
        kObjectSettlingOnFloor
        kGripperApproach
        kGripperClosing
        kObjectSettlingInGrasp
        kFloorDrop

    *Usage*

    .. code:: python

        from brom_drake.productions.grasping.phases import GraspSelection1Phase
        # or
        # from brom_drake.all import GraspSelection1Phase

        # Selecting the gripper approach phase
        phase = GraspSelection1Phase.kGripperApproach
    """

    kObjectSettlingOnFloor = 1
    kGripperApproach = 2
    kGripperClosing = 3
    kObjectSettlingInGrasp = 4
    kFloorDrop = 5

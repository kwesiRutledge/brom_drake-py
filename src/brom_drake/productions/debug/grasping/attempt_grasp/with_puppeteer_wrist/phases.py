from enum import IntEnum

class AttemptGraspWithPuppeteerWristPhase(IntEnum):
    """
    *Description*
    
    An enum describing the different phases of the attempt grasp
    with puppeteer wrist production.

    Options are:
        kObjectSettlingOnFloor
        kGripperApproach
        kGripperClosing
        kObjectSettlingInGrasp
        kFloorDrop

    *Usage*

    .. code:: python

        from brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.phases import AttemptGraspWithPuppeteerWristPhase
        # or
        # from brom_drake.all import AttemptGraspWithPuppeteerWristPhase

        # Selecting the gripper approach phase
        phase = AttemptGraspWithPuppeteerWristPhase.kGripperApproach
    """
    kObjectSettlingOnFloor = 1
    kGripperApproach = 2
    kGripperClosing = 3
    kObjectSettlingInGrasp = 4
    kFloorDrop = 5
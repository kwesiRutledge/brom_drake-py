from enum import IntEnum

class AttemptGraspWithPuppeteerWristPhase(IntEnum):
    kObjectSettlingOnFloor = 1
    kGripperApproach = 2
    kGripperClosing = 3
    kObjectSettlingInGrasp = 4
    kFloorDrop = 5
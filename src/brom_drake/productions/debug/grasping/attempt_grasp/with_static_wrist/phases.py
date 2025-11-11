from enum import IntEnum

class AttemptGraspWithStaticWristPhase(IntEnum):
    kObjectSettlingOnFloor = 1
    kGripperClosing = 2
    kObjectSettlingInGrasp = 3
    kFloorDrop = 4
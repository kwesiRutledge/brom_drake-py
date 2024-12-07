from enum import IntEnum

class DispenserInternalState(IntEnum):
    kReady = 0
    kPlanSet = 1

class DispenserTransitionRequest(IntEnum):
    kNone = 0
    kRequestSavePlan = 1
    kRequestReset = 2
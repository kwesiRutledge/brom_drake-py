from enum import IntEnum

class DispenserInternalState(IntEnum):
    kReady = 0
    kPlanSet = 1

class DispenserTransitionRequest(IntEnum):
    kNone = 0               # Do not request to a new internal state
    kRequestSavePlan = 1    # Request to save the current plan into memory; Requires that the planner be in the kReady state.
    kRequestReset = 2       # Request to reset the dispenser; Requires that the planner be in the kPlanSet state.
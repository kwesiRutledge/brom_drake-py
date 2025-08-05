from enum import IntEnum

class PickAndPlacePhase(IntEnum):
    kPreGrasp = 1
    kGrasp = 2
    kPostGrasp = 3
    kPrePlace = 4
    kPlace = 5
    kPostPlace = 6

    def to_str(self) -> str:
        match self:
            case PickAndPlacePhase.kPreGrasp:
                return "pre_grasp"
            case PickAndPlacePhase.kGrasp:
                return "grasp"
            case PickAndPlacePhase.kPostGrasp:
                return "post_grasp"
            case PickAndPlacePhase.kPrePlace:
                return "pre_place"
            case PickAndPlacePhase.kPlace:
                return "place"
            case PickAndPlacePhase.kPostPlace:
                return "post_place"
            case _:
                raise NotImplementedError(
                    f"The string for PickAndPlacePhase \"{self}\" does not exist yet. Create an issue on GitHub to see this addressed!"
                )
            
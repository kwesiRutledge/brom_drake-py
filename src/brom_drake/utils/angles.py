from enum import IntEnum

class RollPitchYawAngle(IntEnum):
    kRoll = 0
    kPitch = 1
    kYaw = 2

    def __str__(self) -> str:
        """
        *Description*
        
        This method returns a string representation of the RollPitchYawAngle enum.
        """
        if self == RollPitchYawAngle.kRoll:
            return "Roll"
        elif self == RollPitchYawAngle.kPitch:
            return "Pitch"
        elif self == RollPitchYawAngle.kYaw:
            return "Yaw"
        else:
            raise ValueError(f"Invalid RollPitchYawAngle value: {self}")

def rpy_equivalent_body_rotation_order() -> list[RollPitchYawAngle]:
    """
    *Description*
    
    This method returns the equivalent body rotation order for Roll-Pitch-Yaw angles.
    """
    return [
        RollPitchYawAngle.kYaw,
        RollPitchYawAngle.kPitch,
        RollPitchYawAngle.kRoll,
    ]
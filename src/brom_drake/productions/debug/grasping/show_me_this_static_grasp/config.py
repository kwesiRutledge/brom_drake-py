from dataclasses import dataclass
from typing import List

@dataclass
class Configuration:
    meshcat_port_number: int = 7001
    time_step: float = 1e-3
    show_collision_geometries: bool = False
    target_body_on_gripper: str = None # The "Gripper" Frame that we use to define X_GripperObject
    gripper_color: List[float] = None
    show_gripper_base_frame: bool = False
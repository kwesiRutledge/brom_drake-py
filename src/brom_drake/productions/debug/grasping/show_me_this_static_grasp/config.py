from dataclasses import dataclass
from typing import List

from brom_drake.productions.types.base.configuration import Configuration as BaseConfiguration 

@dataclass
class Configuration(BaseConfiguration):
    show_collision_geometries: bool = False
    target_body_on_gripper: str = None # The "Gripper" Frame that we use to define X_GripperObject
    gripper_color: List[float] = None
    show_gripper_base_frame: bool = False
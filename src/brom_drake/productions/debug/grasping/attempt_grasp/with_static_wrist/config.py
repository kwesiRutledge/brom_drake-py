from dataclasses import dataclass, field
import numpy as np
from typing import List, Union

# Internal Imports
from brom_drake.productions.types.base.configuration import Configuration as BaseConfiguration
from .script import Script as AttemptGraspScript

@dataclass
class Configuration:
    base: BaseConfiguration = field(default_factory=BaseConfiguration) #By default, use the standard script
    show_collision_geometries: bool = False
    initial_gripper_joint_positions: Union[List[float], np.ndarray] = None
    target_body_on_gripper: str = None # The "Gripper" Frame that we use to define X_GripperObject
    gripper_color: List[float] = None
    show_gripper_base_frame: bool = False
    script: AttemptGraspScript = field(default_factory=AttemptGraspScript) #By default, use the standard script

    @property
    def meshcat_port_number(self):
        return self.base.meshcat_port_number

    @meshcat_port_number.setter
    def meshcat_port_number(self, new_value: int):
        self.base.meshcat_port_number = new_value
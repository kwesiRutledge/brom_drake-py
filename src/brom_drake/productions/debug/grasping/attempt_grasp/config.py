from dataclasses import dataclass, field
import numpy as np
from typing import List, Union

# Internal Imports
from .script import Script as AttemptGraspScript

@dataclass
class Configuration:
    meshcat_port_number: int = 7001
    time_step: float = 1e-3
    show_collision_geometries: bool = False
    initial_gripper_joint_positions: Union[List[float], np.ndarray] = None
    target_body_on_gripper: str = None # The "Gripper" Frame that we use to define X_GripperObject
    gripper_color: List[float] = None
    show_gripper_base_frame: bool = False
    script: AttemptGraspScript = field(default_factory=AttemptGraspScript) #By default, use the standard script
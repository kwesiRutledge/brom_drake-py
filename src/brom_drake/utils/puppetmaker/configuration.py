from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    Frame
)

@dataclass
class Configuration:
    frame_on_parent: Frame
    name: str
    frame_on_child: Frame = None
    sphere_radius: float = 0.1
    sphere_color: np.ndarray = None
    create_actuators_on_init: bool = True

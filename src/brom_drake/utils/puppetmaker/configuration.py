from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    Frame
)

@dataclass
class Configuration:
    frame_on_parent: Frame
    name: str
    frame_on_child: Frame
    sphere_radius: float = 0.1
    sphere_color: np.ndarray = None
    
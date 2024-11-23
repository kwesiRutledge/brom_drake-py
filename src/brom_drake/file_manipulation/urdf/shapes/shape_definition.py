from dataclasses import dataclass
from enum import IntEnum
import numpy as np
from typing import Union
import xml.etree.ElementTree as ET


class ShapeEnum(IntEnum):
    """
    The different simple shapes that can be created.
    """
    kBox = 1
    kSphere = 2
    kCylinder = 3
    kCapsule = 4

# Shape Definition Can be any one of these
@dataclass
class ShapeDefinition:
    def add_geometry_to_element(self, target_element: ET.Element):
        raise NotImplementedError("This method must be implemented in the subclass.")

    @property
    def type(self):
        raise NotImplementedError("This method must be implemented in the subclass.")

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

@dataclass
class SphereDefinition(ShapeDefinition):
    radius: float

    def add_geometry_to_element(self, target_element: ET.Element):
        return ET.SubElement(
            target_element,
            "sphere",
            {"radius": f"{self.radius}"},
        )

    @property
    def type(self):
        return ShapeEnum.kSphere



@dataclass
class BoxDefinition(ShapeDefinition):
    size: Union[float, list, tuple, np.ndarray]

    def add_geometry_to_element(self, target_element: ET.Element):
        # Setup

        # Convert the size to a numpy array
        if isinstance(self.size, (list, tuple, np.ndarray)):
            self.size = np.array(self.size)
        elif isinstance(self.size, float):
            self.size = np.array([self.size, self.size, self.size])
        else:
            raise ValueError(f"Size {self.size} (type \"{type(self.size)}\") not supported.")

        # Return the element
        return ET.SubElement(
            target_element,
            "box",
            {"size": f"{self.size[0]} {self.size[1]} {self.size[2]}"},
        )

    @property
    def type(self):
        return ShapeEnum.kBox
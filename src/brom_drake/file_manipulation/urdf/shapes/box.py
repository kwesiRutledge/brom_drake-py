from dataclasses import dataclass
import numpy as np
from typing import Union
import xml.etree.ElementTree as ET

# Internal Import
from .shape_definition import ShapeDefinition, ShapeEnum

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
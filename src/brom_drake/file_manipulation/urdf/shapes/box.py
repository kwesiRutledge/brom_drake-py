from dataclasses import dataclass
import numpy as np
from typing import Union
import xml.etree.ElementTree as ET

# Internal Import
from .shape_definition import ShapeDefinition, ShapeEnum

@dataclass
class BoxDefinition(ShapeDefinition):
    """
    *Description*

    Defines a box shape as per URDF specifications.

    *Attributes*

    size: Union[float, list, tuple, np.ndarray]
        The size of the box along the x, y, and z axes.
        If this is a float, it is assumed to be a cube with equal sides.
    """
    size: Union[float, list, tuple, np.ndarray]

    def add_geometry_to_element(self, target_element: ET.Element) -> ET.Element:
        """
        *Description*

        Adds the box geometry as a sub-element to the given target element.

        This method is an implementation of the abstract method defined in the
        ShapeDefinition base class.

        *Parameters*

        target_element: xml.etree.ElementTree.Element
            The target XML element to which the box geometry will be added.

        *Returns*

        xml.etree.ElementTree.Element
            The element `target_element` with the box geometry added as a sub-element.
        """
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
    def type(self) -> ShapeEnum:
        """
        *Description*

        Always returns ShapeEnum.kBox to indicate this is a box shape.

        This method is an implementation of the abstract method defined in the
        ShapeDefinition base class.

        *Returns*

        ShapeEnum
            The shape type, which is always ShapeEnum.kBox for this class.
        """
        return ShapeEnum.kBox
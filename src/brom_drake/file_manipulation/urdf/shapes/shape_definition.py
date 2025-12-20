from dataclasses import dataclass
from enum import IntEnum
import numpy as np
from typing import Union
import xml.etree.ElementTree as ET


class ShapeEnum(IntEnum):
    """
    *Description*
    
    The different simple shapes that can be created.
    """
    kBox = 1
    kSphere = 2
    kCylinder = 3
    kCapsule = 4

# Shape Definition Can be any one of these
@dataclass
class ShapeDefinition:
    """
    *Description*
    
    Base class for defining a shape as per URDF specifications.
    """
    def add_geometry_to_element(self, target_element: ET.Element) -> ET.Element:
        """
        *Description*

        Adds the shape geometry as a sub-element to the given target element.

        This method must be implemented in subclasses.

        *Parameters*

        target_element: xml.etree.ElementTree.Element
            The target XML element to which the shape geometry will be added.

        *Returns*

        xml.etree.ElementTree.Element
            The element `target_element` with the shape geometry added as a sub-element.
        """
        raise NotImplementedError("This method must be implemented in the subclass.")

    @property
    def type(self) -> ShapeEnum:
        """
        *Description*

        Returns the ShapeEnum type corresponding to the shape.

        This method must be implemented in subclasses.

        *Returns*

        ShapeEnum
            The ShapeEnum type for the shape.
        """
        raise NotImplementedError("This method must be implemented in the subclass.")

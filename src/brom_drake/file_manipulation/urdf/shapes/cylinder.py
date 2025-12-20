from dataclasses import dataclass
import xml.etree.ElementTree as ET

# Internal Import
from .shape_definition import ShapeDefinition, ShapeEnum

@dataclass
class CylinderDefinition(ShapeDefinition):
    """
    *Description*
    
    Defines a cylinder shape as per URDF specifications.

    *Attributes*

    radius: float
        The radius of the cylinder.

    length: float
        The length of the cylinder along its central axis.

    """
    radius: float
    length: float

    def add_geometry_to_element(self, target_element: ET.Element) -> ET.Element:
        """
        *Description*
        
        Adds the cylinder geometry as a sub-element to the given target element.
        
        This method is an implementation of the abstract method defined in the
        ShapeDefinition base class.

        *Parameters*

        target_element: xml.etree.ElementTree.Element
            The target XML element to which the cylinder geometry will be added.

        *Returns*

        xml.etree.ElementTree.Element
            The element `target_element` with the cylinder geometry added as a sub-element.
        """
        return ET.SubElement(
            target_element,
            "cylinder",
            {"radius": f"{self.radius}", "length": f"{self.length}"},
        )

    @property
    def type(self) -> ShapeEnum:
        """
        *Description*

        Returns the ShapeEnum type corresponding to a cylinder.

        This method is an implementation of the abstract method defined in the
        ShapeDefinition base class.

        *Returns*

        ShapeEnum
            The ShapeEnum type for a cylinder.
        """

        return ShapeEnum.kCylinder
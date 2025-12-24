from dataclasses import dataclass
import xml.etree.ElementTree as ET

# Internal Import
from .shape_definition import ShapeDefinition, ShapeEnum

@dataclass
class SphereDefinition(ShapeDefinition):
    """
    *Description*

    Defines a sphere shape as per URDF specifications.

    *Attributes*

    radius: float
        The radius of the sphere.
    """
    radius: float

    def add_geometry_to_element(self, target_element: ET.Element) -> ET.Element:
        """
        *Description*
        
        Adds the sphere geometry as a sub-element to the given target element.
        
        This method is an implementation of the abstract method defined in the
        ShapeDefinition base class.

        *Parameters*

        target_element: xml.etree.ElementTree.Element
            The target XML element to which the sphere geometry will be added.

        *Returns*

        xml.etree.ElementTree.Element
            The element `target_element` with the sphere geometry added as a sub-element.
        """
        return ET.SubElement(
            target_element,
            "sphere",
            {"radius": f"{self.radius}"},
        )

    @property
    def type(self) -> ShapeEnum:
        """
        *Description*
        
        Returns the ShapeEnum type corresponding to a sphere.

        This method is an implementation of the abstract method defined in the
        ShapeDefinition base class.

        *Returns*

        ShapeEnum
            The ShapeEnum type for a sphere.
        """
        return ShapeEnum.kSphere
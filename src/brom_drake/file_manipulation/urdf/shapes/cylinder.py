from dataclasses import dataclass
import xml.etree.ElementTree as ET

# Internal Import
from .shape_definition import ShapeDefinition, ShapeEnum

@dataclass
class CylinderDefinition(ShapeDefinition):
    radius: float
    length: float

    def add_geometry_to_element(self, target_element: ET.Element):
        return ET.SubElement(
            target_element,
            "cylinder",
            {"radius": f"{self.radius}", "length": f"{self.length}"},
        )

    @property
    def type(self):
        return ShapeEnum.kCylinder
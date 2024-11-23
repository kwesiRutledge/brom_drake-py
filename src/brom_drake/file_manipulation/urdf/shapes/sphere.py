from dataclasses import dataclass
import xml.etree.ElementTree as ET

# Internal Import
from .shape_definition import ShapeDefinition, ShapeEnum

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
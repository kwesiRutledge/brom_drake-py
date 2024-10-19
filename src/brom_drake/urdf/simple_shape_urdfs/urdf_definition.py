"""
Description:
    This file will contain utilities for quickly creating simple URDF files
    to be used in debugging and testing.
"""
# External Imports
from dataclasses import dataclass
from enum import IntEnum
import os
import xml.etree.ElementTree as ET

import numpy as np

# Internal Imports
from .shape_definition import ShapeEnum, SphereDefinition, ShapeDefinition

@dataclass
class InertiaDefinition:
    """
    A dataclass that defines the inertia of a simple shape.
    """
    ixx: float = 1.0
    ixy: float = 0.0
    ixz: float = 0.0
    iyy: float = 1.0
    iyz: float = 0.0
    izz: float = 1.0

    def as_list(self) -> list:
        """
        Convert the inertia to a list.
        :return: A list of the inertia values.
        """
        return [self.ixx, self.ixy, self.ixz, self.iyy, self.iyz, self.izz]

    def as_map(self) -> dict:
        """
        Convert the inertia to a map.
        :return: A map of the inertia values.
        """
        return {
            "ixx": f"{self.ixx}",
            "ixy": f"{self.ixy}",
            "ixz": f"{self.ixz}",
            "iyy": f"{self.iyy}",
            "iyz": f"{self.iyz}",
            "izz": f"{self.izz}",
        }

@dataclass
class SimpleShapeURDFDefinition:
    """
    A dataclass that defines a simple shape URDF.
    """
    name: str
    shape: ShapeDefinition
    color: np.ndarray = None
    create_collision: bool = True # Whether to create the collision elements of the urdf
    mass: float = 1.0
    inertia: InertiaDefinition = None

    def as_urdf(self) -> ET.Element:
        """
        Create the URDF for the simple shape using python's built-in xml library.
        :return:
        """
        # Setup
        root = ET.Element("robot", {"name": self.name + "_robot"})
        link = ET.SubElement(root, "link", {"name": self.name + "_base_link"})

        # Add inertial elements to link
        self.add_inertial_elements_to(link)

        # Add visual elements to link
        self.add_visual_elements_to(link)

        # Add collision elements to link
        if self.create_collision:
            self.add_collision_elements_to(link)

        return root

    def add_inertial_elements_to(self, link_elt: ET.Element):
        """
        Add the inertial elements to the link element.
        :param link_elt: The ET.Element object for the link.
        :return: Nothing, but modifies the link_elt in place.
        """
        # Setup
        inertial_link = ET.SubElement(link_elt, "inertial")
        inertia_def = self.inertia
        if inertia_def is None:
            inertia_def = InertiaDefinition()

        # Create all child links for inertial elements
        origin_elt = ET.SubElement(
            inertial_link,
            "origin",
            {"xyz": "0 0 0", "rpy": "0 0 0"},
        )
        mass = ET.SubElement(inertial_link, "mass", {"value": f"{self.mass}"})
        inertia = ET.SubElement(
            inertial_link,
            "inertia",
            inertia_def.as_map(),
        )

        # Add inertial elements to link
        link_elt.append(inertial_link)

    def add_visual_elements_to(self, link_elt: ET.Element):
        """
        Add the visual elements to the link element.
        :param link_elt: The ET.Element object for the link.
        :return: Nothing, but modifies the link_elt in place.
        """
        # Setup
        visual_link = ET.SubElement(link_elt, "visual")
        color = self.color
        if color is None:
            color = np.array([0.0, 1.0, 0.0, 0.8])

        # Create all child links for visual elements
        origin_elt = ET.SubElement(
            visual_link,
            "origin",
            {"xyz": "0 0 0", "rpy": "0 0 0"},
        )
        geometry = ET.SubElement(visual_link, "geometry")

        # Add the shape to the geometry element
        if self.shape.type == ShapeEnum.kSphere:
            sphere = ET.SubElement(geometry, "sphere", {"radius": f"{self.shape.radius}"})
        else:
            raise ValueError(f"Shape {self.shape} not supported yet.")

        # Add the material to the visual element
        material = ET.SubElement(visual_link, "material", {"name": "blue"})
        color = ET.SubElement(
            material,
            "color",
            {"rgba": f"{color[0]} {color[1]} {color[2]} {color[3]}"},
        )

        # Add visual elements to link
        link_elt.append(visual_link)

    def add_collision_elements_to(self, link_elt: ET.Element):
        """
        Add the collision elements to the link element.
        :param link_elt: The ET.Element object for the link.
        :return:
        """
        # Setup
        collision_link = ET.SubElement(link_elt, "collision")
        origin_elt = ET.SubElement(
            collision_link,
            "origin",
            {"xyz": "0 0 0", "rpy": "0 0 0"},
        )
        geometry = ET.SubElement(collision_link, "geometry")

        # Add the shape to the geometry element
        if self.shape.type == ShapeEnum.kSphere:
            sphere = ET.SubElement(geometry, "sphere", {"radius": f"{self.shape.radius}"})
        else:
            raise ValueError(f"Shape {self.shape} not supported yet.")

        # Add collision elements to link
        link_elt.append(collision_link)

    def write_to_file(self, file_path: str):
        """
        Write the URDF to a file.
        :param file_path: The path to the file where the URDF will be written.
        :return: Nothing, but writes the URDF to the file.
        """
        # Setup
        root = self.as_urdf()
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)

        # Write to file
        tree.write(file_path, xml_declaration=True)




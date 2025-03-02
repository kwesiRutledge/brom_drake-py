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
from pydrake.math import RigidTransform

# Internal Imports
from brom_drake.file_manipulation.urdf.shapes.shape_definition import ShapeEnum, ShapeDefinition
from brom_drake.file_manipulation.urdf.simple_writer.inertia_definition import InertiaDefinition

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
    pose: RigidTransform = RigidTransform()
    mu_static: float = 0.7
    mu_dynamic: float = 0.4
    is_hydroelastic: bool = True # Whether to use hydroelastic collision for the shape

    def as_urdf(self) -> ET.Element:
        """
        Create the URDF for the simple shape using python's built-in xml library.
        :return:
        """
        # Setup
        root = ET.Element(
            "robot",
            {
                "name": self.name + "_robot",
                "xmlns:drake": "http://drake.mit.edu",
            }
        )
        link = ET.SubElement(root, "link", {"name": self.base_link_name})

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
        self.add_origin_element_to(inertial_link)
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
        self.add_origin_element_to(visual_link)
        geometry = ET.SubElement(visual_link, "geometry")

        # Add the shape to the geometry element
        shape = self.shape.add_geometry_to_element(geometry)

        # Add the material to the visual element
        material = ET.SubElement(visual_link, "material", {"name": "blue"})
        color = ET.SubElement(
            material,
            "color",
            {"rgba": f"{color[0]} {color[1]} {color[2]} {color[3]}"},
        )

    def add_collision_elements_to(self, link_elt: ET.Element):
        """
        Add the collision elements to the link element.
        :param link_elt: The ET.Element object for the link.
        :return:
        """
        # Setup
        collision_link = ET.SubElement(link_elt, "collision")

        # Add pose to collision element
        self.add_origin_element_to(collision_link)
        geometry = ET.SubElement(collision_link, "geometry")

        # Add the shape to the geometry element
        self.shape.add_geometry_to_element(geometry)

        # Add the proximity properties to the collision element
        self.add_proximity_properties_to(collision_link)

    def add_proximity_properties_to(self, collision_elt: ET.Element):
        """
        Add the proximity properties to the collision element.
        :param collision_elt: The ET.Element object for the collision.
        :return: Nothing, but modifies the collision_elt in place.
        """
        # Setup
        proximity_properties = ET.SubElement(collision_elt, "proximity_properties")
        
        # Create mu_static element
        mu_static= ET.SubElement(
            proximity_properties,
            "drake:mu_static",
            {"value": f"{self.mu_static}"}
            )

        # Create mu_dynamic element
        mu_dynamic = ET.SubElement(
            proximity_properties,
            "drake:mu_dynamic",
            {"value": f"{self.mu_dynamic}"}
            )
        
        # Create is_hydroelastic element
        hydroelastic = ET.SubElement(
            proximity_properties,
            "drake:rigid_hydroelastic",
        )

    def add_origin_element_to(self, target_element: ET.Element):
        """
        Add the origin element to the target element.
        :param target_element: The element to which the origin element will be added.
        :return: Nothing, but modifies the target_element in place.
        """
        # Setup
        translation = self.pose.translation()
        rot_as_rpy = self.pose.rotation().ToRollPitchYaw()
        origin_elt = ET.SubElement(
            target_element,
            "origin",
            {
                "xyz": f"{translation[0]} {translation[1]} {translation[2]}",
                "rpy": f"{rot_as_rpy.roll_angle()} {rot_as_rpy.pitch_angle()} {rot_as_rpy.yaw_angle()}",
            },
        )

    @property
    def base_link_name(self) -> str:
        """
        The name of the base link.
        :return: The name of the base link.
        """
        return self.name + "_base_link"

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

        # Create the directory if it doesn't exist
        os.makedirs(os.path.dirname(file_path), exist_ok=True)

        # Write to file
        tree.write(file_path, xml_declaration=True)




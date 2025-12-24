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
from pathlib import Path
from pydrake.math import RigidTransform

# Internal Imports
from brom_drake.file_manipulation.urdf.shapes.shape_definition import ShapeEnum, ShapeDefinition
from brom_drake.file_manipulation.urdf.simple_writer.inertia_definition import InertiaDefinition
from brom_drake.directories import DEFAULT_BROM_MODELS_DIR

@dataclass
class SimpleShapeURDFDefinition:
    """
    *Description*

    A simple shape along with the optional and required options to fully
    specifiy it in a URDF.

    *Attributes*
    
    name: str
        The name of the shape.

    shape: ShapeDefinition
        The shape definition for the shape.

    color: np.ndarray, optional
        The RGBA color of the shape.

    create_collision: bool, optional
        Whether to create the collision elements of the urdf.

    mass: float, optional
        The mass of the shape.
        Defaults to 1.0.

    inertia: InertiaDefinition, optional
        The inertia of the shape.
        If not specified, a default inertia will be used.

    pose: RigidTransform, optional
        The pose of the shape.
        Defaults to identity.

    mu_static: float, optional
        The static friction coefficient of the shape.
        Defaults to 0.7.

    mu_dynamic: float, optional
        The dynamic friction coefficient of the shape.
        Defaults to 0.4.

    is_hydroelastic: bool, optional
        Whether to use hydroelastic collision for the shape.
        Defaults to True.
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
        *Description*

        Create the URDF for the simple shape using python's built-in xml library.

        *Returns*

        xml.etree.ElementTree.Element
            The root element of the desired URDF.
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
        *Description*

        Add the inertial elements to the link element.
        
        *Parameters*

        link_elt: xml.etree.ElementTree.Element
            The ET.Element object for the link.

        *Returns*

        Nothing, but modifies the link_elt in place.
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
        *Description*

        Add the visual elements to the link element.
        
        *Parameters*

        link_elt: xml.etree.ElementTree.Element
            The ET.Element object for the link.

        *Returns*

        Nothing, but modifies the link_elt in place.
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
        *Description*

        Add the collision elements to the link element.

        *Parameters*

        link_elt: xml.etree.ElementTree.Element
            The ET.Element object for the link.

        *Returns*
        Nothing, but modifies the link_elt in place.
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
        *Description*

        Add the proximity properties to the collision element.

        *Parameters*

        collision_elt: xml.etree.ElementTree.Element
            The ET.Element object for the collision object.

        *Returns*

        Nothing, but modifies the collision_elt in place.
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
        *Description*

        Add the origin element to the target element.
        
        *Parameters*

        target_element: xml.etree.ElementTree.Element
            The ET.Element object to which the origin element will be added.

        *Returns*
        Nothing, but modifies the target_element in place.
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
        *Description*

        The name of the base link.

        *Returns*

        str
            The name of the base link.
        """
        return self.name + "_base_link"

    def write_to_file(self, file_path: str = None) -> str:
        """
        *Description*

        Write the URDF to a file.

        *Parameters*

        file_path: str
            The path to the file where the URDF will be written.
        
        *Returns*
        
        str
            The path to the file where the URDF was written.
        """
        # Setup
        root = self.as_urdf()
        tree = ET.ElementTree(root)
        ET.indent(tree, space="\t", level=0)

        # Input Processing
        if file_path is None:
            file_path = DEFAULT_BROM_MODELS_DIR + "/shapes/" + self.name + ".urdf"

        # Create the directory if it doesn't exist
        fp_as_path = Path(file_path)
        fp_as_path.parent.mkdir(parents=True, exist_ok=True)

        # Write to file
        tree.write(file_path, xml_declaration=True)

        return file_path



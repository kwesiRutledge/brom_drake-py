"""
Description
-----------
This file contains the URDF converter for transforming
arbitrary URDFs into URDFs that the Drake toolbox can
support.
"""
import os
from copy import copy, deepcopy
from typing import Tuple

import trimesh
from doctest import UnexpectedException

import loguru
from multiprocessing.managers import Value
from pathlib import Path
import xml.etree.ElementTree as ET

from .mesh_file_converter import MeshFileConverter
# Internal Supports
from .util import (
    does_drake_parser_support,
    URDF_CONVERSION_LOG_LEVEL_NAME, URDF_CONVERSION_LEVEL,
    tree_contains_transmission_for_joint,
    create_transmission_element_for_joint,
)


class DrakeReadyURDFConverter:
    def __init__(
        self,
        original_urdf_filename: str,
        output_urdf_file_path: str = None,
        overwrite_old_models: bool = False,
        overwrite_old_logs: bool = False,
        log_file_name: str = "conversion.log",
    ):
        # Setup
        self.original_urdf_filename = original_urdf_filename
        self.output_urdf_file_path = output_urdf_file_path
        self.log_file_name = log_file_name

        # Define conversion data directory in brom
        # (If it already exists, then we will overwrite it with the proper flag set)
        self.models_dir = Path("./brom/models/")
        if overwrite_old_logs and os.path.exists(self.models_dir / self.log_file_name):
            os.remove(self.models_dir / self.log_file_name)
        if overwrite_old_models:
            self.clean_up_models_dir()
        os.makedirs(self.models_dir, exist_ok=True)

        # Input Processing
        original_urdf_path = Path(self.original_urdf_filename)
        assert original_urdf_path.exists(), f"URDF file {original_urdf_path} does not exist!"

        if not (".urdf" in original_urdf_path.name):
            raise ValueError("Original URDF file must have a .urdf extension!")

        # Set Up logger
        self.configure_logger()
        DrakeReadyURDFConverter.log(
            f"Created a new DrakeReadyURDFConverter for file \"{self.original_urdf_filename}\"."
        )

        # Keep Track of the Joints In The Diagram
        self.actuated_joint_names = []

    def configure_logger(self):
        """
        Description
        -----------
        This method configures the loguru logger for the URDF conversion.
        :return:
        """
        # Setup
        urdf_conversion_level_exists = False
        log_file_name = self.log_file_name

        # Check to see if the log level exists
        try:
            found_level = loguru.logger.level(URDF_CONVERSION_LOG_LEVEL_NAME)
            urdf_conversion_level_exists = (found_level.name == URDF_CONVERSION_LOG_LEVEL_NAME)
        except Exception as e:
            # If the logger doesn't exist, then we will create it.
            expected_message = f"Level '{URDF_CONVERSION_LOG_LEVEL_NAME}' does not exist"
            if expected_message in str(e):
                pass
            else:
                raise UnexpectedException(f"Unexpected exception: {e}")

        # Configure logger if it doesn't exist
        if not urdf_conversion_level_exists:
            loguru.logger.level(
                URDF_CONVERSION_LOG_LEVEL_NAME,
                no=URDF_CONVERSION_LEVEL,
            )

        # Add logger
        loguru.logger.add(
            self.models_dir / log_file_name,
            level=URDF_CONVERSION_LOG_LEVEL_NAME,
            filter=lambda record: record['level'].name == URDF_CONVERSION_LOG_LEVEL_NAME,
        )

    def convert_urdf(self) -> Path:
        """
        Description
        -----------
        Converts the original URDF file into a Drake-compatible URDF file.
        """
        # Setup
        original_urdf_path = Path(self.original_urdf_filename)
        original_xml = ET.ElementTree(file=original_urdf_path)

        # Use recursive function to convert the tree
        new_tree = self.convert_tree(original_xml)

        # Add transmissions, if needed
        for joint_name in self.actuated_joint_names:
            if tree_contains_transmission_for_joint(new_tree, joint_name):
                continue
            else:
                # If transmission doesn't exist in URDF, then add it!
                transmission_element = create_transmission_element_for_joint(joint_name)
                new_tree.getroot().append(transmission_element)

        # Output the new tree to a file
        output_urdf_path = self.output_file_directory() / self.output_urdf_file_name()
        os.makedirs(output_urdf_path.parent, exist_ok=True)

        new_tree.write(output_urdf_path)

        DrakeReadyURDFConverter.log(
            f"Converted URDF file to Drake-compatible URDF file at {output_urdf_path}.\n\n"
        )
        return output_urdf_path

    def clean_up_models_dir(self):
        """
        Description
        -----------
        This method will clean up the models directory.
        :return:
        """
        # Make sure that the models directory exists
        os.makedirs(self.models_dir, exist_ok=True)
        os.system(f"rm -r {self.models_dir}")

    def convert_mesh_element(self, mesh_elt_in: ET.Element) -> ET.Element:
        # Setup
        new_elt = deepcopy(mesh_elt_in)

        # Algorithm
        self.log(
            f"Found a mesh element with filename \"{mesh_elt_in.attrib['filename']}\"."
        )

        # Check the value of the filename
        if not ("filename" in new_elt.attrib):
            raise ValueError(
                f"Found a mesh element that does not contain the \"filename\" attribute.\n"
                + "Brom doesn't know how to handle this!"
            )

        # Filename exists; Let's check to see if it's obj or not
        if does_drake_parser_support(new_elt.attrib["filename"]):
            pass
        else:
            # If parser does not support the given filename,
            # then let's try to create one
            old_filename = new_elt.attrib["filename"]
            new_filename = self.create_obj_to_replace_mesh_file(old_filename)
            new_elt.set(
                "filename", new_filename,
            )
            self.log(
                f"Replaced the mesh file \"{old_filename}\" with a Drake-compatible .obj file at \"{new_filename}\"."
            )

        return new_elt

    def convert_tree(
        self,
        current_tree: ET.ElementTree
    ) -> ET.ElementTree:
        """
        Description
        -----------`
        This recursive method is used to parse each element of the
        xml element tree.
        :param current_tree:
        :return:
        """
        # Setup
        root = deepcopy(current_tree.getroot())

        # Modify Root if it contains a mesh or file input
        new_root = self.convert_tree_element(root)

        # Remove all children from the new root
        for child in new_root.findall("*"):
            new_root.remove(child)

        # Modify Children
        for child in root:
            new_child = self.convert_tree(
                ET.ElementTree(child)
            )
            new_root.append(new_child.getroot())

        return ET.ElementTree(new_root)


    def convert_tree_element(
        self,
        elt: ET.Element,
    ) -> ET.Element:
        """
        Description
        -----------
        This method is used to transform one element in the XML tree into an eleemnt
        that that Drake URDF parser can handle. If necessary, it will create a new
        3d model file and place it into the right directory.
        :param elt: An element in the ElementTree that we would like to convert
                     into a Drake-ready element.
        :return: The Drake-ready xml tree element.
        """
        # Setup
        new_elt = deepcopy(elt)

        # Convert file paths for mesh elements if necessary
        if new_elt.tag == "mesh":
            new_elt = self.convert_mesh_element(new_elt)
        elif new_elt.tag == "transmission":
            # Ignore transmission elements during the conversion
            return new_elt
        elif new_elt.tag == "joint":
            self.handle_joint_element(new_elt)
        else:
            # If the element is not a mesh element, then we will just return a copy of it.
            self.log(
                f"Found an element with tag \"{elt.tag}\" that is not a mesh element. No modification needed..."
            )
        # Otherwise, just return a copy of the previous element.
        return new_elt

    def create_obj_to_replace_mesh_file(
        self,
        mesh_file_name: str,
    ) -> str:
        """
        Description
        -----------
        This function will create an .obj file to replace the
        .stl or .dae file that is given in mesh file "mesh_file_name".
        :param mesh_file_name:
        :return:
        """
        # Setup
        original_urdf_dir = Path(self.original_urdf_filename).parent

        # Convert the file
        converter = MeshFileConverter(
            mesh_file_path=mesh_file_name,
            urdf_dir=Path(original_urdf_dir),
            new_urdf_dir=self.output_file_directory(),
        )
        new_mesh_path = converter.convert()

        # Clip off all parts of path that include the exported output
        new_mesh_path = str(new_mesh_path).replace(str(self.output_file_directory()), "")

        return "./" + str(new_mesh_path)


    def handle_joint_element(self, joint_elt: ET.Element):
        """
        Description
        -----------
        This method will handle the joint element in the URDF file.
        :param joint_elt:
        :return:
        """
        # Setup

        # Keep track of all FREE joint names

        # If the joint does not have a "type" attribute, then we will assume it is fixed
        # and won't add it.
        if "type" not in joint_elt.attrib:
            return

        # If joint is "fixed", then don't add it to the list of actuated joints
        joint_is_fixed = joint_elt.attrib["type"] == "fixed"
        if joint_is_fixed:
            return

        joint_name = joint_elt.attrib["name"]
        self.actuated_joint_names.append(joint_name)

    @staticmethod
    def log(message: str):
        """
        Description
        -----------
        Logs a message to the logger.
        :param message: A string with the message we want to send to the logs.
        :return:
        """
        loguru.logger.log(URDF_CONVERSION_LOG_LEVEL_NAME, message)

    def output_file_directory(self) -> Path:
        """
        Description
        -----------
        Returns the directory of the output urdf file.
        :return:
        """
        # Setup
        original_file_path = Path(self.original_urdf_filename)

        # Input Processing
        if self.output_urdf_file_path is None:
            return self.models_dir / original_file_path.name.replace(".urdf", "")
        else:
            output_file_path = Path(self.output_urdf_file_path)
            return output_file_path.parent

    def output_urdf_file_name(self) -> str:
        """
        Description
        -----------
        Creates the output filename based on the original filename.
        """
        # Setup
        original_file_path = Path(self.original_urdf_filename)

        if self.output_urdf_file_path is None:
            # Use Pathlib to extract the filename, if it exists
            original_name = original_file_path.name
            original_name = original_name.replace(".urdf", "")

            return f"{original_name}.drake.urdf"
        else:
            return Path(self.output_urdf_file_path).name
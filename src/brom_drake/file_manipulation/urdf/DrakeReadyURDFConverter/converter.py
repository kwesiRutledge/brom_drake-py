"""
Description
-----------
This file contains the URDF converter for transforming
arbitrary URDFs into URDFs that the Drake toolbox can
support.
"""
from enum import IntEnum
import os
from copy import copy, deepcopy

import trimesh
from doctest import UnexpectedException

import loguru
import numpy as np
from pathlib import Path
from pydrake.all import RigidTransform, RotationMatrix, RollPitchYaw
import shutil
import xml.etree.ElementTree as ET

# Internal Imports
from brom_drake.directories import DEFAULT_BROM_MODELS_DIR
from .config import DrakeReadyURDFConverterConfig, MeshReplacementStrategy
from .mesh_file_converter import MeshFileConverter
from .util import (
    does_drake_parser_support,
    URDF_CONVERSION_LOG_LEVEL_NAME, URDF_CONVERSION_LEVEL,
    tree_contains_transmission_for_joint,
    create_transmission_element_for_joint,
)
from brom_drake.file_manipulation.urdf.shapes.cylinder import CylinderDefinition

class DrakeReadyURDFConverter:
    """
    Description
    -----------
    This class is used to convert a URDF file into a Drake-compatible URDF file.
    It makes the following considerations when parsing a URDF file:
    - If a mesh file is not supported by the Drake parser,
      then it will convert it into a .obj file (which is supported).
    - If a joint in the URDF file is not "fixed" (i.e., it is actuated)
      and the parser finds no transmission element for it, then it will add
      a transmission element to the URDF file.
    """

    def __init__(
        self,
        original_urdf_filename: str,
        config: DrakeReadyURDFConverterConfig = DrakeReadyURDFConverterConfig(),
    ):
        """
        Description
        -----------
        This method initializes the DrakeReadyURDFConverter.

        Arguments
        ---------
        original_urdf_filename: str
            The path to the original URDF file.
        output_urdf_file_path: str (optional)
            The path to the output URDF file. If None, then the output file will be placed in the 
            brom models directory.
        overwrite_old_models: bool (optional)
            A flag that indicates whether or not to overwrite old models directory.
            This is dangerous, so use with caution. Default is False.
        overwrite_old_logs: bool (optional)
            A flag that indicates whether or not to overwrite old logs.
            Default is False.
        log_file_name: str (optional)
            The name of the log file. Default is "conversion.log".
        collision_mesh_replacement_strategy: MeshReplacementStrategy (optional)
            The strategy for replacing collision meshes. 
            Each "incompatible" mesh file can be replaced by an .obj file (kWithObj)
            or by a minimal enclosing cylinder (kWithMinimalEnclosingCylinder).
            Default is MeshReplacementStrategy.kWithObj.
        """
        # Setup
        self.config = config
        self.original_urdf_filename = original_urdf_filename

        # Define conversion data directory in brom
        # (If it already exists, then we will overwrite it with the proper flag set)
        self.models_dir = Path(DEFAULT_BROM_MODELS_DIR)
        
        overwrite_old_logs = self.config.overwrite_old_logs
        overwrite_old_models = self.config.overwrite_old_models
        log_file_name = self.config.log_file_name

        if overwrite_old_logs and os.path.exists(self.output_file_directory() / log_file_name):
            os.remove(self.output_file_directory() / log_file_name)
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
        log_file_name = self.config.log_file_name

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

        # Remove the old logger
        loguru.logger.remove()

        # Configure logger if it doesn't exist
        if not urdf_conversion_level_exists:
            loguru.logger.level(
                URDF_CONVERSION_LOG_LEVEL_NAME,
                no=URDF_CONVERSION_LEVEL,
            )

        # Add logger
        loguru.logger.add(
            self.output_file_directory() / log_file_name,
            level=URDF_CONVERSION_LOG_LEVEL_NAME,
            filter=lambda record: record['level'].name == URDF_CONVERSION_LOG_LEVEL_NAME,
        )

    def convert_collision_element(self, collision_elt: ET.Element) -> ET.Element:
        """
        Description
        -----------
        This method will convert the collision element in the URDF file.

        Arguments
        ---------
        collision_elt: ET.Element
            The collision element that we want to convert.
        
        Returns
        -------
        ET.Element
            The new collision element.
        """
        # Setup
        new_elt = deepcopy(collision_elt)
        replacement_strategy = self.config.mesh_replacement_strategies.collision_meshes

        n_replacements_made = 0

        # Announce algorithm start
        self.log(
            f"Converting element with tag \"{collision_elt.tag}\" using strategy {replacement_strategy}."
        )

        # Check to see if the collision element has a mesh element within it
        
        # Iterate through every element of the collision element
        for ii, child_ii in enumerate(collision_elt):
            if child_ii.tag == "geometry": 
                # If the child is a geometry element, then we will check if replacement is needed.
                geometry_elt = child_ii
                
                # Iterate through every element of the geometry element
                for jj, child_jj in enumerate(geometry_elt):
                    if child_jj.tag == "mesh":
                        # If the child is a mesh element, then we will check if replacement is needed.
                        mesh_elt = child_jj
                        
                        # Replace!
                        if replacement_strategy == MeshReplacementStrategy.kWithObj:
                            new_mesh_elt = self.convert_mesh_element(mesh_elt)
                            new_elt[ii][jj] = new_mesh_elt
                        elif replacement_strategy == MeshReplacementStrategy.kWithMinimalEnclosingCylinder:
                            new_elt = self.replace_element_with_enclosing_cylinder(new_elt)
                        else:
                            raise ValueError(
                                f"Invalid mesh replacement strategy: {replacement_strategy}"
                            )    

                        n_replacements_made += 1       

        # Announce the number of replacements made in the logger.
        self.log(
            f"Made {n_replacements_made} replacements in the collision element."
        )

        return new_elt
    
    def convert_visual_element(self, visual_elt: ET.Element) -> ET.Element:
        """
        Description
        -----------
        This method will convert the collision element in the URDF file.

        For now, this is very similar to the collision element handling, but it may
        change in the future.

        Arguments
        ---------
        visual_elt: ET.Element
            The visual element that we want to convert.

        Returns
        -------
        ET.Element
            The new visual element.
        """
        # Setup
        new_elt = deepcopy(visual_elt)
        replacement_strategy = self.config.mesh_replacement_strategies.visual_meshes
        original_urdf_dir = Path(self.original_urdf_filename).parent

        # Algorithm
        self.log(
            f"Converting element with tag \"{visual_elt.tag}\" using the \"{replacement_strategy}\" strategy."
        )

        # Search through all children of the visual element
        n_replacements_made = 0
        for ii, child_ii in enumerate(visual_elt):
            if child_ii.tag == "geometry":
                # If the child is a geometry element, then we will check if replacement is needed.
                geometry_elt = child_ii

                # Iterate through every element of the geometry element
                for jj, child_jj in enumerate(geometry_elt):
                    if child_jj.tag == "mesh":
                        # If the child is a mesh element, then we will check if replacement is needed.
                        mesh_elt = child_jj

                        # Replace!
                        if replacement_strategy == MeshReplacementStrategy.kWithObj:
                            new_mesh_elt = self.convert_mesh_element(mesh_elt)
                            new_elt[ii][jj] = new_mesh_elt
                        # TODO: Implement this
                        # elif replacement_strategy == MeshReplacementStrategy.kDoNotReplace:
                        #     # Copy the associated file into the models directory
                        #     old_mesh_file = mesh_elt.attrib["filename"]

                        #     temp_mfc = MeshFileConverter(
                        #         mesh_file_path=old_mesh_file,
                        #         urdf_dir=original_urdf_dir,
                        #         new_urdf_dir=self.output_file_directory(),
                        #     )

                        #     true_old_mesh_file = temp_mfc.true_mesh_file_path()
                        #     new_mesh_full_path = temp_mfc.define_output_path()

                        #     print(f"Copying {true_old_mesh_file} to {new_mesh_full_path}")

                        #     # Copy!
                        #     shutil.copy(true_old_mesh_file, new_mesh_full_path)
                            
                        else:
                            raise ValueError(
                                f"Invalid mesh replacement strategy: {replacement_strategy}"
                            )

                        n_replacements_made += 1

        return new_elt

    def convert_urdf(self) -> Path:
        """
        Description
        -----------
        Converts the original URDF file into a Drake-compatible URDF file.

        Returns
        -------
        Path
            The path to the new URDF file.
        """
        # Setup
        original_urdf_path = Path(self.original_urdf_filename)
        original_xml = ET.ElementTree(file=original_urdf_path)

        add_missing_actuators = self.config.add_missing_actuators

        # Use recursive function to convert the tree
        new_tree = self.convert_tree(original_xml)

        # Add transmissions, if needed
        if add_missing_actuators:
            for joint_name in self.actuated_joint_names:
                if tree_contains_transmission_for_joint(new_tree, joint_name):
                    continue
                else:
                    # If transmission doesn't exist in URDF, then add it!
                    transmission_element = create_transmission_element_for_joint(joint_name)
                    ET.indent(transmission_element, space="\t", level=0)
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
        xml element tree and replace the components that are not
        supported by the Drake URDF parser.

        Arguments
        ---------
        current_tree: ET.ElementTree
            The current tree that we are parsing.
        
        Returns
        -------
        ET.ElementTree
            The new tree that has been converted.
        """
        # Setup
        root = deepcopy(current_tree.getroot())

        # Modify root, if necessary
        new_root = self.convert_tree_element(root)

        # Modify Children of the root
        initial_children = list(new_root)
        for child in initial_children:
            new_child = self.convert_tree(
                ET.ElementTree(child)
            )
            new_root.remove(child)

            # Add new child to new root
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

        Arguments
        ---------
        elt: ET.Element
            An element in the ElementTree that we would like to convert
            into a Drake-ready element.
        
        Returns
        -------
        ET.Element
            The new element that has been converted.
        """
        # Setup
        new_elt = deepcopy(elt)

        # Convert file paths for mesh elements if necessary
        if new_elt.tag == "collision":
            new_elt = self.convert_collision_element(new_elt)
        elif new_elt.tag == "visual":
            new_elt = self.convert_visual_element(new_elt)
        elif new_elt.tag == "transmission":
            # Ignore transmission elements during the conversion
            return new_elt
        elif new_elt.tag == "joint":
            self.handle_joint_element(new_elt)
        else:
            # If the element is not a mesh element, then we will just return a copy of it.
            self.log(
                f"Found an element with tag \"{new_elt.tag}\" (originally \"{elt.tag}\") that is not a mesh element. No modification needed..."
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

        Arguments
        ---------
        mesh_file_name: str
            The name of the mesh file that we want to convert.
        
        Returns
        -------
        str
            The name of the new .obj file that has been created.
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

        return "." + str(new_mesh_path)


    def handle_joint_element(self, joint_elt: ET.Element):
        """
        Description
        -----------
        This method will handle the joint element in the URDF file.
        Specifically, it will check to see if the joint is actuated
        and (if it is) add it to the list of actuated joints.

        This is useful when attempting to add transmissions to the URDF file
        later on.
        
        Arguments
        ---------
        joint_elt: ET.Element
            The joint element that we want to handle.
        
        Returns
        -------
        None
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

        Arguments
        ---------
        message: str
            The message that we want to log.
        
        Returns
        -------
        None
        """
        loguru.logger.log(URDF_CONVERSION_LOG_LEVEL_NAME, message)

    def output_file_directory(self) -> Path:
        """
        Description
        -----------
        Returns the directory of the output urdf file.
        
        Returns
        -------
        Path
            The path to the directory of the output urdf file.
        """
        # Setup
        original_file_path = Path(self.original_urdf_filename)
        output_urdf_file_path = self.config.output_urdf_file_path

        # Input Processing
        if output_urdf_file_path is None:
            return self.models_dir / original_file_path.name.replace(".urdf", "")
        else:
            output_file_path = Path(output_urdf_file_path)
            return output_file_path.parent

    def output_urdf_file_name(self) -> str:
        """
        Description
        -----------
        Creates the output filename based on the original filename.

        Returns
        -------
        str
            The name of the output URDF file
        """
        # Setup
        original_file_path = Path(self.original_urdf_filename)
        output_urdf_file_path = self.config.output_urdf_file_path

        if output_urdf_file_path is None:
            # Use Pathlib to extract the filename, if it exists
            original_name = original_file_path.name
            original_name = original_name.replace(".urdf", "")

            return f"{original_name}.drake.urdf"
        else:
            return Path(output_urdf_file_path).name

    def replace_element_with_enclosing_cylinder(self, collision_elt: ET.Element) -> ET.Element:
        """
        Description
        -----------
        This method will replace the geometry element with an enclosing cylinder.

        Arguments
        ---------
        collision_elt: ET.Element
            The collision element that we want to replace.

        Returns
        -------
        ET.Element
            The new collision element with the enclosing cylinder.
        """
        # Setup
        new_elt = deepcopy(collision_elt)
        original_urdf_dir = Path(self.original_urdf_filename).parent

        # Save the initial pose of the element
        origin_elt = new_elt.find("origin")
        pose0 = RigidTransform(
            R=RollPitchYaw(np.array(
                [float(elt) for elt in origin_elt.attrib["rpy"].split(" ")]
            )).ToRotationMatrix(),
            p=np.array(
                [float(elt) for elt in origin_elt.attrib["xyz"].split(" ")],
            ),
        )

        # Extract the mesh element and the filename that it references
        mesh_elt = new_elt.find("geometry/mesh")
        mesh_file_name = mesh_elt.attrib["filename"]

        # Load the mesh from the mesh element
        converter = MeshFileConverter(
            mesh_file_path=mesh_file_name,
            urdf_dir=Path(original_urdf_dir),
            new_urdf_dir=self.output_file_directory(),
        )
        mesh = trimesh.load_mesh(
            str(converter.urdf_dir / converter.true_mesh_file_path())
        )

        # Create the enclosing cylinder
        cylinder_result = trimesh.bounds.minimum_cylinder(mesh, sample_count=12)
        cylinder_shape = CylinderDefinition(
            radius=cylinder_result['radius'],
            length=cylinder_result['height'],
        )

        # Remove the mesh element and pose element from the collision element, if they exist
        geometry_elt = new_elt.find("geometry")
        if mesh_elt is not None:
            new_elt.remove(geometry_elt)

        origin_elt = new_elt.find("origin")
        if origin_elt is not None:
            new_elt.remove(origin_elt)

        # Add the origin element to the collision element
        transform = cylinder_result['transform']
        transform_as_matrix = np.array(transform)
        transform_as_rt = RigidTransform(
            RotationMatrix(transform_as_matrix[:3, :3]),
            transform_as_matrix[:3, 3],
        )

        pose_as_rt = pose0.multiply(transform_as_rt)

        ET.SubElement(
            new_elt,
            "origin",
            {
                "xyz": f"{pose_as_rt.translation()[0]} {pose_as_rt.translation()[1]} {pose_as_rt.translation()[2]}",
                "rpy": f"{pose_as_rt.rotation().ToRollPitchYaw().roll_angle()} {pose_as_rt.rotation().ToRollPitchYaw().pitch_angle()} {pose_as_rt.rotation().ToRollPitchYaw().yaw_angle()}",
            },
        )

        # Create a new geometry element and add the cylinder element to it
        new_elt.append(ET.Element("geometry"))
        cylinder_shape.add_geometry_to_element(new_elt.find("geometry"))

        return new_elt






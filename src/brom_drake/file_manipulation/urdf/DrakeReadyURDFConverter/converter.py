"""
Description
-----------
This file contains the URDF converter for transforming
arbitrary URDFs into URDFs that the Drake toolbox can
support.
"""
import coacd
from copy import copy, deepcopy
from doctest import UnexpectedException
from enum import IntEnum
import logging
import os
import trimesh
from typing import List

import numpy as np
from pathlib import Path
from pydrake.all import RigidTransform, RotationMatrix, RollPitchYaw
import shutil
import xml.etree.ElementTree as ET

# Internal Imports
from brom_drake.directories import DEFAULT_BROM_MODELS_DIR
from .config import DrakeReadyURDFConverterConfig, MeshReplacementStrategy
from .file_manager import DrakeReadyURDFConverterFileManager
from .mesh_file_converter import MeshFileConverter
from .util import (
    does_drake_parser_support,
    find_mesh_file_path_in,
    get_mesh_element_in,
    URDF_CONVERSION_LOG_LEVEL_NAME, URDF_CONVERSION_LEVEL,
    tree_contains_transmission_for_joint,
    create_transmission_element_for_joint,
)
from brom_drake.file_manipulation.urdf.shapes.cylinder import CylinderDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_element_creator import URDFElementCreator

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

        # Define the file manager which handles the tricky file paths that
        # we encounter during the conversion process.
        self.file_manager : DrakeReadyURDFConverterFileManager = DrakeReadyURDFConverterFileManager(
            original_urdf_path=Path(self.original_urdf_filename),
            models_directory=Path(DEFAULT_BROM_MODELS_DIR),
            log_file_name=config.log_file_name,
            _output_urdf_path=config.output_urdf_file_path,
        ) 

        # Define conversion data directory in brom
        # (If it already exists, then we will overwrite it with the proper flag set)
        self.file_manager.models_directory = Path(DEFAULT_BROM_MODELS_DIR)
        
        overwrite_old_models = self.config.overwrite_old_models

        if overwrite_old_models:
            self.file_manager.clean_up_models_dir()
        os.makedirs(self.file_manager.models_directory, exist_ok=True)

        # Input Processing
        original_urdf_path = Path(self.original_urdf_filename)
        assert original_urdf_path.exists(), f"URDF file {original_urdf_path} does not exist!"

        if not (".urdf" in original_urdf_path.name):
            raise ValueError("Original URDF file must have a .urdf extension!")

        # Set Up logger
        self.logger = self.create_logger()
        self.log(
            f"Created a new DrakeReadyURDFConverter for file \"{self.original_urdf_filename}\".",
        )

        # Configure coacd, just in case
        coacd.set_log_level(config.coacd_log_level)

        # Keep Track of the Joints In The Diagram
        self.actuated_joint_names = []

    def create_logger(self) -> logging.Logger:
        """
        Description
        -----------
        This method configures the `logging` logger for the URDF conversion.
        :return:
        """
        # Setup
        urdf_conversion_level_exists = False
        log_file_name = self.config.log_file_name
        target_file = self.original_urdf_filename

        # Check if the log level exists in the logging module
        overwrite_old_logs = self.config.overwrite_old_logs
        if overwrite_old_logs and os.path.exists(self.file_manager.output_file_directory() / log_file_name):
            os.remove(self.file_manager.output_file_directory() / log_file_name)

        # Create (or collect) a logger for the given file
        logger = logging.getLogger("DrakeReadyURDFConverter ({target_file})" )

        # Create the file handler
        if not self.file_manager.output_file_directory().exists():
            # Create the parent directory if it does not exist
            self.file_manager.output_file_directory().mkdir(parents=True, exist_ok=True)

        file_handler = logging.FileHandler(
            self.file_manager.output_file_directory() / log_file_name,
            mode='w'
        )
        file_handler.setLevel(logging.INFO)

        # Create a formatter and set it for the file handler
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        file_handler.setFormatter(formatter)

        # Add the file handler to the logger
        logger.addHandler(file_handler)

        # Avoid duplicate logs
        logger.propagate = False

        # Make sure the logger responds to all messages of level DEBUG and above
        logger.setLevel(logging.DEBUG)  # Set to DEBUG to capture all messages

        return logger

    def convert_collision_element(self, collision_elt: ET.Element) -> List[ET.Element]:
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
        replacement_strategy = self.config.mesh_replacement_strategies.collision_meshes

        supported_strategies = [
            MeshReplacementStrategy.kWithObj,
            MeshReplacementStrategy.kWithMinimalEnclosingCylinder,
            MeshReplacementStrategy.kWithConvexDecomposition,
        ]

        # Input Processing
        if replacement_strategy not in supported_strategies:
            error_message = (
                f"Replacement strategy {replacement_strategy} is not supported for collision elements. "
                + "Supported strategies are: "
                + ", ".join([str(strategy) for strategy in supported_strategies])
            )
            self.log(error_message)
            raise ValueError(error_message)

        # Announce algorithm start
        self.log(
            f"Converting element with tag \"{collision_elt.tag}\" using strategy {replacement_strategy}."
        )
        n_replacements_made = 0
        new_elts = []

        # If the strategy is kWithMinimalEnclosingCylinder,
        # then we can replace the entire collision element and don't need to iterate through the children.
        if replacement_strategy == MeshReplacementStrategy.kWithMinimalEnclosingCylinder:
            n_replacements_made += 1
            new_elts = [self.replace_element_with_enclosing_cylinder(
                deepcopy(collision_elt)
            )]

        elif replacement_strategy == MeshReplacementStrategy.kWithObj:
            # Otherwise, let's slowly build the new element
            # by iterating through the children of the collision element.
            new_elt = ET.Element(collision_elt.tag, collision_elt.attrib)
            for ii, child_ii in enumerate(collision_elt):
                # Check to see if the child is a geometry element
                if child_ii.tag == "geometry":
                    new_geometry_elt = self.convert_geometry_element(child_ii, replacement_strategy)
                    new_elt.append(new_geometry_elt)
                else:
                    # If the child is not a geometry element, then we will just copy it over
                    new_elt.append(deepcopy(child_ii))


            # Add the new element to the list of new elements
            new_elts.append(new_elt)

        elif replacement_strategy == MeshReplacementStrategy.kWithConvexDecomposition:
            # Determine if the collision element has a mesh element within it
            original_mesh_path = find_mesh_file_path_in(collision_elt)
            if original_mesh_path is None:
                self.log(
                    "The collision element does not contain a mesh file. Cannot replace with convex decomposition."
                )
                new_elts.append(deepcopy(collision_elt))
                return new_elts
            
            # If the collision element has a mesh element within it, then we will perform convex decomposition
            original_mesh_elt = get_mesh_element_in(collision_elt)
            scale_as_str = original_mesh_elt.attrib.get("scale", "1 1 1")
            original_mesh_scale = np.array(
                [float(x) for x in scale_as_str.split()]
            )

            # Load the mesh file and perform convex decomposition
            self.log(
                f"Found a mesh file at {original_mesh_path}. Performing convex decomposition."
            )
            # Load the mesh file
            mesh: trimesh.Trimesh = trimesh.load(
                self.file_manager.file_path_in_context_of_urdf_dir(original_mesh_path),
                force="mesh",
            )
            mesh = coacd.Mesh(mesh.vertices, mesh.faces)
            parts = coacd.run_coacd(mesh) # a list of convex hulls

            self.log(
                f"coacd created {len(parts)} convex parts from the mesh file \"{original_mesh_path}\""
            )

            for ii, part_ii in enumerate(parts):
                # print(part_ii)
                # print(type(part_ii))
                # print(len(part_ii))

                # Construct new mesh and choose the path to export it to
                mesh_ii = trimesh.Trimesh(
                    vertices=part_ii[0], faces=part_ii[1]
                )
                mesh_file_name_ii = original_mesh_path.name[:original_mesh_path.name.index(".")]
                mesh_ii_file_relative_path = f"meshes/{mesh_file_name_ii}/{mesh_file_name_ii}_part_{ii}.obj"
                mesh_ii_file_path = self.file_manager.output_file_directory() / mesh_ii_file_relative_path
                
                # Export after checking that the parent directory exists
                os.makedirs(mesh_ii_file_path.parent, exist_ok=True)
                mesh_ii.export(mesh_ii_file_path)

                # Create a new collision element for each part
                collision_element_ii = URDFElementCreator.create_collision_element(
                    name=f"{collision_elt.attrib.get('name', 'collision')}_part_{ii}",
                    mesh_file_path="./" + str(mesh_ii_file_relative_path),
                    mesh_scale=original_mesh_scale,
                )
                new_elts.append(collision_element_ii)

                n_replacements_made += 1

        # Check to see if the collision element has a mesh element within it
        
        # # Iterate through every element of the collision element
        # for ii, child_ii in enumerate(collision_elt):
        #     if child_ii.tag == "geometry": 
        #         # If the child is a geometry element, then we will check if replacement is needed.
        #         geometry_elt = child_ii
                
        #         # Iterate through every element of the geometry element
        #         for jj, child_jj in enumerate(geometry_elt):
        #             if child_jj.tag == "mesh":
        #                 # If the child is a mesh element, then we will check if replacement is needed.
        #                 mesh_elt = child_jj
                        
        #                 # Replace!
        #                 if replacement_strategy == MeshReplacementStrategy.kWithObj:
        #                     new_mesh_elt = self.convert_mesh_element(mesh_elt)
        #                     new_elt[ii][jj] = new_mesh_elt
        #                 elif replacement_strategy == MeshReplacementStrategy.kWithMinimalEnclosingCylinder:
        #                     new_elt = self.replace_element_with_enclosing_cylinder(new_elt)
        #                 # elif replacement_strategy == MeshReplacementStrategy.kWithConvexDecomposition:
                            
        #                 else:
        #                     raise ValueError(
        #                         f"Invalid mesh replacement strategy: {replacement_strategy}"
        #                     )    

        #                 n_replacements_made += 1       

        # Announce the number of replacements made in the logger.
        self.log(
            f"Made {n_replacements_made} replacements in the collision element."
        )

        return new_elts
    
    def convert_geometry_element(
        self,
        geometry_elt: ET.Element,
        replacement_strategy: MeshReplacementStrategy
    ) -> ET.Element:
        """
        Description
        -----------
        This method will convert the geometry element in the URDF (i.e., an XML) file
        according to the conversion strategy provided as input.
        
        Arguments
        ---------
        geometry_elt: ET.Element
            The geometry element that we want to convert.
        
        Returns
        -------
        ET.Element
            The new geometry element.
        """
        # Setup
        new_elt = ET.Element(geometry_elt.tag, geometry_elt.attrib)

        # Algorithm
        self.log(
            f"Converting geometry element with strategy \"{replacement_strategy}\"."
        )

        # Iterate through every element of the geometry element
        for ii, child_ii in enumerate(geometry_elt):
            if child_ii.tag == "mesh":
                # If the child is a mesh element, then we will check if replacement is needed.
                mesh_elt = child_ii

                # Replace!
                new_mesh_elt = self.convert_mesh_element(mesh_elt)
                new_elt.append(new_mesh_elt)

            else:
                # If the child is not a mesh element, then we will just copy it over
                new_elt.append(deepcopy(child_ii))

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

        # Algorithm
        self.log(
            f"Converting element with tag \"{visual_elt.tag}\" using the \"{replacement_strategy}\" strategy."
        )

        # Search through all children of the visual element
        n_replacements_made = 0
        added_material = False
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
                        #         new_urdf_dir=self.file_manager.output_file_directory(),
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

            elif child_ii.tag == "material":
                new_material_elt = self.create_new_material_element(child_ii)

                # replace the material element in new_elt with this one
                new_elt[ii] = new_material_elt
                added_material = True
                if self.config.replace_colors_with is not None:
                    self.log(
                        f"Replaced the color of the material element with RGBA values {self.config.replace_colors_with}."
                    )

        # If material did not exist in the visual element, then we will add it.
        if not added_material:
            # Create a new material element
            new_material_elt = ET.Element("material")
            new_elt.append(new_material_elt)

            transformed_material_elt = self.create_new_material_element(new_material_elt)
            new_elt[-1] = transformed_material_elt
            self.log(
                f"Added a new material element to the visual element with RGBA values {self.config.replace_colors_with}."
            )
            n_replacements_made += 1



            added_material = True

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
        new_trees = self.convert_tree(original_xml)
        assert len(new_trees) == 1, \
            f"Expected to get exactly one new tree after conversion, but got {len(new_trees)}."
        new_tree = new_trees[0]

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
        output_urdf_path = self.file_manager.output_urdf_path
        os.makedirs(output_urdf_path.parent, exist_ok=True)

        new_tree.write(output_urdf_path)

        self.log(
            f"Converted URDF file to Drake-compatible URDF file at {output_urdf_path}.\n\n"
        )
        return output_urdf_path

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
            self.log(
                f"\"{new_elt.attrib['filename']}\" is already supported by the Drake parser. Let's copy it into the right place."
            )
            # Let's try to copy the old file into the new directory
            old_filename = new_elt.attrib["filename"]
            new_filename = self.create_obj_to_replace_mesh_file(old_filename)
            new_elt.set(
                "filename", new_filename,
            )
            self.log(
                f"Copied the Drake-compatible mesh file \"{old_filename}\" to \"{new_filename}\"."
            )
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
    ) -> List[ET.ElementTree]:
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
        new_elements = self.convert_tree_element(root)

        for element_ii in new_elements:
            # Modify Children of the root
            initial_children = list(element_ii)
            for child in initial_children:
                new_children = self.convert_tree(
                    ET.ElementTree(child)
                )
                element_ii.remove(child)

                # Add new children to new root
                for new_child in new_children:
                    # Convert the new child element
                    element_ii.append(new_child.getroot())

        return [
            ET.ElementTree(element_ii) for element_ii in new_elements
        ]


    def convert_tree_element(
        self,
        elt: ET.Element,
    ) -> List[ET.Element]:
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
        new_elts = []

        # Convert file paths for mesh elements if necessary
        if elt.tag == "collision":
            new_elts = self.convert_collision_element(elt)
        elif elt.tag == "visual":
            new_elts = [self.convert_visual_element(elt)]
        elif elt.tag == "transmission":
            # Ignore transmission elements during the conversion
            new_elts = [deepcopy(elt)]
        elif elt.tag == "joint":
            self.handle_joint_element(elt)
            new_elts = [deepcopy(elt)]
        else:
            # If the element is not a mesh element, then we will just return a copy of it.
            self.log(
                f"Found an element with tag \"{elt.tag}\" that does not require handling. No modification needed..."
            )
            new_elts = [deepcopy(elt)]

        # Otherwise, just return a copy of the previous element.
        return new_elts

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
            new_urdf_dir=self.file_manager.output_file_directory(),
            logger=self.logger,
        )
        new_mesh_path = converter.convert()

        # Clip off all parts of path that include the exported output
        new_mesh_path = str(new_mesh_path).replace(str(self.file_manager.output_file_directory()), "")

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

    def create_new_material_element(self, material_elt: ET.Element):
        """
        Description
        -----------
        This method will handle the material element in the URDF file.
        Specifically, it will check to see if the user wants us to assign a color to the 
        material element. If so, then we will assign the color to the material element.

        This is useful when attempting to add transmissions to the URDF file
        later on.
        
        Arguments
        ---------
        material_elt: ET.Element
            The material element that we want to handle.
        
        Returns
        -------
        None, but modifies the material element.
        """
        # Setup
        replace_colors_with = self.config.replace_colors_with
        material_elt = deepcopy(material_elt)

        # Input Processing (Abort, if no color replacement is needed)
        if replace_colors_with is None:
            return material_elt
        
        if len(replace_colors_with) != 4:
            raise ValueError(
                "The color replacement list must have exactly 4 elements (RGBA)."
            )
        
        # Algorithm
        # Check to see if the material element has a color element
        color_elt = material_elt.find("color")
        if color_elt is None:
            # If no color element exists, then we will create one
            color_elt = ET.Element("color")
            material_elt.append(color_elt)
        else:
            # If a color element exists, then we will modify it
            material_elt.remove(color_elt)
            color_elt = ET.Element("color")
            material_elt.append(color_elt)

        # Set the color of the material element
        color_elt.set(
            "rgba",
            f"{replace_colors_with[0]} {replace_colors_with[1]} {replace_colors_with[2]} {replace_colors_with[3]}"
        )
        return material_elt

    def log(self, message: str):
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
        self.logger.log(logging.INFO, message)

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
            new_urdf_dir=self.file_manager.output_file_directory(),
            logger=self.logger,
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

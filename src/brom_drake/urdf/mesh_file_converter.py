from distutils.command.clean import clean

import loguru
import os
from pathlib import Path
from typing import Union, Tuple
import xml.etree.ElementTree as ET

import trimesh

# Internal imports
from brom_drake.urdf.util import URDF_CONVERSION_LOG_LEVEL_NAME


class MeshFileConverter:
    def __init__(
        self,
        mesh_file_path: str,
        urdf_dir: Path,
        new_urdf_dir: Path,
    ):
        """
        Description:
            This class will convert a mesh file to a different format.
        :param brom_models_dir:
        :param mesh_file:
        """
        # Input Processing

        # Setup
        self.mesh_file = mesh_file_path
        self.urdf_dir = urdf_dir
        self.new_urdf_dir = new_urdf_dir

    def convert(
        self,
        output_file_path: Path = None,
    ):
        # Setup
        if output_file_path is None:
            output_file_path = self.define_output_path(output_file_path)

        # Use trimesh to convert
        print(f"urdf_dir: {self.urdf_dir}")
        print(f"true_path: {self.true_mesh_file_path()}")
        print(
            str(self.urdf_dir / self.true_mesh_file_path())
        )
        mesh = trimesh.load_mesh(
            str(self.urdf_dir / self.true_mesh_file_path())
        )

        os.makedirs(self.define_output_path(output_file_path).parent, exist_ok=True)
        mesh.export(self.define_output_path(output_file_path))

        # Define the relative output path
        return self.define_output_path(output_file_path)

    @property
    def output_mesh_directory(self) -> Path:
        new_urdf_dir = self.new_urdf_dir
        return new_urdf_dir

    def define_output_path(self, output_mesh_file: Path = None) -> Path:
        """
        Description:
            This function will define the output path for the mesh file.
        :param output_mesh_file:
        :return:
        """
        # Input Processing
        if output_mesh_file is not None:
            return output_mesh_file

        # Setup
        mesh_file_name = self.mesh_file

        # Check to see if the file type is supported
        for suffix in self.supported_suffixes:
            if suffix in str(mesh_file_name):
                # Strip off all parts of the file that might contain:
                # - The package name
                # - "../"
                # - "./"
                clean_mesh_file_name = str(mesh_file_name)
                if "package://" in clean_mesh_file_name:
                    clean_mesh_file_name = clean_mesh_file_name.replace("package://", "")
                    clean_mesh_file_name = clean_mesh_file_name[
                        clean_mesh_file_name.find("/") + 1:
                    ]


                clean_mesh_file_name = clean_mesh_file_name.replace("../", "")
                clean_mesh_file_name = clean_mesh_file_name.replace("./", "")

                # If we pass this point, then we know that the file type is supported
                # Clean all parts of the file name that are not the name
                output_mesh_file = clean_mesh_file_name.replace(suffix, ".obj")

                # Prepend a models directory to the path
                return self.output_mesh_directory / output_mesh_file

        # If we pass this point, then we know that the file type is not supported
        raise ValueError(
            f"File type {mesh_file_name} is not supported by this function!\n" +
            f"Make sure that the file type is one of the following: {self.supported_suffixes}"
        )

    def find_file_path_in_package(
        self,
        max_depth: int = 10,
    ) -> Path:
        """
        Description
        -----------
        This method will find the file path in the package.

        :param max_depth: The maximum depth to search for the package directory.
        :return:
        """

        # Setup
        mesh_file_name = self.mesh_file

        # Extract the package name and the package directory from mesh file name
        package_dir, package_name = self.find_package_directory_including_mesh(max_depth=max_depth)

        # Create the output file path from what we now know
        return package_dir / Path(mesh_file_name.replace(f"package://{package_name}/", ""))

    def find_package_directory_including_mesh(
        self,
        max_depth: int = 10,
    ) -> Tuple[Path, str]:
        # Setup
        original_urdf_dir = Path(self.urdf_dir)

        # Find the path to the package
        package_found = False
        candidate_path = original_urdf_dir
        search_depth = 1
        while not package_found:
            print(f"candidate_path: {candidate_path}")
            # Check to see if "package.xml" exists in the directory
            if (candidate_path / "package.xml").exists():
                self.log(f"Found package path at {candidate_path}.")
                package_found = True
            else:
                candidate_path = candidate_path.parent
                search_depth += 1

            if search_depth > max_depth:
                raise ValueError(
                    f"Could not find package path for file {original_urdf_dir} after searching {max_depth} directories."
                )

        # Should now have found the package directory
        package_dir = candidate_path

        # Open the package.xml file and find the name of the package
        package_xml = ET.ElementTree(file=package_dir / "package.xml")
        package_root = package_xml.getroot()
        package_name = package_root.find("name").text

        return package_dir, package_name

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

    def mesh_file_path_is_relative(self):
        """
        Description:
            This function will check if the mesh file path is relative.
        """
        # Algorithm
        mesh_file_as_path = Path(self.mesh_file)

        # Check to see if file path starts with "./"
        if self.mesh_file.startswith("./"):
            return True

        # Ignore this if the mesh file contains a package prefix
        if self.mesh_file.startswith("package:"):
            return False

        # Check to see if there is only one part of the path
        # Now, check to see if the target file exists
        complete_file_path = self.urdf_dir / mesh_file_as_path
        exists = complete_file_path.exists()
        if not exists:
            raise FileNotFoundError(
                f"File {complete_file_path} does not exist in directory {os.getcwd()}!"
            )
        # Otherwise, we can assume that the file is in the current directory
        return True

        # If we pass this point, we know that the mesh file path is not relative
        return False

    @property
    def supported_suffixes(self):
        return [".stl", ".dae", ".DAE", ".STL"]

    def true_mesh_file_path(self, max_depth: int = 10) -> Path:
        """
        Description:
            This function will return the true mesh file path.
        """
        # Algorithm
        if self.mesh_file.startswith("package:"):
            return self.find_file_path_in_package(max_depth=max_depth)
        elif self.mesh_file_path_is_relative():
            return Path(self.mesh_file)
        else:
            raise ValueError(
                f"Mesh file path {self.mesh_file} is not supported by this function!\n" +
                "Make sure that the \"filename\" attribute contains a relative path (e.g., starts with \"./\")\n"
                " or a package path (i.e., starts with \"package:\")."
            )
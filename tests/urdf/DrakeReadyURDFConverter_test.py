"""
DrakeReadyURDFConverter_test.py
Description:
    This script includes a set of tests for the conversion of URDF
    files into a new URDF.
"""
from importlib import resources as impresources
import loguru
import os
from pathlib import Path
import trimesh
import unittest
from xml.etree.ElementTree import ElementTree

import numpy as np

# Internal Imports
import brom_drake
from brom_drake.urdf import DrakeReadyURDFConverter
from brom_drake.urdf.util import URDF_CONVERSION_LOG_LEVEL_NAME, URDF_CONVERSION_LEVEL


class DrakeReadyURDFConverterTest(unittest.TestCase):
    def setUp(self):
        """
        Description
        -----------
        Set up for all of the tests.
        :return:
        """
        self.test_urdf1_filename = str(
            impresources.files(brom_drake) / "../../tests/urdf/resources/test1.urdf"
        )
        self.test_urdf2_filename = str(
            impresources.files(brom_drake) / "robots/models/ur/ur10e.urdf"
        )

        self.test_urdf3_filename = str(
            impresources.files(brom_drake) / "../../tests/urdf/resources/test_package/urdf/baxter.urdf"
        )

        self.test_urdf4_filename = str(
            impresources.files(brom_drake) / "../../tests/urdf/resources/test3_relative.urdf"
        )

    def test_convert_tree1(self):
        """
        Description
        -----------
        Tests that we can correctly read a URDF.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename
        converter = DrakeReadyURDFConverter(test_urdf1, overwrite_old_logs=True)
        test_tree = ElementTree(file=test_urdf1)

        # Algorithm
        tree_out = converter.convert_tree(test_tree)

        # Test
        # Verify that the tree's root has the same tag as the original
        self.assertEqual(test_tree.getroot().tag, tree_out.getroot().tag)

        # Verify that the tree's root has the same attributes as the original
        self.assertEqual(test_tree.getroot().attrib, tree_out.getroot().attrib)

    def test_convert_tree_element1(self):
        """
        Description
        -----------
        Tests the ability to correctly convert a single element of a tree,
        when that element is not a mesh element.
        Nothing should be changed by the function.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename
        test_tree = ElementTree(file=test_urdf1)

        loguru.logger.remove()

        # Create converter and call method
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            log_file_name="test.log",
            overwrite_old_logs=True,
        )
        new_root = converter.convert_tree_element(test_tree.getroot())

        # Root of tree should not be modified
        self.assertEqual(test_tree.getroot().tag, new_root.tag)

        self.assertEqual(test_tree.getroot().attrib, new_root.attrib)

    def test_convert_tree_element2(self):
        """
        Description
        -----------
        Tests the ability to correctly convert a single element of a tree,
        when that element is a mesh element.
        The mesh element should be replaced with a Drake-compatible
        obj file.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename
        test_tree = ElementTree(file=test_urdf1)

        loguru.logger.remove()

        # Create converter and call method
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            log_file_name="test_convert_tree_element2.log",
            overwrite_old_logs=True,
        )
        target_mesh_elt = test_tree.find(".//mesh[@filename='./meshes/ur10e/visual/base.dae']")
        new_root = converter.convert_tree_element(target_mesh_elt)

        # Compare the filenames of the mesh and the new obj file
        self.assertNotEqual(
            target_mesh_elt.attrib["filename"],
            new_root.attrib["filename"]
        )

        self.assertIn(
            ".obj",
            new_root.attrib["filename"]
        )

    def test_create_obj_to_replace_mesh_file1(self):
        """
        Description
        -----------
        This test verifies that we can properly create a new obj file
        to replace a given .dae mesh file.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename
        test_tree = ElementTree(file=test_urdf1)

        # Get the tree with the dae file
        dae_tree = test_tree.find(
            ".//mesh[@filename='./meshes/ur10e/visual/base.dae']"
        )

        # Create converter and call method
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            log_file_name="test.log",
            overwrite_old_logs=True,
        )
        new_elt = converter.create_obj_to_replace_mesh_file(dae_tree.attrib["filename"])

        # Check that the file was created
        directory_for_transformed_file = converter.output_file_directory() / Path(dae_tree.attrib["filename"]).parent
        self.assertTrue(
            (directory_for_transformed_file / Path(dae_tree.attrib["filename"]).name.replace(".dae", ".obj")).exists()
        )

        self.assertTrue(True)

    def test_output_file_name1(self):
        """
        Description
        -----------
        This test verifies that our function properly returns the SAME
        output file name when we give it in the construction of the converter.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename
        output_filename = "test.urdf"

        converter = DrakeReadyURDFConverter(
            test_urdf1,
            output_urdf_file_path=output_filename,
            overwrite_old_logs=True,
            log_file_name="test_output_file_name1.log",
        )

        # Test
        self.assertEqual(
            output_filename,
            converter.output_urdf_file_name(),
        )

    def test_convert_urdf1(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file.
        We'll verify that the new URDF exists and that it contains
        mesh elements that only refer to .obj files.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename

        converter = DrakeReadyURDFConverter(
            test_urdf1,
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf1.log",
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the new file exists
        self.assertTrue(
            new_urdf_path.exists()
        )

        # Verify that the new file contains only obj files
        new_tree = ElementTree(file=new_urdf_path)
        for mesh_elt in new_tree.iter("mesh"):
            self.assertIn(
                ".obj",
                mesh_elt.attrib["filename"]
            )

    def test_convert_urdf2(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file. This time, we'll use a more complicated urdf file.
        We'll verify that the new URDF exists and that it contains
        mesh elements that only refer to .obj files.
        :return:
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename

        converter = DrakeReadyURDFConverter(
            test_urdf2,
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf2.log",
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the new file exists
        self.assertTrue(
            new_urdf_path.exists()
        )

        # Verify that the new file contains only obj files
        new_tree = ElementTree(file=new_urdf_path)
        for mesh_elt in new_tree.iter("mesh"):
            self.assertIn(
                ".obj",
                mesh_elt.attrib["filename"]
            )

    def test_convert_urdf3(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file. This time, we'll use a more complicated urdf file.
        We'll verify that the new URDF exists and that it contains
        mesh elements that only refer to .obj files.
        :return:
        """
        # Setup
        test_urdf3 = self.test_urdf3_filename

        converter = DrakeReadyURDFConverter(
            test_urdf3,
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf3.log",
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the new file exists
        self.assertTrue(
            new_urdf_path.exists()
        )

        # Verify that the new file contains only obj files
        new_tree = ElementTree(file=new_urdf_path)
        for mesh_elt in new_tree.iter("mesh"):
            self.assertIn(
                ".obj",
                mesh_elt.attrib["filename"]
            )

    def test_convert_urdf4(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file. This time, we'll use a more complicated urdf file.
        We'll verify that the new URDF exists and that it contains
        mesh elements that only refer to .obj files.
        :return:
        """
        # Setup
        test_urdf4 = self.test_urdf4_filename

        converter = DrakeReadyURDFConverter(
            test_urdf4,
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf4.log",
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the new file exists
        self.assertTrue(
            new_urdf_path.exists()
        )

        # Verify that the new file contains only obj files
        new_tree = ElementTree(file=new_urdf_path)
        for mesh_elt in new_tree.iter("mesh"):
            self.assertIn(
                ".obj",
                mesh_elt.attrib["filename"]
            )

            # Verify that the new mesh files exist
            self.assertTrue(
                (converter.output_file_directory() / Path(mesh_elt.attrib["filename"])).exists()
            )

if __name__ == '__main__':
    unittest.main()
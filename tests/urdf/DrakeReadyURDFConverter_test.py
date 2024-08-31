"""
DrakeReadyURDFConverter_test.py
Description:
    This script includes a set of tests for the conversion of URDF
    files into a new URDF.
"""
import loguru
import os
from pathlib import Path
import unittest
from xml.etree.ElementTree import ElementTree

# Internal Imports
from brom_drake.urdf import DrakeReadyURDFConverter
from brom_drake.urdf.util import URDF_CONVERSION_LOG_LEVEL_NAME, URDF_CONVERSION_LEVEL


class DrakeReadyURDFConverterTest(unittest.TestCase):
    def setUp(self):
        """
        Description
        -----------

        :return:
        """
        self.test_urdf1_filename = "./resources/test1.urdf"

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

        self.assertTrue(True)

    def test_convert_tree_element1(self):
        """
        Description
        -----------
        Tests that we can correctly read a URDF.
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
        test_output_dir = Path("resources/meshes/test_create_obj_to_replace_mesh_file1/")

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
            converter.output_file_name(),
        )


if __name__ == '__main__':
    unittest.main()
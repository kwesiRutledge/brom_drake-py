from importlib import resources as impresources
import unittest
from xml.etree.ElementTree import ElementTree

# Internal imports
import brom_drake
from brom_drake.file_manipulation.urdf import (
    tree_contains_transmission_for_joint,
    create_transmission_element_for_joint,
)
import resources as resources_dir


class UtilTest(unittest.TestCase):
    def setUp(self):
        """
        Description
        -----------
        Set up for all of the tests.
        :return:
        """
        self.test_urdf1_filename = str(
            impresources.files(resources_dir) / "test1.urdf"
        )
        self.test_urdf2_filename = str(
            impresources.files(resources_dir) / "test2.urdf"
        )

    def test_tree_contains_transmission_for_joint1(self):
        """
        Description:
            This test checks if the function tree_contains_transmission_for_joint
            correctly identifies when a tree DOES NOT contain a transmission element
            for a given joint.
            (The associated tree contains 0 transmission elements)
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf1_filename
        test_tree = ElementTree(file=test_urdf1)

        # Run the function
        self.assertFalse(
            tree_contains_transmission_for_joint(
                test_tree,
                "joint1"
            ),
        )

    def test_tree_contains_transmission_for_joint2(self):
        """
        Description:
            This test checks if the function tree_contains_transmission_for_joint
            correctly identifies when a tree DOES NOT contain a transmission element
            for a given joint.
            (The associated tree contains 1 transmission element)
        :return:
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename
        test_tree = ElementTree(file=test_urdf2)

        # Run the function
        self.assertFalse(
            tree_contains_transmission_for_joint(
                test_tree,
                "wrist_2_joint"
            ),
        )

    def test_tree_contains_transmission_for_joint3(self):
        """
        Description:
            This test checks if the function tree_contains_transmission_for_joint
            correctly identifies when a tree DOES contain a transmission element
            for a given joint.
            (The associated tree contains 1 transmission element)
        :return:
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename
        test_tree = ElementTree(file=test_urdf2)

        # Run the function
        self.assertTrue(
            tree_contains_transmission_for_joint(
                test_tree,
                "wrist_3_joint"
            ),
        )

    def test_create_transmission_element_for_joint1(self):
        """
        Description
        -----------
        This test verifies that the element created by create_transmission_element_for_joint
        is correct.
        It should contain:
            - A transmission element
            - A type element
            - A joint element
            - An actuator element
        :return:
        """
        # Setup
        actuated_joint_name = "joint1"

        # Run the function
        transmission_element = create_transmission_element_for_joint(actuated_joint_name)

        # Check the transmission element
        for child in transmission_element:
            if child.tag == "type":
                self.assertEqual(
                    child.text,
                    "transmission_interface/SimpleTransmission"
                )

            if child.tag == "joint":
                self.assertEqual(
                    child.attrib["name"],
                    actuated_joint_name
                )

            if child.tag == "actuator":
                self.assertEqual(
                    child.attrib["name"],
                    f"{actuated_joint_name}_actuator"
                )

if __name__ == "__main__":
    unittest.main()
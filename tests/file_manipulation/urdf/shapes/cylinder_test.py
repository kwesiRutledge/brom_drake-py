"""
Description:
    This script tests the cylinder class in the shapes module.
"""
import numpy as np
from pydrake.all import (
    RigidTransform, RollPitchYaw,
    Simulator,
)
import unittest
import xml.etree.ElementTree as ET

# Internal Imports
from brom_drake.directories import DEFAULT_BROM_MODELS_DIR
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import SimpleShapeURDFDefinition
from brom_drake.file_manipulation.urdf.shapes.cylinder import CylinderDefinition
from brom_drake.productions.debug.show_me import ShowMeThisModel

class TestCylinder(unittest.TestCase):
    """
    A class to test the Cylinder class.
    """
    def test_init1(self):
        """
        Test the __init__ method.
        """
        cylinder = CylinderDefinition(1, 2)
        self.assertEqual(cylinder.radius, 1)
        self.assertEqual(cylinder.length, 2)

    def test_add_geometry_to_element1(self):
        """
        Test the add_geometry_to_element method properly creates the cylinder element.
        """
        # Setup
        dummy_element = ET.Element("dummy")

        # Run
        cylinder = CylinderDefinition(1, 2)
        element = cylinder.add_geometry_to_element(dummy_element)
        self.assertEqual(element.tag, "cylinder")
        self.assertEqual(element.attrib, {"radius": "1", "length": "2"})

    def test_in_urdf1(self):
        """
        Test the in_urdf method.
        """
        # Setup
        urdf_location = DEFAULT_BROM_MODELS_DIR + "/test_cylinder.urdf"

        # Create cylinder
        cylinder = CylinderDefinition(1, 2)

        # Add to URDF Definition
        defn1 = SimpleShapeURDFDefinition(
            name="cylinder",
            shape=cylinder,
            create_collision=True,
            pose=RigidTransform(
                R=RollPitchYaw(np.pi/4, 0, 0).ToRotationMatrix(),
            ),
        )
        defn1.write_to_file(urdf_location)

        # Try to load this into a simple simulation
        scene = ShowMeThisModel(urdf_location, meshcat_port_number=None)
        diagram, diagram_context = scene.add_cast_and_build()

        # Simulate the scene
        sim = Simulator(diagram, diagram_context)
        sim.AdvanceTo(1.0)

        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
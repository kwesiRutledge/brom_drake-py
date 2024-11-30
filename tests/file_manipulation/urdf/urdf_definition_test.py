import unittest
import xml.etree.ElementTree as ET

# Internal imports
from brom_drake.file_manipulation.urdf.shapes.box import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import SimpleShapeURDFDefinition

class URDFDefinitionTest(unittest.TestCase):
    def test_add_visual_elements_to1(self):
        """
        Test the add_visual_elements_to() method of the URDFDefinition class.
        :return:
        """
        # Setup
        shape1 = BoxDefinition(
            size=(1.0, 1.0, 1.0),
        )

        # Create the SimpleShapeURDF object
        defn1 = SimpleShapeURDFDefinition(
            name="test1",
            shape=shape1,
        )
        root = ET.Element("robot", {"name": f"{defn1.name}_robot"})
        link = ET.SubElement(root, "link", {"name": defn1.name + "_base_link"})

        self.assertEqual(0, len(link.findall("visual")))

        # Call the method
        defn1.add_visual_elements_to(link)

        print(ET.tostring(root).decode())

        # Check on the elements on the link; there should be:
        # - an element with the visual tag
        # - an element with the box tag exists

        self.assertEqual(1, len(link.findall("visual")))
        self.assertEqual(1, len(link.findall("visual/geometry/box")))


if __name__ == '__main__':
    unittest.main()

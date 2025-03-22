"""
DrakeReadyURDFConverter_test.py
Description:
    This script includes a set of tests for the conversion of URDF
    files into a new URDF.
"""
from importlib import resources as impresources
import loguru
import numpy as np
from pathlib import Path
import unittest
from xml.etree.ElementTree import ElementTree

from pydrake.systems.analysis import Simulator
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant

# Internal Imports
import brom_drake
from brom_drake import robots
from brom_drake.all import drakeify_my_urdf
from brom_drake.file_manipulation.urdf import (
    DrakeReadyURDFConverter, 
    DrakeReadyURDFConverterConfig,
    MeshReplacementStrategies,
    MeshReplacementStrategy, 
)
import resources as resources_dir
from brom_drake.productions.debug import ShowMeThisModel


class DrakeReadyURDFConverterTest(unittest.TestCase):
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
            impresources.files(brom_drake) / "robots/models/ur/ur10e.urdf"
        )

        self.test_urdf3_filename = str(
            impresources.files(resources_dir) / "test_package/urdf/baxter.urdf"
        )

        self.test_urdf4_filename = str(
            impresources.files(resources_dir) / "test3_relative.urdf"
        )

        self.test_urdf5_filename = str(
            impresources.files(resources_dir) / "test4_relative.urdf"
        )

        self.test_urdf6_filename = str(
            impresources.files(resources_dir) / "test5_absolute.urdf"
        )

        self.test_urdf7_filename = str(
            impresources.files(resources_dir) / "test6_multiple_collision_geometries.urdf"
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
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_tree1.log",
        )
        converter = DrakeReadyURDFConverter(test_urdf1, config=config)
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

        # Create configuration for the converter
        config = DrakeReadyURDFConverterConfig(
            log_file_name="test_convert_tree_element1.log",
            overwrite_old_logs=True,
        )

        # Create converter and call method
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            config=config,
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

        # Create configuration for the converter
        config = DrakeReadyURDFConverterConfig(
            log_file_name="test_convert_tree_element2.log",
            overwrite_old_logs=True,
        )

        # Create converter and call method
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            config=config,
        )
        # Find an element that references a .dae file
        target_visual_elt = test_tree.getroot().find("link/visual")
        new_visual_elt = converter.convert_tree_element(target_visual_elt)

        # Compare the filenames of the mesh and the new obj file
        self.assertNotEqual(
            target_visual_elt.find("geometry/mesh").attrib["filename"],
            new_visual_elt.find("geometry/mesh").attrib["filename"]
        )

        self.assertIn(
            ".obj",
            new_visual_elt.find("geometry/mesh").attrib["filename"]
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

        # Define configuration for the converter
        config = DrakeReadyURDFConverterConfig(
            log_file_name="test_create_obj_to_replace_mesh_file1.log",
            overwrite_old_logs=True,
        )

        # Create converter and call method
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            config=config,
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

        # Create config
        config = DrakeReadyURDFConverterConfig(
            output_urdf_file_path=output_filename,
            overwrite_old_logs=True,
            log_file_name="test_output_file_name1.log",
        )

        # Create converter
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            config=config,
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
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf1.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            config=config,
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
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf2.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf2,
            config=config,
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

        # Verify that the new file contains transmission elements
        transmissions_found = new_tree.findall(".//transmission")
        self.assertGreater(len(transmissions_found), 0)


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
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf3.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf3,
            config=config,
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
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf4.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf4,
            config=config,
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

    def test_convert_urdf5(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file. This time, we'll use a more complicated urdf file
        that contains relative paths expressed with the "path://" prefix.
        We'll verify that the new URDF exists and that it contains
        mesh elements that only refer to .obj files.
        :return:
        """
        # Setup
        test_urdf5 = self.test_urdf5_filename
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf5.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf5,
            config=config,
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

    def test_convert_urdf6(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file. This time, we'll use a more complicated urdf file
        that contains ABSOLUTE paths expressed with the "path://" prefix.
        We'll verify that the new URDF exists and that it contains
        mesh elements that only refer to .obj files.
        :return:
        """
        # Setup
        test_urdf5 = self.test_urdf5_filename
        test_urdf6 = self.test_urdf6_filename

        # Create pay to the collision base file
        test_dir = Path(__file__).parent

        # Create a copy of the xml in test_urdf5 but with absolute paths
        urdf6_tree = ElementTree(file=test_urdf5)
        for mesh_elt in urdf6_tree.iter("mesh"):
            mesh_elt.attrib["filename"] = f"file://{str(test_dir)}/resources/meshes/ur10e/collision/base.stl"

        urdf6_tree.write(
            test_urdf6,
            encoding="utf-8",
            xml_declaration=True,
        )
        # Use converter on urdf6
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf6.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf6,
            config=config,
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the new file exists
        self.assertTrue(new_urdf_path.exists())

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

        # Make sure that this can be added to a plant
        temp_plant = MultibodyPlant(time_step=1e-3)
        added_models = Parser(temp_plant).AddModels(str(new_urdf_path))
        self.assertTrue(True)

    def test_convert_urdf7(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        with the option to replace collision geometries with visual geometries.
        """
        # Setup
        test_urdf = self.test_urdf1_filename

        # Create pay to the collision base file
        test_dir = Path(__file__).parent

        # Create config for the converter
        config = DrakeReadyURDFConverterConfig(
            output_urdf_file_path="./brom/resources/test_convert_urdf7.urdf",
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf7.log",
            mesh_replacement_strategies=MeshReplacementStrategies(
                collision_meshes=MeshReplacementStrategy.kWithMinimalEnclosingCylinder,
            ),
        )

        # Use converter on urdf6
        converter = DrakeReadyURDFConverter(
            test_urdf,
            config=config,
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the new file exists
        self.assertTrue(new_urdf_path.exists())

        # Verify that the new file's collision elements DO NOT contain mesh elements
        new_tree = ElementTree(file=new_urdf_path)
        all_mesh_elts = list(new_tree.iter("mesh"))
        self.assertTrue(
            len(all_mesh_elts) == 1
        )

        # Search through all the collision elements and make sure they are cylinders
        for collision_elt in new_tree.iter("collision"):
            self.assertEqual(
                collision_elt.find("geometry").find("cylinder").tag,
                "cylinder"
            )

        # Make sure that this can be added to a plant
        temp_plant = MultibodyPlant(time_step=1e-3)
        added_models = Parser(temp_plant).AddModels(str(new_urdf_path))
        self.assertTrue(True)

    def test_convert_urdf8(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file when the urdf contains MULTIPLE collision geometries
        in one of the links.

        We'll verify that:
        - the new URDF exists,
        - that it contains mesh elements that only refer to .obj files, and
        - that the multiple collision geometries exist.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdf7_filename
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf8.log",
        )
        converter = DrakeReadyURDFConverter(
            test_urdf1,
            config=config,
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

        # Verify that the new file contains multiple collision geometries
        collision_elt = new_tree.find(".//link/collision")
        self.assertTrue(collision_elt is not None)
        
        self.assertEqual(
            len(list(collision_elt.iter("geometry"))),
            2,
        )

        # at least one of the geometries should be a cylinder
        self.assertTrue(
            any(
                geom.find("cylinder") is not None
                for geom in collision_elt.iter("geometry")
            )
        )

    def test_convert_urdf9(self):
        """
        Description
        -----------
        This test verifies that we can convert a full URDF file
        into a new URDF file. This time, we'll use a more complicated urdf file
        AND we will set the config object to NOT add missing actuators.
        We'll verify that:
        - the new URDF exists,
        - that it contains mesh elements that only refer to .obj files, and
        - that the new URDF does not contain any transmission elements.
        :return:
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf9.log",
            add_missing_actuators=False,
        )
        converter = DrakeReadyURDFConverter(
            test_urdf2,
            config=config,
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

        # Verify that the new file contains transmission elements
        transmissions_found = new_tree.findall(".//transmission")
        self.assertEqual(len(transmissions_found), 0)

    def test_convert_urdf10(self):
        """
        Description
        -----------
        In this test, we verify that the converter can properly convert a 
        URDF file composed of all mesh files that are supported by Drake.
        We do this by running a URDF through the converter twice.
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf10-1.log",
            add_missing_actuators=False,
        )
        converter = DrakeReadyURDFConverter(
            test_urdf2,
            config=config,
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Run conversion a second time
        second_urdf_path = "./brom/resources/test_convert_urdf10-2.urdf"
        converter2 = DrakeReadyURDFConverter(
            new_urdf_path,
            config=DrakeReadyURDFConverterConfig(
                overwrite_old_logs=True,
                log_file_name="test_convert_urdf10-2.log",
                output_urdf_file_path=second_urdf_path,
            ),
        )
        second_urdf_path = converter2.convert_urdf()

        # Verify that the first new file exists
        self.assertTrue(
            new_urdf_path.exists()
        )

        # Verify that the second new file exists
        self.assertTrue(
            second_urdf_path.exists()
        )

    def test_convert_urdf11(self):
        """
        Description
        -----------
        In this test, we verify that the converter properly throws an error
        when we provide a color to the converter that is not in the rgba format.
        In other words, it is not a list of four floats.
        Let's catch the exception and verify that it is problematic.
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf11.log",
            replace_colors_with=[1.0, 0.5, 0.2],
        )
        converter = DrakeReadyURDFConverter(
            test_urdf2,
            config=config,
        )

        # Test
        with self.assertRaises(ValueError) as context:
            new_urdf_path = converter.convert_urdf()

        self.assertTrue(
            "The color replacement list must have exactly 4 elements (RGBA)." in str(context.exception)
        )

    def test_convert_urdf12(self):
        """
        Description
        -----------
        In this test, we verify that the converter properly inserts RGBA values
        and material fields into a urdf after conversion when the `replace_colors_with`
        field is defined.
        """
        # Setup
        test_urdf2 = self.test_urdf2_filename
        config = DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
            log_file_name="test_convert_urdf12.log",
            replace_colors_with=[1.0, 0.5, 0.2, 0.1],
        )
        converter = DrakeReadyURDFConverter(
            test_urdf2,
            config=config,
        )

        # Test
        new_urdf_path = converter.convert_urdf()

        # Verify that the material elements were added to the urdf
        new_tree = ElementTree(file=new_urdf_path)
        n_materials_found = len(new_tree.findall(".//material"))
        self.assertGreater(n_materials_found, 0)

        for material_elt in new_tree.iter("material"):
            self.assertEqual(
                material_elt.find("color").attrib["rgba"],
                "1.0 0.5 0.2 0.1"
            )

    # TODO: Fix this test

    # def test_convert_urdf10(self):
    #     """
    #     Description
    #     -----------
    #     This test verifies that we can convert a full URDF file
    #     into a new URDF file. This time, we'll use a more complicated urdf file
    #     and we will set the configuration to NOT MODIFY the visual elements.
    #     We'll verify that:
    #     - the new URDF exists,
    #     - that it contains collision mesh elements that only refer to .obj files, and
    #     - that the new URDF's visual mesh elements are the same as the original.
    #     :return:
    #     """
    #     # Setup
    #     test_urdf3 = self.test_urdf3_filename
    #     config = DrakeReadyURDFConverterConfig(
    #         overwrite_old_logs=True,
    #         log_file_name="test_convert_urdf10.log",
    #         output_urdf_file_path="./brom/resources/test_convert_urdf10.urdf",
    #         mesh_replacement_strategies=MeshReplacementStrategies(
    #             visual_meshes=MeshReplacementStrategy.kDoNotReplace,
    #         ),
    #     )
    #     converter = DrakeReadyURDFConverter(
    #         test_urdf3,
    #         config=config,
    #     )

    #     # Test
    #     new_urdf_path = converter.convert_urdf()

    #     # Verify that the new file exists
    #     self.assertTrue(
    #         new_urdf_path.exists()
    #     )

    #     # Verify that the new file contains only obj files
    #     new_tree = ElementTree(file=new_urdf_path)
    #     old_tree = ElementTree(file=test_urdf3)

    #     # Find all the visual elements
    #     old_visual_elts = list(old_tree.iter("visual"))
    #     new_visual_elts = list(new_tree.iter("visual"))

    #     # Compare the filenames of the meshes in the old and the new obj file (should be the same)
    #     for ii, old_visual_elt in enumerate(old_visual_elts[0]):
    #         # Select the geometry element inside
    #         if old_visual_elt.tag == "geometry":
    #             old_geometry_elt = old_visual_elt
    #             new_geometry_elt = new_visual_elts[0][ii]
    #             # Select the mesh element inside
    #             for jj, old_geometry_child in enumerate(old_geometry_elt.iter("mesh")):
    #                 if old_geometry_child.tag == "mesh":
    #                     old_mesh_elt = old_geometry_child
    #                     new_mesh_elt = new_geometry_elt[jj]

    #                     self.assertEqual(
    #                         old_mesh_elt.attrib["filename"],
    #                         new_mesh_elt.attrib["filename"]
    #                     )

    #                     # Check that the file exists
    #                     print("output file directory")
    #                     print(converter.output_file_directory())
    #                     print("new mesh elt")
    #                     print(new_mesh_elt.attrib["filename"])
    #                     print(new_mesh_elt.attrib["filename"].replace("package://baxter_description/", ""))
    #                     self.assertTrue(
    #                         (
    #                             converter.output_file_directory() / 
    #                             Path(new_mesh_elt.attrib["filename"].replace("package://baxter_description/", ""))
    #                         ).exists()
    #                     )

    #     # Verify that the new collision mesh files are obj files
    #     collision_geometry_elts = new_tree.findall(".//collision/geometry")
    #     for collision_geometry_elt in collision_geometry_elts:
    #         self.assertIn(
    #             ".obj",
    #             collision_geometry_elt.find("mesh").attrib["filename"]
    #         )
    
    def test_vis1(self):

        # Setup
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        # Convert the URDF
        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
            collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithMinimalEnclosingCylinder,
        )

        # Visualize the URDF using the "show-me-this-model" feature
        time_step = 1e-3
        production = ShowMeThisModel(
            str(new_urdf_path),
            with_these_joint_positions=[0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0],
            time_step=time_step,
            meshcat_port_number=7003,
        )

        # Build Diagram
        diagram, diagram_context = production.add_cast_and_build()

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        # simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(5.0)

if __name__ == '__main__':
    unittest.main()
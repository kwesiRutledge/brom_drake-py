import unittest

# Brom Imports
from brom_drake.file_manipulation.urdf import (
    DrakeReadyURDFConverter, 
    DrakeReadyURDFConverterConfig,
    MeshReplacementStrategies,
    MeshReplacementStrategy, 
)

class DrakeReadyURDFConverterFileManagerTest(unittest.TestCase):
    def setUp(self):
        """
        Description
        -----------
        Set up for all of the tests.
        :return:
        """
        import importlib.resources as impresources
        import resources as resources_dir

        self.test_urdfs = [
            str(impresources.files(resources_dir) / "test1.urdf"),
            str(impresources.files(resources_dir) / "test2.urdf"),
            str(impresources.files(resources_dir) / "test7_no_mesh_in_collision.urdf"),
        ]
    def test_output_file_name1(self):
        """
        Description
        -----------
        This test verifies that our function properly returns the SAME
        output file name when we give it in the construction of the converter.
        :return:
        """
        # Setup
        test_urdf1 = self.test_urdfs[0]
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
            converter.file_manager.output_urdf_file_name(),
        )
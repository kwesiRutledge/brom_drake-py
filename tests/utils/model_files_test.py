import os
from pathlib import Path
import unittest

# Internal Imports
from brom_drake.utils.model_files import is_package_url, resolve_model_path

# Ships inside the drake wheel, so resolving it needs no download.
DRAKE_PACKAGE_SDF = "package://drake/examples/planar_gripper/planar_brick.sdf"


class ResolveModelPathTest(unittest.TestCase):
    def test_file_paths_are_unchanged1(self):
        self.assertEqual(
            resolve_model_path("some/dir/model.urdf"), "some/dir/model.urdf"
        )

    def test_path_objects_become_strings1(self):
        self.assertEqual(resolve_model_path(Path("a/b.sdf")), "a/b.sdf")

    def test_package_url_is_resolved_to_a_file1(self):
        resolved = resolve_model_path(DRAKE_PACKAGE_SDF)
        self.assertFalse(is_package_url(resolved))
        self.assertTrue(os.path.isfile(resolved))
        self.assertTrue(resolved.endswith("planar_brick.sdf"))

    def test_unknown_package_raises_value_error1(self):
        with self.assertRaisesRegex(ValueError, "no_such_package"):
            resolve_model_path("package://no_such_package/model.sdf")

    def test_missing_file_in_known_package_raises_value_error1(self):
        with self.assertRaises(ValueError):
            resolve_model_path("package://drake/no_such_file.sdf")

    def test_is_package_url1(self):
        self.assertTrue(is_package_url("package://drake_models/ycb/004_sugar_box.sdf"))
        self.assertFalse(is_package_url("/abs/path.sdf"))
        self.assertFalse(is_package_url(Path("package:/x")))


if __name__ == "__main__":
    unittest.main()

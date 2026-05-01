"""
mesh_file_converter_test.py
Description:
    Tests for MeshFileConverter, with a focus on resolving mesh files
    that are referenced via ROS 2 package:// URIs.

    The ROS 2 ament path (Option 3) is tested by mocking
    _AMENT_INDEX_AVAILABLE and get_package_share_directory so that the
    tests run without requiring a live ROS 2 installation.
"""

import logging
import tempfile
import unittest
from importlib import resources as impresources
from pathlib import Path
from unittest.mock import patch

import resources as resources_dir

from brom_drake.file_manipulation.urdf.drake_ready_urdf_converter.mesh_file_converter import (
    MeshFileConverter,
)

# Module path used for patching
_MODULE = (
    "brom_drake.file_manipulation.urdf.drake_ready_urdf_converter.mesh_file_converter"
)

# Shared test data
_TEST_PACKAGE_DIR = Path(str(impresources.files(resources_dir) / "test_package"))
_PACKAGE_NAME = "baxter_description"
_MESH_URI = f"package://{_PACKAGE_NAME}/meshes/torso/base_link.DAE"
_EXPECTED_RELATIVE_MESH_PATH = Path("meshes/torso/base_link.DAE")
_PARENT_RELATIVE_MESH_PATH = "../meshes/torso/base_link_collision.DAE"


def _make_converter(mesh_uri: str, new_urdf_dir: Path) -> MeshFileConverter:
    return MeshFileConverter(
        mesh_file_path=mesh_uri,
        urdf_dir=_TEST_PACKAGE_DIR / "urdf",
        new_urdf_dir=new_urdf_dir,
        logger=logging.getLogger("MeshFileConverterTest"),
    )


class MeshFileConverterAmentTest(unittest.TestCase):
    """
    Tests for the ament_index_python code path in MeshFileConverter.
    The ament index is mocked so no ROS 2 installation is required.
    """

    def _ament_patches(self, package_share_dir: Path):
        """Return context-manager patches that simulate ament being available."""
        return (
            patch(f"{_MODULE}._AMENT_INDEX_AVAILABLE", True),
            patch(
                f"{_MODULE}.get_package_share_directory",
                return_value=str(package_share_dir),
                create=True,
            ),
        )

    # ------------------------------------------------------------------
    # find_package_directory_including_mesh
    # ------------------------------------------------------------------

    def test_find_package_directory_uses_ament_when_available(self):
        """
        When _AMENT_INDEX_AVAILABLE is True,
        find_package_directory_including_mesh should delegate to
        get_package_share_directory and return the correct
        (package_dir, package_name) pair.
        """
        with tempfile.TemporaryDirectory() as tmp:
            converter = _make_converter(_MESH_URI, Path(tmp))

            p1, p2 = self._ament_patches(_TEST_PACKAGE_DIR)
            with p1, p2:
                package_dir, package_name = (
                    converter.find_package_directory_including_mesh()
                )

        self.assertEqual(package_dir, _TEST_PACKAGE_DIR)
        self.assertEqual(package_name, _PACKAGE_NAME)

    def test_find_package_directory_passes_correct_package_name_to_ament(self):
        """
        get_package_share_directory must be called with exactly the
        package name extracted from the package:// URI.
        """
        with tempfile.TemporaryDirectory() as tmp:
            converter = _make_converter(_MESH_URI, Path(tmp))

            p1, p2 = self._ament_patches(_TEST_PACKAGE_DIR)
            with p1, p2 as mock_get_share:
                converter.find_package_directory_including_mesh()
                mock_get_share.assert_called_once_with(_PACKAGE_NAME)

    # ------------------------------------------------------------------
    # true_mesh_file_path
    # ------------------------------------------------------------------

    def test_true_mesh_file_path_resolves_via_ament(self):
        """
        true_mesh_file_path should return the full path to the mesh file
        inside the package share directory when ament is available.
        """
        with tempfile.TemporaryDirectory() as tmp:
            converter = _make_converter(_MESH_URI, Path(tmp))

            p1, p2 = self._ament_patches(_TEST_PACKAGE_DIR)
            with p1, p2:
                resolved = converter.true_mesh_file_path()

        expected = _TEST_PACKAGE_DIR / _EXPECTED_RELATIVE_MESH_PATH
        self.assertEqual(resolved, expected)

    def test_true_mesh_file_path_accepts_parent_relative_path(self):
        """
        true_mesh_file_path should accept parent-directory relative paths
        (e.g., ../meshes/...) and return them as relative paths.
        """
        with tempfile.TemporaryDirectory() as tmp:
            converter = _make_converter(_PARENT_RELATIVE_MESH_PATH, Path(tmp))

            resolved = converter.true_mesh_file_path()

        self.assertEqual(resolved, Path(_PARENT_RELATIVE_MESH_PATH))
        self.assertTrue((converter.urdf_dir / resolved).exists())

    # ------------------------------------------------------------------
    # convert (full pipeline)
    # ------------------------------------------------------------------

    def test_convert_produces_obj_file_via_ament(self):
        """
        The full convert() pipeline should:
        1. Resolve the mesh through ament.
        2. Load the DAE file with trimesh.
        3. Export an .obj to new_urdf_dir.
        """
        with tempfile.TemporaryDirectory() as tmp:
            new_urdf_dir = Path(tmp)
            converter = _make_converter(_MESH_URI, new_urdf_dir)

            p1, p2 = self._ament_patches(_TEST_PACKAGE_DIR)
            with p1, p2:
                output_path = converter.convert()

            self.assertTrue(
                output_path.exists(),
                f"Expected output file at {output_path} but it was not created.",
            )
            self.assertEqual(
                output_path.suffix,
                ".obj",
                f"Expected a .obj file, got {output_path.suffix}.",
            )

    # ------------------------------------------------------------------
    # Fallback: ament unavailable → upward walk
    # ------------------------------------------------------------------

    def test_find_package_directory_falls_back_to_walk_when_ament_unavailable(self):
        """
        When _AMENT_INDEX_AVAILABLE is False (the default for non-ROS users),
        find_package_directory_including_mesh should fall back to walking up
        the directory tree and reading package.xml.
        """
        with tempfile.TemporaryDirectory() as tmp:
            converter = _make_converter(_MESH_URI, Path(tmp))

            with patch(f"{_MODULE}._AMENT_INDEX_AVAILABLE", False):
                package_dir, package_name = (
                    converter.find_package_directory_including_mesh()
                )

        self.assertEqual(package_dir, _TEST_PACKAGE_DIR)
        self.assertEqual(package_name, _PACKAGE_NAME)


if __name__ == "__main__":
    unittest.main()

import numpy as np
from pathlib import Path
from pydrake.all import HPolyhedron
import unittest

# Internal Imports
from brom_drake.utils.pick_and_place.target_description import (
    PickAndPlaceTargetDescription,
)


class PickAndPlaceTargetDescriptionTest(unittest.TestCase):
    def test_name1(self):
        """
        Description
        -----------
        This function verifies that we can create a name for the target
        when ONLY the file path (and the other required fields) are given.
        """
        # Setup
        expected_name = "hasan"
        test_path: Path = Path("./outputs/" + expected_name + ".urdf")
        descr0 = PickAndPlaceTargetDescription(
            file_path=test_path,
            goal_region=HPolyhedron.MakeBox(lb=-np.ones((3,)), ub=np.ones((3,))),
        )

        # Test
        self.assertEqual(expected_name, descr0.name)

    def test_name2(self):
        """
        Description
        -----------
        This function verifies that we can create a name for the target
        when the file path (and the other required fields) are given
        AS WELL AS the optional _name field.
        """
        # Setup
        expected_name = "hasan"
        file_path_name = "rabbit"
        test_path: Path = Path("./outputs/" + file_path_name + ".urdf")
        descr0 = PickAndPlaceTargetDescription(
            file_path=test_path,
            goal_region=HPolyhedron.MakeBox(lb=-np.ones((3,)), ub=np.ones((3,))),
            _name=expected_name,
        )

        # Test
        self.assertEqual(expected_name, descr0.name)
        self.assertNotEqual(file_path_name, descr0.name)


if __name__ == "__main__":
    unittest.main()

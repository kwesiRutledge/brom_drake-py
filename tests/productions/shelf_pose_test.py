import unittest

import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw

# Internal Imports
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2
from brom_drake.productions.motion_planning.offline.kinematic.chem_lab1 import ChemLab1
from brom_drake.productions.pick_and_place.chem_lab3 import ChemLab3


class ShelfPoseTest(unittest.TestCase):
    """
    Description
    -----------
    Regression tests for the ``shelf_pose`` constructor argument of the
    ChemLab productions. Passing a pose used to leave ``pose_WorldShelf``
    unset, so the shelf was never placed where asked.
    """

    production_classes = [ChemLab1, ChemLab2, ChemLab3]

    def test_passed_shelf_pose_reaches_pose_WorldShelf1(self):
        """
        Description
        -----------
        A non-default shelf_pose is used as pose_WorldShelf.
        """
        # Setup
        custom_pose = RigidTransform(
            RollPitchYaw(0.0, 0.0, 0.3).ToQuaternion(),
            np.array([0.1, 0.8, 0.5]),
        )

        for production_class in self.production_classes:
            with self.subTest(production=production_class.__name__):
                production = production_class(
                    meshcat_port_number=None,
                    shelf_pose=custom_pose,
                )
                self.assertTrue(
                    production.pose_WorldShelf.IsExactlyEqualTo(custom_pose)
                )

    def test_default_shelf_pose_is_set1(self):
        """
        Description
        -----------
        Without a shelf_pose, pose_WorldShelf still gets the production's default.
        """
        for production_class in self.production_classes:
            with self.subTest(production=production_class.__name__):
                production = production_class(meshcat_port_number=None)
                self.assertIsInstance(production.pose_WorldShelf, RigidTransform)


if __name__ == "__main__":
    unittest.main()

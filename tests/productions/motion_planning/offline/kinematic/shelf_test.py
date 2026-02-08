import numpy as np
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
import unittest

from brom_drake.productions.ids import ProductionID
from brom_drake.productions.motion_planning.offline.kinematic.shelf import (
    ShelfPlanning1,
)
from brom_drake.productions.roles.all import kKinematicMotionPlanner


class ShelfTest(unittest.TestCase):
    def test_add_all_secondary_cast_members_to_builder1(self):
        """
        Test the add_all_secondary_cast_members_to_builder() method
        of the ShelfPlanning Production class.
        :return:
        """
        # Setup
        shelf_planning_production = ShelfPlanning1(meshcat_port_number=None)

        # Check that defaults are in place
        self.assertEqual(shelf_planning_production.id, ProductionID.kShelfPlanning1)
        self.assertEqual(
            shelf_planning_production.suggested_roles(), [kKinematicMotionPlanner]
        )

        # Populate production with builder
        shelf_planning_production.add_supporting_cast()
        self.assertTrue(True)

    def test_add_cast_and_build1(self):
        pass  # TODO(kwesi) Implement this test

    def test_goal_pose1(self):
        """
        Description
        -----------
        This test verifies that the ShelfPlanning1's goal_pose method.
        The method should not raise an error when the object
        is initialized with a goal configuration input (instead
        of a goal pose).
        """
        # Setup
        production = ShelfPlanning1(
            meshcat_port_number=None,
            goal_configuration=np.zeros((6,)),
        )

        # Call goal_pose
        goal_pose = production.goal_pose

        # Return true if we get here
        self.assertTrue(True)

    def test_id1(self):
        """
        Test the id() method of the ShelfPlanning Production class.
        """
        # Setup
        shelf_planning_production = ShelfPlanning1(meshcat_port_number=None)

        # Check that the id is correct
        self.assertEqual(shelf_planning_production.id, ProductionID.kShelfPlanning1)

    def test_start_pose1(self):
        """
        Description
        -----------
        This test verifies that the ShelfPlanning1's start_pose method.
        The method should not raise an error when the object
        is initialized with a start configuration input (instead
        of a start pose).
        """
        # Setup
        production = ShelfPlanning1(
            meshcat_port_number=None,
            start_configuration=np.zeros((6,)),
        )

        # Call start_pose
        start_pose = production.start_pose

        # Return true if we get here
        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()

from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
import unittest

from brom_drake.productions import ProductionID
from brom_drake.productions.motion_planning.offline.kinematic.shelf import ShelfPlanning1
from brom_drake.productions.roles import kKinematicMotionPlanner


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
        self.assertEqual(shelf_planning_production.suggested_roles(), [kKinematicMotionPlanner])

        # Populate production with builder
        shelf_planning_production.add_supporting_cast()
        self.assertTrue(True)

    def test_add_cast_and_build1(self):
        pass # TODO(kwesi) Implement this test

    def test_id1(self):
        """
        Test the id() method of the ShelfPlanning Production class.
        """
        # Setup
        shelf_planning_production = ShelfPlanning1(meshcat_port_number=None)

        # Check that the id is correct
        self.assertEqual(shelf_planning_production.id, ProductionID.kShelfPlanning1)

if __name__ == '__main__':
    unittest.main()
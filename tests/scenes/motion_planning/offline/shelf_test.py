from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
import unittest

from brom_drake.productions import SceneID
from brom_drake.productions.motion_planning.offline.shelf import ShelfPlanning1
from brom_drake.productions.roles import kKinematicMotionPlanner


class ShelfTest(unittest.TestCase):
    def test_add_all_secondary_cast_members_to_builder1(self):
        """
        Test the add_all_secondary_cast_members_to_builder() method
        of the ShelfPlanningScene class.
        :return:
        """
        # Setup
        shelf_planning_scene = ShelfPlanning1(meshcat_port_number=None)

        # Check that defaults are in place
        self.assertEqual(shelf_planning_scene.id, SceneID.kShelfPlanning1)
        self.assertEqual(shelf_planning_scene.suggested_roles(), [kKinematicMotionPlanner])

        # Populate scene with builder
        shelf_planning_scene.add_all_secondary_cast_members_to_builder()
        self.assertTrue(True)

    def test_cast_scene_and_build1(self):
        pass # TODO(kwesi) Implement this test

if __name__ == '__main__':
    unittest.main()
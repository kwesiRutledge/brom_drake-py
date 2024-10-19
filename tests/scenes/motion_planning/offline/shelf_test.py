from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
import unittest

from brom_drake.scenes import SceneID
from brom_drake.scenes.motion_planning.offline.shelf import ShelfPlanningScene
from brom_drake.scenes.roles import kOfflineMotionPlanner


class ShelfTest(unittest.TestCase):
    def test_add_all_secondary_cast_members_to_builder1(self):
        """
        Test the add_all_secondary_cast_members_to_builder() method
        of the ShelfPlanningScene class.
        :return:
        """
        # Setup
        shelf_planning_scene = ShelfPlanningScene(use_meshcat=False)

        # Check that defaults are in place
        self.assertEqual(shelf_planning_scene.id, SceneID.kShelfPlanning1)
        self.assertEqual(shelf_planning_scene.suggested_roles(), [kOfflineMotionPlanner])

        # Populate scene with builder
        diagram = shelf_planning_scene.cast_scene_and_build()

        # Simulate the diagram
        diagram_context = diagram.CreateDefaultContext()
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(15.0)

        pass

if __name__ == '__main__':
    unittest.main()
"""
offline_test.py
Description:
    This script tests the offline motion planning role.
"""

import unittest

from pydrake.systems.analysis import Simulator

from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner
from brom_drake.scenes.motion_planning.offline import ShelfPlanningScene


class OfflineMotionPlanningSceneTest(unittest.TestCase):
    def test_easy_cast_and_build1(self):
        """
        Description:
            This test checks the easy_cast_and_build method of the OfflineMotionPlanningScene class.
            We will provide a single function as input to the function and it will
            define the cast for us.
        :return:
        """
        # Setup
        scene1 = ShelfPlanningScene(meshcat_port_number=None)
        planner2 = BaseRRTPlanner(
            scene1.arm,
            scene1.plant,
            scene1.scene_graph,
        )

        # Call the method
        diagram, diagram_context = scene1.easy_cast_and_build(
            planner2.plan,
        )

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
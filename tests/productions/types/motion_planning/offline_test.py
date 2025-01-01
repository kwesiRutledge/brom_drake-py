"""
offline_test.py
Description:
    This script tests the offline motion planning production type.
"""

import unittest

from pydrake.systems.analysis import Simulator

from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner
from brom_drake.productions.motion_planning.offline import ShelfPlanning1


class OfflineMotionPlanningProductionTest(unittest.TestCase):
    def test_easy_cast_and_build1(self):
        """
        Description:
            This test checks the easy_cast_and_build method of the OfflineMotionPlanning 
            production class. We will provide a single function as input to the function 
            and it will define the cast for us.
        :return:
        """
        # Setup
        production1 = ShelfPlanning1(meshcat_port_number=None)
        planner2 = BaseRRTPlanner(
            production1.arm,
            production1.plant,
            production1.scene_graph,
        )

        # Call the method
        diagram, diagram_context = production1.easy_cast_and_build(
            planner2.plan,
        )

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(0.1)

        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
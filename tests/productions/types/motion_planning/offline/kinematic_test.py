"""
offline_test.py
Description:
    This script tests the offline motion planning production type.
"""

import unittest

import numpy as np
from pydrake.systems.analysis import Simulator

from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline import ChemLab1, ShelfPlanning1


class OfflineKinematicMotionPlanningProductionTest(unittest.TestCase):
    def test_easy_cast_and_build1(self):
        """
        Description:
            This test checks the easy_cast_and_build method of the OfflineMotionPlanning
            production class. We will provide a single function as input to the function
            and it will define the cast for us.
        :return:
        """
        # Setup
        # A short, collision-free motion (the production's usual start, with the
        # elbow bent by pi/8). The default shelf goal is too far for
        # BaseRRTPlanner's default iteration budget.
        easy_start_config = np.array([-0.323, -1.566, 0.283, -0.287, -1.571, -1.893])
        easy_goal_config = easy_start_config + np.array(
            [0.0, 0.0, np.pi / 8.0, 0.0, 0.0, 0.0]
        )
        production1 = ShelfPlanning1(
            meshcat_port_number=None,
            start_configuration=easy_start_config,
            goal_configuration=easy_goal_config,
        )
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


class KinematicPlanningWithoutWatcherTest(unittest.TestCase):
    """
    Regression tests: the kinematic motion planning productions used to plan
    and move ONLY when a DiagramWatcher was attached (the watcher was the only
    thing evaluating the arm controller, which in turn pulled the plan). Built
    with with_watcher=False they silently stood still.
    """

    def _assert_reaches_goal(self, production):
        planner = BidirectionalRRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
        )
        diagram, diagram_context = production.easy_cast_and_build(
            planner.plan,
            with_watcher=False,
        )

        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()
        simulator.AdvanceTo(0.1)

        # The plan must have been computed during the first steps
        planned_trajectory = production.plan_dispenser.planned_trajectory
        self.assertIsNotNone(
            planned_trajectory,
            "No plan was computed in the first 0.1 s of simulation.",
        )

        # ... and executing it must bring the arm to the goal configuration
        simulator.AdvanceTo(planned_trajectory.end_time() + 0.5)
        plant_context = production.plant.GetMyContextFromRoot(simulator.get_context())
        np.testing.assert_allclose(
            production.plant.GetPositions(plant_context, production.arm),
            production.goal_configuration,
            atol=1e-3,
        )

    def test_shelf_planning1_without_watcher1(self):
        self._assert_reaches_goal(ShelfPlanning1(meshcat_port_number=None))

    def test_chem_lab1_without_watcher1(self):
        self._assert_reaches_goal(ChemLab1(meshcat_port_number=None))


if __name__ == "__main__":
    unittest.main()

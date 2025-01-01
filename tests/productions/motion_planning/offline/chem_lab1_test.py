import unittest
import numpy as np
from pydrake.systems.analysis import Simulator

# Internal Imports
from brom_drake.motion_planning.systems.prototypical_planner import PrototypicalPlannerSystem
from brom_drake.motion_planning.systems.rrt_plan_generator import RRTPlanGenerator
from brom_drake.productions.motion_planning.offline.chem_lab1 import ChemLab1
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner

class ChemLab1Test(unittest.TestCase):
    def test_add_all_secondary_cast_members_to_builder1(self):
        # Setup
        production = ChemLab1(meshcat_port_number=None)

        # Call the method
        production.add_supporting_cast()

        # Build the diagram
        diagram = production.builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(0.010)

        self.assertTrue(True)

    def test_add_cast_and_build1(self):
        """
        Description
        -----------
        This test checks the prototypical planner's ability to plan a motion
        from a start pose to a goal pose.
        :return:
        """
        # Setup
        q_easy_start = np.array([0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0])
        q_easy_goal = np.array([0.0, 0.0, -np.pi/8.0, 0.0, 0.0, 0.0])
        production1 = ChemLab1(
            meshcat_port_number=None,
            start_config=q_easy_start,
            goal_config=q_easy_goal,
        )

        # Create the Planning System
        planning_system = RRTPlanGenerator(
            production1.plant,
            production1.scene_graph,
            production1.robot_model_idx_,
            dim_config=6,
        )

        # Add the prototypical planner to the production
        role1 = production1.suggested_roles()[0]
        diagram, diagram_context = production1.add_cast_and_build(
            cast=[(role1, planning_system)],
        )

        # Create simulator
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        
        self.assertTrue(True) # Production built successfully, which is good.

    def test_easy_cast_and_build1(self):
        # Setup
        easy_start_config = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ])
        easy_goal_config = np.array([
            0.0, 0.0, np.pi/8., 0.0, 0.0, 0.0
        ])
        production = ChemLab1(
            meshcat_port_number=None,
            start_configuration=easy_start_config,
            goal_configuration=easy_goal_config,
            )

        # Create planner with the now finalized arm
        planner1 = BaseRRTPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
        )

        # Call the method
        diagram, diagram_context = production.easy_cast_and_build(
            planner1.plan,
        )

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(0.010)

        self.assertTrue(True)


if __name__ == '__main__':
    unittest.main()
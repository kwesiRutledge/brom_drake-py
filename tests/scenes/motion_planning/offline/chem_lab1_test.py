import unittest
import numpy as np
from pydrake.systems.analysis import Simulator

# Internal Imports
from brom_drake.scenes.motion_planning.offline.chem_lab1 import ChemLab1Scene
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner

class ChemLab1Test(unittest.TestCase):
    def test_add_all_secondary_cast_members_to_builder1(self):
        # Setup
        scene = ChemLab1Scene(meshcat_port_number=None)

        # Call the method
        scene.add_all_secondary_cast_members_to_builder()

        # Build the diagram
        diagram = scene.builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(0.010)

        self.assertTrue(True)

    def test_easy_cast_scene_and_build1(self):
        # Setup
        easy_start_config = np.array([
            0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ])
        easy_goal_config = np.array([
            0.0, 0.0, np.pi/8., 0.0, 0.0, 0.0
        ])
        scene = ChemLab1Scene(
            meshcat_port_number=None,
            start_configuration=easy_start_config,
            goal_configuration=easy_goal_config,
            )

        # Create planner with the now finalized arm
        planner1 = BaseRRTPlanner(
            scene.arm,
            scene.plant,
            scene.scene_graph,
        )

        # Call the method
        diagram, diagram_context = scene.easy_cast_and_build(
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
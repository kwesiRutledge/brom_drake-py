"""
Description
-----------
This file defines a test class that is used to test the prototypical planner
system that I've defined in the motion_planning/systems/prototypical_planner.py file.
"""
from importlib import resources as impresources

from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder
import unittest

from brom_drake.file_manipulation.urdf import drakeify_my_urdf
# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner
from brom_drake.motion_planning.systems.prototypical_planner import PrototypicalPlannerSystem
import brom_drake.robots as robots
from brom_drake.scenes.motion_planning.offline import ShelfPlanningScene


class TestPrototypicalPlannerSystem(unittest.TestCase):
    def setUp(self):
        """
        Description
        -----------
        Set up for all of the tests.
        :return:
        """
        self.builder = DiagramBuilder()
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            self.builder,
            time_step=1e-3,
        )

        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        # Convert the URDF
        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
        )

        # Add the UR10e to the plant
        self.arm = Parser(self.plant).AddModels(str(new_urdf_path))[0]

    def test_init1(self):
        """
        Description
        -----------
        This test checks the prototypical planner's ability to plan a motion
        from a start pose to a goal pose.
        :return:
        """
        # Setup
        planner1 = lambda q_start, q_goal, collision_fcn: q_goal - q_start

        # Create the System
        self.plant.Finalize()
        prototypical_planner = PrototypicalPlannerSystem(
            self.plant, self.scene_graph,
            planner1,
        )

        self.assertTrue(True)

    def test_use_in_mp_scene1(self):
        """
        Description
        -----------
        This test checks the prototypical planner's ability to plan a motion
        from a start pose to a goal pose.
        :return:
        """
        # Setup
        scene1 = ShelfPlanningScene(
            meshcat_port_number=7002,
        )
        planner2 = BaseRRTPlanner(
            scene1.arm,
            scene1.plant,
            scene1.scene_graph,
        )

        # Create the System
        prototypical_planner = PrototypicalPlannerSystem(
            scene1.plant, scene1.scene_graph,
            planner2.plan,
        )

        # Add the prototypical planner to the scene
        role1 = scene1.suggested_roles()[0]
        diagram, diagram_context = scene1.cast_scene_and_build([
            (role1, prototypical_planner),
        ])

        # Create simulator
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

if __name__ == '__main__':
    unittest.main()
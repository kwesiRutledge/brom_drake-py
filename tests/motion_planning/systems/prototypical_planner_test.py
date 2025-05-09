"""
Description
-----------
This file defines a test class that is used to test the prototypical planner
system that I've defined in the motion_planning/systems/prototypical_planner.py file.
"""
from importlib import resources as impresources
import networkx as nx
import numpy as np
from pydrake.all import (
    Parser, DiagramBuilder,
    AddMultibodyPlantSceneGraph,
    RigidTransform, RollPitchYaw,
    Simulator,
)
import unittest

from brom_drake.file_manipulation.urdf import drakeify_my_urdf
# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner
from brom_drake.motion_planning.systems.prototypical_planner import PrototypicalPlannerSystem
import brom_drake.robots as robots
from brom_drake.productions.motion_planning.offline import ShelfPlanning1


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

    def test_check_collision_in_config1(self):
        """
        Description
        -----------
        This test checks the prototypical planner's ability to correctly identify when
        a configuration is in collision.
        We will place the arm in a known collision configuration with
        the shelf placed in the way.
        """
        # Setup
        q_collision = np.array([0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0])

        bad_shelf_position = np.array([0.5, 0.0, 0.0])
        bad_shelf_orientation = RollPitchYaw(np.pi/2.0, 0.0, 0.0)
        bad_shelf_pose = RigidTransform(bad_shelf_orientation, bad_shelf_position)

        # Create the Production
        production1 = ShelfPlanning1(
            meshcat_port_number=None,
            start_config=q_collision,
            goal_config=q_collision,
            shelf_pose=bad_shelf_pose,
        )

        # Cast all secondary cast members
        production1.add_supporting_cast()

        # Create planner with the now finalized arm
        planner1 = BaseRRTPlanner(
            production1.arm,
            production1.plant,
            production1.scene_graph,
        )

        system1 = PrototypicalPlannerSystem(
            production1.plant,
            production1.scene_graph,
            planner1.plan,
            robot_model_idx=production1.arm,
        )

        # Build production
        production1.fill_role(production1.suggested_roles()[0], system1)
        diagram, diagram_context = production1.build_production()
        
        system1.set_internal_root_context(diagram_context)

        # Check collision
        in_collision = system1.check_collision_in_config(q_collision)
        self.assertTrue(in_collision)

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
            robot_model_idx=self.arm,
        )

        self.assertTrue(True)

    def test_use_in_mp_production1(self):
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
        production1 = ShelfPlanning1(
            meshcat_port_number=None,
            start_config=q_easy_start,
            goal_config=q_easy_goal,
        )

        # Cast all secondary cast members
        production1.add_supporting_cast()

        # Create planner with the now finalized arm
        planner2 = BaseRRTPlanner(
            production1.arm,
            production1.plant,
            production1.scene_graph,
        )

        # Create the System
        prototypical_planner = PrototypicalPlannerSystem(
            production1.plant,
            production1.scene_graph,
            planner2.plan,
            robot_model_idx=production1.arm,
        )

        # Add the prototypical planner to the production
        role1 = production1.suggested_roles()[0]
        production1.fill_role(role1, prototypical_planner)

        diagram, diagram_context = production1.build_production()

        # Add the connections that we need for the performer
        prototypical_planner.set_internal_root_context(
            diagram_context
        )

        # Create simulator
        # simulator = Simulator(diagram, diagram_context)
        # simulator.set_target_realtime_rate(1.0)
        # simulator.Initialize()
        
        self.assertTrue(True) # Production built successfully, which is good.

        #TODO(kwesi): Test that the simulation works for some simple case...

    def test_planning_result_to_array1(self):
        """
        Description
        -----------
        This test checks the prototypical planner's ability to create a
        trajectory from an nx.DiGraph object using the planning_result_to_array()
        function.
        """
        # Setup
        q_easy_start = np.array([0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0])
        q_easy_goal = np.array([0.0, 0.0, -np.pi/8.0, 0.0, 0.0, 0.0])
        production1 = ShelfPlanning1(
            meshcat_port_number=None,
            start_config=q_easy_start,
            goal_config=q_easy_goal,
        )

        # Cast all secondary cast members
        production1.add_supporting_cast()

        # Create planner with the now finalized arm
        planner2 = BaseRRTPlanner(
            production1.arm,
            production1.plant,
            production1.scene_graph,
        )

        # Create the System
        prototypical_planner = PrototypicalPlannerSystem(
            production1.plant,
            production1.scene_graph,
            planner2.plan,
            robot_model_idx=production1.arm,
        )

        # Create a dummy nx.DiGraph object
        G = nx.DiGraph()
        G.add_node(0, q=q_easy_start)
        G.add_node(1, q=q_easy_goal)
        G.add_edge(0, 1)

        # Call the planning_result_to_array method
        trajectory = prototypical_planner.planning_result_to_array(G)

        # Check that the trajectory is correct
        # - The trajectory should have 2 waypoints
        # - The first waypoint should be the start configuration
        # - The second waypoint should be the goal configuration
        self.assertEqual(trajectory.shape[0], 2)
        np.testing.assert_array_equal(trajectory[0], q_easy_start)
        np.testing.assert_array_equal(trajectory[1], q_easy_goal)

    def test_planning_result_to_array2(self):
        """
        Description
        -----------
        This test checks the prototypical planner's ability to create a
        trajectory from an np.ndarray object using the planning_result_to_array()
        function.
        """
        # Setup
        q_easy_start = np.array([0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0])
        q_easy_goal = np.array([0.0, 0.0, -np.pi/8.0, 0.0, 0.0, 0.0])
        production1 = ShelfPlanning1(
            meshcat_port_number=None,
            start_config=q_easy_start,
            goal_config=q_easy_goal,
        )

        # Cast all secondary cast members
        production1.add_supporting_cast()

        # Create planner with the now finalized arm
        planner2 = BaseRRTPlanner(
            production1.arm,
            production1.plant,
            production1.scene_graph,
        )

        # Create the System
        prototypical_planner = PrototypicalPlannerSystem(
            production1.plant,
            production1.scene_graph,
            planner2.plan,
            robot_model_idx=production1.arm,
        )

        # Create a dummy array object
        plan_array = np.array([
            q_easy_start,
            q_easy_goal
        ])

        # Call the planning_result_to_array method
        trajectory = prototypical_planner.planning_result_to_array(plan_array)

        # Check that the trajectory is correct
        # - The trajectory should have 2 waypoints
        # - The first waypoint should be the start configuration
        # - The second waypoint should be the goal configuration
        self.assertEqual(trajectory.shape[0], 2)
        np.testing.assert_array_equal(trajectory[0], q_easy_start)
        np.testing.assert_array_equal(trajectory[1], q_easy_goal)

if __name__ == '__main__':
    unittest.main()
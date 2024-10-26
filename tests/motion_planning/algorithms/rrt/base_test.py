"""
base_test.py
Description:

    This file contains unit tests for the BaseRRTPlanner class.
"""

from importlib import resources as impresources

import numpy as np
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
import unittest

# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlanner as BaseRRTPlanner
import brom_drake.robots as robots
from brom_drake.urdf import drakeify_my_urdf


class TestBaseRRT(unittest.TestCase):
    def test_joint_limits1(self):
        """
        Test the joint limits retrieval functionality.
        :return:
        """
        # Setup
        plant = MultibodyPlant(time_step=1e-3)

        # Add the UR10e
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
        )

        model_idcs = Parser(plant).AddModels(str(new_urdf_path))
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("base_link", model_idcs[0]),
        )
        plant.Finalize()

        # Create a BaseRRTPlanner instance and compute the joint limits
        base_rrt = BaseRRTPlanner(model_idcs[0], plant)
        joint_limits = base_rrt.get_joint_limits()

        print("Joint limits:", joint_limits)

        self.assertEqual(
            joint_limits.shape[1], 2,
            "Joint limits should have two columns (lower and upper limits)."
        )

        self.assertEqual(
            joint_limits.shape[0], plant.num_actuated_dofs(model_idcs[0]),
            "Joint limits should have the same number of rows as actuated DOFs."
        )

    def test_sample_random_configuration1(self):
        """
        Test the random configuration sampling functionality.
        In this case, we just verify that 10 separate samples of the configuration
        space are within the limits we've established.
        :return:
        """
        # Setup
        plant = MultibodyPlant(time_step=1e-3)

        # Add the UR10e
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
        )

        model_idcs = Parser(plant).AddModels(str(new_urdf_path))
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("base_link", model_idcs[0]),
        )
        plant.Finalize()

        # Create a BaseRRTPlanner instance and compute the joint limits
        base_rrt = BaseRRTPlanner(model_idcs[0], plant)
        joint_limits = base_rrt.get_joint_limits()

        for _ in range(10):
            random_configuration = base_rrt.sample_random_configuration()
            for joint_idx in range(len(random_configuration)):
                self.assertGreaterEqual(
                    random_configuration[joint_idx],
                    joint_limits[joint_idx, 0],
                    f"Joint {joint_idx} is below its lower limit."
                )
                self.assertLessEqual(
                    random_configuration[joint_idx],
                    joint_limits[joint_idx, 1],
                    f"Joint {joint_idx} is above its upper limit."
                )

    def test_plan1(self):
        """
        Description:
            In this test, we verify that our RRT planner generates a valid RRT with nodes
            that are getting CLOSER to the goal configuration over time.
        :return:
        """
        # Setup
        plant = MultibodyPlant(time_step=1e-3)

        # Add the UR10e
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
        )

        model_idcs = Parser(plant).AddModels(str(new_urdf_path))
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("base_link", model_idcs[0]),
        )
        plant.Finalize()

        # Create a BaseRRTPlanner instance
        base_rrt = BaseRRTPlanner(model_idcs[0], plant)
        base_rrt.plant_context = plant.CreateDefaultContext()

        # Define start and goal configurations
        start_configuration = np.zeros(base_rrt.dim_q)
        goal_configuration = np.ones(base_rrt.dim_q)

        # Plan
        rrt = base_rrt.plan(start_configuration, goal_configuration)

        # Check if the RRT has nodes and if they are getting closer to the goal
        self.assertGreater(len(rrt.nodes), 0, "RRT should have nodes.")

        # Check if the distance from the last node to the goal is less than the distance from the start
        last_node = rrt.nodes[len(rrt.nodes)-1]
        distance_to_goal = np.linalg.norm(last_node['q'] - goal_configuration)
        distance_to_start = np.linalg.norm(start_configuration - goal_configuration)

        self.assertLess(distance_to_goal, distance_to_start, "Last node should be closer to the goal than the start.")

if __name__ == "__main__":
    unittest.main()
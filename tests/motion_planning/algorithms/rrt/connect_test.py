from brom_drake.all import drakeify_my_urdf
import brom_drake.robots as robots
import importlib.resources as impresources
import networkx as nx
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Parser,
    DiagramBuilder,
)
import unittest

# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.connect import (
    RRTConnectPlannerConfig,
    RRTConnectPlanner,
)


class TestRRTConnect(unittest.TestCase):
    def setUp(self) -> None:
        # Setup
        self.builder1 = DiagramBuilder()

        # Create a config
        self.config1 = RRTConnectPlannerConfig(
            steering_step_size=0.025,
            prob_sample_goal=0.25,
            max_iterations=int(1e4),
            convergence_threshold=1e-3,
        )

        # Create a plant and scene graph
        self.plant1, self.scene_graph1 = AddMultibodyPlantSceneGraph(
            self.builder1,
            time_step=1e-3,
        )

        # Collect an arm model
        original_arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        arm_urdf = drakeify_my_urdf(
            original_arm_urdf_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
        )
        self.arm_model_index = Parser(self.plant1).AddModels(str(arm_urdf))[0]

        # Weld arm to world
        self.plant1.WeldFrames(
            self.plant1.world_frame(),
            self.plant1.GetFrameByName("base_link", self.arm_model_index),
        )

        # Define RRTConnectPlanner # 1
        self.planner1 = RRTConnectPlanner(
            self.arm_model_index,
            self.plant1,
            self.scene_graph1,
            config=self.config1,
        )

    def test_dim_q1(self):
        # Setup

        # Finalize plant
        self.plant1.Finalize()

        # Get the dimension of the robot's configuration space
        self.assertEqual(self.planner1.dim_q, 6)

    def test_find_nearest_node1(self):
        # Setup
        q_rand = np.array([0.0, 0.0, 0.2, 0.0, 0.0, 0.0])

        # Define RRTConnectPlanner
        planner = RRTConnectPlanner(
            self.arm_model_index,
            self.plant1,
            self.scene_graph1,
            config=self.config1,
        )

        # Finalize plant
        self.plant1.Finalize()

        # Create a digraph for the RRT
        rrt = nx.DiGraph()
        rrt.add_node(0, q=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        rrt.add_node(1, q=np.array([0.0, 0.0, 0.1, 0.0, 0.0, 0.0]))

        # Find the nearest node
        nearest_node, nearest_node_idx = planner.find_nearest_node(rrt, q_rand)

        # Check
        self.assertEqual(nearest_node_idx, 1)

    def test_steer1(self):
        """
        Test the steering functionality.
        In this test, we will specifically check that the steering function properly returns
        a configuration that is closer to the target configuration than the initial one.
        We will make sure that the steering function DOES not return the goal when it is too far away.
        """
        # Setup
        q_current = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        q_target = np.array([0.0, 0.0, 0.2, 0.0, 0.0, 0.0])

        # Define RRTConnectPlanner
        planner = RRTConnectPlanner(
            self.arm_model_index,
            self.plant1,
            self.scene_graph1,
            config=self.config1,
        )

        # Finalize plant
        self.plant1.Finalize()

        # Steer
        q_new = planner.steer(q_current, q_target)

        # Check that
        # 1. The new configuration is closer to the target than the current configuration
        # 2. The new configuration is not the target configuration
        self.assertLess(
            np.linalg.norm(q_new - q_target), np.linalg.norm(q_current - q_target)
        )
        self.assertFalse(np.isclose(q_new, q_target).all())

    def test_plan1(self):
        """
        Description
        -----------
        Test the planning functionality.
        We will define a simple planning problem where the start
        and goal configurations are close AND in an obstacle-free space.
        Connect should rapidly find a solution.
        """
        # Setup
        start_configuration = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        goal_configuration = np.array([0.0, 0.0, 0.2, 0.0, 0.0, 0.0])

        config2 = RRTConnectPlannerConfig(
            steering_step_size=0.025,
            prob_sample_goal=0.99,
            max_iterations=int(1e4),
            convergence_threshold=1e-3,
        )

        # Define RRTConnectPlanner
        planner = RRTConnectPlanner(
            self.arm_model_index,
            self.plant1,
            self.scene_graph1,
            config=self.config1,
        )

        # Finalize plant + build
        self.plant1.Finalize()
        diagram = self.builder1.Build()
        diagram_context = diagram.CreateDefaultContext()

        planner.root_context = diagram_context

        # Plan
        rrt, success = planner.plan(start_configuration, goal_configuration)

        # Check
        # - Path was successfully found
        # - Path is not empty
        self.assertTrue(success)
        self.assertTrue(len(rrt.edges) > 0)


if __name__ == "__main__":
    unittest.main()

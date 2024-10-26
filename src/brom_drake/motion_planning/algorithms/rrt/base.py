"""
base.py
"""
from typing import Tuple

import networkx as nx
import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
# Internal Imports
from brom_drake.motion_planning.algorithms.motion_planner import MotionPlanner

class BaseRRTPlanner(MotionPlanner):
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        steering_step_size: float = 0.1,
        prob_sample_goal: float = 0.075,
    ):
        super().__init__(robot_model_idx, plant)
        # Input Processing

        # Setup
        self.steering_step_size = steering_step_size
        self.prob_sample_goal = prob_sample_goal
        self.dim_q = plant.num_actuated_dofs(robot_model_idx)

        # Prepare for planning
        self.joint_limits = self.get_joint_limits()  # Initialize joint limits array

    def get_joint_limits(self):
        """
        Description:
            This function retrieves the joint limits of the robot.
        """
        # Setup
        joint_indicies = self.plant.GetJointIndices(self.robot_model_idx)

        joint_limits = np.zeros((0, 2))
        for ii, joint_index in enumerate(joint_indicies):
            # Check to see if joint is actuated; if not, then ignore
            joint_ii = self.plant.get_joint(joint_index)
            joint_can_move = joint_ii.can_rotate() or joint_ii.can_translate()
            if not joint_can_move:
                continue

            lower_limits = self.plant.get_joint(joint_index).position_lower_limits()
            if len(lower_limits) == 0:
                lower_limits = [-np.inf]

            upper_limits = self.plant.get_joint(joint_index).position_upper_limits()
            if len(upper_limits) == 0:
                upper_limits = [np.inf]

            # Append limits to the joint_limits array
            joint_limits = np.vstack((
                joint_limits,
                [lower_limits[0], upper_limits[0]]
            ))

        return joint_limits

    def find_nearest_node(
        self,
        rrt: nx.DiGraph,
        q_random: np.ndarray
    ) -> Tuple[nx.classes.reportviews.NodeView, int]:
        """
        Description:
            This function finds the nearest node in the RRT to the random configuration.
        """
        nearest_node = None
        min_distance = float('inf')

        for node in rrt.nodes:
            distance = np.linalg.norm(rrt.nodes[node]['q'] - q_random)
            if distance < min_distance:
                min_distance = distance
                nearest_node_idx = node
                nearest_node = rrt.nodes[node]

        return nearest_node, nearest_node_idx

    def plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        max_iterations: int = 1000,
    ) -> Tuple[nx.DiGraph, bool]:
        """
        Description:
            This function executes the RRT planning algorithm.
        """
        # Input Processing
        q_start = q_start.flatten()
        q_goal = q_goal.flatten()
        if q_start.shape[0] != self.dim_q or q_goal.shape[0] != self.dim_q:
            raise ValueError(
                f"Start configuration shape ({q_start.shape}) and goal configuration " +
                f"shape ({q_goal.shape}) must match the dimension of the robot ({self.dim_q})."
            )

        # Setup
        rrt = nx.DiGraph()
        rrt.add_node(0, q=q_start) # Make this graph contain all configurations
        prob_sample_goal = self.prob_sample_goal

        # RRT Planner
        for iteration in range(max_iterations):
            # Sample random configuration OR goal
            if np.random.rand() < prob_sample_goal:
                q_random = q_goal.copy()
            else:
                q_random = self.sample_random_configuration()

            # Find nearest node in the tree
            nearest_node, nearest_node_idx = self.find_nearest_node(rrt, q_random)

            # Steer from nearest node to random configuration
            q_new = self.steer(nearest_node['q'], q_random)

            # Check for collisions
            if self.check_collision_in_config(q_new):
                continue

            # Add new node to the tree
            rrt.add_node(rrt.number_of_nodes(), q=q_new)
            rrt.add_edge(nearest_node_idx, rrt.number_of_nodes()-1)

            # Check if we have reached the goal
            if np.linalg.norm(q_new - q_goal) < 1e-3:
                print(f"Goal reached after {iteration} iterations!")
                return rrt, True

        # If we exit the loop without finding a path to the goal,
        # return the RRT and indicate failure
        print("Max iterations reached without finding a path to the goal.")
        return rrt, False



    def sample_random_configuration(self) -> np.ndarray:
        """
        Description:
            This function samples a random configuration within the joint limits.
        """
        return np.random.uniform(
            self.joint_limits[:, 0],
            self.joint_limits[:, 1]
        )

    def steer(
        self,
        nearest_node,
        q_random: np.ndarray
    ) -> np.ndarray:
        """
        Description:
            This function steers from the nearest node towards the random configuration.
        """
        # Setup
        step_size = self.steering_step_size
        q_nearest = nearest_node

        # Calculate the direction vector
        direction = q_random - q_nearest
        distance = np.linalg.norm(direction)

        # Either:
        # 1. Move a full step towards the random configuration
        # 2. Or, if the distance is less than the step size, return the random configuration
        if distance < step_size:
            return q_random

        return q_nearest + step_size * (direction / distance)

"""
connect.py
"""
from dataclasses import dataclass
from typing import Tuple, Callable

import networkx as nx
import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
# Internal Imports
from brom_drake.motion_planning.algorithms.motion_planner import MotionPlanner

@dataclass
class RRTConnectPlannerConfig:
    convergence_threshold: float = 1e-3
    max_iterations: int = int(1e4)
    prob_sample_goal: float = 0.05
    random_seed: int = 23
    steering_step_size: float = 0.025
    debug_flag: bool = False
    

class RRTConnectPlanner(MotionPlanner):
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        config: RRTConnectPlannerConfig = None,
    ):

        # Setup
        # self.dim_q = plant.num_actuated_dofs(robot_model_idx)

        self.config = config
        if self.config is None:
            self.config = RRTConnectPlannerConfig()

        # Prepare for planning
        # self.joint_limits = self.get_joint_limits()  # Initialize joint limits array

        # Use the parent class constructor
        super().__init__(
            robot_model_idx, plant, scene_graph, 
            random_seed=self.config.random_seed,
        )

    def connect(
        self,
        nearest_node_idx: int,
        q_target: np.ndarray,
        rrt: nx.DiGraph,
        collision_check_fcn: Callable[[np.ndarray], bool] = None,
    ) -> np.ndarray:
        # Input Processing
        if collision_check_fcn is None:
            collision_check_fcn = self.check_collision_in_config

        # Setup
        nearest_node = rrt.nodes[nearest_node_idx]
        debug_flag = self.config.debug_flag

        # Steer from nearest node to random configuration
        q_current = nearest_node['q']
        last_node_idx = nearest_node_idx

        n_loops = 0
        while not np.isclose(q_current, q_target).all():
            # Create next node
            q_new = self.steer(q_current, q_target)

            # Check for collisions
            if collision_check_fcn(q_new):
                if debug_flag:
                    print(f"Collision detected at {q_new} (q_current = {q_current})")
                break

            # If the next point is collision free, then add new node to the tree
            rrt.add_node(rrt.number_of_nodes(), q=q_new)
            rrt.add_edge(last_node_idx, rrt.number_of_nodes()-1)

            # Prepare for next iteration
            last_node_idx = rrt.number_of_nodes()-1
            q_current = q_new

            n_loops += 1

        if debug_flag:
            print(f"Number of loops in connection: {n_loops}")
            print(f"RRT Size: {rrt.number_of_nodes()}")

        return q_current # Return the last configuration we visited

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
        collision_check_fcn: Callable[[np.ndarray], bool] = None,
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

        if collision_check_fcn is None:
            collision_check_fcn =  self.check_collision_in_config

        # Setup
        rrt = nx.DiGraph()
        rrt.add_node(0, q=q_start) # Make this graph contain all configurations
        prob_sample_goal = self.config.prob_sample_goal

        # RRT Planner
        for iteration in range(self.config.max_iterations):
            # Sample random configuration OR goal
            if np.random.rand() < prob_sample_goal:
                q_random = q_goal.copy()
            else:
                q_random = self.sample_random_configuration()

            # Find nearest node in the tree
            nearest_node, nearest_node_idx = self.find_nearest_node(rrt, q_random)

            q_new = self.connect(nearest_node_idx, q_random, rrt, collision_check_fcn)

            # Check if we have reached the goal
            if np.linalg.norm(q_new - q_goal) < self.config.convergence_threshold:
                print(f"Goal reached after {iteration} iterations of RRT-Connect!")
                # print(f"Check all points in path: {rrt.nodes}")
                # for node in rrt.nodes:
                #     print(f"Node {node}: {rrt.nodes[node]}")
                #     collision_check_value = collision_check_fcn(rrt.nodes[node]['q'])
                #     if collision_check_value:
                #         print(f"Collision check node: {collision_check_fcn(rrt.nodes[node]['q'])}")
                return rrt, rrt.number_of_nodes()-1

        # If we exit the loop without finding a path to the goal,
        # return the RRT and indicate failure
        print("Max iterations reached without finding a path to the goal.")
        return rrt, -1

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
        step_size = self.config.steering_step_size
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

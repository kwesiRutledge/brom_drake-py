"""
bidirectional.py
Description
-----------
This file contains an algorithm for performing bidirectional RRT
based search.
"""

from dataclasses import dataclass
import networkx as nx
import numpy as np
from pydrake.all import (
    ModelInstanceIndex,
    MultibodyPlant,
    SceneGraph,
)
from typing import Callable, Tuple

# Internal Imports
from brom_drake.motion_planning.algorithms.motion_planner import MotionPlanner

# Define config dataclass
@dataclass(frozen=True)
class BiRRTSamplingProbabilities:
    """
    A dataclass that defines the sampling probabilities for a bidirectional RRT planner.
    """
    sample_goal_tree: float = 0.10
    sample_opposite_tree: float = 0.10
    

@dataclass
class BidirectionalRRTPlannerConfig:
    """
    A dataclass that defines the configuration for a bidirectional RRT planner.
    """
    convergence_threshold: float = 1e-3
    max_tree_nodes: int = int(1e5)
    probabilities: BiRRTSamplingProbabilities = BiRRTSamplingProbabilities()
    random_seed: int = 23
    steering_step_size: float = 0.1

class BidirectionalRRTPlanner(MotionPlanner):
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        config: BidirectionalRRTPlannerConfig = None,
    ):
        # Input Processing
        if config is None:
            config = BidirectionalRRTPlannerConfig()

        # Setup
        self.config = config

        # Use the parent class constructor
        super().__init__(
            robot_model_idx, plant, scene_graph, 
            random_seed=self.config.random_seed,
        )

    def find_nearest_node(
        self,
        rrt: nx.DiGraph,
        q_random: np.ndarray
    ) -> Tuple[nx.classes.reportviews.NodeView, int]:
        """
        Description
        -----------
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
        Description
        -----------
        This function plans a path from q_start to q_goal using the Bidirectional
        RRT algorithm.
        """
        # Input Processing
        q_start = np.array(q_start)
        q_goal = np.array(q_goal)
        if q_start.shape[0] != self.dim_q or q_goal.shape[0] != self.dim_q:
            raise ValueError(
                f"Start configuration shape ({q_start.shape}) and goal configuration " +
                f"shape ({q_goal.shape}) must match the dimension of the robot ({self.dim_q})."
            )

        if collision_check_fcn is None:
            collision_check_fcn = self.check_collision_in_config

        # Setup
        rrt_start = nx.DiGraph()
        rrt_goal = nx.DiGraph()
        rrt_start.add_node(0, q=q_start)
        rrt_goal.add_node(0, q=q_goal)

        prob_sample_goal_tree = self.config.probabilities.sample_goal_tree
        prob_sample_opposite_tree = self.config.probabilities.sample_opposite_tree
        max_tree_nodes = self.config.max_tree_nodes

        # Initialize the trees
        n_nodes_start = 1
        n_nodes_goal = 1
        sample_from_goal_tree = None
        while (n_nodes_start + n_nodes_goal) <= max_tree_nodes:
            # Decide whether to sample from the start or goal tree
            sample_from_goal_tree = np.random.rand() < prob_sample_goal_tree
            if sample_from_goal_tree: # Sample from the goal tree
                current_tree = rrt_goal
            else: # Sample from the start tree
                current_tree = rrt_start

            q_current, current_node_idx = self.sample_from_tree(current_tree)

            # Choose whether or not to sample from the opposite tree or randomly
            if np.random.rand() < prob_sample_opposite_tree:
                # Sample from the opposite tree
                opposite_tree = rrt_start if sample_from_goal_tree else rrt_goal
                node_idx_in_opposite_tree, min_distance = self.sample_nearest_in_tree(opposite_tree, q_current)
                
                node_in_opposite_tree = opposite_tree.nodes[node_idx_in_opposite_tree]
                sampled_config = node_in_opposite_tree['q']

                # Steer from the sampled configuration to the goal 
                q_new, reached_new = self.steer(sampled_config, q_goal)
                if reached_new:
                    combined_rrt = nx.disjoint_union(rrt_start, rrt_goal)
                    # add edge between the two trees
                    if sample_from_goal_tree:
                        combined_rrt.add_edge(
                            node_idx_in_opposite_tree,
                            rrt_start.number_of_nodes() + current_node_idx,
                        )
                    else:
                        combined_rrt.add_edge(
                            current_node_idx,
                            rrt_start.number_of_nodes() + node_idx_in_opposite_tree,
                        )
                    return combined_rrt, rrt_start.number_of_nodes()
                else:
                    # If we did not reach the goal,
                    # add the new configuration to the current tree
                    current_tree.add_node(n_nodes_start, q=q_new)
                    current_tree.add_edge(sampled_node_idx, n_nodes_start)
                    n_nodes_start += 1

            else:
                # Sample from a random configuration
                q_random = self.sample_random_configuration()

                # Find the nearest node in the tree and steer towards it
                nearest_node, nearest_node_idx = self.find_nearest_node(current_tree, q_random)
                q_new, reached_new = self.steer(nearest_node['q'], q_random)

                # Check if the new configuration is in collision
                if collision_check_fcn(q_new):
                    continue

                # If the configuration is not in collision, then add it to the current tree
                current_tree.add_node(n_nodes_start, q=q_new)
                current_tree.add_edge(nearest_node_idx, n_nodes_start)

                if sample_from_goal_tree:
                    n_nodes_goal += 1
                else:
                    n_nodes_start += 1

        # If we exit the loop without finding a path to the goal,
        # return the RRT and indicate failure
        print("Max iterations reached without finding a path to the goal.")
        return nx.disjoint_union(rrt_start, rrt_goal), -1

    def sample_from_tree(
        self,
        rrt: nx.DiGraph,
    ) -> Tuple[np.ndarray, int]:
        """
        Description
        -----------
        This function samples a configuration from the RRT.
        """
        # Setup
        n_tree = rrt.number_of_nodes()

        # Select a random number from the range [0, n_tree)
        random_index = np.random.randint(0, n_tree)
        q_random = rrt.nodes[random_index]['q']

        return q_random, random_index
    
    def sample_nearest_in_tree(
        self,
        rrt: nx.DiGraph,
        q_random: np.ndarray,
    ) -> Tuple[int, float]:
        """
        Description
        -----------
        This function samples a configuration from the RRT.
        """
        # Setup
        n_tree = rrt.number_of_nodes()

        # Search through the tree to find the node that is nearest to the random configuration
        min_distance = float('inf')
        nearest_node_idx = -1
        for ii, node in enumerate(rrt.nodes):
            distance = np.linalg.norm(rrt.nodes[node]['q'] - q_random)
            if distance < min_distance:
                min_distance = distance
                nearest_node_idx = ii

        return nearest_node_idx, min_distance
    
    def steer(
        self,
        q_current: np.ndarray,
        q_target: np.ndarray,
    ) -> Tuple[np.ndarray, bool]:
        """
        Description
        -----------
        This function steers the RRT from the start configuration to the goal configuration.

        Arguments
        ---------
        q_current: np.ndarray
            The current configuration of the robot that we wish to steer from.
        q_target: np.ndarray
            The target configuration of the robot that we wish to steer to.

        Returns
        -------
        q_new: np.ndarray
            The new configuration of the robot after steering.
        reached_config: bool
            Whether or not we reached the target configuration with our step size or not.
        """
        # Setup
        step_size = self.config.steering_step_size

        # Calculate the direction vector
        direction = q_target - q_current
        distance = np.linalg.norm(direction)

        # Either:
        # 1. Move a full step towards the random configuration
        # 2. If the distance is less than the step size, return the random configuration

        if distance < step_size:
            return q_target, True
        
        return q_current + step_size * (direction / distance), False
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
    steering_step_size: float = 0.1
    probabilities: BiRRTSamplingProbabilities = BiRRTSamplingProbabilities()
    max_tree_nodes: int = int(1e5)
    convergence_threshold: float = 1e-3

class BidirectionalRRTPlanner(MotionPlanner):
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        config: BidirectionalRRTPlannerConfig = None,
    ):
        super().__init__(robot_model_idx, plant, scene_graph)
        # Input Processing
        if config is None:
            config = BidirectionalRRTPlannerConfig()

        # Setup
        self.config = config

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

            # Choose whether or not to sample from the opposite tree or randomly
            if np.random.rand() < prob_sample_opposite_tree:
                # Sample from the opposite tree
                q_random = self.sample_random_configuration()
                nearest_node, nearest_node_idx = self.find_nearest_node(rrt_goal, q_random)
                q_new = self.steer(nearest_node['q'], q_random)
                if not collision_check_fcn(q_new):
                    rrt_goal.add_node(n_nodes_goal, q=q_new)
                    rrt_goal.add_edge(nearest_node_idx, n_nodes_goal)
                    n_nodes_goal += 1
            else:
                # Sample from a random configuration
                pass

    def sample_fraom_tree(
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
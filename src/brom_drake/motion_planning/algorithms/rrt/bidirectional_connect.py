"""
bidirectional_connect.py
Description
-----------
This file contains an algorithm for performing bidirectional RRT-Connect
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
# TODO(Kwesi): Introduce planning node object

# Define config dataclass
@dataclass(frozen=True)
class BiRRTConnectSamplingProbabilities:
    """
    A dataclass that defines the sampling probabilities for a bidirectional RRT planner.
    """
    sample_goal_tree: float = 0.10
    sample_opposite_tree: float = 0.10
    

@dataclass
class BidirectionalRRTConnectPlannerConfig:
    """
    A dataclass that defines the configuration for a bidirectional RRT planner.
    """
    convergence_threshold: float = 1e-3
    max_tree_nodes: int = int(1e5)
    probabilities: BiRRTConnectSamplingProbabilities = BiRRTConnectSamplingProbabilities()
    random_seed: int = 23
    steering_step_size: float = 0.1
    debug: bool = False

class BidirectionalRRTConnectPlanner(MotionPlanner):
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        config: BidirectionalRRTConnectPlannerConfig = None,
    ):
        # Input Processing
        if config is None:
            config = BidirectionalRRTConnectPlannerConfig()

        # Setup
        self.config = config

        # Use the parent class constructor
        super().__init__(
            robot_model_idx, plant, scene_graph, 
            random_seed=self.config.random_seed,
        )

    def add_new_node_to(
        self,
        target_tree: nx.DiGraph,
        prev_node: nx.classes.reportviews.NodeView,
        q_new: np.ndarray,
        tree_is_goal: bool,
    ):
        """
        Description
        -----------
        This function adds a new node to the RRT.

        Arguments
        ---------
        target_tree: nx.DiGraph
            The RRT to add the node to.
        q_new: np.ndarray
            The new configuration to add to the RRT.
        """
        # Setup
        n_nodes = target_tree.number_of_nodes()

        # Add the new node to the RRT
        target_tree.add_node(n_nodes, q=q_new)
        if tree_is_goal:
            target_tree.add_edge(
                target_tree.number_of_nodes()-1,
                prev_node,
            )
        else:
            target_tree.add_edge(
                prev_node,
                target_tree.number_of_nodes()-1,
            )

    def connect(
        self,
        nearest_node_view: nx.classes.reportviews.NodeView,
        q_target: np.ndarray,
        rrt: nx.DiGraph,
        tree_is_goal: bool,
        collision_check_fcn: Callable[[np.ndarray], bool] = None,
        target_config_already_exists: bool = False,
    ) -> Tuple[np.ndarray, bool]:
        """
        Description
        -----------
        This function connects the RRT from the nearest node to the target configuration.

        Arguments
        ---------
        target_config_already_exists: bool
            Whether or not the target configuration already exists in the combined RRT.
        """
        # Input Processing
        if collision_check_fcn is None:
            collision_check_fcn = self.check_collision_in_config

        # Setup
        nearest_node = rrt.nodes[nearest_node_view]
        debug = self.config.debug

        # Steer from nearest node to random configuration
        q_current = nearest_node['q']
        last_node_view = nearest_node_view

        n_loops = 0
        reached_new = False
        while not np.isclose(q_current, q_target).all():
            # Create next node
            q_new, reached_new = self.steer(q_current, q_target)

            # Check for collisions
            if collision_check_fcn(q_new):
                break

            # Check if we reached the target configuration
            if reached_new and target_config_already_exists:
                break

            # If the next point is collision free, then add new node to the tree
            self.add_new_node_to(
                rrt,
                last_node_view,
                q_new,
                tree_is_goal=tree_is_goal,
            )

            # Prepare for next iteration
            last_node_view = list(rrt.nodes)[-1]
            q_current = q_new

            n_loops += 1

        if debug:
            print(f"Number of loops in connection: {n_loops}")
            print(f"RRT Size: {rrt.number_of_nodes()}")

        return q_current, reached_new # Return the last configuration we visited

    def plan(
        self,
        q_start: np.ndarray,
        q_goal: np.ndarray,
        collision_check_fcn: Callable[[np.ndarray], bool] = None,
    ) -> Tuple[nx.DiGraph, int]:
        """
        Description
        -----------
        This function plans a path from q_start to q_goal using the Bidirectional
        RRT algorithm.

        Returns
        -------
        nx.DiGraph
            The RRT that was created during the planning process.
        int
            The index of the goal node in the RRT.
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
                combined_tree, connected_two_trees = self.steer_towards_tree(
                    rrt_start,
                    rrt_goal,
                    current_tree_is_goal=sample_from_goal_tree,
                    collision_check_fcn=collision_check_fcn,
                )
                if connected_two_trees:
                    return combined_tree, rrt_start.number_of_nodes()

            else:
                # Sample from a random configuration
                q_random = self.sample_random_configuration()

                # Find the nearest node in the tree and steer towards it
                nearest_node_view, nearest_node_idx = self.sample_nearest_in_tree(current_tree, q_random)
                q_new, reached_new = self.connect(
                    nearest_node_view,
                    q_random,
                    current_tree,
                    tree_is_goal=sample_from_goal_tree,
                    collision_check_fcn=collision_check_fcn,
                )
                # q_new, reached_new = self.steer(
                #     current_tree.nodes[nearest_node_view]['q'],
                #     q_random,
                # )

                # # Check if the new configuration is in collision
                # if collision_check_fcn(q_new):
                #     continue

                # # If the configuration is not in collision,
                # # then add it to the current tree
                # self.add_new_node_to(
                #     current_tree,
                #     nearest_node_view,
                #     q_new,
                #     tree_is_goal=sample_from_goal_tree,
                # )

            # Update the number of nodes in the appropriate tree
            n_nodes_goal = rrt_goal.number_of_nodes()
            n_nodes_start = rrt_start.number_of_nodes()

        # If we exit the loop without finding a path to the goal,
        # return the RRT and indicate failure
        print("Max iterations reached without finding a path to the goal.")
        return nx.disjoint_union(rrt_start, rrt_goal), -1

    def sample_from_tree(
        self,
        rrt: nx.DiGraph,
    ) -> nx.classes.reportviews.NodeView:
        """
        Description
        -----------
        This function samples a configuration from the RRT.
        """
        # Setup
        n_tree = rrt.number_of_nodes()

        # Select a random number from the range [0, n_tree)
        random_index = np.random.randint(0, n_tree)
        for ii, node_ii in enumerate(rrt.nodes):
            if ii == random_index:
                return node_ii

        raise ValueError(
            f"Random index ({random_index}) not found in RRT with {n_tree} nodes."
        )
    
    def sample_nearest_in_tree(
        self,
        rrt: nx.DiGraph,
        q_random: np.ndarray,
    ) -> Tuple[nx.classes.reportviews.NodeView, float]:
        """
        Description
        -----------
        This function samples a configuration from the RRT.

        Arguments
        ---------
        rrt: nx.DiGraph
            The RRT to sample from.
        q_random: np.ndarray
            The random configuration to sample from the RRT.

        Returns
        -------
        node_view: nx.classes.reportviews.NodeView
            A reference to the nearest node in the RRT.
            Use this to access the node as follows rrt.nodes[node_view].
        distance: float
            The distance from the random configuration to the nearest node in the RRT.
            
        """
        # Setup

        # Search through the tree to find the node that is nearest to the random configuration
        min_distance = float('inf')
        nearest_node_view = None
        for node in rrt.nodes:
            distance = np.linalg.norm(rrt.nodes[node]['q'] - q_random)
            if distance < min_distance:
                min_distance = distance
                nearest_node_view = node

        return nearest_node_view, min_distance
    
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
    
    def steer_towards_tree(
        self,
        rrt_start: nx.DiGraph,
        rrt_goal: nx.DiGraph,
        current_tree_is_goal: bool,
        collision_check_fcn: Callable[[np.ndarray], bool] = None,
    ) -> Tuple[nx.DiGraph, bool]:
        """
        Description
        -----------
        This function steers the RRT from the start configuration to the goal configuration.

        Arguments
        ---------

        Returns
        -------
        bool
            Whether or not we reached the target configuration with our step size or not.
        """
        # Setup
        debug = self.config.debug
        current_tree = rrt_goal if current_tree_is_goal else rrt_start

        if collision_check_fcn is None:
            collision_check_fcn = self.check_collision_in_config

        # Sample a node in the current tree
        current_node_view = self.sample_from_tree(current_tree)
        q_current = current_tree.nodes[current_node_view]['q']

        # Sample from the opposite tree
        opposite_tree = rrt_start if current_tree_is_goal else rrt_goal
        node_idx_in_opposite_tree, min_distance = self.sample_nearest_in_tree(opposite_tree, q_current)
        
        node_in_opposite_tree = opposite_tree.nodes[node_idx_in_opposite_tree]
        sampled_config = node_in_opposite_tree['q']

        # Steer from the current configuration to the sampled one
        # q_new, reached_new = self.steer(q_current, sampled_config)
        q_new, reached_new = self.connect(
            current_node_view,
            sampled_config,
            current_tree,
            tree_is_goal=current_tree_is_goal,
            collision_check_fcn=collision_check_fcn,
            target_config_already_exists=True,
        )

        # Check if the new configuration is in collision
        if collision_check_fcn(q_new):
            return None, False

        if reached_new:
            combined_rrt = nx.disjoint_union(rrt_start, rrt_goal)
            # add edge between the two trees
            if current_tree_is_goal:
                if debug:
                    print(f"Adding edge between {node_idx_in_opposite_tree} and {rrt_start.number_of_nodes() + int(current_node_view)}")

                combined_rrt.add_edge(
                    node_idx_in_opposite_tree,
                    rrt_start.number_of_nodes() + int(current_node_view),
                )
            else:
                if debug:
                    print(f"Adding edge between {current_node_view} and {rrt_start.number_of_nodes() + node_idx_in_opposite_tree}")

                combined_rrt.add_edge(
                    int(current_node_view),
                    rrt_start.number_of_nodes() + node_idx_in_opposite_tree,
                )
            return combined_rrt, True
        else:
            # If we did not reach the goal,
            # add the new configuration to the current tree
            self.add_new_node_to(
                current_tree,
                current_node_view,
                q_new,
                tree_is_goal=current_tree_is_goal,
            )

        # We did not finish, so do not return the combined RRT
        # and tell the caller that we did not finish
        return None, False
"""
motion_planner.py
Description:

        This file defines the MotionPlanner class.
        This class is used to plan motion for a robot.
"""

import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex

from brom_drake.utils.collision_checking import CollisionCheckMethod, find_collisions


class MotionPlanner:
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
        random_seed: int = 23,
        collision_threshold: float = 1e-2,
    ):
        """
        Description:
            This function initializes the motion planner.

        Arguments:
            collision_threshold: Safety margin in meters. A configuration counts as
                colliding when any pair of geometries is closer than this
                (signed distance). Defaults to 1e-2.
        """
        # Input Processing
        self.robot_model_idx = robot_model_idx
        self.plant = plant
        self.scene_graph = scene_graph
        self.collision_threshold = collision_threshold

        # Setup
        self.root_context = None  # Usually, the diagram context that we can use to extract the plant's context

        # Set the random seed
        self.random_seed = random_seed
        np.random.seed(self.random_seed)

    @property
    def dim_q(self) -> int:
        if not self.plant.is_finalized():
            raise ValueError(
                "Plant has not been finalized yet! Can not compute num_actuated_dofs()."
            )

        return self.plant.num_actuated_dofs(self.robot_model_idx)

    def check_collision_in_config(
        self,
        q_model: np.ndarray,
    ) -> bool:
        """
        Description:
            This function checks for collisions in the robot's environment.
        """
        # Input Processing
        if self.root_context is None:
            raise ValueError("Plant context is not initialized yet!")

        # Set the configuration
        plant_context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        self.plant.SetPositions(plant_context, self.robot_model_idx, q_model)

        # Any pair closer than collision_threshold counts as a collision
        collisions = find_collisions(
            self.scene_graph,
            self.root_context,
            threshold=self.collision_threshold,
            method=CollisionCheckMethod.SIGNED_DISTANCE,
        )
        return len(collisions) > 0

    @property
    def joint_limits(self) -> np.ndarray:
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
            joint_limits = np.vstack((joint_limits, [lower_limits[0], upper_limits[0]]))

        return joint_limits

    def sample_random_configuration(self) -> np.ndarray:
        """
        Description
        -----------
        This function samples a random configuration within the joint limits.
        """
        return np.random.uniform(self.joint_limits[:, 0], self.joint_limits[:, 1])

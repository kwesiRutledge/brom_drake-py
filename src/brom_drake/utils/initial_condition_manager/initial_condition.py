from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    ModelInstanceIndex,
    MultibodyPlant,
    RigidTransform
)

@dataclass
class InitialCondition:
    model_instance_index: ModelInstanceIndex
    pose_wrt_parent: RigidTransform = None # Is parent always world?
    configuration: np.ndarray = None
    target_body_index: int = 0

    def set_initial_configuration(self, plant: MultibodyPlant):
        """
        Description
        -----------
        Set the initial configuration for the given model instance in the plant.
        """
        # Input Processing
        # - If no configuration is provided, then skip this function
        if self.configuration is None:
            return

        # - If the configuration value exists, then verify that it has the correct size
        if len(self.configuration) != plant.num_positions(self.model_instance_index):
            raise ValueError(f"Configuration size {len(self.configuration)} does not match plant's num_positions {plant.num_positions()}")

        # Set initial configuration
        plant.SetDefaultPositions(self.model_instance_index, self.configuration)
        # plant.SetPositions(self.model_instance_index, self.configuration)

    def set_initial_pose(self, plant: MultibodyPlant):
        """
        Description
        -----------
        Set the initial pose for the given model instance in the plant.
        """
        # Input Processing
        # - If no pose is provided, then skip this function
        if not self.pose_wrt_parent:
            return

        # Get the body frame
        attached_bodies = plant.GetBodyIndices(self.model_instance_index)
        target_body_index = attached_bodies[self.target_body_index]

        # Set initial pose
        plant.SetDefaultFreeBodyPose(
            plant.get_body(target_body_index),
            self.pose_wrt_parent)
        
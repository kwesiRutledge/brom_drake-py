from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    Context,
    ModelInstanceIndex,
    MultibodyPlant,
    RigidTransform,
    SpatialVelocity
)

@dataclass
class InitialCondition:
    model_instance_index: ModelInstanceIndex
    pose_wrt_parent: RigidTransform = None # Is parent always world?
    configuration: np.ndarray = None
    target_body_index: int = 0

    def set_initial_configuration(
        self,
        plant: MultibodyPlant,
        diagram_context: Context = None
    ):
        """
        *Description*
        
        Set the initial configuration for the given model instance in the plant.

        Note: Defining the default positions will fail, if the context for the plant
        has already been created. In that case, we also set the positions in the provided
        diagram context (if any).
        """
        # Input Processing
        # - If no configuration is provided, then skip this function
        if self.configuration is None:
            return

        # - If the plant is not yet finalized, then raise an exception!
        if not plant.is_finalized():
            raise RuntimeError("Plant is not yet finalized! Finalize before calling set_initial_configuration()!")

        # - If the configuration value exists, then verify that it has the correct size
        if len(self.configuration) != plant.num_positions(self.model_instance_index):
            raise ValueError(f"Configuration size {len(self.configuration)} does not match plant's num_positions {plant.num_positions(self.model_instance_index)}")

        # Set initial configuration
        plant.SetDefaultPositions(self.model_instance_index, self.configuration)
        print(f"Set default positions for model instance {self.model_instance_index} to {self.configuration}")
        if diagram_context is not None:
            plant.SetPositions(
                plant.GetMyMutableContextFromRoot(diagram_context),
                self.model_instance_index,
                self.configuration
            )

    def set_initial_pose(self, plant: MultibodyPlant, diagram_context: Context = None):
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
        
        if diagram_context is not None:
            plant.SetFreeBodyPose(
                plant.GetMyMutableContextFromRoot(diagram_context),
                plant.get_body(target_body_index),
                self.pose_wrt_parent)
            
            plant.SetFreeBodySpatialVelocity(
                body=plant.get_body(target_body_index),
                V_PB=SpatialVelocity.Zero(),
                context=plant.GetMyMutableContextFromRoot(diagram_context),
            )
        
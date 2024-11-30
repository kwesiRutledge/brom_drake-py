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


class MotionPlanner:
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
        scene_graph: SceneGraph,
    ):
        """
        Description:
            This function initializes the motion planner.
        """
        # Input Processing
        self.robot_model_idx = robot_model_idx
        self.plant = plant
        self.scene_graph = scene_graph

        # Setup
        self.root_context = None # Usually, the diagram context that we can use to extract the plant's context

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

        # Setup
        plant_context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        scene_graph_context = self.scene_graph.GetMyMutableContextFromRoot(self.root_context)

        # Set the configuration
        self.plant.SetPositions(
            plant_context,
            self.robot_model_idx,
            q_model
        )

        # Note: This method will be inaccurate if using arbitrary, nonconvex meshes!
        # https://drake.mit.edu/pydrake/pydrake.geometry.html?highlight=queryobject#pydrake.geometry.QueryObject.HasCollisions
        query_object = self.scene_graph.get_query_output_port().Eval(scene_graph_context)
        # return query_object.HasCollisions()

        # Alternative method to check for collisions
        closest_points = query_object.ComputeSignedDistancePairwiseClosestPoints()
        eps0 = 1e-2
        for pair in closest_points:
            if pair.distance < eps0:
                return True
            
        # Otherwise return false
        return False


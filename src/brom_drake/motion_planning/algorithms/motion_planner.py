"""
motion_planner.py
Description:

        This file defines the MotionPlanner class.
        This class is used to plan motion for a robot.
"""
import numpy as np
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex


class MotionPlanner:
    def __init__(
        self,
        robot_model_idx: ModelInstanceIndex,
        plant: MultibodyPlant,
    ):
        """
        Description:
            This function initializes the motion planner.
        """
        # Input Processing
        self.robot_model_idx = robot_model_idx
        self.plant = plant

        # Setup
        self.plant_context = None

    def check_collision_in_config(
        self,
        q_model: np.ndarray,
    ):
        """
        Description:
            This function checks for collisions in the robot's environment.
        """
        # Input Processing
        if self.plant_context is None:
            raise ValueError("Plant context is not initialized yet!")

        # Set the configuration
        self.plant.SetPositions(
            self.plant_context,
            self.robot_model_idx,
            q_model
        )

    def plan_motion(self):
        """
        Description:
            This function plans the motion for the robot.
        """
        # Motion planning logic goes here
        pass
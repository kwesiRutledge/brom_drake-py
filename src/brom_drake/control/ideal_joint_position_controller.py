from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, CoulombFriction
from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector, Context

import numpy as np

# Internal Imports
from brom_drake.robots.utils import find_base_link_name_in


class IdealJointPositionController(LeafSystem):
    def __init__(
        self,
        robot_model_file_name: str,
        plant_dt: float = 1e-3,
        robot_base_link_name: str = None,
        plant_context: Context = None,
        q0: np.ndarray = None,
    ):
        LeafSystem.__init__(self)
        # Input Processing

        # Set Up
        self.plant, self.controller_arm = None, None
        self.plant_context = plant_context
        self.create_plant_containing_just_arm(robot_model_file_name, plant_dt, robot_base_link_name)

        self.q0 = q0

        # Create Input Port for the Body's Joint Positions
        self.desired_joint_positions_port = self.DeclareVectorInputPort(
            "desired_joint_positions",
            BasicVector(self.plant.num_positions()),
        )

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
            "measured_joint_positions",
            BasicVector(self.plant.num_positions()),
            self.SetJointPositions,
            {self.time_ticket()}    # indicate that this doesn't depend on any inputs,
        )                           # but should still be updated each timestep

    def create_plant_containing_just_arm(
        self,
        robot_model_file_name: str,
        plant_dt: float = 1e-3,
        robot_base_link_name: str = None,
    ):
        """
        Description:
            Creates a new plant containing just the robot arm
        :param robot_model_file_name: The path to the robot's urdf file
        :param plant_dt: The time step for the plant
        :return:
        """
        # Create a new plant for use only by this controller and add the robot to it
        self.plant = MultibodyPlant(plant_dt)
        self.controller_arm = Parser(plant=self.plant).AddModels(
            robot_model_file_name
        )[0]

        # Weld The Robot's Base to the world frame
        if robot_base_link_name is None:
            # Use utility function to try to find the base link
            robot_base_link_name = find_base_link_name_in(robot_model_file_name)

        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName(robot_base_link_name, self.controller_arm),
        )

        # Finalize Plant
        self.plant.Finalize()


    def SetJointPositions(self, context, output):
        """
        Description:
            This function sets the desired pose of the block.
        """

        # Get Desired Pose from Port
        plant_context = self.plant_context
        joint_positions = self.desired_joint_positions_port.Eval(context)

        # Get plant context from station context
        # station_context = self.station_context
        # if station_context is None:
        #     raise RuntimeError("station_context is None")

        self.plant.SetPositions(
            plant_context,
            joint_positions,
        )

        self.plant.SetVelocities(
            plant_context,
            np.zeros(self.plant.num_velocities())
        )

        output.SetFromVector(joint_positions)

    def SetInitialJointPositions(self, diagram_context):
        """
        Description:
            Sets the initial position .
        """
        # Setup
        q0 = self.q0
        if q0 is None:
            q0 = np.zeros(self.plant.num_positions())

        station_context = self.station_context
        if station_context is None:
            station_context = diagram_context
        plant_context = self.GetSubsystemContext(self.plant, station_context)

        # Set Joint Positions and velocities
        self.plant.SetPositions(plant_context, q0)

        self.plant.SetVelocities(
            plant_context,
            np.zeros(self.plant.num_velocities())
        )
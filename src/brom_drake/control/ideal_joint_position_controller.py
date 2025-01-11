from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, CoulombFriction
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import LeafSystem, PortDataType, BasicVector, Context

import numpy as np

# Internal Imports
from brom_drake.robots.utils import find_base_link_name_in


class IdealJointPositionController(LeafSystem):
    def __init__(
        self,
        robot_model: ModelInstanceIndex,
        plant: MultibodyPlant,
        q0: np.ndarray = None,
        plant_context: Context = None,
    ):
        LeafSystem.__init__(self)
        # Input Processing

        # Set Up
        self.plant, self.robot_model = plant, robot_model
        self.plant_context = plant_context

        self.q0 = q0

        self.arm = robot_model

        # Create Input Port for the Body's Joint Positions
        self.desired_joint_positions_port = self.DeclareVectorInputPort(
            "desired_joint_positions",
            BasicVector(self.plant.num_actuated_dofs(self.arm)),
        )

        # Create Output Port which should share the pose of the block
        self.DeclareVectorOutputPort(
            "measured_joint_positions",
            BasicVector(self.plant.num_actuated_dofs(self.arm)),
            self.SetJointPositions,
            {self.time_ticket()}    # indicate that this doesn't depend on any inputs,
        )                           # but should still be updated each timestep


    def SetJointPositions(self, context, output):
        """
        Description:
            This function sets the desired pose of the block.
        """
        # Setup
        plant_context = self.plant_context
        robot_model = self.robot_model

        # Get Desired Pose from Port
        joint_positions = self.desired_joint_positions_port.Eval(context)

        # Get plant context from station context
        # station_context = self.station_context
        # if station_context is None:
        #     raise RuntimeError("station_context is None")

        self.plant.SetPositions(plant_context, robot_model, joint_positions)
        self.plant.SetVelocities(
            plant_context,
            robot_model,
            np.zeros(self.plant.num_velocities(robot_model))
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
        self.plant.SetPositions(plant_context, self.robot_model, q0)

        self.plant.SetVelocities(
            plant_context,
            self.robot_model,
            np.zeros(self.plant.num_velocities(self.robot_model))
        )
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.math import SpatialVelocity
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, CoulombFriction
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import (
    BasicVector,
    Context,
    EventStatus,
    LeafSystem,
    PortDataType,
)

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
            {self.time_ticket()},  # indicate that this doesn't depend on any inputs,
        )  # but should still be updated each timestep

        # Drive the plant on every simulation step (and at Simulator.Initialize()).
        # Without this event, the plant was only updated when something happened
        # to evaluate the "measured_joint_positions" output (e.g., a DiagramWatcher),
        # so productions built without a watcher never moved the arm or planned.
        self.DeclarePerStepPublishEvent(self._apply_desired_positions_event)

    def _apply_desired_positions(self, context: Context) -> np.ndarray:
        """
        **Description**

        Reads the desired joint positions from the input port and writes them
        (with zero velocities) into the plant context.

        **Returns**

        joint_positions: np.ndarray
            The joint positions that were applied.
        """
        if self.plant_context is None:
            raise RuntimeError(
                "IdealJointPositionController.plant_context is not set. Assign the "
                "plant's context from the built diagram before simulating, e.g. "
                "controller.plant_context = diagram.GetMutableSubsystemContext(plant, diagram_context)."
            )

        joint_positions = self.desired_joint_positions_port.Eval(context)

        self.plant.SetPositions(self.plant_context, self.robot_model, joint_positions)
        self.plant.SetVelocities(
            self.plant_context,
            self.robot_model,
            np.zeros(self.plant.num_velocities(self.robot_model)),
        )
        return joint_positions

    def _apply_desired_positions_event(self, context: Context) -> EventStatus:
        # Nothing is commanding the arm (e.g., a station simulated on its own):
        # leave the plant alone instead of failing.
        if not self.desired_joint_positions_port.HasValue(context):
            return EventStatus.DidNothing()

        self._apply_desired_positions(context)
        return EventStatus.Succeeded()

    def SetJointPositions(self, context, output):
        """
        Description:
            Calculation for the "measured_joint_positions" output port.
            Applies the desired joint positions to the plant (so the output and
            the plant agree) and reports them.
        """
        output.SetFromVector(self._apply_desired_positions(context))

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
            np.zeros(self.plant.num_velocities(self.robot_model)),
        )

"""
gripper_controller.py
Description:

        This file defines the GripperController class.
"""
# External Imports
import importlib.resources as impresources

from pydrake.common.value import AbstractValue
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import LeafSystem
from pydrake.systems.framework import BasicVector

import os

import numpy as np

# Internal Imports
from .gripper_target import GripperTarget
from brom_drake.robots.gripper_type import GripperType

from .. import robots


# Constants
package_dir = os.path.dirname(os.path.abspath(__file__))


class GripperController(LeafSystem):
    """
    A simple gripper controller with two modes: position and velocity.
    Both modes are essentially simple PD controllers.

                            -------------------------
                            |                       |
                            |                       |
    gripper_target -------> |   GripperController   | ---> applied_gripper_torques
    gripper_target_type --> |                       |
                            |                       | ---> measured_gripper_position
                            |                       | ---> measured_gripper_velocity
    gripper_state --------> |                       |
                            |                       |
                            |                       |
                            -------------------------
    """
    def __init__(self, gripper_type: GripperType):
        """
        Description
        -----------
        Constructor for the GripperController class.
        :param gripper_type: The type can be "Robotiq_2f_85" or "NoGripper"
        """
        LeafSystem.__init__(self)
        self.type = gripper_type
        self.context = None

        # State input port size depends on what type of gripper we're using
        if self.type == GripperType.Robotiq_2f_85:
            state_size = 12

            # We'll create a simple model of the gripper which is welded to the floor.
            # This will allow us to compute the distance between fingers.
            self.plant = MultibodyPlant(time_step=1.0) # time step doesn't matter
            gripper_urdf = str(
                impresources.files(robots) / "models/2f_85_gripper/urdf/robotiq_2f_85.urdf"
            )
            self.gripper = Parser(plant=self.plant).AddModels(gripper_urdf)[0]
            self.plant.WeldFrames(
                self.plant.world_frame(),
                self.plant.GetFrameByName("robotiq_arg2f_base_link"),
            )
            # Finalize the plant used for the gripper
            self.plant.Finalize()
            self.context = self.plant.CreateDefaultContext()

        else:
            raise RuntimeError("Invalid gripper type: %s" % self.type)

        # Declare input ports
        self.target_port = self.DeclareVectorInputPort("gripper_target", BasicVector(1))
        self.target_type_port = self.DeclareAbstractInputPort(
            "gripper_target_type",
            AbstractValue.Make(GripperTarget.kPosition),
        )
        self.state_port = self.DeclareVectorInputPort("gripper_state", BasicVector(state_size))

        # Declare output ports
        self.DeclareVectorOutputPort("applied_gripper_torque", BasicVector(2), self.CalcGripperTorque)
        self.DeclareVectorOutputPort(
            "measured_gripper_position", BasicVector(1), self.CalcGripperPosition,
            {self.time_ticket()}    # indicate that this doesn't depend on any inputs,
        )                           # but should still be updated each timestep
        self.DeclareVectorOutputPort(
            "measured_gripper_velocity", BasicVector(1), self.CalcGripperVelocity,
            {self.time_ticket()}    # indicate that this doesn't depend on any inputs,
        )                           # but should still be updated each timestep

    def ComputePosition(self, state):
        """
        Compute the gripper position from state data.
        This is especially useful for the 2F-85 gripper, since the
        state does not map neatly to the finger positions.
        """
        # Setup
        finger_position = None

        # Get the finger positions
        if self.type == GripperType.Robotiq_2f_85:
            # For the more complex 2F-85 gripper, we need to do some kinematics
            # calculations to figure out the gripper position
            self.plant.SetPositionsAndVelocities(self.context, state)

            right_finger = self.plant.GetFrameByName("right_inner_finger_pad")
            left_finger = self.plant.GetFrameByName("left_inner_finger_pad")
            base = self.plant.GetFrameByName("robotiq_arg2f_base_link")

            X_lf = self.plant.CalcRelativeTransform(self.context,
                                                    left_finger,
                                                    base)
            X_rf = self.plant.CalcRelativeTransform(self.context,
                                                    right_finger,
                                                    base)

            lf_pos = -X_lf.translation()[1]
            rf_pos = -X_rf.translation()[1]

            finger_position = np.array([lf_pos, rf_pos])

        else:
            raise NotImplementedError("Gripper type %s does not have ComputePosition() implemented" % self.type)

        return finger_position

    def ComputeVelocity(self, state):
        """
        Compute the gripper velocity from state data.
        This is especially useful for the 2F-85 gripper, since the
        state does not map neatly to the finger positions.
        """
        if self.type == GripperType.Robotiq_2f_85:
            # For the more complex 2F-85 gripper, we need to do some kinematics
            self.plant.SetPositionsAndVelocities(self.context, state)
            v = state[-self.plant.num_velocities():]

            right_finger = self.plant.GetFrameByName("right_inner_finger_pad")
            left_finger = self.plant.GetFrameByName("left_inner_finger_pad")
            base = self.plant.GetFrameByName("robotiq_arg2f_base_link")

            J_lf = self.plant.CalcJacobianTranslationalVelocity(
                self.context, JacobianWrtVariable.kV, left_finger, np.zeros(3), base, base,
            )
            J_rf = self.plant.CalcJacobianTranslationalVelocity(
                self.context, JacobianWrtVariable.kV, right_finger, np.zeros(3), base, base,
            )

            lf_vel = -(J_lf @ v)[1]
            rf_vel = (J_rf @ v)[1]

            finger_velocity = np.array([lf_vel, rf_vel])

        else:
            raise NotImplementedError("Gripper type %s does not have ComputeVelocity() implemented" % self.type)

        return finger_velocity

    def CalcGripperPosition(self, context, output):
        state = self.state_port.Eval(context)

        if self.type == "hande":
            width = 0.03
        elif self.type == GripperType.Robotiq_2f_85:  #2f_85
            width = 0.06
        else:
            raise NotImplementedError("Gripper type %s does not have CalcGripperPosition() implemented" % self.type)

        # Send a single number to match the hardware
        both_finger_positions = self.ComputePosition(state)
        net_position = 1/width* np.mean(both_finger_positions)

        output.SetFromVector([net_position])

    def CalcGripperVelocity(self, context, output):
        state = self.state_port.Eval(context)

        if self.type == "hande":
            width = 0.03
        elif self.type == GripperType.Robotiq_2f_85:  # 2f_85
            width = 0.06
        else:
            raise NotImplementedError("Gripper type %s does not have CalcGripperVelocity() implemented" % self.type)

        # Send a single number to match the hardware
        both_finger_velocity = self.ComputeVelocity(state)
        net_velocity = 1 / width * np.mean(both_finger_velocity)

        output.SetFromVector([net_velocity])

    def CalcGripperTorque(self, context, output):
        state = self.state_port.Eval(context)
        target = self.target_port.Eval(context)
        target_type = self.target_type_port.Eval(context)

        finger_position = self.ComputePosition(state)
        finger_velocity = self.ComputeVelocity(state)

        # Set PD gains depending on gripper type
        if self.type == "hande":
            width = 0.03
            Kp = 100 * np.eye(2)
            Kd = 2 * np.sqrt(Kp)
        elif self.type == GripperType.Robotiq_2f_85:
            width = 0.06
            Kp = 10 * np.eye(2)
            Kd = 2 * np.sqrt(0.01 * Kp)
        else:
            raise NotImplementedError("Gripper type %s does not have CalcGripperTorque() implemented" % self.type)

        # Set target positions and velocities based on the current control mode
        if target_type == GripperTarget.kPosition:
            target = width - width * target * np.ones(2)
            target_finger_position = target
            target_finger_velocity = np.zeros(2)
        elif target_type == GripperTarget.kVelocity:
            target_finger_position = finger_position
            target_finger_velocity = -width * target
        else:
            raise RuntimeError("Invalid gripper target type: %s" % target_type)

        # Determine applied torques with PD controller
        position_err = target_finger_position - finger_position
        velocity_err = target_finger_velocity - finger_velocity
        tau = -Kp @ (position_err) - Kd @ (velocity_err)

        output.SetFromVector(tau)

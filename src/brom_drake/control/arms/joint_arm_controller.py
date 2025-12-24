import numpy as np
from pydrake.all import Context
from pydrake.common.value import AbstractValue
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.framework import BasicVector

# Internal imports
from brom_drake.control.arms.base_arm_controller import BaseArmController
from brom_drake.control.arms.joint_target import JointTarget


class JointArmController(BaseArmController):
    """
    *Description*

    A controller which imitates the joint control mode of the Kinova gen3 arm. 
    This is designed to work with any arm.

    The controller is an extension of the pydrake.systems.framework.LeafSystem.
    As such, it has input and output ports which are described in detail in the following
    block diagram ::

                                    ------------------------
                                    |                      |
                                    |                      |
        joint_target -------------> |  JointArmController  | ----> applied_arm_torque
        joint_target_type --------> |                      |
                                    |                      |
                                    |                      | ----> measured_ee_pose
        arm_joint_position -------> |                      | ----> measured_ee_twist
        arm_joint_velocity ------>  |                      |
                                    |                      |
                                    |                      |
                                    ------------------------

    The type of target is determined by ee_target_type, and the options are defined in the
    end effector target.
    """
    def __init__(
        self,
        plant: MultibodyPlant,
        arm_model: ModelInstanceIndex,
        end_effector_frame_name: str = "end_effector_frame",
        joint_target_type: JointTarget = JointTarget.kPosition,
    ):
        BaseArmController.__init__(
            self, plant, arm_model,
            end_effector_frame_name=end_effector_frame_name,
        )

        # Define input and Output Ports
        self.joint_target_port, self.joint_target_type_port = None, None
        self.define_joint_target_input_ports(joint_target_type)

        # Define output ports
        self.DeclareVectorOutputPort(
            "applied_arm_torque",
            BasicVector(self.plant.num_actuators()),
            self.CalcArmTorques
        )

    def define_joint_target_input_ports(self, joint_target_type: JointTarget):
        """
        *Description*
        
        Define the input ports for the JointArmController system
        
        *Parameters*

        joint_target_type: JointTarget
            The type of joint target being provided to the controller.
        """
        self.joint_target_port = self.DeclareVectorInputPort(
            "joint_target",
            BasicVector(self.plant.num_positions(self.arm)),
        )
        self.joint_target_type_port = self.DeclareAbstractInputPort(
            "joint_target_type",
            AbstractValue.Make(joint_target_type),
        )

    def CalcArmTorques(self, context: Context, output: BasicVector):
        """
        *Description*
        
        Calculate the arm torques based on the joint target and the current state of the arm.
        
        *Parameters*

        context: pydrake.systems.framework.Context
            The context of the system at the current timestep.

        output: pydrake.systems.framework.BasicVector
            The output vector where the calculated torques will be stored.
        """
        # Get the current state of the arm
        q = self.arm_joint_position_port.Eval(context)
        qd = self.arm_joint_velocity_port.Eval(context)

        # Set our "internal plant's state" to the current state of the arm
        self.plant.SetPositions(self.context, self.arm, q)
        self.plant.SetVelocities(self.context, self.arm, qd)

        # Calculate the torque needed to compensate for gravity
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # Indicate what type of command we're recieving
        target_type = self.joint_target_type_port.Eval(context)

        if target_type == JointTarget.kPosition:
            q_des = self.joint_target_port.Eval(context)
            q_err = q_des - q
            qd_des = 0 * q_err
            kp = 3e3
            kd = 2*np.sqrt(3e3)
            tau = kp * q_err + kd * (qd_des - qd) + tau_g

        else:
            raise NotImplementedError(
                f"Only position joint control is currently supported; received {target_type}"
            )

        output.SetFromVector(tau)
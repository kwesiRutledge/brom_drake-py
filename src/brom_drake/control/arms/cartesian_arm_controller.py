"""
cartesian_controller.py
Description:

    This file defines the CartesianController class. This class is used to control different manipulators and generates
    torque based control commands for the targeted arm.
"""
# External Imports
from pydrake.common.value import AbstractValue
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsParameters,
    DoDifferentialInverseKinematics, InverseKinematics
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import MultibodyForces, JacobianWrtVariable
from pydrake.solvers import Solve
from pydrake.systems.framework import BasicVector

import numpy as np

# Internal Imports
from brom_drake.control.arms.end_effector_target import EndEffectorTarget
from .base_arm_controller import BaseArmController


class CartesianArmController(BaseArmController):
    """
    A controller which imitates the cartesian control mode of the Kinova gen3 arm.

                                -----------------------------
                                |                           |
                                |                           |
    ee_target ----------------> |  CartesianArmController   | ----> applied_arm_torque
    ee_target_type -----------> |                           |
                                |                           |
                                |                           | ----> measured_ee_pose
    arm_joint_position -------> |                           | ----> measured_ee_twist
    arm_joint_velocity ------>  |                           |
                                |                           |
                                |                           |
                                -----------------------------

    Note that many ports are defined by the base class (BaseArmController).
    """

    def __init__(
        self,
        plant: MultibodyPlant,
        arm_model,
        end_effector_frame_name: str = "end_effector_frame",
    ):
        BaseArmController.__init__(
            self,
            plant,
            arm_model,
            end_effector_frame_name=end_effector_frame_name,
        )

        # Define input and Output Ports
        self.ee_target_port, self.ee_target_type_port = None, None
        self.define_cartesian_target_input_ports()

        # Define output ports
        self.DeclareVectorOutputPort(
            "applied_arm_torque",
            BasicVector(self.plant.num_actuators()),
            self.CalcArmTorques)

        # Store desired end-effector pose and corresponding joint
        # angles so we only run full IK when we need to
        self.last_ee_pose_target = None
        self.last_q_target = None

    def define_cartesian_target_input_ports(self):
        """
        Description
        -----------
        Define the input ports for the CartesianArmController system
        :return:
        """
        self.ee_target_port = self.DeclareVectorInputPort(
            "ee_target",
            BasicVector(7),
        )
        self.ee_target_type_port = self.DeclareAbstractInputPort(
            "ee_target_type",
            AbstractValue.Make(EndEffectorTarget.kPose),
        )

    def CalcArmTorques(self, context, output):
        """
        This method is called each timestep to determine output torques
        """
        q = self.arm_joint_position_port.Eval(context)
        qd = self.arm_joint_velocity_port.Eval(context)
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, qd)

        # Some dynamics computations
        tau_g = -self.plant.CalcGravityGeneralizedForces(self.context)

        # Indicate what type of command we're recieving
        target_type = self.ee_target_type_port.Eval(context)

        if target_type == EndEffectorTarget.kWrench:
            # Compute joint torques consistent with the desired wrench
            wrench_des = self.ee_target_port.Eval(context)

            # Compute end-effector jacobian
            J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                       JacobianWrtVariable.kV,
                                                       self.ee_frame,
                                                       np.zeros(3),
                                                       self.world_frame,
                                                       self.world_frame)

            tau = tau_g + J.T @ wrench_des

            print("Calculated wrench torques")

        elif target_type == EndEffectorTarget.kTwist:
            # Compue joint torques consistent with the desired twist
            twist_des = self.ee_target_port.Eval(context)

            # Use DoDifferentialInverseKinematics to determine desired qd
            params = DifferentialInverseKinematicsParameters(self.plant.num_positions(),
                                                             self.plant.num_velocities())
            params.set_time_step(0.005)
            params.set_joint_velocity_limits((self.qd_min, self.qd_max))
            params.set_joint_position_limits((self.q_min, self.q_max))

            result = DoDifferentialInverseKinematics(self.plant,
                                                     self.context,
                                                     twist_des,
                                                     self.ee_frame,
                                                     params)

            if result.status == DifferentialInverseKinematicsStatus.kSolutionFound:
                qd_nom = result.joint_velocities
            else:
                print("Differential inverse kinematics failed!")
                qd_nom = np.zeros(self.plant.num_velocities())

            # Select desired accelerations using a proportional controller
            Kp = 10 * np.eye(self.plant.num_velocities())
            qdd_nom = Kp @ (qd_nom - qd)

            # Compute joint torques consistent with these desired accelerations
            f_ext = MultibodyForces(self.plant)
            tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        elif target_type == EndEffectorTarget.kPose:
            # Compute joint torques which move the end effector to the desired pose
            rpy_xyz_des = self.ee_target_port.Eval(context)

            # Only do a full inverse kinematics solve if the target pose has changed
            if (rpy_xyz_des != self.last_ee_pose_target).any():

                X_WE_des = RigidTransform(
                    RollPitchYaw(rpy_xyz_des[:3]),
                    rpy_xyz_des[-3:],
                )

                # First compute joint angles consistent with the desired pose using Drake's IK.
                # This sets up a nonconvex optimization problem to find joint angles consistent
                # with the given constraints
                ik = InverseKinematics(self.plant, self.context)
                ik.AddPositionConstraint(self.ee_frame,
                                         [0, 0, 0],
                                         self.world_frame,
                                         X_WE_des.translation(),
                                         X_WE_des.translation())
                ik.AddOrientationConstraint(self.ee_frame,
                                            RotationMatrix(),
                                            self.world_frame,
                                            X_WE_des.rotation(),
                                            0.001)

                prog = ik.get_mutable_prog()
                q_var = ik.q()
                prog.AddQuadraticErrorCost(np.eye(len(q_var)), q, q_var)
                prog.SetInitialGuess(q_var, q)
                result = Solve(ik.prog())

                if not result.is_success():
                    print("Inverse Kinematics Failed!")
                    q_nom = np.zeros(self.plant.num_positions())
                else:
                    q_nom = result.GetSolution(q_var)

                    # Save the results of this solve for later
                    self.last_ee_pose_target = rpy_xyz_des
                    self.last_q_target = q_nom

            else:
                q_nom = self.last_q_target

            qd_nom = np.zeros(self.plant.num_velocities())

            # Use PD controller to map desired q, qd to desired qdd
            Kp = 1 * np.eye(self.plant.num_positions())
            Kd = 2 * np.sqrt(Kp)  # critical damping
            qdd_nom = Kp @ (q_nom - q) + Kd @ (qd_nom - qd)

            # Compute joint torques consistent with these desired qdd
            f_ext = MultibodyForces(self.plant)
            tau = tau_g + self.plant.CalcInverseDynamics(self.context, qdd_nom, f_ext)

        else:
            raise RuntimeError("Invalid target type %s" % target_type)
            print("Invalid target type")

        output.SetFromVector(tau)

        print("Calculated arm torques")
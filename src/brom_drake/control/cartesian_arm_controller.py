"""
cartesian_controller.py
Description:

    This file defines the CartesianController class. This class is used to control different manipulators and generates
    torque based control commands for the targeted arm.
"""
# External Imports
from pydrake.common.value import AbstractValue
from pydrake.math import RollPitchYaw
from pydrake.multibody.inverse_kinematics import (
    DifferentialInverseKinematicsParameters,
    DoDifferentialInverseKinematics
)
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import MultibodyForces
from pydrake.systems.framework import LeafSystem, BasicVector

import numpy as np

# Internal Imports
from .end_effector_target import EndEffectorTarget
from .gripper_target import GripperTarget


class CartesianArmController(LeafSystem):
    """
    A controller which imitates the cartesian control mode of the Kinova gen3 arm.

                         -------------------------
                         |                       |
                         |                       |
    ee_target ---------> |  CartesianArmController  | ----> applied_arm_torque
    ee_target_type ----> |                       |
                         |                       |
                         |                       | ----> measured_ee_pose
    arm_position ------> |                       | ----> measured_ee_twist
    arm_velocity ------> |                       |
                         |                       |
                         |                       |
                         -------------------------

    The type of target is determined by ee_target_type, and can be
        EndEffectorTarget.kPose,
        EndEffectorTarget.kTwist,
        EndEffectorTarget.kWrench.

    """

    def __init__(self, plant: MultibodyPlant, arm_model):
        LeafSystem.__init__(self)

        self.plant = plant
        self.arm = arm_model
        self.context = self.plant.CreateDefaultContext()

        # Define input ports
        self.ee_target_port = self.DeclareVectorInputPort(
            "ee_target",
            BasicVector(6))
        self.ee_target_type_port = self.DeclareAbstractInputPort(
            "ee_target_type",
            AbstractValue.Make(EndEffectorTarget.kPose))

        self.arm_position_port = self.DeclareVectorInputPort(
            "arm_position",
            BasicVector(self.plant.num_positions(self.arm)))
        self.arm_velocity_port = self.DeclareVectorInputPort(
            "arm_velocity",
            BasicVector(self.plant.num_velocities(self.arm)))

        # Define output ports
        self.DeclareVectorOutputPort(
            "applied_arm_torque",
            BasicVector(self.plant.num_actuators()),
            self.CalcArmTorques)

        self.DeclareVectorOutputPort(
            "measured_ee_pose",
            BasicVector(6),
            self.CalcEndEffectorPose,
            {self.time_ticket()}  # indicate that this doesn't depend on any inputs,
        )  # but should still be updated each timestep
        self.DeclareVectorOutputPort(
            "measured_ee_twist",
            BasicVector(6),
            self.CalcEndEffectorTwist,
            {self.time_ticket()})

        # Define some relevant frames
        self.world_frame = self.plant.world_frame()
        self.ee_frame = self.plant.GetFrameByName("end_effector")

        # Set joint limits (set self.{q,qd}_{min,max})
        self.GetJointLimits()

        # Store desired end-effector pose and corresponding joint
        # angles so we only run full IK when we need to
        self.last_ee_pose_target = None
        self.last_q_target = None

    def GetJointLimits(self):
        """
        Iterate through self.plant to establish joint angle
        and velocity limits.

        Sets:

            self.q_min
            self.q_max
            self.qd_min
            self.qd_max

        """
        q_min = []
        q_max = []
        qd_min = []
        qd_max = []

        joint_indices = self.plant.GetJointIndices(self.arm)

        for idx in joint_indices:
            joint = self.plant.get_joint(idx)

            if joint.type_name() == "revolute":  # ignore the joint welded to the world
                q_min.append(joint.position_lower_limit())
                q_max.append(joint.position_upper_limit())
                qd_min.append(joint.velocity_lower_limit())  # note that higher limits
                qd_max.append(joint.velocity_upper_limit())  # are availible in cartesian mode

        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)
        self.qd_min = np.array(qd_min)
        self.qd_max = np.array(qd_max)

    def CalcEndEffectorPose(self, context, output):
        """
        This method is called each timestep to determine the end-effector pose
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, qd)

        # Compute the rigid transform between the world and end-effector frames
        X_ee = self.plant.CalcRelativeTransform(self.context,
                                                self.world_frame,
                                                self.ee_frame)

        ee_pose = np.hstack([RollPitchYaw(X_ee.rotation()).vector(), X_ee.translation()])

        output.SetFromVector(ee_pose)

    def CalcEndEffectorTwist(self, context, output):
        """
        This method is called each timestep to determine the end-effector twist
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, qd)

        # Compute end-effector Jacobian
        J = self.plant.CalcJacobianSpatialVelocity(self.context,
                                                   JacobianWrtVariable.kV,
                                                   self.ee_frame,
                                                   np.zeros(3),
                                                   self.world_frame,
                                                   self.world_frame)

        ee_twist = J @ qd
        output.SetFromVector(ee_twist)

    def CalcArmTorques(self, context, output):
        """
        This method is called each timestep to determine output torques
        """
        q = self.arm_position_port.Eval(context)
        qd = self.arm_velocity_port.Eval(context)
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

                X_WE_des = RigidTransform(RollPitchYaw(rpy_xyz_des[:3]),
                                          rpy_xyz_des[-3:])

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

        output.SetFromVector(tau)
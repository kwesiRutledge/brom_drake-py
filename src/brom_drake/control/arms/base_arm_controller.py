import numpy as np
from pydrake.math import RollPitchYaw
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import JacobianWrtVariable
from pydrake.systems.framework import LeafSystem, BasicVector


class BaseArmController(LeafSystem):
    """
    A controller which provides the basic functionality for controlling
    a robotic arm.
                                ------------------------
                                |                      |
                                |                      |
    ee_target ----------------> |  BaseArmController   | ----> applied_arm_torque
    ee_target_type -----------> |                      |
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
        arm_model,
        end_effector_frame_name: str = "end_effector_frame",
    ):
        LeafSystem.__init__(self)

        self.plant = plant
        self.arm = arm_model
        self.context = self.plant.CreateDefaultContext()

        # Define some relevant frames
        self.world_frame = self.plant.world_frame()
        self.ee_frame = self.plant.GetFrameByName(end_effector_frame_name)

        # Define the base input and output ports
        self.arm_joint_position_port, self.arm_joint_velocity_port = None, None
        self.define_input_ports_for_arm_state()

        self.define_output_measurement_ports()

        # Set joint limits (set self.{q,qd}_{min,max})
        self.q_min, self.q_max = None, None
        self.qd_min, self.qd_max = None, None
        self.GetJointLimits()

    def define_input_ports_for_arm_state(self):
        """
        Description
        -----------
        Defines the input ports for the arm state. This is useful for
        connecting the arm state to the controller.
        :return:
        """
        self.arm_joint_position_port = self.DeclareVectorInputPort(
            "arm_joint_position",
            BasicVector(self.plant.num_positions(self.arm)),
        )
        self.arm_joint_velocity_port = self.DeclareVectorInputPort(
            "arm_joint_velocity",
            BasicVector(self.plant.num_velocities(self.arm)),
        )

    def define_output_measurement_ports(self):
        """
        Description
        -----------
        Defines the output ports for the controller that have to deal with
        the sensed state of the input arm.
        :return:
        """
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

    def CalcEndEffectorPose(self, context, output):
        """
        This method is called each timestep to determine the end-effector pose
        """
        q = self.arm_joint_position_port.Eval(context)
        qd = self.arm_joint_velocity_port.Eval(context)
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, qd)

        # Compute the rigid transform between the world and end-effector frames
        X_ee = self.plant.CalcRelativeTransform(
            self.context,
            self.world_frame,
            self.ee_frame,
        )

        ee_pose = np.hstack([
            RollPitchYaw(X_ee.rotation()).vector(),
            X_ee.translation()
        ])

        output.SetFromVector(ee_pose)

    def CalcEndEffectorTwist(self, context, output):
        """
        This method is called each timestep to determine the end-effector twist
        """
        q = self.arm_joint_position_port.Eval(context)
        qd = self.arm_joint_velocity_port.Eval(context)
        self.plant.SetPositions(self.context, q)
        self.plant.SetVelocities(self.context, qd)

        # Compute end-effector Jacobian
        J = self.plant.CalcJacobianSpatialVelocity(
            self.context,
            JacobianWrtVariable.kV,
            self.ee_frame,
            np.zeros(3),
            self.world_frame,
            self.world_frame,
        )

        ee_twist = J @ qd
        output.SetFromVector(ee_twist)

        # print("Calculated end effector twist")

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

        # print("Got joint limits")
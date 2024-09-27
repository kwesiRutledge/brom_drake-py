from enum import IntEnum
from importlib import resources as impresources
import numpy as np
from pydrake.geometry import SceneGraph, Meshcat, MeshcatVisualizer
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import Demultiplexer

from . import EndEffectorWrenchCalculator
# Local imports
from .gripper_type import GripperType
from ..control import GripperController
from brom_drake.control.arms.cartesian_arm_controller import CartesianArmController
from ..control.arms import (
    JointArmController, CartesianArmController,
    JointTarget, ArmControlMode,
)
from ..urdf import DrakeReadyURDFConverter

from brom_drake import robots

class UR10eStation(Diagram):
    """
    A template system diagram for controlling a UR10e robot in a simulated environment.
    # TODO(kwesi): Draw the diagram for the two different control modes.
    """

    def __init__(
        self,
        time_step: float = 0.002,
        gripper_type: GripperType = GripperType.NoGripper,
        control_mode: ArmControlMode = ArmControlMode.kEndEffector,
    ):
        """
        Description:

            This class defines the
        """
        # Input Processing

        # Initialize the Diagram
        Diagram.__init__(self)
        self.set_name("UR10e_Station")

        # Create a diagram builder
        self.builder = DiagramBuilder()

        # Create scene_graph and plant
        self.scene_graph = self.builder.AddSystem(SceneGraph())
        self.scene_graph.set_name("UR10e_Station_SceneGraph")

        self.plant = self.builder.AddSystem(
            MultibodyPlant(time_step=time_step)
        )
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name("UR10e_Station_Plant")

        # Add a shadow plant that only has access to the robot arm + gripper mass,
        # and is used for the controller. (Doesn't know about anything else in the scene)
        self.controller_plant = MultibodyPlant(time_step=time_step)
        self.controller_plant.set_name("UR10e_Station_ControllerPlant")

        # Body ID's and Poses for Anything else in the scene
        self.object_ids = []
        self.object_poses = []

        # Whether we have a camera in the simulation
        # self.has_camera = False

        self.arm_control_mode = control_mode
        self.AddArm()

        # Which sort of gripper we're using (if any)
        self.gripper_type = gripper_type
        if gripper_type == GripperType.Robotiq_2f_85:
            self.Add2f85Gripper()


    def AddArm(self):
        """
        Add the UR10e arm to the system.
        """
        # Setup
        arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        arm_urdf = DrakeReadyURDFConverter(arm_urdf_path).convert_urdf()
        arm_urdf = str(arm_urdf)

        self.end_effector_frame_name = "tool0"

        # The hardware system has lots of damping so this is more realistic,
        # but requires a simulation with small timesteps.

        self.arm = Parser(plant=self.plant).AddModels(arm_urdf)[0]
        self.controller_arm = Parser(plant=self.controller_plant).AddModels(arm_urdf)[0]

        # Fix the base of the arm to the world
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link", self.arm),
        )

        self.controller_plant.WeldFrames(
            self.controller_plant.world_frame(),
            self.controller_plant.GetFrameByName("base_link", self.controller_arm),
        )

        # Create a new frame with the actual end-effector position.
        self.X_ee = RigidTransform()
        self.X_ee.set_translation([0, 0, 0.13])
        self.plant.AddFrame(FixedOffsetFrame(
            "end_effector",
            self.plant.GetFrameByName("tool0"),
            self.X_ee, self.arm))
        self.controller_plant.AddFrame(FixedOffsetFrame(
            "end_effector",
            self.controller_plant.GetFrameByName("tool0"),
            self.X_ee, self.controller_arm))

    def Add2f85Gripper(self):
        """
        Add the Robotiq 2F-85 gripper to the system. The arm must be added first.
        """
        # Setup

        # Add a gripper with actuation to the full simulated plant
        gripper_urdf_path = str(
            impresources.files(robots) / "models/2f_85_gripper/urdf/robotiq_2f_85.urdf"
        )
        self.gripper = Parser(plant=self.plant).AddModels(gripper_urdf_path)[0]

        X_grip = RigidTransform()
        X_grip.set_rotation(
            RotationMatrix(RollPitchYaw([0.0, 0.0, np.pi / 2]))
        )
        self.plant.WeldFrames(
            self.plant.GetFrameByName("tool0", self.arm),
            self.plant.GetFrameByName("robotiq_arg2f_base_link", self.gripper),
            X_grip,
        )

        # Add a gripper without actuation to the controller plant
        gripper_static_urdf = str(
            impresources.files(robots) / "models/2f_85_gripper/urdf/robotiq_2f_85_static.urdf"
        )
        static_gripper = Parser(plant=self.controller_plant).AddModels(
            gripper_static_urdf
        )[0]

        self.controller_plant.WeldFrames(
            self.controller_plant.GetFrameByName("tool0", self.controller_arm),
            self.controller_plant.GetFrameByName("robotiq_arg2f_base_link", static_gripper),
            X_grip)

    def ConnectToMeshcatVisualizer(self, port=None):
        self.meshcat = Meshcat(port)
        m = MeshcatVisualizer(self.meshcat)
        m.AddToBuilder(self.builder, self.scene_graph, self.meshcat)

        print("Open %s in a browser to view the meshcat visualizer." % self.meshcat.web_url())

    def Finalize(self):
        """
        Description
        ----------
        This finalizes the diagram by:
        - Finalizing all plants in the diagram
        :return: Nothing
        """
        # Setup

        # Finalize all plants
        self.plant.Finalize()
        self.controller_plant.Finalize()

        # Set up the scene graph
        self.builder.Connect(
            self.scene_graph.get_query_output_port(),
            self.plant.get_geometry_query_input_port()
        )

        self.builder.Connect(
            self.plant.get_geometry_pose_output_port(),
            self.scene_graph.get_source_pose_port(self.plant.get_source_id())
        )

        # Create Arm and Gripper Controllers
        self.CreateArmControllerAndConnect(self.arm_control_mode)

        if self.gripper_type != GripperType.NoGripper:
            self.CreateGripperControllerAndConnect()

        # Build the diagram
        self.builder.BuildInto(self)

    def CreateArmControllerAndConnect(
        self,
        arm_control_mode: ArmControlMode,
    ):
        """
        Description
        -----------
        This function creates a Cartesian arm controller and connects it to the rest of the system.
        :return:
        """

        controller = None

        if arm_control_mode == ArmControlMode.kJoint:
            controller = self.CreateJointArmControllerAndExportInputs()

        elif arm_control_mode == ArmControlMode.kEndEffector:
            controller = self.CreateCartesianArmControllerAndExportInputs()

        else:
            raise ValueError("Invalid Arm Control Mode: {}".format(arm_control_mode))

        # Output measured arm position and velocity
        demux = self.builder.AddSystem(
            Demultiplexer(self.plant.num_multibody_states(self.arm), self.plant.num_positions(self.arm)),
        )
        demux.set_name("UR10e_Station_Demux")

        self.builder.Connect(
            self.plant.get_state_output_port(self.arm),
            demux.get_input_port(0),
        )
        self.builder.ExportOutput(
            demux.get_output_port(0),
            "measured_arm_position",
        )
        self.builder.ExportOutput(
            demux.get_output_port(1),
            "measured_arm_velocity",
        )

        # Measured Arm Position and Velocity are sent to the controller
        self.builder.Connect(
            demux.get_output_port(0),
            controller.arm_joint_position_port,
        )
        self.builder.Connect(
            demux.get_output_port(1),
            controller.arm_joint_velocity_port,
        )

        # Torques from controller go to the "true" plant
        self.builder.Connect(
            controller.GetOutputPort("applied_arm_torque"),
            self.plant.get_actuation_input_port(self.arm),
        )

        # Controller outputs measured arm torques, end-effector pose, end-effector twist
        controller_output_port_names = ["applied_arm_torque", "measured_ee_pose", "measured_ee_twist"]
        for output_port_name in controller_output_port_names:
            self.builder.ExportOutput(
                controller.GetOutputPort(output_port_name),
                output_port_name,
            )

        # Compute and output end-effector wrenches based on measured joint torques
        wrench_calculator = self.builder.AddSystem(
            EndEffectorWrenchCalculator(
                self.controller_plant,
                self.controller_plant.GetFrameByName("end_effector"),
            ),
        )
        wrench_calculator.set_name("wrench_calculator")

        self.builder.Connect(
            demux.get_output_port(0),
            wrench_calculator.GetInputPort("joint_positions"))
        self.builder.Connect(
            demux.get_output_port(1),
            wrench_calculator.GetInputPort("joint_velocities"))
        self.builder.Connect(
            controller.GetOutputPort("applied_arm_torque"),
            wrench_calculator.GetInputPort("joint_torques"))

        self.builder.ExportOutput(
            wrench_calculator.get_output_port(),
            "measured_ee_wrench")

    def CreateCartesianArmControllerAndExportInputs(self) -> CartesianArmController:
        """
        Description
        -----------
        Defines the input ports for the CartesianArmController system

        :return:
        """
        # Create controller that uses the shadow plant to control the robot
        cartesian_controller = self.builder.AddSystem(
            CartesianArmController(
                self.controller_plant,
                self.controller_arm,
                end_effector_frame_name=self.end_effector_frame_name,
            ),
        )
        cartesian_controller.set_name("UR10e_Station_CartesianController")

        # End effector target and target type go to the controller
        self.builder.ExportInput(
            cartesian_controller.ee_target_port,
            "ee_target",
        )
        self.builder.ExportInput(
            cartesian_controller.ee_target_type_port,
            "ee_target_type",
        )

        return cartesian_controller


    def CreateJointArmControllerAndExportInputs(self)-> JointArmController:
        """
        Description
        -----------
        :return:
        """
        # Setup

        # Create Controller
        joint_controller = self.builder.AddSystem(
            JointArmController(
                self.controller_plant,
                self.controller_arm,
                end_effector_frame_name=self.end_effector_frame_name,
            ),
        )
        joint_controller.set_name("UR10e_Station_JointController")

        # End effector target and target type go to the controller
        self.builder.ExportInput(
            joint_controller.joint_target_port,
            "joint_target",
        )
        self.builder.ExportInput(
            joint_controller.joint_target_type_port,
            "joint_target_type",
        )

        return joint_controller

    def CreateGripperControllerAndConnect(self):
        """
        Description
        -----------
        This function creates a gripper controller and connects it to the rest of the system.
        Requires that the builder has been created and IS NOT FINALIZED.
        :return: (Nothing)
        """
        # Setup

        # Create gripper controller
        gripper_controller = self.builder.AddSystem(
            GripperController(self.gripper_type)
        )
        gripper_controller.set_name("gripper_controller")

        # Connect gripper controller to the diagram
        self.builder.ExportInput(
            gripper_controller.GetInputPort("gripper_target"),
            "gripper_target")
        self.builder.ExportInput(
            gripper_controller.GetInputPort("gripper_target_type"),
            "gripper_target_type")

        self.builder.Connect(
            self.plant.get_state_output_port(self.gripper),
            gripper_controller.GetInputPort("gripper_state"))
        self.builder.Connect(
            gripper_controller.GetOutputPort("applied_gripper_torque"),
            self.plant.get_actuation_input_port(self.gripper))

        # Send gripper position and velocity as an output
        self.builder.ExportOutput(
            gripper_controller.GetOutputPort("measured_gripper_position"),
            "measured_gripper_position")
        self.builder.ExportOutput(
            gripper_controller.GetOutputPort("measured_gripper_velocity"),
            "measured_gripper_velocity")
from importlib import resources as impresources
from pathlib import Path

import numpy as np
from pydrake.geometry import SceneGraph, Meshcat, MeshcatVisualizer, MeshcatVisualizerParams
from pydrake.geometry import Role as DrakeRole
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import Demultiplexer

# Local imports
from brom_drake.robots.gripper_type import GripperType
from brom_drake.control import IdealJointPositionController
from brom_drake.control.grippers.gripper_controller import GripperController
from brom_drake.file_manipulation.urdf import DrakeReadyURDFConverter, MeshReplacementStrategy

from brom_drake import robots

class UR10eStation(Diagram):
    """
    A template system diagram for controlling a UR10e robot in a simulated environment.
    """

    def __init__(
        self,
        time_step: float = 0.002,
        gripper_type: GripperType = GripperType.NoGripper,
        force_conversion_of_original_urdf: bool = False,
        meshcat_port_number: int = None,
    ):
        """
        Description:

            This class defines the KINEMATIC UR10e Manipulation Station.
            This station is based off of the Drake Manipulation Station showcased
            in Russ Tedrake's manipulation course.
        """
        # Input Processing
        self.force_conversion_of_original_urdf = force_conversion_of_original_urdf
        self.meshcat_port_number = meshcat_port_number

        # Initialize the Diagram
        Diagram.__init__(self)
        self.set_name("UR10e_Station")

        # Create a diagram builder
        self.builder = DiagramBuilder()

        # Create scene_graph and plant
        self.plant, self.scene_graph = None, None
        self.time_step = time_step
        self.create_plant_and_scene_graph()

        # Body ID's and Poses for Anything else in the scene
        self.object_ids = []
        self.object_poses = []

        # Whether we have a camera in the simulation
        # self.has_camera = False
        self.arm = None # Assign the Arm's Model Index later
        self.AddArm()

        # Which sort of gripper we're using (if any)
        self.gripper_type = gripper_type
        if gripper_type == GripperType.Robotiq_2f_85:
            self.Add2f85Gripper()

        # Visualization
        self.meshcat = None


    def AddArm(self):
        """
        Add the UR10e arm to the system.
        """
        # Setup
        original_arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        expected_arm_urdf_path = Path("./brom/models/ur10e/ur10e.drake.urdf")

        # Convert the arm urdf if necessary
        arm_urdf = None
        if (not expected_arm_urdf_path.exists()) or self.force_conversion_of_original_urdf:
            # If drake-compatible URDF does not exist, then we need to convert
            # the original URDF to a drake-compatible URDF.
            arm_urdf = DrakeReadyURDFConverter(
                original_arm_urdf_path,
                overwrite_old_logs=True,
                collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithMinimalEnclosingCylinder,
            ).convert_urdf()
            arm_urdf = str(arm_urdf)
        else:
            # Otherwise, let's just read the drake-compatible URDF
            arm_urdf = str(expected_arm_urdf_path)

        self.end_effector_frame_name = "tool0"

        # The hardware system has lots of damping so this is more realistic,
        # but requires a simulation with small timesteps.

        self.arm = Parser(plant=self.plant).AddModels(arm_urdf)[0]

        # Fix the base of the arm to the world
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link", self.arm),
        )

        # Create a new frame with the actual end-effector position.
        self.X_ee = RigidTransform()
        self.X_ee.set_translation([0, 0, 0.13])
        self.plant.AddFrame(FixedOffsetFrame(
            "end_effector",
            self.plant.GetFrameByName("tool0"),
            self.X_ee, self.arm))

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
            X_grip,
        )

    def ConnectToMeshcatVisualizer(self, port=None):
        self.meshcat = Meshcat(port)
        m = MeshcatVisualizer(self.meshcat)
        m.AddToBuilder(
            self.builder, self.scene_graph, self.meshcat,
            # params=MeshcatVisualizerParams(
            #         role=DrakeRole.kProximity,
            #     ),
        )

        print("Open %s in a browser to view the meshcat visualizer." % self.meshcat.web_url())

    def create_plant_and_scene_graph(self):
        """
        Description
        -----------
        This function creates the plant for the UR10e station, if needed.
        :return:
        """
        # Setup

        # Create SceneGraph
        self.scene_graph = self.builder.AddSystem(
            SceneGraph()
        )
        self.scene_graph.set_name(f"{self.get_name()}_SceneGraph")

        # Create plant
        self.plant = self.builder.AddSystem(
            MultibodyPlant(time_step=self.time_step)
        )
        self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        self.plant.set_name(f"{self.get_name()}_Plant")

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

        # Set up the scene graph
        self.builder.Connect(
            self.scene_graph.get_query_output_port(),
            self.plant.get_geometry_query_input_port()
        )

        self.builder.Connect(
            self.plant.get_geometry_pose_output_port(),
            self.scene_graph.get_source_pose_port(self.plant.get_source_id())
        )

        self.builder.ExportOutput(
            self.scene_graph.get_query_output_port(),
            "query_object",
        )

        # Connect Meshcat, if desired
        if self.use_meshcat():
            self.ConnectToMeshcatVisualizer(port=self.meshcat_port_number)

        # Create Arm and Gripper Controllers
        self.CreateArmPorts()

        if self.gripper_type != GripperType.NoGripper:
            self.CreateGripperControllerAndConnect()

        # Build the diagram
        self.builder.BuildInto(self)

    def CreateArmPorts(
        self,
    ):
        """
        Description
        -----------
        This function creates a Cartesian arm controller and connects it to the rest of the system.
        :return:
        """
        # Setup

        # Create the input ports for the joint positions
        self.arm_controller = self.CreateJointArmControllerAndExportIO()

        # Output measured arm position and velocity
        demux = self.builder.AddSystem(
            Demultiplexer(self.plant.num_multibody_states(self.arm), self.plant.num_positions(self.arm)),
        )
        demux.set_name(f"{self.get_name()}_Demux")

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

    def CreateJointArmControllerAndExportIO(
        self,
    )-> IdealJointPositionController:
        """
        Description
        -----------
        Creates an ideal joint position controller and exports the necessary inputs + outputs.
        :return:
        """
        # Setup
        arm = self.arm
        plant = self.plant

        # Create the joint controller
        joint_controller = self.builder.AddSystem(
            IdealJointPositionController(arm, plant),
        )

        # Export the input and output ports
        self.builder.ExportInput(
            joint_controller.desired_joint_positions_port,
            "desired_joint_positions",
        )

        self.builder.ExportOutput(
            joint_controller.GetOutputPort("measured_joint_positions"),
            "measured_joint_positions",
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

    def use_meshcat(self):
        return self.meshcat_port_number is not None
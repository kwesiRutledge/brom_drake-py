from importlib import resources as impresources
from pathlib import Path

import numpy as np
from pydrake.all import (
    Frame,
    ModelInstanceIndex,
)
from pydrake.geometry import SceneGraph, Meshcat, MeshcatVisualizer, MeshcatVisualizerParams
from pydrake.geometry import Role as DrakeRole
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import Demultiplexer
from typing import Tuple

# Local imports
from brom_drake.robots.gripper_type import GripperType
from brom_drake.control import IdealJointPositionController
from brom_drake.control.grippers.gripper_controller import GripperController
from brom_drake.file_manipulation.urdf import (
    DrakeReadyURDFConverter, 
    DrakeReadyURDFConverterConfig,
    MeshReplacementStrategy,
    MeshReplacementStrategies,
)

from brom_drake import robots

class UR10eStation(Diagram):
    """
    Description
    -----------
    A template system diagram for controlling a UR10e robot in a simulated environment.

    Diagram
    -------
    
    """
    def __init__(
        self,
        time_step: float = 0.002,
        gripper_type: GripperType = GripperType.NoGripper,
        force_conversion_of_original_urdf: bool = False,
        meshcat_port_number: int = None,
        end_effector_frame_name: str = "tool0",
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
        self.end_effector_frame_name = end_effector_frame_name

        # Initialize the Diagram
        Diagram.__init__(self)
        self.set_name("UR10e_Station")

        # Create a diagram builder
        self.builder = DiagramBuilder()

        # Create scene_graph and plant
        self.plant, self.scene_graph = None, None
        self.controller_plant = None
        self.time_step = time_step
        self.create_plants_and_scene_graph()

        # Body ID's and Poses for Anything else in the scene
        self.object_ids = []
        self.object_poses = []

        # Whether we have a camera in the simulation
        # self.has_camera = False
        self.arm = None # Assign the Arm's Model Index later
        self.AddArm()

        # Which sort of gripper we're using (if any)
        self.gripper_type = gripper_type
        self.gripper = None
        self.gripper_controller = None
        if gripper_type == GripperType.Robotiq_2f_85:
            self.Add2f85Gripper()
            self.add_gripper_controller()

        # Visualization
        self.meshcat = None

    def add_arm_to_plant_with_ee_frame(
        self,
        arm_urdf_path: str,
        plant: MultibodyPlant,
        new_frame_name: str = "end_effector",
        X_ee: RigidTransform = RigidTransform(),
    ) -> Tuple[ModelInstanceIndex, Frame]:
        """
        Description
        -----------
        This function adds the "arm" object at the arm_urdf_path location to the plant.

        Arguments
        ---------
        arm_urdf_path : str
            The path to the URDF file for the arm.
        plant : MultibodyPlant
            The plant to which the arm will be added.
        :param arm_urdf_path:
        :param plant:
        :return:
        """
        # Setup

        # Add the arm to the provided plant
        arm = Parser(plant=plant).AddModels(arm_urdf_path)[0]

        # Fix the base of the arm to the world
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("base_link", arm),
        )

        # Add Frame for the end effector
        new_frame = plant.AddFrame(
            FixedOffsetFrame(
                new_frame_name,
                plant.GetFrameByName(self.end_effector_frame_name, arm),
                X_ee,
                arm,
            )
        )

        return arm, new_frame

    def Add2f85Gripper(self):
        """
        Add the Robotiq 2F-85 gripper to the system. The arm must be added first.
        """
        # Setup

        # Add a gripper with actuation to the full simulated plant
        gripper_urdf_path = str(
            impresources.files(robots) / "models/robotiq/2f_85_gripper-no-mimic/urdf/robotiq_2f_85.urdf"
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
            impresources.files(robots) / "models/robotiq/2f_85_gripper-no-mimic/urdf/robotiq_2f_85_static.urdf"
        )
        static_gripper = Parser(plant=self.controller_plant).AddModels(
            gripper_static_urdf
        )[0]

        self.controller_plant.WeldFrames(
            self.controller_plant.GetFrameByName("tool0", self.controller_arm),
            self.controller_plant.GetFrameByName("robotiq_arg2f_base_link", static_gripper),
            X_grip,
        )

    def add_gripper_controller(self):
        """
        Description
        -----------
        This funciton adds a controller for the gripper to the station.
        It is optional and may not be necessary for all stations.
        """
        # Setup

        # Create gripper controller
        self.gripper_controller = self.builder.AddSystem(
            GripperController(self.gripper_type)
        )
        self.gripper_controller.set_name(f"{self.get_name()}_gripper_controller")

    def AddArm(self):
        """
        Add the UR10e arm to the system.
        """
        # Setup
        original_arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        expected_arm_urdf_path = Path("./brom/models/ur10e/ur10e.drake.urdf")

        # Define Transform from tool0 to end_effector frame
        self.X_ee = RigidTransform()
        self.X_ee.set_translation([0, 0, 0.13])

        # Convert the arm urdf if necessary
        arm_urdf = None
        if (not expected_arm_urdf_path.exists()) or self.force_conversion_of_original_urdf:
            # If drake-compatible URDF does not exist, then we need to convert
            # the original URDF to a drake-compatible URDF.
            config = DrakeReadyURDFConverterConfig(
                overwrite_old_logs=True,
                mesh_replacement_strategies=MeshReplacementStrategies(
                    collision_meshes=MeshReplacementStrategy.kWithObj,
                )
            )
            arm_urdf = DrakeReadyURDFConverter(
                original_arm_urdf_path,
                config=config,
            ).convert_urdf()
            arm_urdf = str(arm_urdf)
        else:
            # Otherwise, let's just read the drake-compatible URDF
            arm_urdf = str(expected_arm_urdf_path)

        # Add the arm to the plant using our convenience function
        self.arm, self.arm_ee_frame = self.add_arm_to_plant_with_ee_frame(
            arm_urdf, self.plant,
            X_ee=self.X_ee,
        )

        # Add the arm to the controller plant using our convenience function
        self.controller_arm, self.controller_arm_ee_frame = self.add_arm_to_plant_with_ee_frame(
            arm_urdf, self.controller_plant,
            X_ee=self.X_ee,
        )

    def connect_gripper_controller(self):
        """
        Description
        -----------
        Connects the gripper controller to:
        - The Gripper Actuators (done via the plant of the station)
        
        and exports some of the gripper controller's inputs and outputs
        (so that they become the inputs and outputs of the station).
        """
        # Setup
        gripper_controller = self.gripper_controller

        # Export the inputs of the gripper controller to the diagram
        self.builder.ExportInput(
            gripper_controller.GetInputPort("gripper_target"),
            "gripper_target",
        )
        self.builder.ExportInput(
            gripper_controller.GetInputPort("gripper_target_type"),
            "gripper_target_type")

        # Connect gripper controller to the plant
        self.builder.Connect(
            self.plant.get_state_output_port(self.gripper),
            gripper_controller.GetInputPort("gripper_state"),
        )
        self.builder.Connect(
            gripper_controller.GetOutputPort("applied_gripper_torque"),
            self.plant.get_actuation_input_port(self.gripper),
        )

        # Send gripper position and velocity as an output
        self.builder.ExportOutput(
            gripper_controller.GetOutputPort("measured_gripper_position"),
            "measured_gripper_position")
        self.builder.ExportOutput(
            gripper_controller.GetOutputPort("measured_gripper_velocity"),
            "measured_gripper_velocity",
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

    def create_plants_and_scene_graph(self):
        """
        Description
        -----------
        This function creates the plant for the UR10e station, if needed.
        :return:
        """
        # Setup

        # Create SceneGraph
        # self.scene_graph = self.builder.AddSystem(
        #     SceneGraph()
        # )
        # self.scene_graph.set_name(f"{self.get_name()}_SceneGraph")

        # # Create plant (will contain ALL elements of scene)
        # self.plant = self.builder.AddSystem(
        #     MultibodyPlant(time_step=self.time_step)
        # )
        # self.plant.RegisterAsSourceForSceneGraph(self.scene_graph)
        # self.plant.set_name(f"{self.get_name()}_Plant")

        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder=self.builder,
            time_step=self.time_step,
        )
        self.plant.set_name(f"{self.get_name()}_Plant")
        self.scene_graph.set_name(f"{self.get_name()}_SceneGraph")

        # Create plant for controller
        self.controller_plant = MultibodyPlant(time_step=self.time_step)
        self.controller_plant.set_name(f"{self.get_name()}_ControllerPlant")

    def Finalize(self):
        """
        Description
        ----------
        This finalizes the diagram by:
        - Finalizing all plants in the diagram
        :return: Nothing
        """
        # Setup

        # Announce that we are finalizing the station
        # print("Finalizing station!")

        # Finalize all plants
        self.plant.Finalize()
        self.controller_plant.Finalize()

        # Set up the scene graph
        # self.builder.Connect(
        #     self.scene_graph.get_query_output_port(),
        #     self.plant.get_geometry_query_input_port()
        # )

        # self.builder.Connect(
        #     self.plant.get_geometry_pose_output_port(),
        #     self.scene_graph.get_source_pose_port(self.plant.get_source_id())
        # )

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
            self.connect_gripper_controller()

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

    def use_meshcat(self):
        return self.meshcat_port_number is not None
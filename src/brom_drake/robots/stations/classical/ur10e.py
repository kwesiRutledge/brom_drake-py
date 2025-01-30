from importlib import resources as impresources
import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ConstantVectorSource,
    Frame,
    InverseDynamicsController,
    ModelInstanceIndex,
    MultibodyPlant,
    Multiplexer,
    Parser,
    RigidTransform,
)
from pydrake.geometry import SceneGraph, Meshcat, MeshcatVisualizer
from pydrake.math import RollPitchYaw, RigidTransform, RotationMatrix
from pydrake.multibody.tree import FixedOffsetFrame
from pydrake.systems.framework import Diagram, DiagramBuilder
from pydrake.systems.primitives import Demultiplexer
from typing import Tuple

# Local imports
from brom_drake.robots.gripper_type import GripperType
from brom_drake.control.grippers import GripperController
from brom_drake.file_manipulation.urdf import DrakeReadyURDFConverter
from brom_drake.utils.leaf_systems import EndEffectorWrenchCalculator

from brom_drake import robots

class UR10eStation(Diagram):
    """
    A template system diagram for controlling a UR10e robot in a simulated environment.
    
    Diagram
    -------
                                        -----------------
                                        |               |
    gripper_target -------------------->|               |
    (Vector, np.array)                  |   UR10e       |
                                        |               |
    gripper_target_type --------------->|   Station     |
    (Abstract, GripperTarget Enum)      |               |
                                        |               |
    desired_joint_positions ----------->|               |
    (Vector, np.array)                  |               |
                                        |               |
                                        -----------------
    """

    def __init__(
        self,
        time_step: float = 0.002,
        meshcat_port_number: int = 7001,
        gripper_type: GripperType = GripperType.Robotiq_2f_85,
    ):
        """
        Description:

            This class defines the UR10e Station.
        """
        # Input Processing
        self.meshcat_port_number = meshcat_port_number
        self.time_step = time_step

        # Initialize the Diagram
        Diagram.__init__(self)
        self.set_name("UR10e_Station")

        # Create a diagram builder
        self.builder = DiagramBuilder()

        # Create the plant and scene graph
        self.plant, self.scene_graph = None, None
        self.controller_plant = None
        self.create_plants_and_scene_graph(time_step)

        # Body ID's and Poses for Anything else in the scene
        self.object_ids = []
        self.object_poses = []

        # Whether we have a camera in the simulation
        # self.has_camera = False
        self.AddArm()

        # Which sort of gripper we're using (if any)
        self.gripper_type = gripper_type
        self.gripper_controller = None
        if gripper_type == GripperType.Robotiq_2f_85:
            self.Add2f85Gripper()
            self.add_gripper_controller()


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
        
        Returns
        -------
        Tuple[ModelInstanceIndex, Frame]
            The model instance index of the arm and the frame of the end effector.
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
        arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        arm_urdf = DrakeReadyURDFConverter(arm_urdf_path).convert_urdf()
        arm_urdf = str(arm_urdf)

        self.end_effector_frame_name = "tool0"

        # Create transform for EE
        self.X_ee = RigidTransform()
        self.X_ee.set_translation([0, 0, 0.13])

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

        # # Add a gripper without actuation to the controller plant
        # gripper_static_urdf = str(
        #     impresources.files(robots) / "models/robotiq/2f_85_gripper-no-mimic/urdf/robotiq_2f_85_static.urdf"
        # )
        # static_gripper = Parser(plant=self.controller_plant).AddModels(
        #     gripper_static_urdf
        # )[0]

        # self.controller_plant.WeldFrames(
        #     self.controller_plant.GetFrameByName("tool0", self.controller_arm),
        #     self.controller_plant.GetFrameByName("robotiq_arg2f_base_link", static_gripper),
        #     X_grip)
    
    def ConnectToMeshcatVisualizer(self, port=None):
        self.meshcat = Meshcat(port)
        m = MeshcatVisualizer(self.meshcat)
        m.AddToBuilder(self.builder, self.scene_graph, self.meshcat)

        print("Open %s in a browser to view the meshcat visualizer." % self.meshcat.web_url())

    def CreateArmControllerAndConnect(self) -> InverseDynamicsController:
        """
        Description
        -----------
        This function creates a Cartesian arm controller and connects it to the rest of the system.
        :return:
        """
        # Setup
        kp = 1.0e3 * np.ones((self.plant.num_actuated_dofs(self.arm),))
        kd = 2.0 * np.sqrt(kp)

        # Create InverseDynamicsController
        self.controller = self.builder.AddSystem(
            InverseDynamicsController(
                self.controller_plant,
                kp=kp,
                ki=0.0*kp,
                kd=kd,
                has_reference_acceleration=False,
            )
        )

        # Torques from controller go to the "true" plant
        self.builder.Connect(
            self.controller.get_output_port_control(),
            self.plant.get_actuation_input_port(self.arm),
        )
        self.builder.Connect(
            self.controller.get_output_port_control(),
            self.controller_plant.get_actuation_input_port(self.controller_arm),
        )

        # Connect plant's state to controller
        self.builder.Connect(
            self.plant.get_state_output_port(self.arm),
            self.controller.get_input_port_estimated_state(),
        )

        return self.controller

    def CreateGripperControllerAndConnect(self):
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
            "gripper_target_type",
        )

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
        # self.builder.ExportOutput(
        #     gripper_controller.GetOutputPort("measured_gripper_position"),
        #     "measured_gripper_position",
        # )
        # self.builder.ExportOutput(
        #     gripper_controller.GetOutputPort("measured_gripper_velocity"),
        #     "measured_gripper_velocity",
        # )

    def create_plants_and_scene_graph(self, time_step: float = 0.001):
        """
        Description
        -----------
        This function creates the plant and scene graph for the diagram.

        Outputs
        -------
        None
        """
        # Setup

        # Algorithm
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(
            builder=self.builder,
            time_step=time_step,
        )
        self.plant.set_name(f"{self.get_name()}_Plant")
        self.scene_graph.set_name(f"{self.get_name()}_SceneGraph")

        # Create plant for controller
        self.controller_plant = self.builder.AddSystem(
            MultibodyPlant(time_step=time_step)
        )
        self.controller_plant.set_name(f"{self.get_name()}_ControllerPlant")


    def ExportArmPositionInputPort(self):
        """
        Description
        -----------
        Creates the input port for the arm position (i.e. the joint position)
        for the station.
        This requires creating a special connection to the arm controller
        via a multiplexer.
        """
        # Setup
        controller = self.controller
        controller_plant = controller.get_multibody_plant_for_control()

        # Create multiplexer to provide input to arm controller
        mux_for_controller = self.builder.AddSystem(
            Multiplexer([
                self.plant.num_positions(self.arm),
                self.plant.num_velocities(self.arm),
            ]),
        )

        # Create constant velocity source
        constant_velocity_source = self.builder.AddSystem(
            ConstantVectorSource(
                np.zeros((controller_plant.num_actuated_dofs(self.arm),)),
            ),
        )

        # Connect the multiplexer to the controller
        self.builder.Connect(
            constant_velocity_source.get_output_port(),
            mux_for_controller.get_input_port(1),
        )
        self.builder.Connect(
            mux_for_controller.get_output_port(),
            controller.get_input_port_desired_state(),
        )

        # Export the second input port of the multiplexer
        self.builder.ExportInput(
            mux_for_controller.get_input_port(0),
            "desired_joint_positions",
        )

    def ExportArmControllerPorts(self):
        """
        Description
        -----------
        This function creates the input and output ports for the diagram
        that are related to the arm controller.
        """
        # Setup
        controller = self.controller
        controller_plant = controller.get_multibody_plant_for_control()

        # Create commanded arm torque output
        self.builder.ExportOutput(
            self.controller.get_output_port_control(),
            "ur10e_arm_torque_commanded",
        )

        # Create desired position input
        self.ExportArmPositionInputPort()
        
        # Create Multiplexer to extract the arm position and velocity
        demux = self.builder.AddSystem(
            Demultiplexer(
                self.plant.num_multibody_states(self.arm),
                self.plant.num_positions(self.arm),
            ),
        )
        demux.set_name("UR10e_Station_Demux")

        self.builder.Connect(
            self.plant.get_state_output_port(self.arm),
            demux.get_input_port(0),
        )
        self.builder.ExportOutput(
            demux.get_output_port(0),
            "ur10e_arm_position_measured",
        )
        self.builder.ExportOutput(
            demux.get_output_port(1),
            "ur10e_arm_velocity_measured",
        )

        # Compute and output end-effector wrenches based on measured joint torques
        wrench_calculator = self.builder.AddSystem(
            EndEffectorWrenchCalculator(
                controller_plant,
                controller_plant.GetFrameByName("end_effector"),
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
            self.controller.get_output_port_control(),
            wrench_calculator.GetInputPort("joint_torques"))

        self.builder.ExportOutput(
            wrench_calculator.get_output_port(),
            "measured_ee_wrench",
        )

    def ExportGripperStatePort(self):
        """
        Description
        -----------
        This function exports the gripper's state
        as a port of the diagram.
        """
        # Setup

        # Export the state of the gripper
        self.builder.ExportOutput(
            self.plant.get_state_output_port(self.gripper),
            "gripper_state",
        )
    
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

        # # Set up the scene graph
        # self.builder.Connect(
        #     self.scene_graph.get_query_output_port(),
        #     self.plant.get_geometry_query_input_port()
        # )

        # self.builder.Connect(
        #     self.plant.get_geometry_pose_output_port(),
        #     self.scene_graph.get_source_pose_port(self.plant.get_source_id())
        # )

        # Connect Meshcat, if desired
        if self.use_meshcat():
            self.ConnectToMeshcatVisualizer(port=self.meshcat_port_number)

        # Create Arm and Gripper Controllers
        self.CreateArmControllerAndConnect()
        self.ExportArmControllerPorts()

        if self.gripper_type != GripperType.NoGripper:
            self.CreateGripperControllerAndConnect()
            self.ExportGripperStatePort()

        # Build the diagram
        self.builder.BuildInto(self)
    
    def use_meshcat(self) -> bool:
        """
        Description
        -----------
        This function returns whether or not to use meshcat for visualization.
        :return: (bool) Whether or not to use meshcat for visualization.
        """
        return self.meshcat_port_number is not None
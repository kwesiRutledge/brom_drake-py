from importlib import resources as impresources
from typing import List, Tuple, Union
from dataclasses import dataclass, field
import networkx as nx
import numpy as np
from pydrake.all import (
    AbstractValue,
    AffineSystem,
    Adder,
    ConstantValueSource,
    FixedOffsetFrame,
    Frame,
    GeometryProperties,
    InputPortIndex,
    IllustrationProperties,
    JointActuator,
    ModelInstanceIndex,
    PassThrough,
    PidController,
    PiecewisePolynomial,
    PiecewisePose,
    PrismaticJoint,
    RigidBodyFrame,
    RigidTransform,
    SceneGraphInspector,
    TrajectorySource,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, VectorLogSink

# Internal Imports
from brom_drake.control.grippers import GripperController, GripperTarget
from brom_drake.file_manipulation.urdf.drakeify import drakeify_my_urdf
from brom_drake.file_manipulation.urdf.shapes.box import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_writer import SimpleShapeURDFDefinition
from brom_drake.motion_planning.systems.open_loop_dispensers.pose_trajectory_source import PoseTrajectorySource
from brom_drake.robots import find_base_link_name_in, GripperType
from brom_drake import robots
from brom_drake.motion_planning.systems import OpenLoopPlanDispenser, OpenLoopPosePlanDispenser
from brom_drake.productions.types.debug import BasicGraspingDebuggingProduction
from brom_drake.productions import ProductionID
from brom_drake.productions.roles.role import Role
from brom_drake.systems.abstract_list_selection_system import AbstractListSelectionSystem
from brom_drake.systems.abstract_port_switch_system import AbstractPortSwitch
from brom_drake.systems.pose_composition import PoseCompositionSystem
from brom_drake.utils import (
    Performer, collision_checking, NetworkXFSM, FlexiblePortSwitch, FSMTransitionCondition,
    FSMOutputDefinition, FSMTransitionConditionType, Puppetmaker, PuppetmakerConfiguration, PuppetSignature,
)
from brom_drake.utils.initial_condition_manager import InitialConditionManager
from brom_drake.utils.leaf_systems import RigidTransformToVectorSystem, define_named_vector_selection_system
from brom_drake.utils.leaf_systems.rigid_transform_to_vector_system.configuration import Configuration as RigidTransformToVectorSystemConfiguration
from brom_drake.utils.model_instances import (
    get_name_of_first_body_in_urdf,
    find_number_of_positions_in_welded_model,
)
from brom_drake.utils.triads import AddMultibodyTriad
from .config import Configuration
from .phases import AttemptGraspWithPuppeteerWristPhase
from .script import Script as AttemptGraspWithPuppeteerWristScript

class AttemptGraspWithPuppeteerWrist(BasicGraspingDebuggingProduction):
    """
    *Description*

    A production where the user can provide:
    
    - An object to be grasped
    - A gripper
    - The joint positions for the gripper to be in when performing the grasp
    - A pose trajectory for a given gripper frame (usually at the base of the gripper)

    and test whether or not the gripper can successfully grasp the object using the desired approach
    trajectory.

    This production uses a Puppetmaker to control the wrist of the gripper, allowing for
    more complex wrist motions during the grasping attempt.

    The order of events is controlled by a *script*
    (an object of :py:class:`the corresponding Script type<brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.script.Script>`)
    accessible by accessing the ``config.script`` variable of this object.

    *Parameters*

    path_to_object: str
        The file path to the URDF/SDF of the object to be grasped.

    gripper_choice: GripperType
        An enum value from GripperType that specifies which gripper to use for the grasping attempt.

    grasp_joint_positions: np.ndarray
        An array of floats representing the joint positions the gripper should be in when performing the grasp.

    X_ObjectGripper_trajectory: List[RigidTransform], optional
        The desired trajectory for the Gripper Frame.
        A sequence of poses of the Gripper Frame (usually attached to the base of the gripper)
        with respect to the world frame.
        Default is None.

    meshcat_port_number: int, optional
        The port number for Meshcat visualization.

    config: Configuration, optional
        The configuration for the production.
        See the Configuration dataclass for more details.
        Default is the standard Configuration.

    *Usage*

    The user can create an instance of this production and simulate it to see if the grasp is successful in the following way: ::

        from brom_drake.all import drakeify_my_urdf, GripperType, MeshReplacementStrategy
        from brom_drake.productions import AttemptGraspWithPuppeteerWrist

        # Create erlenmeyer flask urdf
        model_file = ... # Some path to your urdf file

        # Create the following poses (transforms):
        # - The Grasp Pose (i.e. the target pose of the gripper wrist when grasping the object)
        X_ObjectTarget = RigidTransform(
            p=np.array([-0.08, 0.05, 0.2]),
            rpy=RollPitchYaw(0.0, np.pi/2.0, 0.0),
        )

        # - The Pre-Grasp Pose (i.e. the pose of the gripper wrist just before reaching to grasp the object)
        X_WorldPreGrasp = X_ObjectTarget.multiply(
            RigidTransform(
                p=np.array([0.0, 0.0, -0.2]),
                rpy=RollPitchYaw(0.0, 0.0, 0.0),
            )
        )

        # Create the production
        production = AttemptGraspWithPuppeteerWrist(
            path_to_object=model_file,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            X_ObjectGripper_trajectory=[X_WorldPreGrasp, X_ObjectTarget],
            meshcat_port_number=7001, # Use None for CI
        )

        # Build with watcher (so we can view the simulation's data in `brom/watcher/plots` which is helpful for debugging)
        diagram, diagram_context = production.add_cast_and_build()
        script = production.config.script

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)
        simulator.Initialize()
        simulator.AdvanceTo(script.total_time())

    There are also several examples in the `examples/productions/debug/grasping/attempt_grasp_with_puppeteer_wrist` folder.
        
    """
    def __init__(
        self,
        path_to_object: str,
        gripper_choice: GripperType,
        grasp_joint_positions: np.ndarray,
        X_ObjectGripper_trajectory: List[RigidTransform] = None,
        meshcat_port_number: int = None,
        config: Configuration = Configuration(),
    ):
        """
        *Description*
        

        *Parameters*

        path_to_object: str
            The file path to the URDF/SDF of the object to be grasped.

        gripper_choice: GripperType
            An enum value from GripperType that specifies which gripper to use for the grasping attempt.

        grasp_joint_positions: np.ndarray
            An array of floats representing the joint positions the gripper should be in when performing the grasp.
        
        X_ObjectGripper_trajectory: List[RigidTransform]
            The desired trajectory for the Gripper Frame.
            A sequence of poses of the Gripper Frame (usually attached to the base of the gripper)
            with respect to the object being grasped.

        meshcat_port_number: int, optional
            The port number for Meshcat visualization.

        config: Configuration, optional
            The configuration for the production.
            See the Configuration dataclass for more details.
            Default is the standard Configuration.
        """
        # Use the enum to choose the gripper URDF from a set of supported grippers
        if gripper_choice == GripperType.Robotiq_2f_85:
            path_to_gripper = str(
                impresources.files(robots) / "models/robotiq/2f_85_gripper/urdf/robotiq_2f_85.urdf"
            )
        else:
            raise ValueError(f"Gripper type {gripper_choice} not supported for this scene! Create an issue on GitHub if you want it to be!")

        # Handle config
        config.base.meshcat_port_number = meshcat_port_number

        # Call the parent constructor
        super().__init__(
            path_to_object=path_to_object,
            path_to_gripper=path_to_gripper,
            X_ObjectGrasp=X_ObjectGripper_trajectory[-1],
            meshcat_port_number=config.base.meshcat_port_number,
            time_step=config.base.time_step,
            target_body_on_gripper=config.target_body_on_gripper,
            gripper_color=config.gripper_color,
            show_gripper_base_frame=config.show_gripper_base_frame,
            show_collision_geometries=config.show_collision_geometries,
        )

        # This will define the following fields for the production:
        # - path_to_object
        # - path_to_gripper
        # - X_ObjectTarget
        # - meshcat_port_number
        # - time_step
        # - target_body_on_gripper
        # - gripper_color
        # - show_gripper_base_frame
        # - show_collision_geometries
        # - plant
        # - scene_graph

        # TODO(kwesi): Figure out a better way to CHECK the initial joint positions
        # Create INITIAL joint position array
        self.initial_gripper_joint_positions = np.zeros(grasp_joint_positions.shape)# initial_gripper_joint_positions

        # Save the trajectory of the gripper frame
        self.X_ObjectGripper_trajectory = X_ObjectGripper_trajectory
        self.add_frames_for_gripper_base_trajectory(self.X_ObjectGripper_trajectory)
        
        self.grasp_joint_positions = grasp_joint_positions
        self.config = config
        
        # Add Name to plantPlant and Scene Graph for easy simulation
        self.plant.set_name("AttemptDynamicGrasp_plant")

        # Create show me system for holding object in place
        self.show_me_system = None

        self.floor_model_index = None
        self.floor_actuator = None
        self.floor_shape = None

        # Create an executive/brain
        self.executive = self.create_executive_system() # The executive system that coordinates ALL the systems

    def add_floor_controller_and_connect(self, floor_mass: float):
        """
        *Description*

        This method will add a controller to the "actuated floor" in this production.
        The controller will be a PID controller that maintains the floor at a constant height
        during the first few phases of the production, but eventually we will tell the floor to drop.

        *Parameters*

        floor_mass: float
            The mass of the floor object.
        """
        # Setup
        plant: MultibodyPlant = self.plant

        # Find the z position of the floor
        z_floor = self.find_floor_z_via_line_search()

        # Connect a PID Controller to the floor actuator
        kp = np.array([[floor_mass * 9.81 * 1.0e0]]) #Idk how I'm picking this number.
        ki = np.zeros(kp.shape) # 0.1 * np.sqrt(kp)
        kd = 4.0 * np.sqrt(kp)
        floor_controller = self.builder.AddSystem(
            PidController(kp=kp, ki=ki, kd=kd),
        )
        floor_controller.set_name("[Floor] PID Controller")

        # Create a feedforward term that we will add to the PID controller's
        # output to keep the floor at a constant height
        # (i.e., the weight of the object)
        feedforward_term = self.builder.AddSystem(
            ConstantVectorSource(np.array([[floor_mass * 9.81]])),  # Weight of the object
        )
        feedforward_term.set_name("[Floor] Feedforward")

        # Summer to combine the PID output and the feedforward term
        summer = self.builder.AddSystem(
            Adder(2, 1),
        )

        self.builder.Connect(
            floor_controller.get_output_port(0),  # PID output
            summer.get_input_port(0),  # summer input
        )

        self.builder.Connect(
            feedforward_term.get_output_port(0),  # Weight of the object
            summer.get_input_port(1),  # Feedforward term
        )

        # Connect the target height to the PID controller
        target_height_source = self.create_floor_trajectory_source(z_floor=z_floor)

        # Connect the source to the floor actuator
        self.builder.Connect(
            summer.get_output_port(0),
            plant.get_actuation_input_port(self.floor_model_index),
        )

        # Connect the target height to the PID controller
        self.builder.Connect(
            target_height_source.get_output_port(0),
            floor_controller.get_input_port_desired_state(),
        )

        # Connect the current height of the floor to the PID controller
        self.builder.Connect(
            plant.get_state_output_port(self.floor_model_index),
            floor_controller.get_input_port_estimated_state(),
        )

    def add_floor_to_plant(
        self,
        floor_mass: float = 100.0,
        plant: MultibodyPlant = None,
    ) -> Tuple[
        float, List[float], ModelInstanceIndex, JointActuator
    ]:
        """
        *Description*
        
        This method will add a "floor" (i.e., a rectangular prism with fixed mass)
        to the simulation (i.e., to the plant). It also adds an actuated joint
        between the world origin and the floor, that allows the floor to move up and down.

        *Parameters*

        floor_mass: float, optional
            The mass of the floor object. Default is 100.0 kg.

        plant: MultibodyPlant, optional
            The plant to which the floor will be added.
            If None, the production's plant will be used.
            Default is None.

        *Returns*

        floor_mass: float
            The mass of the floor object.

        floor_shape: List[float]
            The shape of the floor object as [length, width, height].

        floor_model_index: ModelInstanceIndex
            The model instance index of the floor in the plant.

        floor_actuator: JointActuator
            The joint actuator that controls the floor's vertical movement.
        """
        # Setup
        if plant is None:
            plant: MultibodyPlant = self.plant

        # Create a box urdf for the floor
        floor_shape = [10.0, 10.0, 0.1] # Length, Width, Height
        floor_geometry_defn = BoxDefinition(size=floor_shape)

        # Create a urdf for the floor
        floor_urdf_defn = SimpleShapeURDFDefinition(
            name="floor",
            shape=floor_geometry_defn,
            mass=floor_mass,
        ) 
        floor_urdf = floor_urdf_defn.write_to_file()

        # Add the floor to the plant
        floor_model_idcs = Parser(plant=plant).AddModels(floor_urdf)
        floor_model_index = floor_model_idcs[0]

        # Create joint + actuator for the floor
        floor_joint = plant.AddJoint(
            PrismaticJoint(
                name="floor_joint",
                frame_on_parent=plant.world_frame(),
                frame_on_child=plant.GetFrameByName(get_name_of_first_body_in_urdf(floor_urdf)),
                axis=[0, 0, 1],
            )
        )

        floor_actuator = plant.AddJointActuator(
            "floor_elevation_actuator",
            floor_joint,
        )

        return floor_mass, floor_shape, floor_model_index, floor_actuator

    def add_frames_for_gripper_base_trajectory(
        self,
        pose_trajectory: List[RigidTransform]
    ):
        """
        *Description*
        
        This method will add visualizations for the frames
        that are the waypoints in the given gripper base trajectory.

        *Parameters*

        pose_trajectory: List[RigidTransform]
            The desired trajectory for the Gripper Frame.
            A sequence of poses of the Gripper Frame (usually attached to the base of the gripper)
            with respect to the object being grasped.
            TODO(Kwesi): Change the name of this parameter to include frame information.
        """
        # Create frames for each pose in the trajectory
        gripper_base_frames = []
        for i, pose_i in enumerate(pose_trajectory):
            frame_i = FixedOffsetFrame(
                name = "Gripper Base Frame Traj Pt " + str(i),
                P= self.plant.world_frame(), # Parent Frame
                X_PF=pose_i,
            )
            frame_i_added = self.plant.AddFrame(frame_i)
            gripper_base_frames.append(frame_i_added)

        for i, gripper_base_frame_i in enumerate(gripper_base_frames):
            # Add a triad for each frame
            AddMultibodyTriad(
                frame=gripper_base_frame_i,
                scene_graph=self.scene_graph,
                length=0.05,
                radius=0.005,
                opacity=1.0,
            )

        return gripper_base_frames

    def add_gripper_controllers_and_connect(
        self,
        gripper_puppet_input: RigidTransformToVectorSystem,
        gripper_joint_input: PassThrough,
    ):
        # Setup
        builder = self.builder
        script = self.config.script
        plant: MultibodyPlant = self.plant

        gripper_base_pose_source = self.add_gripper_sources_and_connect(builder, plant, script)

        # Combine gripper pose source (X_ManipulandGripper) with the current pose of the
        # manipuland in the world (X_WorldManipuland) and send that signal to the puppet (X_WorldGripperDesired)
        builder.Connect(
            gripper_base_pose_source.get_output_port(),
            gripper_puppet_input.get_input_port(),
        )

        debug_sys2 = builder.AddNamedSystem(
            name="[AttemptGrasp] Gripper Puppet Input Debug",
            system=RigidTransformToVectorSystem(
                RigidTransformToVectorSystemConfiguration(
                    name=f"debug_attempt_grasp_gripper_puppet_input",
                    output_format="vector_xyz_euler(rpy)",
                )
            ),
        )
        builder.Connect(
            gripper_base_pose_source.get_output_port(),
            debug_sys2.get_input_port(),
        )

        # Connect controller for all of the joints
        self.add_gripper_joint_controllers_and_connect(gripper_joint_input)

    def add_gripper_sources_and_connect(
        self,
        builder: DiagramBuilder,
        plant: MultibodyPlant,
        script: AttemptGraspWithPuppeteerWristScript,
    ) -> Tuple[AbstractPortSwitch]:
        # Setup
        executive: NetworkXFSM = self.executive

        # Create a trajectory source for the gripper base (X_ManipulandGripper)
        pose_ManipulandGripper_trajectory = script.build_gripper_approach_trajectory_from_waypoints(self.X_ObjectGripper_trajectory)
        gripper_pose_source = builder.AddNamedSystem(
            name="Source for Gripper Pose in Object Frame",
            system=PoseTrajectorySource(pose_ManipulandGripper_trajectory),
        )

        # Create a system that will convert the reference trajectory
        # from the object frame to the world frame
        object_to_world_converter = builder.AddNamedSystem(
            name="Object to World Frame Converter for Gripper Pose",
            system=PoseCompositionSystem(),
        )

        # - Connect the pose of the object in the world to the FIRST input port
        manipuland_body_indices = plant.GetBodyIndices(self.manipuland_index)
        assert len(manipuland_body_indices) == 1, \
            f"This production currently supports manipulands containing one body; please create an issue if you want more support for objects with multiple parts!"

        pose_selector = builder.AddNamedSystem(
            system=AbstractListSelectionSystem(
                index=int(manipuland_body_indices[0]), 
                output_type=RigidTransform
            ),
            name="Selector for Manipuland Pose in World Frame",
        )
        builder.Connect(
            plant.get_body_poses_output_port(),
            pose_selector.get_input_port(),
        )

        # - Connect the selected pose to the FIRST input port of the converter
        builder.Connect(
            pose_selector.get_output_port(),
            object_to_world_converter.get_pose_AB_port(),
        )

        # - Connect the gripper pose source to the SECOND input port of the converter
        builder.Connect(
            gripper_pose_source.get_output_port(),
            object_to_world_converter.get_pose_BC_port(),
        )

        # Create a system that will "remember" the last
        # desired pose of the gripper in world when we switch away from it
        memory_system = builder.AddNamedSystem(
            name="[AttemptGrasp] Gripper Desired Pose Memory",
            system = ConstantValueSource(
                AbstractValue.Make(self.X_ObjectGripper_trajectory[-1])
            ),
        )

        # Create a flexible port switch to choose between
        # the trajectory source and the memory system
        gripper_pose_port_switch: AbstractPortSwitch = builder.AddNamedSystem(
            name="[AttemptGrasp] Gripper Desired Pose Port Switch",
            system=AbstractPortSwitch(selector_type_in=str, input_type=RigidTransform),
        )

        gripper_pose_port_switch.DeclareInputPort(name="trajectory1")
        gripper_pose_port_switch.DeclareInputPort(name="memory2")

        # - Connect the output of the converter to input 0 of the port switch
        builder.Connect(
            object_to_world_converter.get_output_port(),
            gripper_pose_port_switch.GetInputPort("trajectory1"),
        )

        # Connect the output of the memory system to input 1 of the port switch
        builder.Connect(
            memory_system.get_output_port(),
            gripper_pose_port_switch.GetInputPort("memory2"),
        )

        # # Connect the output of the port switch to the input of the memory system
        # builder.Connect(
        #     gripper_pose_port_switch.get_output_port(),
        #     memory_system.get_input_port(),
        # )

        # - The switch is controlled by the executive
        builder.Connect(
            executive.GetOutputPort("switch_to_memory"),
            gripper_pose_port_switch.get_port_selector_input_port(),
        )

        return gripper_pose_port_switch



    def add_gripper_joint_controllers_and_connect(
        self,
        gripper_joint_input: PassThrough
    ):
        # Setup
        builder: DiagramBuilder = self.builder
        plant: MultibodyPlant = self.plant

        # Create a trajectory source for the gripper joints
        gripper_trajectory_source = self.add_gripper_joints_trajectory_system()

        # Create a simple piece of logic to turn on/off the gripper (i.e., to close it)
        gripper_controller = builder.AddSystem(
            GripperController(
                GripperType.Robotiq_2f_85,
                Kp=1e2 * np.eye(2),
            ),
        )
        gripper_controller.set_name("[Gripper] Gripper Controller")

        # Connect the gripper controller to the gripper
        builder.Connect(
            gripper_controller.GetOutputPort("applied_gripper_torque"),
            gripper_joint_input.get_input_port(),
        )

        # Connect plan to the controller
        builder.Connect(
            gripper_trajectory_source.get_output_port(),
            gripper_controller.GetInputPort("gripper_target"),
        )

        # Create a signal that tells the controller to use position control
        gripper_position_control_source = builder.AddSystem(
            ConstantValueSource(
                AbstractValue.Make(GripperTarget.kPosition),
            )
        )
        gripper_position_control_source.set_name("[Gripper] Target Type Signal")
        builder.Connect(
            gripper_position_control_source.get_output_port(),
            gripper_controller.GetInputPort("gripper_target_type"),
        )

        # Create a system that selects ONLY the elements of the gripper state
        # that are relevant to the gripper controller (i.e., the gripper joint
        # positions and velocities, but none of the puppeteer states)
        states_to_pass_to_gripper_controller = [
            state_name 
            for state_name in plant.GetStateNames(self.gripper_model_index)
            if "puppetmaker" not in state_name
        ]

        state_compressor = builder.AddNamedSystem(
            name=f"[AttemptGrasp] Plant State To Gripper State Converter",
            system=define_named_vector_selection_system(
                all_element_names=plant.GetStateNames(self.gripper_model_index),
                sequence_of_names_for_output=states_to_pass_to_gripper_controller,
            )
        )

        builder.Connect(
            plant.get_state_output_port(self.gripper_model_index),
            state_compressor.get_input_port(),
        )

        builder.Connect(
            state_compressor.get_output_port(),
            gripper_controller.GetInputPort("gripper_state"),
        )

    def add_gripper_joints_trajectory_system(self) -> TrajectorySource:
        """
        *Description*

        This method creates a LeafSystem that outputs a PiecewisePolynomial
        trajectory for the gripper joints to follow during the grasping attempt.

        *Returns*

        gripper_trajectory_source: TrajectorySource
            A TrajectorySource system that outputs the desired gripper joint trajectory.
        """
        # Setup
        builder: DiagramBuilder = self.builder
        script: AttemptGraspWithPuppeteerWristScript = self.config.script

        # Collect gripper initial and final positions
        p_gripper0 = self.initial_gripper_joint_positions
        p_gripper_final = self.grasp_joint_positions

        # Find times for each phase
        t_gripper_close_start = script.start_time_of_phase(
            phase=AttemptGraspWithPuppeteerWristPhase.kGripperClosing
        )
        t_object_settling_in_grasp_start = script.start_time_of_phase(
            phase=AttemptGraspWithPuppeteerWristPhase.kObjectSettlingInGrasp
        )

        # Create a piecewise trajectory for the gripper to follow
        joint_position_samples = []
        joint_position_samples.append(p_gripper0.reshape(-1, 1).tolist()) # This wild compression is needed because of bugs in Drake. :'(
        joint_position_samples.append(p_gripper0.reshape(-1, 1).tolist())
        joint_position_samples.append(p_gripper_final.reshape(-1, 1).tolist())
        gripper_trajectory = PiecewisePolynomial.FirstOrderHold(
            samples=joint_position_samples,
            breaks=[0.0, t_gripper_close_start, t_object_settling_in_grasp_start],
        )

        # Add a trajectory source object to the builder
        gripper_trajectory_source = builder.AddSystem(
            TrajectorySource(gripper_trajectory),
        )
        gripper_trajectory_source.set_name("[Gripper] Trajectory Source")
        return gripper_trajectory_source

    def add_puppeteer_for_gripper(self) -> Tuple[Puppetmaker, PuppetSignature]:
        """
        *Description*

        This method will make the gripper model a "puppet" and return
        the puppetmaker and puppet signature for the gripper.

        By making the gripper a puppet, we can then control its wrist
        using the Puppetmaker's controller.

        *Returns*

        puppetmaker0: Puppetmaker
            The Puppetmaker system that CREATES the gripper puppet.

        puppet_signature0: PuppetSignature
            The PuppetSignature that describes the puppet strings
            for the gripper puppet (i.e., the actuators on the puppet).
        """

        # Setup
        plant : MultibodyPlant = self.plant

        # Create a puppet maker
        puppetmaker0 = Puppetmaker(
            plant=plant,
            config=PuppetmakerConfiguration(
                frame_on_parent=plant.world_frame(),
                name="attempt_grasp_puppetmaker",
                sphere_radius=0.02,
                sphere_color=np.array([0.0,1.0,0.0,0.8]), # Change the color of the puppet links to green
            )
        )

        # Add puppet strings for the gripper
        puppet_signature0 = puppetmaker0.add_strings_for(self.gripper_model_index)

        return puppetmaker0, puppet_signature0

    def add_supporting_cast(self):
        """
        *Description*
        
        This method adds the supporting cast to the production.
        This creates the plant (which contains the object being grasped,
        the gripper, the floor), the controllers for the gripper and floor,
        and the puppetmaker for the gripper wrist.

        This method also is required in order to implement the BaseProduction class.
        """
        # Setup

        # Add the object to the builder
        self.add_manipuland_to_plant()

        # Add the gripper to the builder
        self.add_gripper_to_plant()
        maker0, self.puppet_signature = self.add_puppeteer_for_gripper()

        # Add the floor
        floor_mass, self.floor_shape, self.floor_model_index, self.floor_actuator = self.add_floor_to_plant()

        # Connect the plant to the meshcat, if requested
        if self.meshcat_port_number is not None:
            self.connect_to_meshcat()

        # Finalize the plant
        self.plant.Finalize()
        puppet_pose_input, replacement_gripper_actuator_inputs = maker0.add_puppet_controller_for(
            self.puppet_signature,
            self.builder,
            Kp=np.array([1e2, 1e2, 1e2, 1e1, 1e1, 6e0])
        )

        # Add controllers for gripper AND floor
        self.add_gripper_controllers_and_connect(puppet_pose_input, replacement_gripper_actuator_inputs)
        self.add_floor_controller_and_connect(floor_mass)

        # Create defaults for plant
        self.set_initial_conditions()
        # self.add_initial_conditions_to_plant()    

    def connect_executive_to_gripper_puppet_controller(self, trajectory_dispenser: OpenLoopPosePlanDispenser):
        """
        *Description*

        This method connects the executive system to the trajectory dispenser
        (i.e., provides the trigger signal to start the reference trajectory
        for the gripper wrist).
        """
        # Setup
        builder: DiagramBuilder = self.builder
        executive: NetworkXFSM = self.executive

        # Connect the executive to the trajectory dispenser
        builder.Connect(
            executive.GetOutputPort("enable_gripper_approach"),
            trajectory_dispenser.GetInputPort("plan_ready"),  # Trigger input
        )

    def create_executive_system(self) -> NetworkXFSM:
        """
        *Description*
        
        This method creates the executive system for the production.
        The executive system is responsible for coordinating the various
        phases of the production.
        """
        # Setup
        builder: DiagramBuilder = self.builder
        script: AttemptGraspWithPuppeteerWristScript = self.config.script

        # Create the NetworkXFSM from the graph
        executive = builder.AddSystem(script.to_fsm())

        return executive

    def create_floor_trajectory_source(
        self,
        z_floor: float
    ) -> Tuple[FlexiblePortSwitch]:
        """
        *Description*

        This method creates a LeafSystem which outputs a reference trajectory
        for the floor height during the production.

        *Parameters*

        z_floor: float
            The initial height of the floor.

        *Returns*

        floor_target_height_source: FlexiblePortSwitch
            A LeafSystem which outputs a reference trajectory for the floor height during the production.
            It is a "switch" system that can switch between different input ports
            based on the executive's commands.
        """
        # Setup
        builder: DiagramBuilder = self.builder

        #Create a proper trajectory source for the floor so that it slowly falls away
        floor_target_height_source = builder.AddSystem(
            OpenLoopPlanDispenser(2, speed=0.25),
        )

        # Create the trajectory that will be run
        simple_plan = np.array([
            [z_floor, 0.0],
            [z_floor - 10.0, 0.0]
        ])
        floor_trajectory_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(simple_plan))
        )
        
        builder.Connect(
            floor_trajectory_source.get_output_port(),
            floor_target_height_source.GetInputPort("plan"),
        )

        # Connect timer to the plan dispenser
        builder.Connect(
            self.executive.GetOutputPort("start_floor"),
            floor_target_height_source.GetInputPort("plan_ready"),  # Trigger input
        )

        return floor_target_height_source

    def find_floor_z_via_bounding_box(
        self,
        debug_flag: bool = True,
    ) -> float:
        """
        *Description*
        
        This method will find the transform needed to put the floor at so that
        the object sits "just" on top of it.

        WARNING: This method does not work and I don't know why. (i.e., sometimes
        the floor is in collision with the object)

        *Parameters*

        debug_flag: bool, optional
            If True, will print debug information during the computation. Default is True.

        *Returns*

        floor_z_translation: float
            The z translation for the floor such that the object sits just on top of it.
        """
        # Setup
        shadow_builder = DiagramBuilder()

        # Create shadow plant + scene graph
        shadow_plant, shadow_scene_graph = AddMultibodyPlantSceneGraph(
            shadow_builder,
            time_step=self.time_step,
        )
        shadow_plant.set_name("DemonstrateStaticGrasp_shadow_plant")
        shadow_scene_graph.set_name("DemonstrateStaticGrasp_shadow_scene_graph")

        # Add the object to the shadow plant
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_object)
        assert len(model_idcs) == 1, f"Only one model should be added; received {len(model_idcs)}"
        model_idx = model_idcs[0]
        model_name = shadow_plant.GetModelInstanceName(model_idx)

        # Weld the base link to the world frame
        # manipuland_body_idcs = shadow_plant.GetBodyIndices(model_idx)
        # assert len(manipuland_body_idcs) > 0, \
        #     f"Expected at least one body in the manipuland; received {len(manipuland_body_idcs)}"
        # manipuland_body = shadow_plant.get_body(manipuland_body_idcs[0])
        # frame0 = manipuland_body.body_frame()
        # shadow_plant.WeldFrames(
        #     shadow_plant.world_frame(),
        #     frame0,
        # )

        # Finalize the shadow plant
        shadow_plant.Finalize()

        # Search through the geometries in the scene and find the manipuland's
        # geometry. Attempt to compute the bounding box around it.
        shadow_sg_inspector: SceneGraphInspector = shadow_scene_graph.model_inspector()
        all_geometries = shadow_scene_graph.model_inspector().GetAllGeometryIds()
        objects_minimum_z = 1_000.0
        for ii, geometry_ii in enumerate(all_geometries):
            # Announce geometry value:
            if debug_flag:
                print(f"Geometry ID {ii}:")
                print(f"- Raw value: {geometry_ii}")

            # Get shape of the geometry
            shape_ii = shadow_sg_inspector.GetShape(geometry_ii)
            if debug_flag:
                print(f"- Shape {ii}:")
                print(f"  + Raw value: {shape_ii}")
                print(f"  + Type: {type(shape_ii)}")

            # Get the convex hull
            ch_ii = shape_ii.GetConvexHull()
            if debug_flag:
                print(f"- Convex Hull {ii}:")
                print(f"  + Raw value: {ch_ii}")

            bb_ii = ch_ii.CalcBoundingBox()
            if debug_flag:
                print(f"- Bounding Box {ii}:")
                print(f"  + Raw value: {bb_ii}")
            lower_left, upper_right = bb_ii

            if lower_left[2] < objects_minimum_z:
                objects_minimum_z = lower_left[2]
                print(f"  + New z_out: {objects_minimum_z}")
        
        # If we didn't find any geometry, raise an error
        if objects_minimum_z == 1_000.0:
            raise ValueError("Unable to find geometry for the manipuland.")
        
        # If we found a geometry, then we need to set the floor height
        # using the known shape of the table

        return objects_minimum_z - self.floor_shape[2]*0.5  # Subtract the height of the floor

    def find_floor_z_via_line_search(
        self,
        debug_flag: bool = False,
    ) -> float:
        """
        *Description*
        
        Use bounding box method to generate an initial guess for the floor height
        and then perform a line search to find the height that actually
        is collision free.

        *Parameters*

        debug_flag: bool, optional
            If True, will print debug information during the computation. Default is False.

        *Returns*

        floor_z_translation: float
            The z translation for the floor such that the object sits just on top of it
        """
        # Setup
        z_floor = self.find_floor_z_via_bounding_box(debug_flag=debug_flag)
        floor_shape = self.floor_shape

        # Check for collisions at the initial guess
        collision_detected = self.floor_z_creates_collisions(
            floor_z_translation=z_floor,
            debug_flag=debug_flag,
        )
        if not collision_detected:
            # If no collision detected, return the initial guess
            return z_floor
        
        # If we have a collision, then we need to perform a line search
        # to find the height that is collision free
        
        # Start with a small step size
        step_size = floor_shape[2] * 0.25  # 25% of the floor height

        # Perform a line search to find the height that is collision free
        candidate_z_floor = z_floor - step_size
        while True:
            # Check for collisions at the current height
            collision_detected = self.floor_z_creates_collisions(
                floor_z_translation=candidate_z_floor,
                debug_flag=debug_flag,
            )
            if not collision_detected:
                # If no collision detected, return the current height
                print(f"Found collision-free height: {candidate_z_floor}")
                return candidate_z_floor
            

            print(f"Collision detected at height: {candidate_z_floor}")
            # If we have a collision, then increase the height
            candidate_z_floor -= step_size

            # If we exceed a certain height, break out of the loop
            if candidate_z_floor < -10.0:
                break        
                
        raise ValueError("Unable to find a collision-free height for the floor.")

    def find_X_WorldGripper(
        self,
        X_ObjectGripper: RigidTransform,
        target_frame_name: str,
        desired_joint_positions: List[float],
    ) -> RigidTransform:
        """
        *Description*
        
        This method will return the transform from the world frame to the gripper frame.

        *Parameters*

        X_ObjectGripper: RigidTransform
            The desired transform from the object frame to the gripper target frame.

        target_frame_name: str
            The name of the target frame on the gripper.
            (i.e., the frame that we define as "Gripper" in the X_ObjectGripper transform).

        desired_joint_positions: List[float]
            The desired joint positions for the gripper.
            TODO(Kwesi): Is this needed? Maybe when we use the fingers as the target frame name.

        *Returns*

        X_WorldGripper: RigidTransform
            The transform from the world frame to the gripper frame.
        """
        # Setup
        desired_joint_positions = np.array(desired_joint_positions)

        # Create shadow plant containing just the gripper
        shadow_plant = MultibodyPlant(self.time_step)
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        shadow_model_idx = model_idcs[0]
        
        # Get the first body on the gripper
        name_of_first_body_in_gripper = get_name_of_first_body_in_urdf(self.path_to_gripper)

        # Weld the base link to the world frame
        shadow_plant.WeldFrames(
            shadow_plant.world_frame(),
            shadow_plant.GetFrameByName(name_of_first_body_in_gripper),
        )

        # Finalize Plant
        shadow_plant.Finalize()

        shadow_builder = DiagramBuilder()
        shadow_builder.AddSystem(shadow_plant)
        diagram = shadow_builder.Build()
        shadow_diagram_context = diagram.CreateDefaultContext()

        # Set the joint positions
        # shadow_plant.SetPositions(
        #     shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
        #     shadow_model_idx,
        #     desired_joint_positions,
        # )

        # Get the transform of the gripper's base in the static world frame
        X_GripperBase_GripperTarget = shadow_plant.EvalBodyPoseInWorld(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_plant.GetBodyByName(target_frame_name),
        )

        X_Object_GripperTarget = X_ObjectGripper
        X_WorldObject = RigidTransform.Identity()
        X_Object_GripperBase = X_Object_GripperTarget.multiply(
            X_GripperBase_GripperTarget.inverse(),
        )

        return X_WorldObject.multiply(X_Object_GripperBase)

    def floor_z_creates_collisions(
        self,
        floor_z_translation: float = 0.0,
        debug_flag: bool = False,
    ) -> bool:
        """
        *Description*
        
        This method will check if the given floor z translation
        creates collisions with the object in the production.

        *Parameters*

        floor_z_translation: float, optional
            The z translation of the floor to check for collisions. Default is 0.0.

        debug_flag: bool, optional
            If True, will print debug information during the computation. Default is False.
            TODO(Kwesi): Implement debug information.

        *Returns*

        collision_detected: bool
            True if a collision is detected between the floor and the object, False otherwise.
        """
        # Setup
        shadow_builder = DiagramBuilder()

        # Create shadow plant + scene graph
        shadow_plant, shadow_scene_graph = AddMultibodyPlantSceneGraph(
            shadow_builder,
            time_step=self.time_step,
        )
        shadow_plant.set_name("DemonstrateStaticGrasp_shadow_plant")
        shadow_scene_graph.set_name("DemonstrateStaticGrasp_shadow_scene_graph")

        # Add the object to the shadow plant
        model_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_object)
        assert len(model_idcs) == 1, f"Only one model should be added; received {len(model_idcs)}"
        model_idx = model_idcs[0]
        model_name = shadow_plant.GetModelInstanceName(model_idx)

        # # Add the gripper to the shadow plant
        # temp_idcs = Parser(plant=shadow_plant).AddModels(self.path_to_gripper)
        # assert len(temp_idcs) == 1, f"Only one model should be added; received {len(temp_idcs)}"
        # shadow_gripper_model_index = temp_idcs[0]
        # shadow_gripper_model_name = shadow_plant.GetModelInstanceName(shadow_gripper_model_index)

        # Add the floor to the shadow plant
        _, _, shadow_floor_model_index, _ = self.add_floor_to_plant(plant=shadow_plant)

        # Finalize the shadow plant
        shadow_plant.Finalize()

        # Create diagram and set position of the floor to be floor_z_translation
        shadow_diagram = shadow_builder.Build()
        shadow_diagram_context = shadow_diagram.CreateDefaultContext()

        shadow_plant.SetPositions(
            shadow_plant.GetMyContextFromRoot(shadow_diagram_context),
            shadow_floor_model_index,
            np.array([floor_z_translation]),
        )

        return collision_checking.using_point_pair_penetration(
            shadow_plant, shadow_scene_graph, shadow_diagram_context,
        )

    @property
    def id(self):
        """
        *Description*

        Always returns ProductionID.kAttemptGraspWithPuppeteer.
        
        This property is required to implement the BaseProduction class.
        """
        return ProductionID.kAttemptGraspWithPuppeteer

    def set_initial_conditions(self):
        """
        *Description*
        
        This method will set the default state of the plant to the desired joint positions.
        """
        # Setup
        plant: MultibodyPlant = self.plant
        ic_manager: InitialConditionManager = self.initial_condition_manager

        # Find the z position of the floor
        z_floor = self.find_floor_z_via_line_search()

        # Set the default state of the floor to be at the desired height
        ic_manager.add_initial_configuration(
            model_instance_index=self.floor_model_index,
            configuration=np.array([z_floor]),
        )

        # Set the initial pose of the object
        ic_manager.add_initial_pose(
            model_instance_index=self.manipuland_index,
            pose_wrt_parent=RigidTransform.Identity(),
        )

        # # - Set the initial configuration of some of the puppet joints to match the gripper
        # #   (i.e., so that the puppet and gripper start in the same position)
        # X_WorldObject = X_WorldObject.multiply(self.X_ObjectGripper)
        # for jj in range(3):
        #     puppet_model_jj = self.puppet_signature.all_models[jj]
        #     ic_manager.add_initial_configuration(
        #         model_instance_index=puppet_model_jj,
        #         configuration=np.array([self.X_World_PreGrasp.translation()[jj]]),
        #     )

        # for jj in range(2):
        #     puppet_model_jj = self.puppet_signature.all_models[jj + 3]
        #     ic_manager.add_initial_configuration(
        #         model_instance_index=puppet_model_jj,
        #         configuration=np.array([self.X_World_PreGrasp.rotation().ToRollPitchYaw().vector()[jj]]),
        #     )

        # Set initial conditions for the puppeteer joints
        X_World_PreGrasp = self.X_ObjectGripper_trajectory[0]
        assert len(self.puppet_signature.all_joint_actuators) == 6, \
            f"Expected 6 puppet joint actuators; received {len(self.puppet_signature.all_joint_actuators)}"

        for joint_index, joint_actuator_i in enumerate(self.puppet_signature.all_joint_actuators):
            if joint_index < 3:
                self.initial_condition_manager.add_initial_configuration(
                    model_instance_index=joint_actuator_i.model_instance(),
                    configuration=np.array([X_World_PreGrasp.translation()[joint_index]])
                )
            elif joint_index == 3: # Yaw
                self.initial_condition_manager.add_initial_configuration(
                    model_instance_index=joint_actuator_i.model_instance(),
                    configuration=np.array([X_World_PreGrasp.rotation().ToRollPitchYaw().yaw_angle()])
                )
            elif joint_index == 4: # Pitch
                self.initial_condition_manager.add_initial_configuration(
                    model_instance_index=joint_actuator_i.model_instance(),
                    configuration=np.array([X_World_PreGrasp.rotation().ToRollPitchYaw().pitch_angle()])
                )

        # Set the initial configuration of the gripper
        # - Set the gripper positions (i.e., initial positions of the joints)
        #   (Because of the puppeteer, this has an additional fixed joint at the base)
        n_gripper_positions1 = plant.num_positions(self.gripper_model_index)

        initial_gripper_configuration = np.zeros((n_gripper_positions1,))
        initial_gripper_configuration[0] = X_World_PreGrasp.rotation().ToRollPitchYaw().roll_angle()        
        initial_gripper_configuration[1:] = self.initial_gripper_joint_positions
        
        ic_manager.add_initial_configuration(
            model_instance_index=self.gripper_model_index,
            configuration=initial_gripper_configuration,
        )

        # # - Set the gripper's base pose
        # X_WorldObject = X_WorldObject.multiply(self.X_ObjectGripper)
        # ic_manager.add_initial_pose(
        #     model_instance_index=self.gripper_model_index,
        #     pose_wrt_parent=X_WorldObject,
        # )

        

    @property
    def suggested_roles(self) -> List[Role]:
        """
        *Description*
        
        This property suggests the roles that are relevant for this production.

        No roles are necessary here, so this method returns an empty list.
        
        *Returns*
        suggested_roles: List[Role]
            A list of Role enums that are relevant for this production.
        """
        return []
    
    
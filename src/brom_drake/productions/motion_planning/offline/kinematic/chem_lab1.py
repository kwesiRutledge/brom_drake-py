
from importlib import resources as impresources
import networkx as nx
import numpy as np
from pydrake.all import (
    AbstractValue,
    CollisionFilterDeclaration,
    ConstantValueSource,
    ConstantVectorSource,
    Context,
    Diagram,
    GeometrySet,
    MultibodyPlant,
    Parser,
    Quaternion,
)
from pydrake.math import RollPitchYaw, RigidTransform
from typing import Callable, List, Tuple

# Internal Imports
from brom_drake.directories import DEFAULT_BROM_MODELS_DIR
from brom_drake.control import (
    GripperTarget,
)
from brom_drake.file_manipulation.urdf import drakeify_my_urdf
from brom_drake.file_manipulation.urdf.shapes.box import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import SimpleShapeURDFDefinition, \
    InertiaDefinition
from brom_drake.motion_planning.systems.open_loop_dispensers.open_loop_plan_dispenser import OpenLoopPlanDispenser
import brom_drake.robots as robots
from brom_drake.robots.gripper_type import GripperType
from brom_drake.stations.kinematic import UR10eStation as KinematicUR10eStation
from brom_drake.productions import ProductionID
from brom_drake.productions.roles import Role
from brom_drake.productions.types import KinematicMotionPlanningProduction
from brom_drake.utils import Performer, AddGround, MotionPlan


class ChemLab1(KinematicMotionPlanningProduction):
    """
    *Description*

    This production is the first in the chemistry lab series.
    It is used to test the motion planning capabilities of the robot
    in a chemistry lab setting with minimal constraints.
    
    *Parameters*

    time_step: float, optional
        The time step for the production, by default 1e-3.

    meshcat_port_number: int, optional
        The port number for meshcat visualization, by default 7001.

    plan_execution_speed: float, optional
        The speed at which to execute the motion plan, by default 0.2.

    pose_WorldBeaker: RigidTransform, optional
        The pose of the beaker in the world frame, by default None.

    table_length: float, optional
        The length of the table, by default 0.6.

    table_width: float, optional
        The width of the table, by default 2.0.

    table_height: float, optional
        The height of the table, by default 0.1.

    shelf_pose: RigidTransform, optional
        The pose of the shelf in the world frame, by default None.

    *Usage*

    To use this production, create an instance of the ChemLab1 class
    provide it with a motion planning function, and then build it. ::

        # Create the production
        production = ChemLab1()

        # Create a planner object which will be used to plan the motion
        config = RRTConnectPlannerConfig(
            steering_step_size=0.1,
            prob_sample_goal=0.30,
            max_iterations=int(1e5),
            convergence_threshold=1e-3,
            debug_flag=True,
        )
        planner2 = RRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=config,
        )

        # To build the production, we only need to provide a planning function
        # (can come from anywhere, not just a BaseRRTPlanner object)
        diagram, diagram_context = production.easy_cast_and_build(
            planner2.plan,
            with_watcher=True,
        )

        print("Simulating...")

        # Simulate the diagram
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(0.1)
        planned_trajectory = production.plan_dispenser.planned_trajectory
        print(f"Expected end time of trajectory: {planned_trajectory.end_time()}")

        # Advance to the end of the simulation
        simulator.AdvanceTo(planned_trajectory.end_time()+1.0)
    """
    def __init__(
        self,
        time_step: float =1e-3,
        meshcat_port_number: int = 7000,
        plan_execution_speed: float = 0.2,
        table_length: float = 0.6,
        table_width: float = 2.0,
        table_height: float = 0.1,
        shelf_pose: RigidTransform = None,
        **kwargs,
    ):
        """
        *Description*
            
        Constructor for the ChemLab1Scene class.

        *Parameters*

        time_step: float, optional
            The time step for the production, by default 1e-3.

        meshcat_port_number: int, optional
            The port number for meshcat visualization, by default 7001.

        plan_execution_speed: float, optional
            The speed at which to execute the motion plan, by default 0.2.

        pose_WorldBeaker: RigidTransform, optional
            The pose of the beaker in the world frame, by default None.

        table_length: float, optional
            The length of the table, by default 0.6.

        table_width: float, optional
            The width of the table, by default 2.0.

        table_height: float, optional
            The height of the table, by default 0.1.

        shelf_pose: RigidTransform, optional
            The pose of the shelf in the world frame, by default None.
        
        """
        # Superclass constructor
        super().__init__(**kwargs)

        # Input Processing
        self.time_step = time_step
        self.meshcat_port_number = meshcat_port_number
        self.plan_execution_speed = plan_execution_speed
        self.table_height, self.table_width = table_height, table_width
        self.table_length = table_length

        # Set shelf pose
        self.shelf_pose = shelf_pose
        if self.shelf_pose is None:
            self.pose_WorldShelf = RigidTransform(
                RollPitchYaw(0.0, 0.0, -np.pi/2.0).ToQuaternion(),
                np.array([0.0, 0.75, 0.66]),
            )

        # Set Beaker Pose
        self.pose_WorldBeaker = RigidTransform(
            RollPitchYaw(np.pi/2.0, 0.0, 0.0).ToQuaternion(),
            np.array([-0.6, 0.45, 0.075]),
        )

        # Set Holder Pose
        self.pose_WorldHolder = RigidTransform(
            RollPitchYaw(np.pi/2.0, 0.0, 0.0).ToQuaternion(),
            np.array([self.table_width*0.5*0.7, 0.6+self.table_length/4., self.table_height-0.015]),
        )

        # Set station
        self.station = KinematicUR10eStation(
            time_step=self.time_step,
            meshcat_port_number=self.meshcat_port_number,
            gripper_type=GripperType.NoGripper,
        )
        self.arm = self.station.arm
        self.robot_model_idx_ = self.arm

        # self.gripper = self.station.gripper

        # Set Names of Plant and scene graph
        self.plant = self.station.plant
        self.scene_graph = self.station.scene_graph

        self.plant.set_name(f"ChemLab1_Production_Plant")
        self.scene_graph.set_name(f"ChemLab1_Production_SceneGraph")

        # Define placeholder variables for models
        self.test_tube_holder1 = None

    def add_supporting_cast(self):
        """
        *Description*
        
        This method adds all secondary cast members to the builder.
        The secondary cast members in the production are the:
        
        - Table, where the robot exists
        - Test Tube Holders
        - Component which share's the robot model reference
        - Motion Planning components (e.g., dispensers, etc.)
        - Start and Goal sources
        
        """
        # Call the superclass method
        super().add_supporting_cast()

        # Setup

        # Create the table and the items on it
        self.add_table()
        self.add_test_tube_holders()
        self.add_beaker()
        self.add_shelf()
        AddGround(self.plant)

        # Add the station
        self.builder.AddSystem(
            self.station,
        )

        self.station.Finalize()

        # Add the motion planning components
        self.add_robot_source_system()
        self.add_motion_planning_components()
        self.add_start_and_goal_sources_to_builder()

        # Add initial conditions for the arm
        self.initial_condition_manager.add_initial_configuration(
            model_instance_index=self.arm,
            configuration=self.start_configuration,
        )
        # The initial condition will be set for the plant when we build the production.

        print("Completed adding supporting cast members.")

    def add_beaker(self):
        """
        *Description*
        
        This method adds a model of a beaker to the production.
        The beaker is welded in place at the pose pose_WorldBeaker
        with respect to the world origin.

        The beaker's URDF is specified in the `brom_drake` robots
        package.
        """
        # Setup
        plant = self.plant
        urdf_file_path = str(
            impresources.files(robots) / "models/beaker/beaker.urdf"
        )

        # Use Drakeify my urdf to create the beaker
        new_beaker_urdf = drakeify_my_urdf(urdf_file_path)

        # Add the beaker to the plant
        model_idcs = Parser(plant=plant).AddModels(str(new_beaker_urdf))
        self.beaker_model_index = model_idcs[0]

        # Weld the beaker to the world frame
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("beaker_base_link", self.beaker_model_index),
            self.pose_WorldBeaker,
        )

    def add_motion_planning_components(self):
        """
        *Description*

        Add the motion planning components to the builder.
        """
        # Setup
        arm = self.arm
        n_actuated_dof = self.plant.num_actuated_dofs(arm) # Number of actuated DOF in arm

        # Add the Plan Dispenser and connect it to the station
        self.plan_dispenser = self.builder.AddSystem(
            OpenLoopPlanDispenser(n_actuated_dof, self.plan_execution_speed)
        )

        self.builder.Connect(
            self.plan_dispenser.GetOutputPort("point_in_plan"),
            self.station.GetInputPort("desired_joint_positions"),
        )

    def add_shelf(self):
        """
        *Description*

        Add a model of a shelf to the production's plant.
        The shelf's pydrake.multibody.tree.ModelInstanceIndex will be saved in the internal variable
        ``self.shelf_model_index``.
        The shelf is fixed at the pose ``self.pose_WorldShelf`` with respect to the world origin.
        """
        # Setup
        plant = self.plant
        urdf_file_path = str(
            impresources.files(robots) / "models/cupboard/cupboard.sdf"
        )

        # Add the shelf to the plant
        model_idcs = Parser(plant=plant).AddModels(urdf_file_path)
        self.shelf_model_index = model_idcs[0]

        # Weld the bookshelf to the world frame
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("cupboard_body", self.shelf_model_index),
            self.pose_WorldShelf,
        )

    def add_table(self):
        """
        *Description*
        
        This method adds a model of a table to the production's plant.
        The table will be a simple shape that the robot will interact with.
        The table is fixed in a pose with respect to the origin (pose chosen in this function).
        """
        # Setup

        # Create a new shape urdf
        table_defn = SimpleShapeURDFDefinition(
            name="table",
            shape=BoxDefinition(
                size=(self.table_width, self.table_length, self.table_height),
            ),
            mass=100.0, # kg
            inertia=InertiaDefinition(
                ixx=10.0,
                iyy=10.0,
                izz=10.0,
            ),
            color=np.array([0.1, 0.1, 0.1, 0.5]),
        )
        table_urdf_path = DEFAULT_BROM_MODELS_DIR + "/table/table.urdf"
        table_defn.write_to_file(table_urdf_path)

        # Add the table to the production
        table_model_index = Parser(self.plant).AddModels(table_urdf_path)[0]

        # Weld the table to the world frame
        table_pose = RigidTransform(
            RollPitchYaw(0.0, 0.0, 0.0).ToQuaternion(),
            np.array([0.0, 0.6, 0.0]),
        )
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("table_base_link", table_model_index),
            table_pose,
        )

    def add_test_tube_holders(self):
        """
        *Description*
        
        This method adds the test tube holders to the production.
        The test tube holder is fixed at pose ``self.pose_WorldHolder``
        with respect to the world origin.
        """
        # Setup
        test_tube_urdf1 = str(
            impresources.files(robots) / "models/test_tube_holder/test_tube_holder.urdf"
        )

        # Use Drakeify my urdf to create the test tube holders
        new_test_tube_holder_urdf1 = drakeify_my_urdf(test_tube_urdf1)

        self.test_tube_holder1 = Parser(self.plant).AddModels(
            str(new_test_tube_holder_urdf1)
        )[0]

        # Weld the test tube holders to the world frame
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("test_tube_holder_base_link", self.test_tube_holder1),
            self.pose_WorldHolder,
        )


    def add_cast_and_build(
        self,
        cast: List[Tuple[Role, Performer]] = [],
        with_watcher: bool = False,
    ) -> Tuple[Diagram, Context]:
        """
        *Description*
        
        Modifies the normal add_cast_and_build, so that
        we share the context of the plant with the appropriate
        parts of the system.

        In addition to using the parent class's implementation of add_cast_and_build, this:
        - Configures the collision filter to avoid spurious collisions between the cupboard's 
        various parts and itself
        - Insert the motion planner role into the diagram and connect it properly
        
        *Parameters*

        cast: List[Tuple[Role, Performer]], optional
            The main cast members that we would like to assign to the production.
            Default is None.
            TODO(Kwesi): Will using the default lead to errors? Should we even allow that?
        
        with_watcher: bool, optional
            A Boolean that determines whether to add a watcher to the diagram.
            Default is False.
        
        *Returns*

        diagram: pydrake.systems.framework.Diagram
            The diagram that represents the fully assembled cast, all connected together.

        diagram_context: pdrake.systems.framework.Context
            The context for the diagram (used by the diagram watcher to monitor the system, if defined).

        .. note::

            This method returns a `pydrake.systems.framework.Diagram` and its associated `pydrake.systems.framework.Context`.
            When constructing a Drake simulation of this, you should use these objects to create the `pydrake.systems.analysis.Simulator`.
            
        """
        diagram, diagram_context = super().add_cast_and_build(
            main_cast_members=cast,
            with_watcher=with_watcher,
        )

        # Configure the scene graph for collision detection
        self.configure_collision_filter(
            diagram.GetSubsystemContext(
                self.scene_graph, diagram_context,
            )
        )

        # Connect arm controller to the appropriate plant_context
        self.station.arm_controller.plant_context = diagram.GetSubsystemContext(
            self.station.arm_controller.plant, diagram_context,
        )

        for role_ii, performer_ii in cast:
            if role_ii.name == "OfflineMotionPlanner":
                performer_ii.set_internal_root_context(
                    diagram_context
                )

        return diagram, diagram_context
    
    def configure_collision_filter(self, scene_graph_context: Context):
        """
        *Description*
        
        This method configures the collision filter, so that:
        
        - self collisions between the shelf's pieces,
        
        are ignored during simulation.

        *Parameters*

        scene_graph_context: pydrake.systems.framework.Context
            The context of the scenegraph for the constructed production (i.e., the diagram
            for the built Production).
        """
        # Setup
        scene_graph = self.scene_graph
        shelf_model_index = self.shelf_model_index

        # Ignore self collisions of the shelf, by:
        # - Getting the model instance index of the shelf
        # - Collecting the Geometry IDs of all the geometries in the shelf
        shelf_geometry_ids = []
        for body_index in self.plant.GetBodyIndices(shelf_model_index):
            # Get the geometry IDs
            shelf_geometry_ids.extend(
                self.plant.GetCollisionGeometriesForBody(
                    self.plant.get_body(body_index)
                )
            )
            # print(self.plant.get_body(body_index))

        self.shelf_geometry_ids = shelf_geometry_ids

        # Apply the collision filter
        scene_graph.collision_filter_manager(scene_graph_context).Apply(
            CollisionFilterDeclaration().ExcludeWithin(
                GeometrySet(self.shelf_geometry_ids)
            )
        )

        # # Ignore collisions between the robot links
        # # TODO: Actually implement this for adjacent links? Or is this not necessary?
        # arm_geometry_ids = []
        # for body_index in self.plant.GetBodyIndices(self.arm):
        #     arm_geometry_ids.extend(
        #         self.plant.GetCollisionGeometriesForBody(
        #             self.plant.get_body(body_index)
        #         )
        #     )

        # self.arm_geometry_ids = arm_geometry_ids

        # # Apply the collision filter
        # scene_graph.collision_filter_manager(scene_graph_context).Apply(
        #     CollisionFilterDeclaration().ExcludeWithin(
        #         GeometrySet(self.arm_geometry_ids)
        #     )
        # )
    
    def easy_cast_and_build(
        self,
        planning_algorithm: Callable[
            [np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],
            Tuple[MotionPlan, bool],
        ],
        with_watcher: bool = False,
    ) -> Tuple[Diagram, Context]:
        """
        *Description*
        
        This function is used to easily cast and build the production.

        *Parameters*

        planning_algorithm: Callable[[np.ndarray, np.ndarray, Callable[[np.ndarray], bool]], Tuple[MotionPlan, bool]
            The algorithm that produces a path from the start configuration to goal configuration
            using the collision checker for the scene.

        with_watcher: bool, optional
            A Boolean that determines whether to add a watcher to the diagram.
            Default is False.
        
        *Returns*

        diagram: pydrake.systems.framework.Diagram
            The diagram that represents the fully assembled cast, all connected together.

        diagram_context: pdrake.systems.framework.Context
            The context for the diagram (used by the diagram watcher to monitor the system, if defined).

        .. note::

            This method returns a `pydrake.systems.framework.Diagram` and its associated `pydrake.systems.framework.Context`.
            When constructing a Drake simulation of this, you should use these objects to create the `pydrake.systems.analysis.Simulator`.
        """
        # Setup

        # Use Base class implementation to start
        diagram, diagram_context = super().easy_cast_and_build(
            planning_algorithm,
            with_watcher=with_watcher,
        )

        # print("completed cast and build.")

        # Configure the scene graph for collision detection
        self.configure_collision_filter(
            diagram.GetSubsystemContext(
                self.scene_graph, diagram_context,
            )
        )

        # Connect arm controller to the appropriate plant_context
        self.station.arm_controller.plant_context = diagram.GetSubsystemContext(
            self.station.arm_controller.plant, diagram_context,
        )

        self.performers[0].set_internal_root_context(
            diagram_context
        )

        return diagram, diagram_context

    @property
    def goal_configuration(self) -> np.ndarray:
        """
        *Description*

        Goal configuration of the robot in the production.

        Depending on the inputs, this may be computed the first time it is called.

        *Returns*

        q_goal: np.ndarray of shape (6,)
            The goal configuration of the robot.
        """
        # Setup
        hardcoded_robot_joint_names = [
            'ur10e-test_shoulder_pan_joint_q',
            'ur10e-test_shoulder_lift_joint_q',
            'ur10e-test_elbow_joint_q',
            'ur10e-test_wrist_1_joint_q',
            'ur10e-test_wrist_2_joint_q',
            'ur10e-test_wrist_3_joint_q',
        ]

        # Retrieve goal_configuraiton value
        if self._goal_config is None:
            beaker_to_goal_translation = np.array([+0.0, 0.2, 0.0]) 
            beaker_to_goal_orientation = RollPitchYaw(0., 0., 0.0).ToQuaternion()
            X_BeakerGoal = RigidTransform(  
                beaker_to_goal_orientation,
                beaker_to_goal_translation,
            )

            pose_WorldGoal = self.pose_WorldBeaker.multiply(X_BeakerGoal)

            # Use Inverse Kinematics to get the goal configuration of the robot
            self._goal_config = self.solve_pose_ik_problem(
                pose_WorldGoal,
                robot_joint_names=hardcoded_robot_joint_names,
            )

            return self._goal_config
        else:
            return self._goal_config

    @property
    def goal_pose(self) -> RigidTransform:
        """
        *Description*
        
        Get the goal pose of the end effector frame.

        *Returns*
        
        goal_pose: RigidTransform
            The goal pose of the robot.
        """
        # Setup

        # Algorithm
        if self._goal_pose is not None:
            # If the goal pose is already defined, return it
            return self._goal_pose
        elif (self._goal_pose is None) and (self._goal_config is None):
            # If we have no guidance on where goal is, then we will use the default goal pose
            beaker_to_goal_translation = np.array([+0.0, 0.2, 0.0]) 
            beaker_to_goal_orientation = RollPitchYaw(0., 0., 0.0).ToQuaternion()
            X_BeakerGoal = RigidTransform(  
                beaker_to_goal_orientation,
                beaker_to_goal_translation,
            )

            pose_WorldGoal = self.pose_WorldBeaker.multiply(X_BeakerGoal)
            self._goal_pose = pose_WorldGoal

            return self._goal_pose
        
        elif (self._goal_pose is None) and (self._goal_config is not None):
            # Use the goal configuration to get the goal pose
            # Using a "forward kinematics solver"
            self._goal_pose = self.solve_forward_kinematics_problem_for_arm(self._goal_config)
            return self._goal_pose
        
        else:
            raise ValueError(
                f"Unexpected behavior. This should never happen."
            )

    @property
    def id(self) -> ProductionID:
        """
        Always returns ``ProductionID.kChemLab1``.
        """
        return ProductionID.kChemLab1
        
    def solve_forward_kinematics_problem_for_arm(
        self,
        robot_joint_positions: np.ndarray,
        target_frame_name: str = "ft_frame",
    ) -> RigidTransform:
        """
        *Description*
        
        This method solves the forward kinematics problem for the UR10e arm
        by itself.

        *Parameters*

        robot_joint_positions: np.ndarray of shape (6,)
            The joint configuration of the robot in the production.

        target_frame_name: str, optional
            The frame that we compute the pose of with respect to the world origin frame.
            Default is "ft_frame"
        
        *Returns*
        
        pose_WorldTarget: RigidTransform
            The end effector pose of the robot expressed in the world frame.
        """
        # Setup

        # Create shadow plant and populate it with the UR10e arm
        shadow_station = KinematicUR10eStation(
            time_step=self.time_step,
            meshcat_port_number=None,
        )
        shadow_station.Finalize()

        # Use station's plant to solve the forward kinematics problem
        shadow_plant = shadow_station.plant
        
        temp_context = shadow_plant.CreateDefaultContext()
        shadow_plant.SetPositions(
            temp_context,
            shadow_station.arm,
            robot_joint_positions,
        )

        # Get the pose of the end effector
        end_effector_pose = shadow_plant.EvalBodyPoseInWorld(
            temp_context,
            shadow_plant.GetBodyByName(target_frame_name),
        )

        return end_effector_pose

    @property
    def start_configuration(self):
        """
        *Description*

        Get the start configuration.

        This may be computed using inverse kinematics on the first time it is requested,
        depending on the construction of the production.
        
        *Returns*

        q_start: np.ndarray of shape(6,)
            The starting configuration of the production's robot.
        """
        # Setup
        hardcoded_robot_joint_names = [
            'ur10e-test_shoulder_pan_joint_q',
            'ur10e-test_shoulder_lift_joint_q',
            'ur10e-test_elbow_joint_q',
            'ur10e-test_wrist_1_joint_q',
            'ur10e-test_wrist_2_joint_q',
            'ur10e-test_wrist_3_joint_q',
        ]

        # Algorithm
        if self._start_config is not None:
            return self._start_config
        elif self._start_pose is not None:
            # Use the start pose to get the start configuration
            # Using the IK solver (potentially buggy because default ik problem ignores obstacles)
            return self.solve_pose_ik_problem(
                self._start_pose,
                robot_joint_names=hardcoded_robot_joint_names,
            )
        else:
            raise NotImplementedError(
                "This function should be implemented by the subclass."
            )

    @property
    def start_pose(self) -> RigidTransform:
        """
        *Description*
        
        Get the start pose.
        
        *Returns*

        pose_WorldStart: RigidTransform
            The start pose of the robot's end effector frame with respect to the world frame.
        """
        # Setup

        # Algorithm
        if self._start_pose is not None:
            # If the start pose is already defined, return it
            return self._start_pose
        
        elif (self._start_pose is None) and (self._start_config is None):
            # If we have no guidance on how to start, then we will use the default start pose
            holder_to_start_translation = np.array([+0.0, 0.2, 0.025])
            holder_to_start_orientation = Quaternion(1, 0, 0, 0)
            X_HolderStart = RigidTransform(
                holder_to_start_orientation,
                holder_to_start_translation,
            )
            pose_WorldStart = self.pose_WorldHolder.multiply(X_HolderStart)
            self._start_pose = pose_WorldStart
            return self._start_pose
        
        elif (self._start_pose is None) and (self._start_config is not None):
            # Use the start configuration to get the starting pose
            # Using a "forward kinematics solver"
            self._start_pose = self.solve_forward_kinematics_problem_for_arm(self._start_config)
            return self._start_pose
        
        else:
            raise ValueError(
                f"Unexpected behavior. This should never happen."
            )


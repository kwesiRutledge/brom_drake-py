from importlib import resources as impresources
from typing import List, Tuple, Callable

import networkx as nx
import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.common.value import AbstractValue
from pydrake.geometry import GeometrySet, CollisionFilterDeclaration
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, Context
from pydrake.systems.primitives import ConstantValueSource

# Internal Imports
import brom_drake.robots as robots
from brom_drake.motion_planning.systems.open_loop_dispensers.open_loop_plan_dispenser import OpenLoopPlanDispenser
from brom_drake.stations.kinematic import UR10eStation as KinematicUR10eStation
from brom_drake.productions import ProductionID
from brom_drake.productions.roles import Role
from brom_drake.productions.types.motion_planning import KinematicMotionPlanningProduction
from brom_drake.utils import Performer, GroundShape, AddGround, MotionPlan


class ShelfPlanning1(KinematicMotionPlanningProduction):
    """
    *Description*

    This production contains a simple motion planning task
    where we attempt to reach to different places around the shelfs
    of a bookcase.

    This is a Kinematic motion planning production, which means that the
    the robot's actuators will not be driven. Instead, the robot's links will
    be reset to "exactly" the configurations in the motion plan. This is not realistic
    but tests the correctness of the plan without worrying about the control problem
    of moving the arm.

    *Parameters*

    time_step: float, optional
        The time step for the production, by default 1e-3.

    shelf_pose: RigidTransform, optional
        The pose of the shelf in the world frame, by default None.

    meshcat_port_number: int, optional
        The port number for meshcat visualization, by default 7001.

    plan_execution_speed: float, optional
        The speed at which to execute the motion plan, by default 0.2

    *Usage*

    You may use this in a similar way to all other KinematicMotionPlanningProduction
    objects. ::

        # Other imports above...
        from brom_drake.motion_planning.algorithms.rrt import RRTConnectPlannerConfig, RRTConnectPlanner
        from brom_drake.productions.motion_planning.offline import ShelfPlanning1

        # Define the goal pose
        easy_goal_position = np.array([+0.0, 1.0, 1.05])
        goal_orientation = RollPitchYaw(np.pi / 2.0, np.pi / 2.0, 0.0).ToQuaternion()
        goal_pose = RigidTransform(goal_orientation, easy_goal_position)

        # Create the production
        production = ShelfPlanning1(
            meshcat_port_number=meshcat_port_number, # Use None for CI
            goal_pose=goal_pose,
        )

        # Create a planner object which will be used to plan the motion
        config = RRTConnectPlannerConfig(
            steering_step_size=0.01,
            prob_sample_goal=0.05,
            max_iterations=int(1e5),
            convergence_threshold=1e-3,
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
        print(f"Expected end time of the planned trajectory: {planned_trajectory.end_time()}")
        # return
        simulator.AdvanceTo(planned_trajectory.end_time()+1.0)


    """
    def __init__(
        self,
        time_step: float = 1e-3,
        shelf_pose: RigidTransform = None, # The pose of the shelf
        meshcat_port_number: int = 7001, # Usually turn off for CI (i.e., make it None)
        plan_execution_speed: float = 0.2,
        **kwargs,
    ):
        """
        *Description*

        Constructor for ShelfPlanning1.
        
        This:
        
        - Stores all inputs in the appropriate places
        - Selects the default shelf_pose, if none is given
        - Sets the cupboard joints to be in the "open" configuration
        - constructs the KinematicUR10eStation used in testing
        - Sets the names of the plant and scene graph to use the ShelfPlanning tag

        *Parameters*

        time_step: float, optional
            The time step for the production, by default 1e-3.

        shelf_pose: RigidTransform, optional
            The pose of the shelf in the world frame, by default None.

        meshcat_port_number: int, optional
            The port number for meshcat visualization, by default 7001.

        plan_execution_speed: float, optional
            The speed at which to execute the motion plan, by default 0.2        
        """
        super().__init__(**kwargs)

        self.time_step = time_step
        self.shelf_pose = shelf_pose
        self.meshcat_port_number = meshcat_port_number
        self.plan_execution_speed = plan_execution_speed

        # Input Processing
        if self.shelf_pose is None:
            self.shelf_pose = RigidTransform(
                RollPitchYaw(0.0, 0.0, +np.pi/2.0).ToQuaternion(),
                np.array([0.0, 1.0, 0.65]),
            )

        self.desired_cupboard_positions = np.array([0.2, 0.3])

        # Create containers for meshcat and other values we will set later
        self.meshcat = None
        self.plan_dispenser = None

        # Set station
        self.station = KinematicUR10eStation(
            time_step=self.time_step,
            meshcat_port_number=self.meshcat_port_number,
        )
        self.arm = self.station.arm
        self.robot_model_idx_ = self.station.arm

        # Set Names of Plant and scene graph
        self.plant = self.station.plant
        self.scene_graph = self.station.scene_graph

        self.plant.set_name(f"Shelf1_Production_Plant")
        self.scene_graph.set_name(f"Shelf1_Production_SceneGraph")

        # Create variables for the models we have
        self.shelf_model_index = None
        self.geometry_ids_to_ignore = []

    def add_supporting_cast(self):
        """
        *Description*

        Modifies the plant and the builder by:
        - Adding UR10e station to the plant
        - Adding shelf to the plant
        - Adding a "ground" to the plant
        - Adding features required by the KinematicMotionPlanningProductionFeatures
          + Robot's ModelInstanceIndex Source
          + Motion Planning Components
          + Start and Goal Pose sources
        - Setting initial configuration of the arm for the plant
        
        """
        # Call the parent class method
        super().add_supporting_cast()

        # Setup
        self.add_ur10e_station() # Use the UR10e station's plant + scene graph for all other objects

        # Add obstacles
        self.add_shelf(self.plant)

        # Add the ground as well as the start and goal locations
        AddGround(self.plant)
        self.station.Finalize()

        # Add The Motion Planning Components (e.g., the interpolator)
        self.add_robot_source_system()

        self.add_motion_planning_components()

        self.add_start_and_goal_sources_to_builder()

        # Add initial conditions for the arm
        self.initial_condition_manager.add_initial_configuration(
            model_instance_index=self.arm,
            configuration=self.start_configuration,
        )
        # The initial condition will be set for the plant when we build the production.

        # Print message to user
        print("Added all secondary cast members to the builder.")

    def add_motion_planning_components(self):
        """
        *Description*

        Connects the plan dispenser's output port (i.e., the reference signal for the
        arm's joint controller) to the input of the UR10eStation.

        """
        # Setup
        n_actuated_dof = self.plant.num_actuated_dofs()

        # Add the Plan Dispenser and connect it to the station
        self.plan_dispenser = self.builder.AddSystem(
            OpenLoopPlanDispenser(n_actuated_dof, self.plan_execution_speed)
        )

        self.builder.Connect(
            self.plan_dispenser.GetOutputPort("point_in_plan"),
            self.station.GetInputPort("desired_joint_positions"),
        )

    def add_shelf(self, plant: MultibodyPlant):
        """
        *Description*

        Adds a model of the cupboard to the production's plant.

        *Parameter*

        plant: pydrake.multibody.plant.MultibodyPlant
            The plant that we want to add the cupboard to.
        """
        # Setup
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
            self.shelf_pose,
        )

    def add_ur10e_station(self):
        """
        *Description*
        
        Add the Kinematic UR10e station to the production's plant for simple motion planning.
        """
        # Setup

        # Add station
        self.builder.AddSystem(self.station)

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

        TODO(kwesi): Consider making this a standard part of the
        OfflineKinematicMotionPlanningProduction class.

        *Parameters*

        cast: List[Tuple[Role, Performer]], optional
            The cast to add to the production, by default [].

        with_watcher: bool, optional
            A Boolean that determines whether to add a watcher to the diagram, by default False.

        *Returns*

        diagram: Diagram
            The built diagram.

        diagram_context: Context
            The context for the built diagram.
            This context will be used to initialize the watcher.
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
        
        - self collisions between the shelf's pieces, and
        - self collisions between the arm's parts,
        
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

        # Ignore collisions between the robot links
        # TODO: Actually implement this for adjacent links? Or is this not necessary?
        arm_geometry_ids = []
        for body_index in self.plant.GetBodyIndices(self.arm):
            arm_geometry_ids.extend(
                self.plant.GetCollisionGeometriesForBody(
                    self.plant.get_body(body_index)
                )
            )

        self.arm_geometry_ids = arm_geometry_ids

        # Apply the collision filter
        scene_graph.collision_filter_manager(scene_graph_context).Apply(
            CollisionFilterDeclaration().ExcludeWithin(
                GeometrySet(self.arm_geometry_ids)
            )
        )

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

        # Set the initial positions of the arm
        # self.station.plant.SetPositions(
        #     self.station.plant.GetMyMutableContextFromRoot(diagram_context),
        #     self.arm,
        #     self.start_configuration,
        # )

        return diagram, diagram_context

    @property
    def goal_pose(self) -> RigidTransform:
        """
        *Description*

        Get the goal pose. 
        
        *Returns*
        
        pose_WorldGoal: RigidTransform
            The goal pose.
        """

        # Algorithm
        if self._goal_pose is not None:
            # If we have already defined the goal pose, then return it
            return self._goal_pose
        elif (self._goal_pose is None) and (self._goal_config is None):
            # If we have no guidance on how to start, then we will use a default
            goal_position = np.array([+0.0, 1.0, 0.6])
            goal_orientation = RollPitchYaw(np.pi / 2.0, np.pi / 2.0, 0.0).ToQuaternion()
            self._goal_pose = RigidTransform(goal_orientation, goal_position)
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
        """Always returns ``ProductionID.kShelfPlanning1``"""
        return ProductionID.kShelfPlanning1

    def solve_forward_kinematics_problem_for_arm(
        self,
        robot_joint_positions: np.ndarray,
        target_frame_name: str = "ft_frame",
    ) -> RigidTransform:
        """
        *Description*
        
        This method solves the forward kinematics problem for the UR10e arm
        by itself.

        TODO(Kwesi): Attempt to make this a utility (it is duplicated multiple times!)

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
    def start_pose(self) -> RigidTransform:
        """
        *Description*
        
        Get the start pose.
        
        *Returns*

        pose_WorldStart: RigidTransform
            The start pose of the robot's end effector frame with respect to the world frame.
        """
        # Setup

        if self._start_pose is not None:
            # If we have already defined the start pose, then return it
            return self._start_pose
        elif (self._start_pose is None) and (self._start_config is None):
            # If we have no guidance on how to start, then we will use a default
            start_position = np.array([+0.3, 0.1, 1.2])
            start_orientation = Quaternion(1, 0, 0, 0)
            self._start_pose = RigidTransform(start_orientation, start_position)
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
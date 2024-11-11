from importlib import resources as impresources
from typing import Tuple, Callable

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
from brom_drake.example_helpers import AddGround
from brom_drake.motion_planning.systems.open_loop_plan_dispenser import OpenLoopPlanDispenser
from brom_drake.robots.stations.kinematic import UR10eStation as KinematicUR10eStation
from brom_drake.scenes import SceneID
from brom_drake.scenes.roles import Role
from brom_drake.scenes.types.motion_planning import OfflineMotionPlanningScene
from brom_drake.utils import Performer


class ShelfPlanningScene(OfflineMotionPlanningScene):
    def __init__(
        self,
        time_step=1e-3,
        shelf_pose: RigidTransform = None, # The pose of the shelf
        meshcat_port_number: int = 7001, # Usually turn off for CI (i.e., make it None)
        plan_execution_speed: float = 0.2,
        **kwargs,
    ):
        super().__init__(**kwargs)

        self.time_step = time_step
        self.shelf_pose = shelf_pose
        self.meshcat_port_number = meshcat_port_number
        self.plan_execution_speed = plan_execution_speed

        # Input Processing
        if self.shelf_pose is None:
            shelf_orientation = Quaternion(1, 0, 0 , 0)
            shelf_position = np.array([-0.4, 0.8, 0.0])
            self.shelf_pose = RigidTransform(
                RollPitchYaw(0.0, 0.0, +np.pi/2.0).ToQuaternion(),
                np.array([0.0, 1.0, 0.6]),
            )

        self.desired_cupboard_positions = np.array([0.0, 0.0])

        # Create containers for meshcat and other values we will set later
        self.meshcat = None
        self.plan_dispenser = None

        # Set station
        self.station = KinematicUR10eStation(
            time_step=self.time_step,
            meshcat_port_number=self.meshcat_port_number,
        )
        self.arm = self.station.arm

        # Set Names of Plant and scene graph
        self.plant = self.station.plant
        self.scene_graph = self.station.scene_graph

        self.plant.set_name(f"ShelfScene_Plant")
        self.scene_graph.set_name(f"ShelfScene_SceneGraph")

        # Create variables for the models we have
        self.shelf_model_index = None
        self.geometry_ids_to_ignore = []

    def add_all_secondary_cast_members_to_builder(self):
        """
        Add all secondary cast members to the builder.
        :return:
        """
        # Setup
        self.add_ur10e_station() # Use the UR10e station's plant + scene graph for all other objects

        # Add obstacles
        self.add_shelf(self.plant)

        # Add the ground as well as the start and goal locations
        AddGround(self.plant)
        self.add_start_and_goal_to_this_plant(self.plant)

        self.station.Finalize()

        # Add The Motion Planning Components (e.g., the interpolator)
        self.add_start_source_system()
        self.add_goal_source_system()
        self.add_robot_source_system()

        self.add_motion_planning_components()

        # Connect motion planning components to station
        # self.connect_motion_planning_components()

        # Connect to Meshcat
        # if self.use_meshcat:
        #     self.meshcat = Meshcat(port=7001)  # Object provides an interface to Meshcat
        #     m_visualizer = MeshcatVisualizer(self.meshcat)
        #     m_visualizer.AddToBuilder(builder, self.scene_graph, self.meshcat)

        # Print message to user
        print("Added all secondary cast members to the builder.")

    def add_motion_planning_components(self):
        """
        Add the motion planning components to the builder.
        :return:
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

    def add_robot_source_system(self):
        """
        Description
        -----------
        This method adds a source for providing the motion planner
        with the model index for the robot that we are trying to control.
        :return:
        """
        # Setup

        # Create AbstractValueSource
        robot_source_system = ConstantValueSource(
            AbstractValue.Make(self.station.arm)
        )
        robot_source_system.set_name("robot_model_index_source")

        self.builder.AddSystem(robot_source_system)

    def add_shelf(self, plant: MultibodyPlant):
        """
        Add the shelf to the scene.
        :return:
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
        Add the Kinematic UR10e station to the scene for simple motion planning.
        :return:
        """
        # Setup

        # Add station
        self.builder.AddSystem(self.station)

    def cast_scene_and_build(
        self,
        cast: Tuple[Role, Performer] = [],
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        Modifies the normal cast_scene_and_build, so that
        we share the context of the plant with the appropriate
        parts of the system.
        :param cast:
        :return:
        """
        diagram, diagram_context = super().cast_scene_and_build(cast=cast)

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
        Description
        -----------
        This method configures the collision filter for the scene.
        :param scene_graph_context:
        :return:
        """
        # Setup
        scene_graph = self.scene_graph
        shelf_model_index = self.shelf_model_index

        # Ignore self collisions of the shelf, by:
        # - Getting the model instance index of the shelf
        # - Collecting the Geometry IDs of all the geometries in the shelf
        geometry_ids = []
        for body_index in self.plant.GetBodyIndices(shelf_model_index):
            # Get the geometry IDs
            geometry_ids.extend(
                self.plant.GetCollisionGeometriesForBody(
                    self.plant.get_body(body_index)
                )
            )
            print(self.plant.get_body(body_index))

        self.geometry_ids_to_ignore = geometry_ids

        # Apply the collision filter
        scene_graph.collision_filter_manager(scene_graph_context).Apply(
            CollisionFilterDeclaration().ExcludeWithin(
                GeometrySet(self.geometry_ids_to_ignore)
            )
        )

    def connect_motion_planning_components(
        self,
    ):
        """
        Description
        -----------
        This method connects the motion planning components directly to the
        kinematic controller.
        :return:
        """

        # Connect the plan dispenser to the station
        pass

    def easy_cast_and_build(
        self,
            planning_algorithm: Callable[
                [np.ndarray, np.ndarray, Callable[[np.ndarray], bool]],
                Tuple[nx.DiGraph, np.ndarray],
            ],
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        This function is used to easily cast and build the scene.
        :param planning_algorithm: The algorithm that we will use to
        :return:
        """
        # Setup

        # Use Base class implementation to start
        diagram, diagram_context = super().easy_cast_and_build(planning_algorithm)

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
    def goal_pose(self):
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        goal_position = np.array([+0.1, 1.0, 0.55]) # np.array([+0.5, 0.7, 0.65])
        goal_orientation = RollPitchYaw(np.pi/2.0, np.pi/2.0, 0.0).ToQuaternion()
        return RigidTransform(goal_orientation, goal_position)

    @property
    def id(self) -> SceneID:
        return SceneID.kShelfPlanning1

    @property
    def start_pose(self):
        """
        Get the start pose. This should be defined by the subclass.
        :return:
        """
        start_position = np.array([+0.3, 0.1, 1.2])
        start_orientation = Quaternion(1, 0, 0, 0)
        return RigidTransform(start_orientation, start_position)
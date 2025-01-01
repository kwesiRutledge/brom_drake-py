
from importlib import resources as impresources
import networkx as nx
import numpy as np
from pydrake.all import (
    MultibodyPlant, Parser,
    CollisionFilterDeclaration, GeometrySet,
)
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.systems.framework import Diagram, Context
from typing import Callable, Tuple

from brom_drake.directories import DEFAULT_BROM_MODELS_DIR
from brom_drake.file_manipulation.urdf import drakeify_my_urdf
from brom_drake.file_manipulation.urdf.shapes.box import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import SimpleShapeURDFDefinition, \
    InertiaDefinition
from brom_drake.motion_planning.systems.open_loop_dispensers.open_loop_plan_dispenser import OpenLoopPlanDispenser
import brom_drake.robots as robots
from brom_drake.robots.stations.kinematic import UR10eStation as KinematicUR10eStation
from brom_drake.productions import ProductionID
from brom_drake.productions.roles import Role
from brom_drake.productions.types import KinematicMotionPlanningProduction
from brom_drake.utils import Performer, AddGround, MotionPlan


class ChemLab1(KinematicMotionPlanningProduction):
    """
    Description:
        This production is the first in the chemistry lab series.
        It is used to test the motion planning capabilities of the robot
        in a chemistry lab setting with minimal constraints.
    """
    def __init__(
        self,
        time_step=1e-3,
        meshcat_port_number: int = 7000,
        plan_execution_speed: float = 0.2,
        table_length: float = 0.6,
        table_width: float = 2.0,
        table_height: float = 0.1,
        shelf_pose: RigidTransform = None,
        **kwargs,
    ):
        """
        Description:
            Constructor for the ChemLab1Scene class.
        :param meshcat_port_number:
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
        )
        self.arm = self.station.arm
        self.robot_model_idx_ = self.arm

        # Set Names of Plant and scene graph
        self.plant = self.station.plant
        self.scene_graph = self.station.scene_graph

        self.plant.set_name(f"ChemLab1_Production_Plant")
        self.scene_graph.set_name(f"ChemLab1_Production_SceneGraph")

        # Define placeholder variables for models
        self.test_tube_holder1 = None

    def add_supporting_cast(self):
        """
        Description
        -----------
        This method adds all secondary cast members to the builder.
        The secondary cast members in the production are the:
        - Table, where the robot exists
        - Test Tube Holders
        - Component which share's the robot model reference
        - Motion Planning components (e.g., dispensers, etc.)
        - Start and Goal sources
        :return:
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

    def add_beaker(self):
        """
        Description
        -----------
        This method adds the beaker to the production.
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

    def add_shelf(self):
        """
        Add the shelf to the production.
        :return:
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
        Description
        -----------
        This method adds the table to the production.
        The table will be a simple shape that the robot will interact with.
        :return:
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
        Description
        -----------
        This method adds the test tube holders to the production.
        :return:
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
        cast: Tuple[Role, Performer] = [],
        with_watcher: bool = False,
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        Modifies the normal add_cast_and_build, so that
        we share the context of the plant with the appropriate
        parts of the system.
        :param cast:
        :param with_watcher: A Boolean that determines whether to add a watcher to the diagram.
        :return:
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
        Description
        -----------
        This method configures the collision filter for the production.
        :param scene_graph_context:
        :return:
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
        Description
        -----------
        This function is used to easily cast and build the production.
        :param planning_algorithm: The algorithm that we will use to
        plan the motion.
        :param with_watcher: A Boolean that determines whether to add a watcher to the diagram.
        :return:
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

        return diagram, diagram_context

    @property
    def goal_configuration(self):
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        if self.goal_config_ is None:
            beaker_to_goal_translation = np.array([+0.0, 0.2, 0.0]) 
            beaker_to_goal_orientation = RollPitchYaw(0., 0., 0.0).ToQuaternion()
            X_BeakerGoal = RigidTransform(  
                beaker_to_goal_orientation,
                beaker_to_goal_translation,
            )

            pose_WorldGoal = self.pose_WorldBeaker.multiply(X_BeakerGoal)

            # Use Inverse Kinematics to get the goal configuration of the robot
            self.goal_config_ = self.solve_pose_ik_problem(
                pose_WorldGoal,
            )

            return self.goal_config_
        else:
            return self.goal_config_

    @property
    def goal_pose(self) -> RigidTransform:
        """
        Description
        -----------
        Get the goal pose. This should be defined by the subclass.
        """
        # Setup

        # Algorithm
        if self.goal_pose_ is None:
            beaker_to_goal_translation = np.array([+0.0, 0.2, 0.0]) 
            beaker_to_goal_orientation = RollPitchYaw(0., 0., 0.0).ToQuaternion()
            X_BeakerGoal = RigidTransform(  
                beaker_to_goal_orientation,
                beaker_to_goal_translation,
            )

            pose_WorldGoal = self.pose_WorldBeaker.multiply(X_BeakerGoal)
            self.goal_pose_ = pose_WorldGoal

            return self.goal_pose_
        else:
            return self.goal_pose_

    @property
    def id(self) -> ProductionID:
        return ProductionID.kShelfPlanning1
        
    # TODO(kwesi): Implement start_configuration method.

    @property
    def start_pose(self) -> RigidTransform:
        """
        Get the start pose. This should be defined by the subclass.
        :return:
        """
        # Setup

        if self.start_pose_ is None:
            # Define Start Pose
            holder_to_start_translation = np.array([+0.0, 0.2, 0.025])
            holder_to_start_orientation = Quaternion(1, 0, 0, 0)
            X_HolderStart = RigidTransform(
                holder_to_start_orientation,
                holder_to_start_translation,
            )
            pose_WorldStart = self.pose_WorldHolder.multiply(X_HolderStart)
            self.start_pose_ = pose_WorldStart
            return self.start_pose_
        else:
            return self.start_pose_

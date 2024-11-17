
from importlib import resources as impresources
import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.systems.framework import Diagram, Context
from typing import Tuple

from brom_drake.DiagramWatcher.constants import DEFAULT_BROM_MODELS_DIR
from brom_drake.file_manipulation.urdf import drakeify_my_urdf
from brom_drake.file_manipulation.urdf.simple_shape_urdfs.shape_definition import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_shape_urdfs.urdf_definition import SimpleShapeURDFDefinition, \
    InertiaDefinition
import brom_drake.robots as robots
from brom_drake.robots.stations.kinematic import UR10eStation as KinematicUR10eStation
from brom_drake.scenes import SceneID
from brom_drake.scenes.roles import Role
from brom_drake.scenes.types import OfflineMotionPlanningScene
from brom_drake.utils import Performer


class ChemLab1Scene(OfflineMotionPlanningScene):
    """
    Description:
        This scene is the first in the chemistry lab series.
    """
    def __init__(
        self,
        time_step=1e-3,
        meshcat_port_number: int = 7000,
        plan_execution_speed: float = 0.2,
        table_length: float = 0.6,
        table_width: float = 2.0,
        table_height: float = 0.1
    ):
        """
        Description:
            Constructor for the ChemLab1Scene class.
        :param meshcat_port_number:
        """
        # Superclass constructor
        super().__init__()

        # Setup
        self.time_step = time_step
        self.meshcat_port_number = meshcat_port_number
        self.plan_execution_speed = plan_execution_speed
        self.table_height, self.table_width = table_height, table_width
        self.table_length = table_length

        # Set station
        self.station = KinematicUR10eStation(
            time_step=self.time_step,
            meshcat_port_number=self.meshcat_port_number,
        )
        self.arm = self.station.arm

        # Set Names of Plant and scene graph
        self.plant = self.station.plant
        self.scene_graph = self.station.scene_graph

        self.plant.set_name(f"ChemLabScene1_Plant")
        self.scene_graph.set_name(f"ChemLabScene1_SceneGraph")

        # Define placeholder variables for models
        self.test_tube_holder1 = None

    def add_all_secondary_cast_members_to_builder(self):
        """
        Description
        -----------
        This method adds all secondary cast members to the builder.
        The secondary cast members in the scene are the:
        - Table, where the robot exists
        - Test Tube Holders
        - ...
        :return:
        """

        # Setup

        # Create the table
        self.add_table()
        self.add_test_tube_holders()

        # Add the station
        self.builder.AddSystem(
            self.station,
        )

        self.station.Finalize()

        pass

    def add_table(self):
        """
        Description
        -----------
        This method adds the table to the scene.
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
            color=np.array([0.1, 0.2, 0.5, 1.0]),
        )
        table_urdf_path = DEFAULT_BROM_MODELS_DIR + "/table/table.urdf"
        table_defn.write_to_file(table_urdf_path)

        # Add the table to the scene
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
        This method adds the test tube holders to the scene.
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
        test_tube_holder_pose1 = RigidTransform(
            RollPitchYaw(np.pi/2.0, 0.0, 0.0).ToQuaternion(),
            np.array([self.table_width*0.5*0.7, 0.6+self.table_length/4., self.table_height-0.015]),
        )
        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("test_tube_holder_base_link", self.test_tube_holder1),
            test_tube_holder_pose1,
        )


    def cast_scene_and_build(
        self,
        cast: Tuple[Role, Performer] = [],
        with_watcher: bool = False,
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        Modifies the normal cast_scene_and_build, so that
        we share the context of the plant with the appropriate
        parts of the system.
        :param cast:
        :param with_watcher: A Boolean that determines whether to add a watcher to the diagram.
        :return:
        """
        diagram, diagram_context = super().cast_scene_and_build(
            cast=cast,
            with_watcher=with_watcher,
        )

        # Configure the scene graph for collision detection
        # self.configure_collision_filter(
        #     diagram.GetSubsystemContext(
        #         self.scene_graph, diagram_context,
        #     )
        # )

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

    @property
    def goal_pose(self):
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        if self.goal_pose_ is None:
            goal_position = np.array([+0.1, 1.0, 0.55])  # np.array([+0.5, 0.7, 0.65])
            goal_orientation = RollPitchYaw(np.pi / 2.0, np.pi / 2.0, 0.0).ToQuaternion()
            return RigidTransform(goal_orientation, goal_position)
        else:
            return self.goal_pose_

    @property
    def id(self) -> SceneID:
        return SceneID.kShelfPlanning1

    @property
    def start_pose(self):
        """
        Get the start pose. This should be defined by the subclass.
        :return:
        """
        if self.start_pose_ is None:
            start_position = np.array([+0.3, 0.1, 1.2])
            start_orientation = Quaternion(1, 0, 0, 0)
            return RigidTransform(start_orientation, start_position)
        else:
            return self.start_pose_
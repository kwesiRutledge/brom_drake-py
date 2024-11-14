from typing import Tuple

import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RollPitchYaw, RigidTransform
from pydrake.systems.framework import Diagram, Context

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
        pass

    def add_table(self):

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
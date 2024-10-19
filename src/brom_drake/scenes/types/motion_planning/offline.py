import os
from pathlib import Path

import numpy as np
from pydrake.geometry import SceneGraph
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import DiagramBuilder

from brom_drake.scenes.types import BaseScene
from brom_drake.scenes.roles import kOfflineMotionPlanner
from brom_drake.urdf.simple_shape_urdfs.shape_definition import SphereDefinition
from brom_drake.urdf.simple_shape_urdfs.urdf_definition import SimpleShapeURDFDefinition


class OfflineMotionPlanningScene(BaseScene):
    def __init__(self, time_step: float = 2e-3, **kwargs):
        super().__init__(**kwargs)

    def add_all_secondary_cast_members_to_builder(self):
        pass

    def add_start_and_goal_to_this_plant(self, plant: MultibodyPlant):
        """
        Description
        -----------
        Add the start and goal to this plant.
        Parameters
        ----------
        :param plant: The plant to add the start and goal to.

        Notes
        -----
        Feel free to overwrite this if you have specific sizes of the start and goal in mind,
        or if you want to add more than just spheres, etc.
        :return:

        """

        # Create a sphere for the start
        start_sphere_defn = SimpleShapeURDFDefinition(
            name="start_sphere",
            shape=SphereDefinition(radius=0.1),
            create_collision=False,
            color=np.array([0, 0.3, 0.8, 0.3]),
        )
        start_sphere_urdf_location = Path("brom/models/spheres/start.urdf")
        os.makedirs(start_sphere_urdf_location.parent, exist_ok=True)

        start_sphere_defn.write_to_file(start_sphere_urdf_location)

        # Load the start sphere into the plant and rigidly attach it at the start_pose
        start_sphere_model_idx = Parser(plant).AddModels(str(start_sphere_urdf_location))[0]
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName("start_sphere_base_link", start_sphere_model_idx),
            self.start_pose,
        )

        # Create a sphere for the goal
        goal_sphere_defn = SimpleShapeURDFDefinition(
            name="end_sphere",
            shape=SphereDefinition(radius=0.1),
            create_collision=False,
            color=np.array([0, 0.8, 0.3, 0.3]),
        )
        goal_sphere_urdf_location = Path("brom/models/spheres/end.urdf")
        os.makedirs(goal_sphere_urdf_location.parent, exist_ok=True)

        goal_sphere_defn.write_to_file(goal_sphere_urdf_location)

        # Load the start sphere into the plant and rigidly attach it at the start_pose
        goal_sphere_model_idx = Parser(plant).AddModels(str(goal_sphere_urdf_location))[0]
        plant.WeldFrames(
            plant.world_frame(),
            plant.GetFrameByName(f"{goal_sphere_defn.name}_base_link", goal_sphere_model_idx),
            self.goal_pose,
        )

    @property
    def start_pose(self) -> RigidTransform:
        """
        Get the start pose. This should be defined by the subclass.
        :return:
        """
        return None

    @property
    def goal_pose(self) -> RigidTransform:
        """
        Get the goal pose. This should be defined by the subclass.
        :return:
        """
        return None

    def suggested_roles(self):
        return [kOfflineMotionPlanner]
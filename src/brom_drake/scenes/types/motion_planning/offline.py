import os
from pathlib import Path
from typing import Tuple

import numpy as np
from pydrake.common.value import AbstractValue
from pydrake.math import RigidTransform
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.primitives import ConstantVectorSource, ConstantValueSource

from brom_drake.scenes.types import BaseScene
from brom_drake.scenes.roles import kOfflineMotionPlanner, Role
from brom_drake.file_manipulation.urdf.simple_shape_urdfs.shape_definition import SphereDefinition
from brom_drake.file_manipulation.urdf.simple_shape_urdfs.urdf_definition import SimpleShapeURDFDefinition
from brom_drake.utils import Performer


class OfflineMotionPlanningScene(BaseScene):
    def __init__(
        self,
        time_step: float = 1e-3,
        **kwargs,
    ):
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

    def add_start_source_system(self):
        """
        Add a source system for the start pose.
        :return:
        """
        # Setup
        X_start = self.start_pose
        p_start = np.hstack((
            X_start.translation(),
            X_start.rotation().ToQuaternion().wxyz()
        ))

        # Adds a source system for the start pose to the builder
        start_source_system = ConstantVectorSource(p_start)
        start_source_system.set_name("start_pose")

        self.builder.AddSystem(start_source_system)

    def add_goal_source_system(self):
        """
        Add a source system for the goal pose.
        :return:
        """
        # Setup
        X_goal = self.goal_pose
        p_goal = np.hstack((
            X_goal.translation(),
            X_goal.rotation().ToQuaternion().wxyz()
        ))

        # Adds a source system for the goal pose to the builder
        goal_source_system = ConstantVectorSource(p_goal)
        goal_source_system.set_name("goal_pose")

        self.builder.AddSystem(goal_source_system)

    def cast_scene(
        self,
        cast: Tuple[Role, Performer] = [],
    ):
        """
        Description
        -----------
        This small modification to the normal cast_scene()
        function includes a call to create the optional inputs if needed.
        :param cast:
        :return:
        """
        # Setup

        # Default
        super().cast_scene(cast=cast)

        # Create optional outputs, if needed
        for role_ii, performer_ii in cast:
            self.create_optional_outputs_if_necessary(
                role_ii, performer_ii
            )

    def create_optional_outputs_if_necessary(
        self,
        role: Role, # This role will always be the Motion Planner, won't it?
        performer: Performer,
    ):
        """
        Description
        -----------
        This method checks to see if all optional outputs were connected
        for the given role. If all of them were, then we do nothing.
        If any of the optional outputs were NOT connected,
        then we add a dummy value to replace it.

        Requirements
        ------------
        This method should only be called after the full scene has
        been built. Otherwise, this may have unexpected behavior.

        :return:
        """
        # Setup

        # Iterate through all OPTIONAL OUTPUTS
        # for assignment_ii in role.port_assignments:
        #     # Should we investigate this port?
        #     investigate_this_assignment = assignment_ii.pairing_type == PairingType.kOutput
        #     investigate_this_assignment = investigate_this_assignment and \
        #                                   (not assignment_ii.is_required)
        #
        #     if not investigate_this_assignment:
        #         continue # Skip this assignment value if it doesn't satisfy this
        #
        #     # Otherwise, check to see if we have the desired output port
        #     if performer.HasOutputPort(assignment_ii.performer_port_name):
        #         continue # If performer has the output port, then we should have already connected it; skip.
        #
        #     # If performer DOES NOT HAVE

        # TODO(kwesi): Maybe move above, general code to somewhere else?
        if role.name != "OfflineMotionPlanner":
            raise ValueError(
                f"Expected role for OfflineMotionPlanning scene to be \"OfflineMotionPlanner\";" +
                f"received {role.name}"
            )

        # Check to see if performer has the LAST assignment in the motion planning role
        last_assignment = role.port_assignments[-1]
        if last_assignment.performer_port_name != "plan_is_ready":
            raise ValueError(
                f"Expected last port assignment to be \"plan_is_ready\" assignment, but" +
                f"received an assignment with performer_port_name \"{last_assignment.performer_port_name}\""
            )

        if performer.HasOutputPort(last_assignment.performer_port_name):
            return # Do nothing; performer should already be connected

        # If the performer does not have plan_is_ready port, then
        # let's create a dummy value and connect it to the right place.
        plan_ready_source = ConstantValueSource(
            AbstractValue.Make(True),
        )
        self.builder.AddSystem(plan_ready_source)

        # Find system we want to connect it to
        systems_list = last_assignment.find_any_matching_input_targets(self.builder)
        assert len(systems_list) == 1, \
            f"Expected 1 system to have port \"{last_assignment.external_target_name}\"," + \
            f" but found {len(systems_list)} systems with that output port."
        # TODO(kwesi): Perhaps move more of these error assertions to a separate file?

        self.builder.Connect(
            plan_ready_source.get_output_port(),
            systems_list[0].GetInputPort(last_assignment.external_target_name)
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
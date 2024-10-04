from typing import Union, Tuple, List

from pydrake.systems.framework import DiagramBuilder, LeafSystem, Diagram

# Internal Imports
from brom_drake.scenes.roles import Role
from brom_drake.scenes.ids import SceneID
from brom_drake.utils import Performer

class BaseScene:
    """
    Base class for all scenes.
    """
    def __init__(self, **kwargs):
        pass

    def add_all_secondary_cast_members_to_builder(self, builder: DiagramBuilder):
        """
        Description
        -----------
        This method must be implemented by the subclass. It should add all
        elements to the builder.
        :param builder:
        :return:
        """
        pass

    def suggested_roles(self) -> List[Role]:
        """
        Description
        -----------
        This method should be implemented by the subclass.
        It should return a list of roles that are suggested for the scene.
        :return:
        """
        return []

    def fill_role(
        self,
        role: Role,
        system: Performer,
        builder: DiagramBuilder,
    ):
        """
        Description
        -----------
        This method should be implemented by the subclass. It should add the
        system to the role.
        :param role:
        :param system:
        :param builder:
        :return:
        """
        # Call the member method of the role object
        role.connect_performer_to_system(builder, system)

    def cast_scene(
        self,
        builder: DiagramBuilder,
        cast: Tuple[Role, Performer] = [],
    ):
        # Setup

        # Add all elements to the builder
        self.add_all_secondary_cast_members_to_builder(builder)

        # Fulfill each role-performer pair in the casting_call list
        for role, performer in cast:
            self.fill_role(role, performer, builder)

    def cast_scene_and_build(
        self,
        builder: DiagramBuilder,
        cast: Tuple[Role, Performer] = [],
    ) -> Diagram:
        # Setup
        self.cast_scene(builder, cast)

        # Build the diagram
        return builder.Build()

    @property
    def id(self) -> SceneID:
        """
        Description
        -----------
        The unique identifier for the scene
        :return:
        """
        return SceneID.kNotDefined
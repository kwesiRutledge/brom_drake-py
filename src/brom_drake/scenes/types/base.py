from typing import Union, Tuple, List

from pydrake.systems.framework import DiagramBuilder, LeafSystem, Diagram, Context

# Internal Imports
from brom_drake.scenes.roles import Role
from brom_drake.scenes.ids import SceneID
from brom_drake.utils import Performer

class BaseScene:
    """
    Base class for all scenes.
    """
    def __init__(self, **kwargs):
        # Create a builder for the scene
        self.builder = DiagramBuilder()

        # Create the diagram and diagram_context placeholders
        self.diagram = None
        self.diagram_context = None

    def add_all_secondary_cast_members_to_builder(self):
        """
        Description
        -----------
        This method must be implemented by the subclass. It should add all
        elements to the builder.
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
    ):
        """
        Description
        -----------
        This method should be implemented by the subclass. It should add the
        system to the role.
        :param role:
        :param system:
        :return:
        """
        # Setup
        builder = self.builder

        # Call the member method of the role object
        role.connect_performer_to_diagram(builder, system)

    def cast_scene(
        self,
        cast: Tuple[Role, Performer] = [],
    ):
        # Setup

        # Add all elements to the builder
        self.add_all_secondary_cast_members_to_builder()

        # Fulfill each role-performer pair in the casting_call list
        for role, performer in cast:
            self.fill_role(role, performer)

    def cast_scene_and_build(
        self,
        cast: Tuple[Role, Performer] = [],
    ) -> Tuple[Diagram, Context]:
        # Setup
        builder = self.builder
        self.cast_scene(cast)

        # Build the diagram
        self.diagram = builder.Build()
        self.diagram_context = self.diagram.CreateDefaultContext()

        return self.diagram, self.diagram_context

    @property
    def id(self) -> SceneID:
        """
        Description
        -----------
        The unique identifier for the scene
        :return:
        """
        return SceneID.kNotDefined
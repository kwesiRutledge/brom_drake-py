from typing import Union, Tuple, List

from pydrake.systems.framework import DiagramBuilder, LeafSystem, Diagram, Context

from brom_drake.all import add_watcher_and_build
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

        # Save a list of the performers
        self.performers = []

        # Create an extra place to save DiagramWatcher objects
        self.watcher = None

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
        role.connect_performer_ports_to(builder, system)

        # Save the performer
        self.performers.append(system)

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

    def build_scene(
        self,
        with_watcher: bool = True,
    ):
        """
        Description
        -----------
        This method builds the scene.
        It assumes that all components have been added to the builder.
        :param with_watcher: A Boolean that determines whether to add a watcher to the diagram.
        :return:
        """
        # Setup
        builder = self.builder

        # Build the diagram
        if with_watcher:
            self.watcher, self.diagram, self.diagram_context = add_watcher_and_build(builder)
        else:
            self.diagram = builder.Build()
            self.diagram_context = self.diagram.CreateDefaultContext()

        return self.diagram, self.diagram_context

    def cast_scene_and_build(
        self,
        cast: Tuple[Role, Performer] = [],
        with_watcher: bool = True,
    ) -> Tuple[Diagram, Context]:
        # Setup
        self.cast_scene(cast)

        return self.build_scene(with_watcher=with_watcher)

    @property
    def id(self) -> SceneID:
        """
        Description
        -----------
        The unique identifier for the scene
        :return:
        """
        return SceneID.kNotDefined
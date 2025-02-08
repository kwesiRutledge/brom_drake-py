from typing import Union, Tuple, List

from pydrake.systems.framework import DiagramBuilder, LeafSystem, Diagram, Context

# Internal Imports
from brom_drake.productions.roles import Role
from brom_drake.productions.ids import ProductionID
from brom_drake.utils import Performer
from brom_drake.utils.watcher import add_watcher_and_build

class BaseProduction:
    """
    Base class for all productions.
    """
    def __init__(self, **kwargs):
        # Create a builder for the production
        self.builder = DiagramBuilder()

        # Create the diagram and diagram_context placeholders
        self.diagram = None
        self.diagram_context = None

        # Save a list of the performers
        self.performers = []

        # Create an extra place to save DiagramWatcher objects
        self.watcher = None

    def add_supporting_cast(self):
        """
        Description
        -----------
        This method must be implemented by the subclass.
        It should add all "secondary" elements to the builder (i.e., the
        cast members that the user doesn't need to worry about).
        :return:
        """
        pass

    def suggested_roles(self) -> List[Role]:
        """
        Description
        -----------
        This method should be implemented by the subclass.
        It should return a list of roles that are suggested for the production.
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

    def add_main_cast(
        self,
        cast: Tuple[Role, Performer] = [],
    ):
        # Setup

        # Fulfill each role-performer pair in the casting_call list
        for role, performer in cast:
            self.fill_role(role, performer)

    def build_production(
        self,
        with_watcher: bool = True,
    ) -> Tuple[Diagram, Context]:
        """
        Description
        -----------
        This method builds the production.
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

    def add_cast_and_build(
        self,
        main_cast_members: Tuple[Role, Performer] = [],
        with_watcher: bool = True,
    ) -> Tuple[Diagram, Context]:
        # Setup

        # Add the cast to the production
        self.add_supporting_cast()
        self.add_main_cast(cast=main_cast_members)

        return self.build_production(with_watcher=with_watcher)

    @property
    def id(self) -> ProductionID:
        """
        Description
        -----------
        The unique identifier for the production.
        :return:
        """
        return ProductionID.kNotDefined
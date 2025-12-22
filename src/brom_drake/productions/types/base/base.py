from typing import Union, Tuple, List

from pydrake.systems.framework import DiagramBuilder, LeafSystem, Diagram, Context

# Internal Imports
from brom_drake.PortWatcher.plotter import FigureNamingConvention
from brom_drake.productions.roles import Role
from brom_drake.productions.ids import ProductionID
from brom_drake.utils import Performer
from brom_drake.utils.watcher import add_watcher_and_build
from brom_drake.utils.initial_condition_manager import InitialConditionManager

class BaseProduction:
    """
    *Description*

    Base class for all productions.

    *Attributes*

    builder : DiagramBuilder
        The DiagramBuilder used to build the production.

    diagram : Diagram
        The Diagram of the production.
        This is None and must be built using the build_production method.

    diagram_context : Context
        The Context of the production's Diagram.
        This is None and must be built using the build_production method.

    performers : List[Performer]
        A list of all performers added to the production.

    watcher : Optional[DiagramWatcher]
        The DiagramWatcher for the production.
        This is None, unless the production is built with a watcher.

    plant : Optional[MultibodyPlant]
        The MultibodyPlant of the production, if applicable.
        This is None by default.

    scene_graph : Optional[SceneGraph]
        The SceneGraph of the production, if applicable.
        This is None by default.

    initial_condition_manager : InitialConditionManager
        The InitialConditionManager used to manage initial conditions
        for the production.
        
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

        # Create a place to save the plant and scene_graph, if needed
        self.plant, self.scene_graph = None, None

        # Create an initial condition manager
        self.initial_condition_manager = InitialConditionManager()

    def add_supporting_cast(self):
        """
        *Description*
        
        This method must be implemented by the subclass.
        It should add all "secondary" elements to the builder (i.e., the
        cast members that the user doesn't need to worry about).
        
        """
        pass

    def suggested_roles(self) -> List[Role]:
        """
        *Description*
        
        This method should be implemented by the subclass.
        It should return a list of roles that are suggested for the production.
        
        *Returns*

        suggested_roles: List[Role]
            A list of roles that are suggested for the production.
        """
        return []

    def fill_role(
        self,
        role: Role,
        system: Performer,
    ):
        """
        *Description*
        
        This method should be implemented by the subclass. It should add the
        system to the role.
        
        *Parameters*

        role: Role
            The role to be filled.

        system: Performer
            The system to fill the role with.
        """
        # Setup
        builder = self.builder

        # Call the member method of the role object
        role.connect_performer_ports_to(builder, system)

        # Save the performer
        self.performers.append(system)

    def add_main_cast(
        self,
        cast: List[Tuple[Role, Performer]] = [],
    ):
        # Setup

        # Fulfill each role-performer pair in the casting_call list
        for role, performer in cast:
            self.fill_role(role, performer)

    def build_production(
        self,
        with_watcher: bool = True,
        figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat,
    ) -> Tuple[Diagram, Context]:
        """
        *Description*
        
        This method builds the production.
        It assumes that all components have been added to the builder.
        
        *Parameters*

        with_watcher: bool, optional
            Whether to build the production with a DiagramWatcher.
            Default is True.

        figure_naming_convention: FigureNamingConvention, optional
            The naming convention to use for figures in the watcher.
            Default is FigureNamingConvention.kFlat.

        *Returns*

        diagram: Diagram
            The built Diagram of the production.

        diagram_context: Context
            The Context of the built Diagram.
            If with_watcher is True, this will be the Context of the DiagramWatcher, as well.
        """
        # Setup
        builder = self.builder

        # Set all initial conditions, if any
        if self.plant is not None:
            self.initial_condition_manager.set_all_initial_conditions(
                plant=self.plant,
                # diagram_context=self.diagram_context,
            )

        # Build the diagram
        if with_watcher:
            self.watcher, self.diagram, self.diagram_context = add_watcher_and_build(
                builder,
                figure_naming_convention=figure_naming_convention
            )
        else:
            self.diagram = builder.Build()
            self.diagram_context = self.diagram.CreateDefaultContext()

        return self.diagram, self.diagram_context

    def add_cast_and_build(
        self,
        main_cast_members: Tuple[Role, Performer] = [],
        with_watcher: bool = True,
        figure_naming_convention: FigureNamingConvention = FigureNamingConvention.kFlat,
    ) -> Tuple[Diagram, Context]:
        # Setup

        # Add the cast to the production
        self.add_supporting_cast()
        self.add_main_cast(cast=main_cast_members)

        return self.build_production(
            with_watcher=with_watcher,
            figure_naming_convention=figure_naming_convention,
        )

    @property
    def id(self) -> ProductionID:
        """
        *Description*
        
        The unique identifier for the production.

        If not defined, returns ProductionID.kNotDefined.
        
        *Returns*
        
        ProductionID
            The unique identifier for the production.
        """
        return ProductionID.kNotDefined
import datetime
import json
from pathlib import Path
from typing import Dict, Union, Tuple, List, Optional, Sequence

from pydrake.systems.framework import (
    Context,
    Diagram,
    DiagramBuilder,
    LeafSystem,
    OutputPort,
)

# Internal Imports
from brom_drake.directories import DEFAULT_PRODUCTION_REPORTS_DIR, DEFAULT_VIDEO_DIR
from brom_drake.watchers.port_watcher.plotter import FigureNamingConvention
from brom_drake.productions.roles.role import Role
from brom_drake.productions.ids import ProductionID
from brom_drake.utils import Performer
from brom_drake.watchers.diagram_watcher.add_watcher import add_watcher_and_build
from brom_drake.utils.initial_condition_manager import InitialConditionManager
from brom_drake.systems.video import CameraSpec, VideoRecorder, add_cameras_to_builder


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

    video_recorders : Dict[str, VideoRecorder]
        The VideoRecorder for each camera requested with add_cameras,
        keyed by camera name. Empty until the production is built.

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

        # Create metadata dictionary
        self._metadata = {}

        # Cameras requested for video recording (attached during build_production)
        self._camera_specs: List[CameraSpec] = []
        self._video_output_dir: Path = DEFAULT_VIDEO_DIR
        self._renderer_backend: str = ""
        self.video_recorders: Dict[str, VideoRecorder] = {}

    def add_cameras(
        self,
        camera_specs: Sequence[CameraSpec],
        output_dir: Union[str, Path] = DEFAULT_VIDEO_DIR,
        renderer_backend: str = "",
    ):
        """
        *Description*

        Requests that the production record a video from each of the given cameras.
        Must be called BEFORE the production is built (e.g., before
        add_cast_and_build). Each camera is written to ``<output_dir>/<name>.mp4``
        once :py:meth:`save_videos` is called after the simulation.

        *Parameters*

        camera_specs: Sequence[CameraSpec]
            The cameras to record from.

        output_dir: Union[str, Path], optional
            The directory to write videos to. Default is ``./brom/videos``.

        renderer_backend: str, optional
            The VTK rendering backend. Use "" for Drake's default (EGL on Linux,
            which works without a display).

        *Usage*

        ::

            production.add_cameras([
                CameraSpec.looking_at("front", position=[1.5, 0, 1], target=[0, 0, 0.3]),
            ])
            diagram, diagram_context = production.add_cast_and_build()

            simulator = Simulator(diagram, diagram_context)
            simulator.AdvanceTo(5.0)

            video_paths = production.save_videos()
        """
        if self.diagram is not None:
            raise RuntimeError(
                "add_cameras() must be called before the production is built."
            )

        existing = {spec.name for spec in self._camera_specs}
        for spec in camera_specs:
            if spec.name in existing:
                raise ValueError(f"A camera named '{spec.name}' was already added.")
            existing.add(spec.name)

        self._camera_specs.extend(camera_specs)
        self._video_output_dir = Path(output_dir)
        self._renderer_backend = renderer_backend

    def _attach_cameras(self):
        """
        *Description*

        Adds a camera and VideoRecorder to the builder for every requested CameraSpec.
        Called by build_production just before the diagram is built.
        """
        if len(self._camera_specs) == 0:
            return

        if self.scene_graph is None:
            raise RuntimeError(
                f"Production {type(self).__name__} has no SceneGraph, "
                "so it cannot record videos."
            )

        query_object_port = self._find_camera_query_object_port()

        recorders = add_cameras_to_builder(
            builder=self.builder,
            scene_graph=self.scene_graph,
            camera_specs=self._camera_specs,
            plant=self.plant,
            output_dir=self._video_output_dir,
            renderer_backend=self._renderer_backend,
            query_object_port=query_object_port,
        )
        self.video_recorders = {
            spec.name: recorder for spec, recorder in zip(self._camera_specs, recorders)
        }

    def _find_camera_query_object_port(self) -> OutputPort:
        """
        *Description*

        Finds the port (in this production's builder) that cameras should read
        geometry from:

        - the SceneGraph's own query port, if the SceneGraph was added directly
          to the builder, or
        - the exported "query_object" port of a nested diagram (e.g., a station)
          that contains the SceneGraph.

        *Returns*

        query_object_port: OutputPort
            The port to connect cameras to.
        """
        top_level_systems = self.builder.GetSystems()
        if self.scene_graph in top_level_systems:
            return self.scene_graph.get_query_output_port()

        for system in top_level_systems:
            if not isinstance(system, Diagram):
                continue
            if not system.HasOutputPort("query_object"):
                continue
            if self.scene_graph in system.GetSystems():
                return system.GetOutputPort("query_object")

        raise NotImplementedError(
            f"Production {type(self).__name__} keeps its SceneGraph inside a nested "
            'diagram that does not export a "query_object" output port, so cameras '
            "cannot be attached. Export the SceneGraph's query output port from that "
            'diagram as "query_object" to enable video recording.'
        )

    def save_videos(self) -> Dict[str, Path]:
        """
        *Description*

        Finalizes every video recorded by the production. Call this after the
        simulation is finished.

        *Returns*

        video_paths: Dict[str, Path]
            The path of each written MP4, keyed by camera name. Cameras that
            captured no frames are left out.
        """
        video_paths = {}
        for name, recorder in self.video_recorders.items():
            path = recorder.save()
            if path is not None:
                video_paths[name] = path
        return video_paths

    def _record_metadata(self):
        """
        *Description*

        This method should be called at the end of the production's build_production method.
        It can be used to record any metadata about the production that may be useful for
        later analysis or debugging.
        """
        # Record the production ID and number of performers, at least.
        self._metadata["production_id"] = self.id.value
        self._metadata["num_performers"] = len(self.performers)
        self._metadata["cameras"] = [spec.name for spec in self._camera_specs]

        # Record metadata about when the "report file" will be written
        record_time_dt = datetime.datetime.now()
        self._metadata["record_time"] = record_time_dt.strftime("%Y-%m-%d_%H-%M-%S")

        # Add more metadata as needed

        # Save the metadata to a json
        record_time_str = record_time_dt.strftime("%Y-%m-%d_%H-%M-%S")
        log_file_path = (
            DEFAULT_PRODUCTION_REPORTS_DIR / f"build_metadata_{record_time_str}.json"
        )
        log_file_path.parent.mkdir(parents=True, exist_ok=True)
        with open(log_file_path, "w") as f:
            json.dump(self._metadata, f, indent=4)

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

        # Attach any cameras requested with add_cameras()
        self._attach_cameras()

        # Build the diagram
        if with_watcher:
            self.watcher, self.diagram, self.diagram_context = add_watcher_and_build(
                builder, figure_naming_convention=figure_naming_convention
            )
        else:
            self.diagram = builder.Build()
            self.diagram_context = self.diagram.CreateDefaultContext()

        # Record metadata about the production
        self._record_metadata()

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

from brom_drake.watchers.port_watcher.file_naming_convention import compute_safe_system_name, PathOrganizationConvention, file_path_for_port_data_dimension, generate_all_file_paths_for_ports_data
from brom_drake.watchers.port_watcher.port_watcher_options import (
    PortFigureArrangement,
    FigureNamingConvention,
    PortWatcherPlottingOptions,
    PortWatcherRawDataOptions,
)
from dataclasses import dataclass
import warnings
from pathlib import Path
from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import OutputPort
from pydrake.systems.primitives import VectorLogSink
from typing import List


@dataclass
class PortWatcherFileManager:
    """
    *Description*

    This class manages file paths and directories for saving data
    collected by the PortWatcher system.
    """

    base_directory: Path
    plotting_options: PortWatcherPlottingOptions
    raw_data_options: PortWatcherRawDataOptions = PortWatcherRawDataOptions()

    def compute_path_for_each_figure(
        self,
        output_port: OutputPort,
        associated_log_sink: VectorLogSink,
        port_component_name: str = None,
    ) -> List[Path]:
        """
        *Description*

        Computes the names of all of the figures that will be produced for


        *Returns*

        figure_paths_out: List[Path]
            The paths of all of the figures that will be produced by
            this PortWatcherPlotter object.

        """
        # Setup
        plotting_options = self.plotting_options
        file_format = plotting_options.file_format
        log_sink_size = associated_log_sink.get_input_port().size()

        # Create the figure paths based on the naming convention given to the
        # PortWatcherPlotter.
        file_organization_convention: PathOrganizationConvention = None
        match plotting_options.figure_naming_convention:
            case FigureNamingConvention.kFlat:
                file_organization_convention = PathOrganizationConvention.kFlat
            case FigureNamingConvention.kHierarchical:
                file_organization_convention = PathOrganizationConvention.kHierarchical
            case _:
                raise NotImplementedError(
                    f"Invalid figure naming convention for figure_names(): {plotting_options.figure_naming_convention}."
                )
            
        # Now define names
        match plotting_options.plot_arrangement:
            case PortFigureArrangement.OnePlotPerPort:
                return [
                    self.plot_dir
                    / file_path_for_port_data_dimension(
                        output_port=output_port,
                        file_format=file_format,
                        dimension_name=port_component_name, 
                        organization_convention=file_organization_convention,
                        component_name=port_component_name,
                    )
                ]
            
            case PortFigureArrangement.OnePlotPerDim:
                # If there is a sub-component name, then we will
                # create a sub-directory for it
                relative_file_paths = generate_all_file_paths_for_ports_data(
                    output_port=output_port,
                    file_format=file_format,
                    organization_convention=file_organization_convention,
                    component_name=port_component_name,
                )
                return [
                    self.plot_dir / relative_file_path for relative_file_path in relative_file_paths
                ]

            case _:
                raise NotImplementedError(
                    f"Invalid plot arrangement for figure naming convention {plotting_options.figure_naming_convention}: {plotting_options.plot_arrangement}."
                )
        

    def figure_names_under_flat_convention(
        self,
        output_port: OutputPort,
        associated_log_sink: VectorLogSink,
        port_component_name: str = None,
    ) -> List[Path]:
        """
        *Description*

        Returns the names associated with each figure that this port will
        generate assuming we are under the kFlat convention.

        *Arguments*

        output_port: OutputPort
            The output port for which we are generating figure names.
            We can extract the system name, port name, and port size from this.

        port_component_name: str
            A "sub-component" of the port that we wish to give a unique name in the
            figures.

        *Returns*

        figure_names: List[Path]
            List of paths where each path is a file name for an associated figure.

        .. deprecated::
            Use :meth:`compute_path_for_each_figure` instead.
            This method will be removed in a future release.
        """
        warnings.warn(
            "figure_names_under_flat_convention is deprecated and will be removed in a future release. "
            "Use compute_path_for_each_figure instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        # Setup
        plotting_options = self.plotting_options
        format = plotting_options.file_format
        plot_dir = self.plot_dir

        # The naming also depends on the arrangement of the plots
        # (i.e., if there is one plot per port, or one plot per dimension)
        system = output_port.get_system()
        system_name = system.get_name()
        safe_system_name = compute_safe_system_name(system_name)

        port_name = output_port.get_name()
        log_sink_size = associated_log_sink.get_input_port().size()

        match plotting_options.plot_arrangement:
            case PortFigureArrangement.OnePlotPerPort:
                return [
                    plot_dir
                    / file_path_for_port_data_dimension(
                        output_port=output_port,
                        file_format=format,
                        dimension_name=port_component_name, 
                    )
                ]

            case PortFigureArrangement.OnePlotPerDim:

                if port_component_name is None:
                    # If there is no sub-component name, then we just
                    # create the files in the main plot directory
                    relative_file_paths = generate_all_file_paths_for_ports_data(
                        output_port=output_port,
                        file_format=format,
                        organization_convention=PathOrganizationConvention.kFlat,
                    )
                    return [
                        plot_dir / relative_file_path for relative_file_path in relative_file_paths 
                    ]
                else:
                    # If there is a sub-component name, then we will
                    # create a sub-directory for it
                    dimension_names = {
                        dimension: f"{port_component_name}/dim{dimension}"
                        for dimension in range(log_sink_size)
                    }
                    relative_file_paths = generate_all_file_paths_for_ports_data(
                        output_port=output_port,
                        file_format=format,
                        dimension_names=dimension_names,
                        organization_convention=PathOrganizationConvention.kFlat,
                    )
                    return [
                        plot_dir / relative_file_path for relative_file_path in relative_file_paths
                    ]
            case _:
                raise NotImplementedError(
                    f"Invalid plot arrangement for figure naming convention {plotting_options.figure_naming_convention}: {plotting_options.plot_arrangement}."
                )

    def figure_names_under_hierarchical_convention(
        self,
        output_port: OutputPort,
        associated_log_sink: VectorLogSink,
        port_component_name: str = None,
    ) -> List[Path]:
        """
        *Description*

        Returns the names associated with each figure that this port will
        generate assuming we are under the kHierarchical convention.

        *Parameters*

        output_port: OutputPort
            The output port for which we are generating figure names.
            We can extract the system name, port name, and port size from this.

        port_component_name: str
            A "sub-component" of the port that we wish to give a unique name in the figures.

        *Returns*

        paths_out: List[Path]
            Each path in this list is a file path for an associated figure.
        """
        # Setup
        plotting_options = self.plotting_options
        format = plotting_options.file_format

        # Compute the figure paths based on the arrangement of the plots
        # (i.e., if there is one plot per port, or one plot per dimension
        system = output_port.get_system()
        system_name = system.get_name()
        safe_system_name = compute_safe_system_name(system_name)

        port_name = output_port.get_name()

        log_sink_size = associated_log_sink.get_input_port().size()

        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return [
                self.plot_dir
                / file_path_for_port_data_dimension(
                    output_port=output_port,
                    file_format=format,
                    dimension_name=port_component_name,
                    organization_convention=PathOrganizationConvention.kHierarchical,
                )
            ]

        elif plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            if port_component_name is None:
                # If there is no sub-component name, then we just
                # create the files in the main plot directory
                relative_file_paths = generate_all_file_paths_for_ports_data(
                        output_port=output_port,
                        file_format=format,
                        organization_convention=PathOrganizationConvention.kHierarchical,
                    )
                return [
                    self.plot_dir / relative_file_path for relative_file_path in relative_file_paths 
                ]
            else:
                # If there is a sub-component name, then we will
                # create a sub-directory for it
                return [
                    self.plot_dir
                    / f"system_{safe_system_name}/port_{port_name}/{port_component_name}/dim_{self.name_of_data_at_index(ii, output_port, associated_log_sink, remove_spaces=True)}.{format}"
                    for ii in range(log_sink_size)
                ]

        else:
            raise NotImplementedError(
                f"Invalid plot arrangement for figure naming convention {plotting_options.figure_naming_convention}: {plotting_options.plot_arrangement}."
            )

    def name_of_data_at_index(
        self,
        dim_index: int,
        target_port: OutputPort,
        associated_log_sink: VectorLogSink,
        remove_spaces: bool = False,
    ) -> str:
        """
        *Description*

        Returns the name of the data which is in index dim_index
        of this vector-valued port.

        TODO(kwesi): Consider moving this to its own utility file, outside of
        the file manager.

        *Parameters*

        self : PortWatcherPlotter
            The PortWatcherPlotter object.
        dim_index : int
            The index of the data in the port.
        remove_spaces : bool
            Whether to remove spaces from the name.

        *Returns*

        name_of_data: str
            The name of the data at index dim_index.
        """
        # Setup
        plotting_options = self.plotting_options

        n_dims_sink = associated_log_sink.get_input_port().size()
        system = target_port.get_system()

        # Input Processing
        if dim_index >= n_dims_sink:
            raise ValueError(
                f"dim_index ({dim_index}) is greater than the number of dimensions in the port ({n_dims_sink})."
            )

        # Default name
        name = f"Dim #{dim_index}"

        # If we are using the OnePlotPerPort config, then use the default name
        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return name

        # Otherwise, try to get a better name
        # - For MultibodyPlants, we can get specific names for certain dimensions of the "state" output port

        if type(system) is MultibodyPlant:
            # The multi-body plant has names for specific ports
            if target_port.get_name() == "state":
                # We can get the names of the state
                state_names = system.GetStateNames()
                name = state_names[dim_index]

        # Filter our spaces, if requested
        if remove_spaces:
            name = name.replace(" ", "_")

        # Return name!
        return name

    @property
    def plot_dir(self) -> Path:
        """
        *Description*

        This function returns the directory where the plots will be saved.

        *Returns*

        plot_dir: Path
            The directory where the plots will be saved.
        """
        return self.base_directory / "plots"

    @property
    def raw_data_dir(self) -> Path:
        """
        *Description*

        This function returns the directory where the raw data will be saved.

        *Returns*

        raw_data_dir: Path
            The directory where the raw data will be saved.
        """
        return self.base_directory / "raw_data"

    def raw_data_file_path(
        self, 
        output_port: OutputPort, 
        port_component_name: str = None
    ) -> Path:
        """
        *Description*

        This function returns the file name for saving raw data.

        *Parameters*

        output_port: OutputPort
            The output port for which to generate the file path.

        port_component_name: str
            A "sub-component" of the port that we wish to give a unique name in the data.

        *Returns*

        raw_data_file_name: Path
            The file name for saving raw data.
        """

        return (
            self.raw_data_dir
            / file_path_for_port_data_dimension(
                output_port=output_port,
                file_format=self.raw_data_options.file_format,
                organization_convention=self.raw_data_options.file_organization_convention,
                component_name=port_component_name,
            )
        )

    def time_data_file_path(self, output_port: OutputPort) -> Path:
        """
        *Description*

        This function returns the file name for saving time data.

        *Returns*

        time_data_file_path: Path
            The file name for saving time data.
        """
        port_name = output_port.get_name()

        return (
            self.raw_data_dir
            / file_path_for_port_data_dimension(
                output_port=output_port,
                file_format=self.raw_data_options.file_format,
                organization_convention=self.raw_data_options.file_organization_convention,
                dimension_name=port_name + "_times",
            )
        )

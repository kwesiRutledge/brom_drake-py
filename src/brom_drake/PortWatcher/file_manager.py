from brom_drake.PortWatcher.port_watcher_options import (
    PortFigureArrangement,
    FigureNamingConvention,
    PortWatcherPlottingOptions, 
)
from dataclasses import dataclass
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
    raw_data_file_format: str = "npy"

    @staticmethod
    def compute_safe_system_name(system_name: str) -> str:
        """
        *Description*
        
        This function returns a filesystem-safe version of the system name.

        *Returns*

        safe_system_name: str
            The filesystem-safe version of the system name.
        """
        # First, let's check to see how many "/" exist in the name
        slash_occurences = [i for i, letter in enumerate(system_name) if letter == "/"]
        if len(slash_occurences) > 0:
            system_name = system_name[slash_occurences[-1] + 1:]  # truncrate string based on the last slash
        
        # Second, replace all spaces with underscores
        system_name = system_name.replace(" ", "_")

        return system_name

    def compute_path_for_each_figure(
        self,
        output_port: OutputPort,
        associated_log_sink: VectorLogSink,
        port_component_name: str = None
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

        # Create the figure paths based on the naming convention given to the
        # PortWatcherPlotter.
        match plotting_options.figure_naming_convention:
            case FigureNamingConvention.kFlat:
                return self.figure_names_under_flat_convention(output_port, associated_log_sink, port_component_name)
            
            case FigureNamingConvention.kHierarchical:
                return self.figure_names_under_hierarchical_convention(output_port, associated_log_sink, port_component_name)
            
            case _:
                raise NotImplementedError(
                    f"Invalid figure naming convention for figure_names(): {plotting_options.figure_naming_convention}."
                )


    def figure_names_under_flat_convention(
        self,
        output_port: OutputPort,
        associated_log_sink: VectorLogSink,
        port_component_name: str = None
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
        """
        # Setup
        plotting_options = self.plotting_options
        format = plotting_options.file_format
        plot_dir = self.plot_dir

        # The naming also depends on the arrangement of the plots
        # (i.e., if there is one plot per port, or one plot per dimension)
        system = output_port.get_system()
        system_name = system.get_name()
        safe_system_name = self.compute_safe_system_name(system_name)

        port_name = output_port.get_name()
        log_sink_size = associated_log_sink.get_input_port().size()

        match plotting_options.plot_arrangement:
            case PortFigureArrangement.OnePlotPerPort:
                if port_component_name is None:
                    # If there is no sub-component name, then we just
                    # create the file in the main plot directory
                    return [
                        plot_dir / f"system_{safe_system_name}_port_{port_name}.{format}"
                    ]
                else:
                    # If there is a sub-component name, then we will
                    # create a sub-directory for it
                    return [
                        plot_dir / f"system_{safe_system_name}_port_{port_name}" / f"{port_component_name}.{format}"
                    ]

            case PortFigureArrangement.OnePlotPerDim:

                if port_component_name is None:
                    # If there is no sub-component name, then we just
                    # create the files in the main plot directory
                    return [
                        plot_dir / f"system_{safe_system_name}_port_{port_name}_dim{ii}.{format}"
                        for ii in range(log_sink_size)
                    ]
                else:
                    # If there is a sub-component name, then we will
                    # create a sub-directory for it
                    return [
                        plot_dir / f"system_{safe_system_name}_port_{port_name}" / f"{port_component_name}_dim{ii}.{format}"
                        for ii in range(log_sink_size)
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
        safe_system_name = self.compute_safe_system_name(system_name)

        port_name = output_port.get_name()
        
        log_sink_size = associated_log_sink.get_input_port().size()

        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            if port_component_name is None:
                # If there is no sub-component name, then we just
                # create the file in the main plot directory
                return [
                    self.plot_dir / f"system_{safe_system_name}/port_{port_name}.{format}"
                ]
            else:
                # If there is a sub-component name, then we will
                # create a sub-directory for it
                return [
                    self.plot_dir / f"system_{safe_system_name}/port_{port_name}/{port_component_name}.{format}"
                ]

        elif plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            if port_component_name is None:
                # If there is no sub-component name, then we just
                # create the files in the main plot directory
                return [
                    self.plot_dir / f"system_{safe_system_name}/port_{port_name}/dim_{self.name_of_data_at_index(ii, output_port, associated_log_sink, remove_spaces=True)}.{format}"
                    for ii in range(log_sink_size)
                ]
            else:
                # If there is a sub-component name, then we will
                # create a sub-directory for it
                return [
                    self.plot_dir / f"system_{safe_system_name}/port_{port_name}/{port_component_name}/dim_{self.name_of_data_at_index(ii, output_port, associated_log_sink, remove_spaces=True)}.{format}"
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
        system_name: str,
        port_name: str,
        port_component_name: str = None
    ) -> Path:
        """
        *Description*
        
        This function returns the file name for saving raw data.

        *Parameters*

        port_component_name: str
            A "sub-component" of the port that we wish to give a unique name in the data.

        *Returns*

        raw_data_file_name: Path
            The file name for saving raw data.
        """
        safe_system_name = self.compute_safe_system_name(system_name)

        if port_component_name is None:    
            return self.raw_data_dir / f"system_{safe_system_name}_port_{port_name}_data.{self.raw_data_file_format}"
        else:
            return self.raw_data_dir / f"system_{safe_system_name}_port_{port_name}" / f"{port_component_name}.{self.raw_data_file_format}"

    def time_data_file_path(self, system_name: str, port_name: str) -> Path:
        """
        *Description*
        
        This function returns the file name for saving time data.

        *Returns*

        time_data_file_path: Path
            The file name for saving time data.
        """
        safe_system_name = self.compute_safe_system_name(system_name)
        return self.raw_data_dir / f"system_{safe_system_name}_port_{port_name}_times.{self.raw_data_file_format}"
    
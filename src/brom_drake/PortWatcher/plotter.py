import loguru
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path
from pydrake.all import (
    Context,
    LeafSystem,
    OutputPort,
    MultibodyPlant,
)
from typing import List, Tuple


# Internal Imports
from brom_drake.PortWatcher.port_figure_arrangement import PortFigureArrangement
from brom_drake.PortWatcher.port_watcher_options import FigureNamingConvention, PortWatcherPlottingOptions
from brom_drake.utils.constants import SupportedLogger

class PortWatcherPlotter:
    def __init__(
        self,
        logger: SupportedLogger,
        port: OutputPort,
        plotting_options: PortWatcherPlottingOptions = PortWatcherPlottingOptions(),
    ):
        # Setup
        self.logger = logger
        self.port = port
        self.plotting_options = plotting_options

    def compute_plot_shape(self, n_dims: int) -> Tuple[int, int]:
        """
        Description:
            Computes the shape of the plot based on the data.
        :param n_dims: The number of dimensions in the data.
        :return:
        """
        if n_dims == 1:
            return 1, 1
        if n_dims == 2:
            return 1, 2

        if n_dims < 9:
            return 2, int(np.ceil(n_dims / 2.0))

        # Otherwise
        return 3, int(np.ceil(n_dims / 3.0))

    def figure_names(self) -> List[str]:
        """
        Description:
            Returns the name of the figure.
        :return:
        """
        # Setup
        plotting_options = self.plotting_options

        # If this has the flat naming convention, then the file should be contained within the plot_dir.
        if plotting_options.figure_naming_convention == FigureNamingConvention.kFlat:
            return self.figure_names_under_flat_convention()


        elif plotting_options.figure_naming_convention == FigureNamingConvention.kHierarchical:
            return self.figure_names_under_hierarchical_convention()

        else:
            raise NotImplementedError(
                f"Invalid figure naming convention for figure_names(): {plotting_options.figure_naming_convention}."
            )


    def figure_names_under_flat_convention(self) -> List[str]:
        """
        Description:
            Returns the names associated with each figure that this port will
            generate assuming we are under the kFlat convention.
        :param self:
        :return: List of strings where each string is a file name for an associated figure.
        """
        # Setup
        plotting_options = self.plotting_options
        format = plotting_options.file_format
        plot_dir = plotting_options.base_directory

        #
        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return [
                f"{plot_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}.{format}"
            ]

        elif plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            return [
                f"{plot_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}_dim{ii}.{format}"
                for ii in range(self.port.size())
            ]

        else:
            raise NotImplementedError(
                f"Invalid plot arrangement for figure naming convention {options.plotting.figure_naming_convention}: {options.plot_arrangement}."
            )

    def figure_names_under_hierarchical_convention(self) -> List[str]:
        """
        Description:
            Returns the names associated with each figure that this port will
            generate assuming we are under the kHierarchical convention.
        :return:
        """
        # Setup
        plotting_options = self.plotting_options
        format = plotting_options.file_format
        plot_dir = plotting_options.base_directory

        # Algorithm
        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return [
                f"{plot_dir}/system_{self.safe_system_name()}/port_{self.port.get_name()}.{format}"
            ]

        elif plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            return [
                f"{plot_dir}/system_{self.safe_system_name()}/port_{self.port.get_name()}/{self.name_of_data_at_index(ii, remove_spaces=True)}.{format}"
                for ii in range(self.port.size())
            ]

        else:
            raise NotImplementedError(
                f"Invalid plot arrangement for figure naming convention {plotting_options.figure_naming_convention}: {options.plot_arrangement}."
            )

    def name_of_data_at_index(
        self,
        dim_index: int,
        remove_spaces: bool = False,
    ) -> str:
        """
        Description:
            Returns the name of the data which is in index dim_index
            of this vector-valued port.
        :param dim_index:
        :param remove_spaces: Whether to remove spaces from the name.
        :return:
        """
        # Setup
        plotting_options = self.plotting_options
        n_dims = self.port.size()
        system = self.port.get_system()

        # Input Processing
        if dim_index >= n_dims:
            raise ValueError(
                f"dim_index ({dim_index}) is greater than the number of dimensions in the port ({n_dims})."
            )

        # Default name
        name = f"Dim #{dim_index}"

        # Only use full names, if we are NOT using the OnePlotPerPort config
        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return name

        if self.system_is_multibody_plant():
            # The multi-body plant has names for specific ports
            if self.port.get_name() == "state":
                # We can get the names of the state
                state_names = system.GetStateNames()
                name = state_names[dim_index]
            else:
                # Announce that we are using the default name
                loguru.logger.warning(
                    f"Using default name for data at index {dim_index} of port {self.port.get_name()} of system {system.get_name()}."
                )

        # Filter our spaces, if requested
        if remove_spaces:
            name = name.replace(" ", "_")

        # Return name!
        return name

    def plot_logger_data(
        self,
        diagram_context: Context,
    ) -> Tuple[List[plt.Figure], List[List[plt.Axes]]]:
        """
        plot_logger_data
        Description:

            This function plots the data in the logger.
        :param diagram_context:
        :return:
        """
        # Setup
        logger = self.logger
        plotting_options = self.plotting_options
        system = self.port.get_system()

        # Get the log from the logger
        temp_log = logger.FindLog(diagram_context)

        log_times = temp_log.sample_times()
        log_data = temp_log.data()

        if (log_data.shape[1] == 0) or (log_data.shape[0] == 0):
            loguru.logger.warning(
                f"No data found for {system.get_name()} - Port {self.port.get_name()}! Skipping...")
            return None, None

        # Plot the data
        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            fig, ax_list = self.plot_logger_data_subplots(log_times, log_data)
            return [fig], [ax_list]

        elif plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            figs, ax_grid = [], []
            for port_index in range(self.port.size()):
                fig_ii = plt.figure()
                ax_ii = fig_ii.add_subplot(1, 1, 1)
                ax_ii.plot(
                    log_times, log_data[port_index, :]
                )

                # Add axis titles and labels
                ax_ii.set_xlabel("Time (s)")
                ax_ii.set_title(self.name_of_data_at_index(port_index))

                # Save figures and axes to lists
                figs.append(fig_ii)
                ax_grid.append([ax_ii])

            return figs, ax_grid

        else:
            raise ValueError(
                f"Invalid plot arrangement: {plotting_options.plot_arrangement}."
            )

    def plot_logger_data_subplots(
        self,
        times: np.array,
        data: np.array,

    ):
        """
        plot_logger_data_subplots
        Description:

            This function plots the data in the logger.
        :param times:
        :param data:
        :param diagram_context:
        :return:
        """
        # Setup
        n_dims = data.shape[0]

        # Plot the data
        n_rows, n_cols = self.compute_plot_shape(n_dims)

        print(f"Plotting {n_dims} dimensions in a {n_rows}x{n_cols} grid.")

        fig, ax_list = plt.subplots(n_rows, n_cols)

        if n_rows == 1 and n_cols == 1:
            ax_list.plot(times, data[0, :])
            ax_list.set_title(self.name_of_data_at_index(0))

        elif (n_rows == 1) or (n_cols == 1):
            for dim_index in range(n_dims):
                ax_list[dim_index].plot(times, data[dim_index, :])
                ax_list[dim_index].set_title(
                    self.name_of_data_at_index(dim_index),
                )

        else:
            for row_index in range(n_rows):
                for col_index in range(n_cols):

                    dim_index = n_cols * row_index + col_index

                    if dim_index >= n_dims:
                        fig.delaxes(ax_list[row_index, col_index])
                        continue

                    ax_list[row_index, col_index].plot(times, data[dim_index, :])
                    ax_list[row_index, col_index].set_title(self.name_of_data_at_index(dim_index))

        return fig, ax_list
    
    def safe_system_name(self) -> str:
        """
        Description:
            Returns a safe name for the system.
        :param name: System's name.
        :return:
        """
        # Setup
        system = self.port.get_system()
        out = system.get_name()

        # First, let's check to see how many "/" exist in the name
        slash_occurences = [i for i, letter in enumerate(out) if letter == "/"]
        if len(slash_occurences) > 0:
            out = out[slash_occurences[-1] + 1:]  # truncrate string based on the last slash

        # Second, replace all spaces with underscores
        out = out.replace(" ", "_")

        return out

    def save_figures(self, diagram_context: Context):
        """
        save_figures
        Description:
            This function saves the figures.
        TODO(kwesi): Make it so that this function computes names + directory structure based on plot arrangement.
        :param diagram_context:
        :return:
        """
        # Setup
        plotting_options = self.plotting_options
        figs, ax_list = self.plot_logger_data(diagram_context)

        # If no figures are returned, then return early!
        if figs is None:
            return

        if len(figs) == 0:
            return # Do nothing

        # Save the figures
        figure_names = self.figure_names()

        if len(figs) == 1:
            # Create directory for the plots, if it doesn't already exist
            os.makedirs(Path(figure_names[0]).parent, exist_ok=True)
            figs[0].savefig(figure_names[0], dpi=plotting_options.plot_dpi)
            plt.close(figs[0])
        else:
            # Plot each figure within this directory
            for ii, fig_ii in enumerate(figs):
                os.makedirs(Path(figure_names[ii]).parent, exist_ok=True)
                fig_ii.savefig(figure_names[ii],dpi=plotting_options.plot_dpi)
                plt.close(fig_ii)

        # Close all figures when done (this should be redundant?)
        # plt.close('all')

    def system_is_multibody_plant(self) -> bool:
        """
        Description:
            Returns True if the system is a MultibodyPlant.
        :return:
        """
        # Setup
        system = self.port.get_system()

        # Return
        return type(system) == MultibodyPlant
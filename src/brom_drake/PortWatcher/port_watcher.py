"""
PortWatcher.py
Description:

    This file defines the PortWatcher class. This class is used to watch the ports of a
    diagram.
"""
from pathlib import Path
from typing import List, Tuple, Union, NamedTuple
import loguru
import numpy as np
import matplotlib.pyplot as plt
import os

from pydrake.systems.framework import OutputPort, PortDataType, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import LogVectorOutput
from pydrake.systems.framework import Context

# Internal Imports
from brom_drake.DiagramWatcher.constants import DEFAULT_PLOT_DIR
from .port_watcher_options import PortWatcherOptions, FigureNamingConvention
from .port_figure_arrangement import PortFigureArrangement


class PortWatcher:
    def __init__(
        self,
        system: LeafSystem,
        output_port: OutputPort,
        builder: DiagramBuilder,
        logger_name: str = None,
        options: PortWatcherOptions = PortWatcherOptions(),
        plot_dir: str = DEFAULT_PLOT_DIR,
    ):
        # Setup
        self.options = options
        self.system = system
        self.port = output_port
        self.data = {}
        self.plot_handles = {}
        self.plot_handles = None
        self.plot_dir = plot_dir

        # Input Processing
        if output_port.get_data_type() != PortDataType.kVectorValued:
            raise ValueError(
                f"This watcher only supports vector valued ports (i.e., of type {PortDataType.kVectorValued}.\n" +
                f"Received port of type {output_port.get_data_type()}."
            )

        if logger_name is None:
            logger_name = f"PortWatcher_{system.get_name()}_{output_port.get_name()}"

        # Processing
        self.logger = LogVectorOutput(output_port, builder)
        self.logger.set_name(logger_name)

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

        # Get the log from the logger
        temp_log = self.logger.FindLog(diagram_context)

        log_times = temp_log.sample_times()
        log_data = temp_log.data()

        if (log_data.shape[1] == 0) or (log_data.shape[0] == 0):
            loguru.logger.warning(
                f"No data found for {self.system.get_name()} - Port {self.port.get_name()}! Skipping...")
            return None, None

        # Plot the data
        if self.options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            fig, ax_list = self.plot_logger_data_subplots(diagram_context, log_times, log_data)
            return [fig], [ax_list]

        elif self.options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            figs, ax_grid = [], []
            for port_index in range(self.port.size()):
                fig_ii = plt.figure()
                ax_ii = fig_ii.add_subplot(1, 1, 1)
                ax_ii.plot(
                    log_times, log_data[port_index, :]
                )

                # Save figures and axes to lists
                figs.append(fig_ii)
                ax_grid.append([ax_ii])

            return figs, ax_grid

        else:
            raise ValueError(
                f"Invalid plot arrangement: {self.options.plot_arrangement}."
            )

    def plot_logger_data_subplots(
        self,
        diagram_context: Context,
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
            ax_list.set_title(f"Dim #0")

        else:
            for row_index in range(n_rows):
                for col_index in range(n_cols):

                    dim_index = n_cols * row_index + col_index

                    if dim_index >= n_dims:
                        fig.delaxes(ax_list[row_index, col_index])
                        continue

                    ax_list[row_index, col_index].plot(times, data[dim_index, :])
                    ax_list[row_index, col_index].set_title(f"Dim #{dim_index}")

        return fig, ax_list

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

    def safe_system_name(self) -> str:
        """
        Description:
            Returns a safe name for the system.
        :param name: System's name.
        :return:
        """
        out = self.system.get_name()
        # First, let's check to see how many "/" exist in the name
        slash_occurences = [i for i, letter in enumerate(out) if letter == "/"]
        if len(slash_occurences) > 0:
            out = out[slash_occurences[-1] + 1:]  # truncrate string based on the last slash

        # Second, replace all spaces with underscores
        out = out.replace(" ", "_")

        return out

    def save_figures(self, diagram_context: Context):
        """
        savefigures
        Description:
            This function saves the figures.
        TODO(kwesi): Make it so that this function computes names + directory structure based on plot arrangement.
        :param diagram_context:
        :return:
        """
        # Setup
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
            figs[0].savefig(figure_names[0], dpi=self.options.plot_dpi)
        else:
            # Plot each figure within this directory
            for ii, fig_ii in enumerate(figs):
                os.makedirs(Path(figure_names[ii]).parent, exist_ok=True)
                fig_ii.savefig(figure_names[ii],dpi=self.options.plot_dpi)

    def figure_names(self) -> List[str]:
        """
        Description:
            Returns the name of the figure.
        :return:
        """
        # Setup
        options = self.options

        # If this has the flat naming convention, then the file should be contained within the plot_dir.
        if options.figure_naming_convention == FigureNamingConvention.kFlat:
            return self.figure_names_under_flat_convention()


        elif options.figure_naming_convention == FigureNamingConvention.kHierarchical:
            return self.figure_names_under_hierarchical_convention()

        else:
            raise NotImplementedError(
                f"Invalid figure naming convention for figure_names(): {options.figure_naming_convention}."
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
        options = self.options
        format = options.file_format

        #
        if options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return [
                f"{self.plot_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}.{format}"
            ]

        elif options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            return [
                f"{self.plot_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}_dim{ii}.{format}"
                for ii in range(self.port.size())
            ]

        else:
            raise NotImplementedError(
                f"Invalid plot arrangement for figure naming convention {options.figure_naming_convention}: {options.plot_arrangement}."
            )

    def figure_names_under_hierarchical_convention(self) -> List[str]:
        """
        Description:
            Returns the names associated with each figure that this port will
            generate assuming we are under the kHierarchical convention.
        :return:
        """
        # Setup
        options = self.options
        format = options.file_format

        # Algorithm
        if options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            return [
                f"{self.plot_dir}/system_{self.safe_system_name()}/port_{self.port.get_name()}.{format}"
            ]

        elif options.plot_arrangement == PortFigureArrangement.OnePlotPerDim:
            return [
                f"{self.plot_dir}/system_{self.safe_system_name()}/port_{self.port.get_name()}/dim{ii}.{format}"
                for ii in range(self.port.size())
            ]

        else:
            raise NotImplementedError(
                f"Invalid plot arrangement for figure naming convention {options.figure_naming_convention}: {options.plot_arrangement}."
            )
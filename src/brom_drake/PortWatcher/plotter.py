import logging
import matplotlib.pyplot as plt
import numpy as np
import os
from pathlib import Path
from pydrake.all import (
    Context,
    OutputPort,
    PortDataType,
    MultibodyPlant,
)
from pydrake.systems.primitives import VectorLogSink
from typing import List, Tuple


# Internal Imports
from brom_drake.PortWatcher.file_manager import PortWatcherFileManager
from brom_drake.PortWatcher.port_figure_arrangement import PortFigureArrangement
from brom_drake.PortWatcher.port_watcher_options import FigureNamingConvention, PortWatcherPlottingOptions
from brom_drake.utils.constants import SupportedLogger
from brom_drake.utils.type_checking import is_rigid_transform
from brom_drake.directories import DEFAULT_PLOT_DIR

class PortWatcherPlotter:
    """
    *Description*

    A plotter for the PortWatcher object. This is responsible for interpreting
    the data from the :py:class:`brom_drake.PortWatcher.PortWatcher` 
    
    TODO(Kwesi): Consider making this a dataclass
    """
    def __init__(
        self,
        port: OutputPort,
        python_logger: logging.Logger,
        file_manager: PortWatcherFileManager,
        plotting_options: PortWatcherPlottingOptions = PortWatcherPlottingOptions(),
    ):
        # Setup
        self.port = port
        self.plotting_options = plotting_options
        self.file_manager = file_manager
        self.python_logger = python_logger

    def compute_plot_shape(self, n_dims: int) -> Tuple[int, int]:
        """
        *Description*
        
        Computes the shape of the plot based on the data.

        *Parameters*
        
        n_dims: int
            The number of dimensions in the data.

        *Returns*
        
        n_rows: int
            The number of rows in the plot.

        n_columns: int
            The number of columns in the plot
        """
        if n_dims == 1:
            return 1, 1
        if n_dims == 2:
            return 1, 2

        if n_dims < 9:
            return 2, int(np.ceil(n_dims / 2.0))

        # Otherwise
        return 3, int(np.ceil(n_dims / 3.0))

    def add_to_python_report(self, message: str):
        """
        Description
        -----------
        Logs a message to the Python logger.
        :param message: A string with the message we want to send to the logs.
        :return:
        """
        self.python_logger.info(message)

    def add_warning_to_python_report(self, message: str):
        self.python_logger.warning(message)

    def data_dimension(self) -> int:
        """
        *Description*
        
        Returns the dimension of the data in the port.
        
        *Parameters*
        
        self : PortWatcherPlotter
            The PortWatcherPlotter object.

        *Returns*
        
        data_dim: int
            The dimension of the data in the port.
        """
        if self.port.get_data_type() == PortDataType.kVectorValued:
            return self.port.size()
        else:
            # If port contains RigidTransform, then the expected data dimension is 7.
            example_allocation = self.port.Allocate()
            example_value = example_allocation.get_value()
            if is_rigid_transform(example_value):
                return 7
            elif type(example_value) == bool: # if the output_value is a boolean
                return 1
        
        # Otherwise, raise an error
        raise ValueError(
            f"Port {self.port.get_name()} of system {self.port.get_system().get_name()} is not of the correct type for plotting."
        )

    def plot_logger_data(
        self,
        drake_vector_log: VectorLogSink,
        diagram_context: Context,
    ) -> Tuple[List[plt.Figure], List[List[plt.Axes]]]:
        """
        *Description*
        
        This function plots the data in the logger.

        *Parameters*
        
        self : PortWatcherPlotter
            The PortWatcherPlotter object.

        diagram_context : Context
            The context of the diagram.
        
        *Returns*
        
        Tuple[List[plt.Figure], List[List[plt.Axes]]]
            A tuple where:
            - the first element is a list of figures and 
            - the second element is a list of lists of axes.
        """
        # Setup
        python_logger = self.python_logger
        plotting_options = self.plotting_options
        system = self.port.get_system()

        # Get the log from the drake_vector_log
        temp_log = drake_vector_log.FindLog(diagram_context)

        log_times = temp_log.sample_times()
        log_data = temp_log.data()

        if (log_data.shape[1] == 0) or (log_data.shape[0] == 0):
            python_logger.warning(
                f"No data found for {system.get_name()} - Port {self.port.get_name()}! Skipping...")
            return None, None

        # Plot the data
        if plotting_options.plot_arrangement == PortFigureArrangement.OnePlotPerPort:
            fig, ax_list = self.plot_logger_data_subplots(log_times, log_data, drake_vector_log)
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
                ax_ii.set_title(self.file_manager.name_of_data_at_index(port_index, self.port, drake_vector_log))

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
        drake_vector_log: VectorLogSink,
    ):
        """
        *Description*
        
        This function plots the data in the logger.

        TODO(Kwesi): Consider adding refactoring this to remove dependecny on drake_vector_log

        *Parameters*
        
        self : PortWatcherPlotter
            The PortWatcherPlotter object.

        times : np.array
            The times at which the data was recorded.

        data : np.array
            The data that was recorded.
        
        *Returns*
        
        fig_out: plt.Figure
            The figure containing subplots that we use in the output.

        axes_list: list[plt.Axes]
            The list of axes.
        """
        # Setup
        n_dims = data.shape[0]

        # Plot the data
        n_rows, n_cols = self.compute_plot_shape(n_dims)

        self.add_to_python_report(
            f"Plotting {n_dims} dimensions in a {n_rows}x{n_cols} grid."
            )

        fig, ax_list = plt.subplots(n_rows, n_cols)

        if n_rows == 1 and n_cols == 1:
            ax_list.plot(times, data[0, :])
            ax_list.set_title(self.file_manager.name_of_data_at_index(0, self.port, drake_vector_log))

        elif (n_rows == 1) or (n_cols == 1):
            for dim_index in range(n_dims):
                ax_list[dim_index].plot(times, data[dim_index, :])
                ax_list[dim_index].set_title(
                    self.file_manager.name_of_data_at_index(dim_index, self.port, drake_vector_log),
                )

        else:
            for row_index in range(n_rows):
                for col_index in range(n_cols):

                    dim_index = n_cols * row_index + col_index

                    if dim_index >= n_dims:
                        fig.delaxes(ax_list[row_index, col_index])
                        continue

                    ax_list[row_index, col_index].plot(times, data[dim_index, :])
                    ax_list[row_index, col_index].set_title(self.file_manager.name_of_data_at_index(dim_index, self.port, drake_vector_log))

        return fig, ax_list

    def save_figures(
        self,
        vector_log_sink: VectorLogSink,
        diagram_context: Context,
        port_component_name: str|None = None,
    ):
        """
        *Description*
        
        This function saves the figures.

        .. note::

            TODO(kwesi): Make it so that this function computes names + directory structure based on plot arrangement.

        *Parameters*
        
        self : PortWatcherPlotter
            The PortWatcherPlotter object.

        diagram_context : Context
            The context of the diagram.
        """
        # Setup
        plotting_options = self.plotting_options
        figs, ax_list = self.plot_logger_data(vector_log_sink, diagram_context)

        # If no figures are returned, then return early!
        if figs is None:
            self.add_warning_to_python_report(
                f"No figures to save for {self.port.get_name()} of system {self.port.get_system().get_name()}; plot_logger_data was empty."
            )
            return

        if len(figs) == 0:
            self.add_warning_to_python_report(
                "Zero figures to save; plot_logger_data was empty."
            )
            return # Do nothing

        # Save the figures
        figure_paths = self.file_manager.compute_path_for_each_figure(
            output_port=self.port,
            associated_log_sink=vector_log_sink,
            port_component_name=port_component_name,
        )

        if len(figs) == 1:
            # Create directory for the plots, if it doesn't already exist
            first_figure_path = figure_paths[0]
            if not first_figure_path.parent.exists():
                first_figure_path.parent.mkdir(parents=True, exist_ok=True)

            # Save the single figure
            figs[0].savefig(first_figure_path, dpi=plotting_options.plot_dpi)
            plt.close(figs[0])
        else:
            # Plot each figure within this directory
            for ii, fig_ii in enumerate(figs):
                figure_path = figure_paths[ii]
                if not figure_path.parent.exists():
                    figure_path.parent.mkdir(parents=True, exist_ok=True)

                # Save figure to file
                fig_ii.savefig(figure_path, dpi=plotting_options.plot_dpi)
                plt.close(fig_ii)

        # Close all figures when done (this should be redundant?)
        # plt.close('all')

    def system_is_multibody_plant(self) -> bool:
        """
        *Description*
        
        Returns True if the system is a MultibodyPlant.
        
        *Parameters*
        
        self : PortWatcherPlotter
            The PortWatcherPlotter object.

        *Returns*
        
        is_plant: bool
            True if the system is a MultibodyPlant.
        """
        # Setup
        system = self.port.get_system()

        # Return
        return type(system) == MultibodyPlant
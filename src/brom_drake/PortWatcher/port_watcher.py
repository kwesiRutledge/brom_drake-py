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

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import OutputPort, PortDataType, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import LogVectorOutput
from pydrake.systems.framework import Context

# Internal Imports
from brom_drake.directories import DEFAULT_PLOT_DIR, DEFAULT_RAW_DATA_DIR
from .port_watcher_options import (
    PortWatcherOptions, FigureNamingConvention,
    PortWatcherPlottingOptions, PortWatcherRawDataOptions,
)
from .port_figure_arrangement import PortFigureArrangement
from .plotter import PortWatcherPlotter


class PortWatcher:
    def __init__(
        self,
        output_port: OutputPort,
        builder: DiagramBuilder,
        logger_name: str = None,
        options: PortWatcherOptions = PortWatcherOptions(),
        plot_dir: str = DEFAULT_PLOT_DIR,
        raw_data_dir: str = DEFAULT_RAW_DATA_DIR,
    ):
        # Setup
        self.options = options
        self.port = output_port
        self.data = {}
        self.plot_handles = {}
        self.plot_handles = None
        
        # Set up directories
        self.options = self.create_new_port_watcher_options(options, plot_dir, raw_data_dir)

        # Input Processing
        if output_port.get_data_type() != PortDataType.kVectorValued:
            raise ValueError(
                f"This watcher only supports vector valued ports (i.e., of type {PortDataType.kVectorValued}.\n" +
                f"Received port of type {output_port.get_data_type()}."
            )
        system = output_port.get_system()

        if logger_name is None:
            logger_name = f"PortWatcher_{system.get_name()}_{output_port.get_name()}"

        # Preparing LogVectorSink
        self.logger = LogVectorOutput(output_port, builder)
        self.logger.set_name(logger_name)

        # Prepare optional members
        self.plotter = None
        if self.options.plotting.save_to_file:
            self.plotter = PortWatcherPlotter(
                logger=self.logger,
                port=self.port,
                plotting_options=self.options.plotting,
            )

    def create_new_port_watcher_options(
        self,
        options: PortWatcherOptions,
        plot_dir: str = DEFAULT_PLOT_DIR,
        raw_data_dir: str = DEFAULT_RAW_DATA_DIR,
    ) -> PortWatcherOptions:
        """
        Description:
            Creates a new set of PortWatcherOptions.
        :param plotting:
        :param raw_data:
        :return:
        """
        return PortWatcherOptions(
            plotting=PortWatcherPlottingOptions(
                plot_arrangement=options.plotting.plot_arrangement,
                plot_dpi=options.plotting.plot_dpi,
                save_to_file=options.plotting.save_to_file,
                base_directory=plot_dir,
                file_format=options.plotting.file_format,
                figure_naming_convention=options.plotting.figure_naming_convention,
            ),
            raw_data=PortWatcherRawDataOptions(
                save_to_file=options.raw_data.save_to_file,
                base_directory=raw_data_dir,
                file_format=options.raw_data.file_format,
            )
        )
    
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

    def save_raw_data(self, diagram_context: Context):
        """
        Description:
            Saves the raw data to a file.
        :param diagram_context:
        :return:
        """
        # Setup
        log = self.logger.FindLog(diagram_context)
        time_data_file_name, raw_data_file_name = self.time_and_raw_data_names()

        # Save time data
        log_times = log.sample_times()
        os.makedirs(Path(time_data_file_name).parent, exist_ok=True)
        np.save(time_data_file_name, log_times)

        # Save the data
        log_data = log.data()
        np.save(raw_data_file_name, log_data)

    def time_and_raw_data_names(self) -> Tuple[str, str]:
        """
        Description:
            Returns the names that will be given to the raw data for this port.
        
        :return: Tuple of strings where:
        - the first string is the name of the time data and,
        - the second string is the name of the data.
        """
        # Setup
        options = self.options
        format = options.raw_data.file_format
        raw_data_dir = options.raw_data.base_directory

        # If this has the flat naming convention, then the file should be contained within the plot_dir.
        return [
            f"{raw_data_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}_times.{format}",
            f"{raw_data_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}.{format}"
        ]

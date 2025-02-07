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

from pydrake.all import (
    RigidTransform,
)
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
from brom_drake.utils import (
    BoolToVectorSystem,
    RigidTransformToVectorSystem,
)
from brom_drake.utils.type_checking import is_rigid_transform

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
        """
        Description
        -----------
        This class is used to watch a single port of a system.

        Arguments
        ---------
        output_port: OutputPort
            The output port that will be watched.
        builder: DiagramBuilder
            The diagram builder that is used to create the logger.
        logger_name: str
            The name of the logger.
        options: PortWatcherOptions
            The options that are used to configure the watcher.
        plot_dir: str
            The directory where the plots will be saved.
        raw_data_dir: str
            The directory where the raw data will be saved.
        """
        # Setup
        self.options = options
        self.port = output_port
        self.data = {}
        self.plot_handles = {}
        self.plot_handles = None
        self.plot_dir = plot_dir
        self.raw_data_dir = raw_data_dir

        # Set up directories
        os.makedirs(self.options.plot_dir(), exist_ok=True)
        os.makedirs(self.options.raw_data_dir(), exist_ok=True)

        # Input Processing
        self.check_port_type()
        
        # Identify port's type and connect it to a logger
        system = output_port.get_system()

        if logger_name is None:
            logger_name = f"PortWatcher_{system.get_name()}_{output_port.get_name()}"

        # Preparing LogVectorSink
        self.logger = None
        self.prepare_logger(builder)
        self.logger.set_name(logger_name)

        # Prepare optional members
        self.plotter = None
        if self.options.plotting.save_to_file:
            self.plotter = PortWatcherPlotter(
                logger=self.logger,
                port=self.port,
                plotting_options=self.options.plotting,
                plot_dir=self.options.plot_dir(),
            )

    def check_port_type(self):
        """
        Description
        -----------
        Checks to see if the port is of the correct type for plotting.

        """

        # Setup
        output_port = self.port

        # Algorithm
        if output_port.get_data_type() == PortDataType.kVectorValued:
            return
        
        # Check to see if AbstractValue port contains RigidTransform
        example_allocation = output_port.Allocate()
        example_value = example_allocation.get_value()
        if is_rigid_transform(example_value):
            return
        elif type(example_value) == bool: # if the output_value is a boolean
            return

        # Raise error otherwise
        raise self.create_port_value_type_error(output_port)
    
    @staticmethod
    def create_port_value_type_error(output_port: OutputPort) -> ValueError:
        """
        Description
        -----------
        Creates an error message for the port value type.
        
        Arguments
        ---------
        output_port: OutputPort
            The output port that is causing the error.

        Returns
        -------
        ValueError
        """
        # Setup
        example_value = output_port.Allocate()

        # Return
        return ValueError(
            f"This watcher only supports output ports that are:\n" +
            f"- Vector valued ports (i.e., of type {PortDataType.kVectorValued}.\n" +
            f"- Abstract valued ports containing:\n" +
            f"  - RigidTransforms\n" +
            f"  - Booleans\n" +
            f"Received port of type {output_port.get_data_type()} with underlying type {type(example_value)}."
        )
    
    def prepare_logger(self, builder: DiagramBuilder):
        """
        Description
        -----------
        Prepares the logger for the port.
        
        Arguments
        ---------
        builder: DiagramBuilder
            The diagram builder that is used to create the logger.
        
        Returns
        -------
        None
        """
        # Setup
        system = self.port.get_system()

        # Create the logger dependent on the type of data in the port
        if self.port.get_data_type() == PortDataType.kVectorValued:
            self.logger = LogVectorOutput(self.port, builder)
        else:
            # Port must be abstract valued

            # Check to see if the port contains a value of type
            # - RigidTransform, or
            # - bool
            example_allocation = self.port.Allocate()
            example_value = example_allocation.get_value()
            if is_rigid_transform(example_value):
                # If it is, then we must create an intermediate system
                # that will convert the RigidTransform to a vector.
                converter_system = builder.AddSystem(
                    RigidTransformToVectorSystem()
                )

                # Connect the system to the port
                builder.Connect(
                    self.port,
                    converter_system.get_input_port(),
                )
                # Then connect the output of the converter to a logger
                self.logger = LogVectorOutput(
                    converter_system.get_output_port(),
                    builder,
                )
            elif type(example_value) == bool:
                # If the value is a boolean,
                # then we must create an intermediate BoolToVectorSystem
                # that will convert the boolean to a vector.
                converter_system = builder.AddSystem(
                    BoolToVectorSystem()
                )

                # Connect the system to the port
                builder.Connect(
                    self.port,
                    converter_system.get_input_port(),
                )

                # Then connect the output of the converter to a logger
                self.logger = LogVectorOutput(
                    converter_system.get_output_port(),
                    builder,
                )
            
            else:
                raise NotImplementedError(
                    f"PortWatcher does not support the type of data contained in the port."
                )
    
    def safe_system_name(self) -> str:
        """
        Description:
            Returns a safe name for the system.

        Arguments
        ---------
        None

        Returns
        -------
        name: System's name.
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

        Arguments
        ---------
        diagram_context: Context
            The context of the diagram.
        
        Returns
        -------
        Nothing
        """
        # Setup
        log = self.logger.FindLog(diagram_context)
        time_data_file_name, raw_data_file_name = self.time_and_raw_data_names()

        # Save times associated with each data point
        log_times = log.sample_times()
        os.makedirs(Path(time_data_file_name).parent, exist_ok=True)
        np.save(time_data_file_name, log_times)

        # Save the data values
        log_data = log.data()
        np.save(raw_data_file_name, log_data)

        # Announce the saving of the raw data
        

    def time_and_raw_data_names(self) -> Tuple[str, str]:
        """
        Description:
            Returns the names that will be given to the raw data for this port.
        
        Arguments
        ---------
        Nothing

        Returns
        -------
        Tuple of strings where:
        - the first string is the name of the time data and,
        - the second string is the name of the data.
        """
        # Setup
        options = self.options
        format = options.raw_data.file_format
        raw_data_dir = options.raw_data_dir()

        # If this has the flat naming convention, then the file should be contained within the plot_dir.
        return [
            f"{raw_data_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}_times.{format}",
            f"{raw_data_dir}/system_{self.safe_system_name()}_port_{self.port.get_name()}.{format}"
        ]

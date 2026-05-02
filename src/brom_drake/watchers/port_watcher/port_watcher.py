"""
PortWatcher.py
Description:

    This file defines the PortWatcher class. This class is used to watch the ports of a
    diagram.
"""

import logging
from pathlib import Path
from typing import Dict
import numpy as np
import os

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import (
    OutputPort,
    PortDataType,
    DiagramBuilder,
    LeafSystem,
)
from pydrake.systems.primitives import LogVectorOutput, VectorLogSink
from pydrake.systems.framework import Context

# Internal Imports
from brom_drake.directories import DEFAULT_BROM_DIR
from .file_manager import PortWatcherFileManager
from brom_drake.watchers.port_watcher.support_types import assert_port_is_supported
from .port_watcher_options import (
    PortWatcherOptions,
)
from .plotter import PortWatcherPlotter
from brom_drake.systems.abstract_list_selection_system import (
    AbstractListSelectionSystem,
)
from brom_drake.systems.conversion import (
    BoolToVectorSystem,
    RigidTransformToVectorSystem,
)
from brom_drake.utils.plant import get_all_associated_body_indices_in_plant
from brom_drake.utils.type_checking import is_rigid_transform

OutputPortNameLike = str


class PortWatcher:
    """
    *Description*

    The real workhorse of the :py:class:`DiagramWatcher<brom_drake.watchers.diagram_watcher.diagram_watcher.DiagramWatcher>` class.
    This class adds the elements to the drake diagram that will monitor a given
    system's output port (**output_port**), if possible.
    """

    def __init__(
        self,
        output_port: OutputPort,
        builder: DiagramBuilder,
        python_logger: logging.Logger,
        logger_name: str = None,
        options: PortWatcherOptions = PortWatcherOptions(),
        base_watcher_dir: Path = DEFAULT_BROM_DIR,
    ):
        """
        *Description*

        This class is used to watch a single port of a system.

        *Parameters*

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
        self.file_manager = PortWatcherFileManager(
            base_directory=base_watcher_dir,
            plotting_options=self.options.plotting,
            raw_data_options=self.options.raw_data,
        )
        self.python_logger = python_logger

        # Set up directories
        os.makedirs(self.file_manager.plot_dir, exist_ok=True)
        os.makedirs(self.file_manager.raw_data_dir, exist_ok=True)

        # Input Processing
        assert_port_is_supported(self.port)

        # Preparing LogVectorSink
        self._drake_vector_logs: Dict[OutputPortNameLike, VectorLogSink] = {}
        self._prepare_vector_logs(builder)

        # Prepare optional members
        self.plotter = None
        if self.options.plotting.save_to_file:
            self.plotter = PortWatcherPlotter(
                port=self.port,
                python_logger=self.python_logger,
                plotting_options=self.options.plotting,
                file_manager=self.file_manager,
            )

    def get_data_dictionary(self, diagram_context: Context) -> Dict[str, np.ndarray]:
        """
        **Description**

        Returns the data recorded by this PortWatcher as a dictionary.

        .. warning::

            The data in the dictionary is only available after the diagram has been simulated and the VectorLogSinks have recorded data.


        The keys of the dictionary are the names of the output ports being watched, and the values are the corresponding data as numpy arrays.
        Importantly, most of the time this data dictionary contains only one key-value pair, since most ports being watched are not list-valued (i.e., they do not have "components" that require multiple VectorLogSinks to monitor).
        However, in the case of list-valued ports, there will be multiple key-value pairs in the dictionary, where each key corresponds to a different component of the port (e.g., "element_0_out", "element_1_out", etc.).
        In this case, the user can use the output port name (which is included in the keys) to determine which component of the port each key-value pair corresponds to.

        .. note::

            The dimensions of the data will always be (n_data_dim, n_timesteps).
            In other words, there will be one row for each dimension of the data, and one column for each time step recorded by the VectorLogSinks.

        **Parameters**

        diagram_context: Context
            The context of the diagram.

        **Returns**

        data: Dict[str, np.ndarray]
            A dictionary containing the data from all of the VectorLogSinks
            associated with this PortWatcher. The keys are the names of the output ports being watched.
        """
        data = {}
        for output_port_name, log_sink in self._drake_vector_logs.items():
            log = log_sink.FindLog(diagram_context)
            data[output_port_name] = log.data()

        return data

    def get_timing_array(self, diagram_context: Context) -> np.ndarray:
        """
        **Description**

        Returns the timing data recorded by this PortWatcher as a numpy array.

        **Parameters**

        diagram_context: Context
            The context of the diagram.

        **Returns**
        timing: np.ndarray
            A numpy array containing the timing data from one of the VectorLogSinks
            associated with this PortWatcher. Since all of the VectorLogSinks should have the same timing, it does not matter which one we pull the timing data from.
        """
        # Check to see if we have any VectorLogSinks
        if len(self._drake_vector_logs) == 0:
            raise ValueError(
                "Cannot get timing data because no VectorLogSinks were created for this PortWatcher."
            )

        # Pull timing data from the first VectorLogSink (they should all have the same timing)
        first_log_sink = list(self._drake_vector_logs.values())[0]
        log = first_log_sink.FindLog(diagram_context)
        timing = log.sample_times()

        return timing

    def get_vector_log_sink(
        self, with_index: int = None, with_output_port_name: str = None
    ) -> VectorLogSink:
        """
        **Description**

        Returns the VectorLogSink corresponding to the given index or output port name.
        By default, this returns the first VectorLogSink in the internal dictionary.

        **Parameters**

        with_index: int, optional
            The index of the VectorLogSink to return.
            By default, None.

        with_output_port_name: str, optional
            The name of the output port of the VectorLogSink to return.
            By default, None.

        **Returns**

        vector_log_sink: VectorLogSink
            The VectorLogSink corresponding to the given index or output port name.
        """
        # Default case
        if with_index is None and with_output_port_name is None:
            all_vector_logs = list(self._drake_vector_logs.values())
            return all_vector_logs[0]

        # If either parameter is provided, use it to select the log
        if with_index is not None:
            all_vector_logs = list(self._drake_vector_logs.values())
            return all_vector_logs[with_index]
        elif with_output_port_name is not None:
            return self._drake_vector_logs[with_output_port_name]
        else:
            raise ValueError(
                "Either with_index or with_output_port_name must be provided."
            )

    def _name_vector_log_sink(
        self,
        current_output_port: OutputPort,
    ):
        """
        **Description**

        Provides a name to the VectorLogSink stored in `self.drake_vector_logs`
        that corresponds to the given system and output_port.
        """
        # Setup
        original_output_port = self.port
        system = original_output_port.get_system()

        # Name the VectorLogSink
        name = f"PortWatcher_{system.get_name()}_{original_output_port.get_name()}"
        if current_output_port.get_name() != original_output_port.get_name():
            name += f"_{current_output_port.get_name()}"

        self._drake_vector_logs[current_output_port.get_name()].set_name(name)

    def _prepare_vector_log_for_rigid_transform_port(
        self,
        current_output_port: OutputPort,
        builder: DiagramBuilder,
    ):
        """
        *Description*

        Adds to the current Drake Diagram a VectorLogSink (and helper system) to measure the value of the RigidTransform
        value coming from the **PortWatcher**'s target output port.

        *Parameters*

        builder: DiagramBuilder
            The diagram builder that is used to create the VectorLogSink.
        """
        # Determine the name of the system that converts
        # RigidTransform objects to vectors
        converter_name = f"RigidTransformToVectorSystem_{self.safe_system_name(current_output_port.get_system())}"

        # First, let's create an intermediate system
        # that will convert the RigidTransform to a vector that is easily logged
        converter_system = builder.AddNamedSystem(
            system=RigidTransformToVectorSystem(),
            name=converter_name,
        )

        # Connect the system to the port
        builder.Connect(
            current_output_port,
            converter_system.get_input_port(),
        )

        # Then connect the output of the converter to a VectorLogSink
        self._drake_vector_logs[current_output_port.get_name()] = LogVectorOutput(
            converter_system.get_output_port(),
            builder,
        )

        # And finally, name the vector log sink (must have unique names to compile the diagram)
        self._name_vector_log_sink(current_output_port=current_output_port)

    def _prepare_vector_log_for_abstract_valued_port(
        self,
        current_output_port: OutputPort,
        builder: DiagramBuilder,
    ):
        """
        *Description*

        .. note::

            While this is usually called with the port ``self.port``,
            it can be called with other ports (as is done with list[T] inputs).
        """
        # Collect the system and output port (their names will be used in the default name)
        system: LeafSystem = current_output_port.get_system()

        # Check to see if the port contains a value of type
        # - RigidTransform, or
        # - bool
        example_allocation = current_output_port.Allocate()
        example_value = example_allocation.get_value()

        # Address the case of list type inputs first
        if type(example_value) is list:
            # A list input should NEVER have zero length
            assert (
                len(example_value) > 0
            ), f"All list output ports should contain at least 1 example allocation value; received 0 from port {current_output_port.get_name()}"

            # If the value is a list, then we will use
            # a special system to select each element of the list
            # and then recursively call this function.

            for idx in range(len(example_value)):
                output_port_name = f"element_{idx}_out"

                # If this is a very specific port (body_poses),
                # then propose a very specific name for the output port
                if current_output_port.get_name() == "body_poses":
                    # TODO(Kwesi): Include test to see if
                    # this system is a multibody plant
                    system_is_multibody_plant = (
                        type(current_output_port.get_system()) is MultibodyPlant
                    )
                    if system_is_multibody_plant:
                        # Extract all body names
                        system_as_multibody_plant: MultibodyPlant = (
                            current_output_port.get_system()
                        )
                        body_indices = get_all_associated_body_indices_in_plant(
                            system_as_multibody_plant
                        )
                        current_body_name = system_as_multibody_plant.get_body(
                            body_index=body_indices[idx]
                        ).name()

                        output_port_name = f"{current_body_name}"

                # Create selection system for this iteration
                selection_i = builder.AddNamedSystem(
                    system=AbstractListSelectionSystem(
                        idx,
                        output_type=type(example_value[0]),
                        output_port_name=output_port_name,
                    ),
                    name=f'System {system.get_name()}\'s Port "{current_output_port.get_name()}" Element #{idx}',
                )

                # Connect selection system to target port
                builder.Connect(current_output_port, selection_i.get_input_port())

                # Use recursion to assign the proper logger to the output of selection_i
                self._prepare_vector_log_for_abstract_valued_port(
                    current_output_port=selection_i.get_output_port(),
                    builder=builder,
                )

            # After the recursion, we should be able to return
            return

        # Non-list types
        if is_rigid_transform(example_value):
            self._prepare_vector_log_for_rigid_transform_port(
                current_output_port,
                builder,
            )
            self._name_vector_log_sink(current_output_port=current_output_port)

        elif type(example_value) == bool:
            # If the value is a boolean,
            # then we must create an intermediate BoolToVectorSystem
            # that will convert the boolean to a vector.
            converter_system = builder.AddSystem(BoolToVectorSystem())

            # Connect the system to the port
            builder.Connect(
                current_output_port,
                converter_system.get_input_port(),
            )

            # Then connect the output of the converter to a logger
            self._drake_vector_logs[current_output_port.get_name()] = LogVectorOutput(
                converter_system.get_output_port(),
                builder,
            )
            self._name_vector_log_sink(current_output_port=current_output_port)

        else:
            raise NotImplementedError(
                f"PortWatcher does not support the type of data ({type(example_value)}) contained in the port."
            )

    def _prepare_vector_logs(self, builder: DiagramBuilder):
        """
        *Description*

        Prepares any VectorLogSink's needed to measure the current port.
        In most cases, only one VectorLogSink is needed.

        *Parameters*

        builder: DiagramBuilder
            The diagram builder that is used to create the logger.

        """
        # Create ALL of the VectorLogSink's needed
        # for each type of data in the port
        if self.port.get_data_type() == PortDataType.kVectorValued:
            self._drake_vector_logs[self.port.get_name()] = LogVectorOutput(
                self.port, builder
            )
            self._name_vector_log_sink(current_output_port=self.port)
        else:
            # Port must be abstract valued
            self._prepare_vector_log_for_abstract_valued_port(self.port, builder)

        # Announce the preparation of the VectorLogSink's
        # with the logger
        for target_port_name, log_sink in self._drake_vector_logs.items():
            self.python_logger.info(
                f'PortWatcher prepared VectorLogSink for system "{self.port.get_system().get_name()}"\'s '
                f'port "{target_port_name}".\n'
                + f' - It\'s name is "{log_sink.get_name()}".'
            )

    def safe_system_name(self, system: LeafSystem = None) -> str:
        """
        *Description*

        Returns a safe name for the system.

        *Returns*

        system: LeafSystem, optional
            The system for which to generate a safe name. If None, uses the system of the
            PortWatcher\'s target port.
        """
        # Setup
        if system is None:
            system = self.port.get_system()

        out = system.get_name()

        # First, let's check to see how many "/" exist in the name
        slash_occurences = [i for i, letter in enumerate(out) if letter == "/"]
        if len(slash_occurences) > 0:
            out = out[
                slash_occurences[-1] + 1 :
            ]  # truncrate string based on the last slash

        # Second, replace all spaces with underscores
        out = out.replace(" ", "_")

        return out

    def save_all_figures(self, diagram_context: Context):
        """
        *Description*

        Saves all of the figures that the PortWatcher should according to its options.
        This may generate one figure for each dimension of the output port being watched,
        or just one figure for the entire port.

        *Parameters*

        diagram_context: Context
            The context of the diagram.
        """
        # Test to see if we have a plotter
        assert (
            self.plotter is not None
        ), "Cannot save figures because no plotter was created for this PortWatcher."

        # For each vector log, save one figure
        for output_port_name, log_sink in self._drake_vector_logs.items():
            # Decide on whether or not there are "components"
            # in this output port (i.e., multiple values in a list output; different from a vector with multiple dimensions)
            port_component_name = None
            if output_port_name != self.port.get_name():
                port_component_name = output_port_name

            # Call the plotter to save the figure
            self.plotter.save_figures(
                log_sink, diagram_context, port_component_name=port_component_name
            )

    def save_raw_data(self, diagram_context: Context):
        """
        **Description**

        Saves the raw data to file(s). The number of files depends on the components in the data AND the options of the PortWatcher (i.e., whether to save each component in a separate file or all in one file).

        **Arguments**

        diagram_context: Context
            The context of the diagram.

        **Notes**

        TODO(Kwesi): Take advantage of get_data_dictionary function to clean up this code.
        """
        # Test to see the number of logs we have to save data for
        n_vector_logs = len(list(self._drake_vector_logs))

        # Iterate through all of the available VectorLogSink's
        for output_port_name, log_sink in self._drake_vector_logs.items():
            # Collect Drake Log of Data
            log = log_sink.FindLog(diagram_context)

            port = self.port
            system_containing_port: LeafSystem = port.get_system()

            # Write the data to file
            # - time data
            time_data_file_name = self.file_manager.time_data_file_path(
                output_port=port,
            )

            log_times = log.sample_times()
            os.makedirs(time_data_file_name.parent, exist_ok=True)
            np.save(time_data_file_name, log_times)

            # - data values
            raw_data_file: Path = None
            if n_vector_logs == 1:
                raw_data_file = self.file_manager.raw_data_file_path(
                    output_port=port,
                )
            else:
                raw_data_file = self.file_manager.raw_data_file_path(
                    output_port=port,
                    port_component_name=output_port_name,
                )

                if raw_data_file.parent.exists() is False:
                    raw_data_file.parent.mkdir(parents=True, exist_ok=True)

            log_data = log.data()
            np.save(raw_data_file, log_data)

            # Announce the saving of the raw data
            # TODO(Kwesi): Include the file name in the log message

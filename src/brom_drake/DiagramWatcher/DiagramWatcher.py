"""
DiagramWatcher.py
Description:

    Creates a DiagramWatcher class that can be used to monitor the state of a Diagram
    and automatically log signals of interest.
"""
import os
from typing import List, Union
import matplotlib.pyplot as plt
import numpy as np
import loguru

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder, LeafSystem, PortDataType
from pydrake.systems.primitives import (
    VectorLogSink, ConstantVectorSource, AffineSystem, LogVectorOutput,
)

# Internal Imports
from brom_drake.DiagramTarget import DiagramTarget
from brom_drake.PortWatcher.port_watcher import PortWatcher
from brom_drake.PortWatcher.port_watcher_options import PortWatcherOptions, PortWatcherPlottingOptions, PortWatcherRawDataOptions
from brom_drake.DiagramWatcher.diagram_watcher_options import DiagramWatcherOptions
from brom_drake.DiagramWatcher.constants import INELIGIBLE_SYSTEM_TYPES
from brom_drake.DiagramWatcher.errors import UnrecognizedTargetError


class DiagramWatcher:
    def __init__(
        self,
        subject: DiagramBuilder,
        targets: List[DiagramTarget] = None,
        options: DiagramWatcherOptions = DiagramWatcherOptions(),
    ):
        """
        Description
        -----------
        Initializes the DiagramWatcher class.

        Arguments
        ---------
        subject : DiagramBuilder
            We will search through the subject (a diagram builder)
            to find all the systems that we want to monitor.
        targets : List[DiagramTarget], optional
            The targets that we want to monitor, by default None
        port_watcher_options : PortWatcherOptions, optional
            The options for the PortWatcher, by default PortWatcherOptions()
        """
        # Setup

        # Needs to be populated by the user of this class AFTER the diagram has been built
        self.diagram = None
        self.diagram_context = None

        # Check subject
        if not isinstance(subject, DiagramBuilder):
            raise ValueError("subject must be a DiagramBuilder!")

        # Save the inputs
        self.subject = subject
        self.options = options

        # Create the .brom directory, to store:
        # - activity_summary.log
        # - all plots
        if os.path.exists(self.options.base_directory):
            os.system(f"rm -r {self.options.base_directory}")

        # Create directory to plot in
        os.makedirs(self.options.base_directory, exist_ok=True)
        self.configure_brom_activity_summary()  # Create an "activity summary" log
                                                # which details what the
                                                # DiagramWatcher is doing.

        # Collect All the Connections and Systems
        self.eligible_systems = self.find_eligible_systems(subject)
        if targets is None:
            targets = [DiagramTarget(system.get_name()) for system in self.eligible_systems]
        else:
            self.check_targets(targets, self.eligible_systems)

        # Log the list of eligible systems
        loguru.logger.info(f"Found {len(self.eligible_systems)} systems in diagram are eligible for targeting:")
        for idx, system in enumerate(self.eligible_systems):
            loguru.logger.info(f"{idx}: {system.get_name()}")

        # For Each Target with None ports, we will try to
        # "smartly" create the targets that we want to monitor
        inferred_targets = self.get_smart_targets(subject, targets)
        self.inferred_targets = inferred_targets

        # For each target's port, we will add a logger
        self.port_watchers = {target.name: {} for target in inferred_targets}
        loguru.logger.info("Adding loggers to the diagram... (via PortWatcher objects)")
        for target in inferred_targets:
            system = subject.GetSubsystemByName(target.name)
            for port_index in target.ports:
                target_port = system.get_output_port(port_index)
                options_for_target_port = options.to_port_watcher_options()

                try:
                    # Configure PortWatcher
                    self.port_watchers[target.name][target_port.get_name()] = PortWatcher(
                        target_port, subject,
                        logger_name=f"{target.name}_logger_{port_index}",
                        plot_dir=options_for_target_port.plot_dir(),
                        options=options_for_target_port,
                    )
                except Exception as e:
                    if not self.options.hide_messages.during_port_watcher_connection:
                        print(f"[Warning] Unable to log port named \"{target_port.get_name()}\" of system \"{target.name}\". See log file ({options_for_target_port.plot_dir()}/activity_summary.log) for more details.")
                    
                    loguru.logger.warning(
                        f"There was an error attempting to add a watcher to port {target_port.get_name()} of system {target.name}"
                    )
                    loguru.logger.warning(f"Error: {e}")
                    continue

                # Announce that we successfully added logger
                loguru.logger.info(f"Added logger to port {target_port.get_name()} of system {target.name}")

    def create_new_port_watcher_options(
        self,
        options: PortWatcherOptions,
        plot_dir: str = None,
        raw_data_dir: str = None,
    ) -> PortWatcherOptions:
        """
        Description
        -----------
        Creates a new set of PortWatcherOptions with the given options.

        Arguments
        ---------
        options : PortWatcherOptions
            The options to use as a base.
        plot_dir : str, optional
            The directory to save the plots in, by default None
        raw_data_dir : str, optional
            The directory to save the raw data in, by default None
        """
        # Setup
        new_plot_dir = options.plotting.base_directory
        if plot_dir is not None: 
            new_plot_dir = plot_dir

        new_raw_data_dir = options.raw_data.base_directory
        if raw_data_dir is not None: 
            new_raw_data_dir = raw_data_dir

        # Return the new options
        return PortWatcherOptions(
            plotting=PortWatcherPlottingOptions(
                plot_arrangement=options.plotting.plot_arrangement,
                plot_dpi=options.plotting.plot_dpi,
                save_to_file=options.plotting.save_to_file,
                base_directory=new_plot_dir,
                file_format=options.plotting.file_format,
                figure_naming_convention=options.plotting.figure_naming_convention,
            ),
            raw_data=PortWatcherRawDataOptions(
                save_to_file=options.raw_data.save_to_file,
                base_directory=new_raw_data_dir,
                file_format=options.raw_data.file_format,
            ),
        )

    def __del__(self):
        """
        Description
        -----------
        Destructor for the Diagram Watcher.
        Will plot the data from all of our loggers if we have access to the diagram context.
        """
        # Setup

        is_ready_to_plot = self.diagram is not None

        if not is_ready_to_plot:
            return # Return early if we don't have access to the diagram context

        # Upon deletion, we will PLOT the data from all of our loggers
        # if we have access to the diagram context
        self.save_figures()
        self.save_raw_data()

    def configure_brom_activity_summary(self):
        """
        Description:
            Configures the "activity summary" a log of brom's activity.
        :return:
        """
        # Setup
        options = self.options

        # Configure the logger
        loguru.logger.remove()  # Remove the default logger
        loguru.logger.add(options.base_directory + "/activity_summary.log")

    def check_targets(
        self,
        targets: List[DiagramTarget],
        eligible_systems: List[Union[MultibodyPlant, AffineSystem, LeafSystem]],
    ) -> List[Union[MultibodyPlant, AffineSystem, LeafSystem]]:
        """
        Description
        -----------
        Finds the systems specified by the targets list that
        we want to watch/monitor.
        We will try to ignore all systems that are:
        - Scene Graphs
        - Loggers
        and raise an error if the target is not found in the eligible systems.

        Arguments
        ---------
        targets : List[DiagramTarget]
            The targets that we want to monitor.
        eligible_systems : List[Union[MultibodyPlant, AffineSystem, LeafSystem]]
            The systems that are eligible for monitoring.
        :return:
        """

        # Find all the systems that are eligible for logging
        eligible_system_dict = {
            system.get_name(): system for system in eligible_systems
        }

        # Search for each target in eligible_systems
        targeted = []
        for target in targets:
            # Check if the target name is in the eligible systems
            if target.name not in eligible_system_dict.keys():
                raise UnrecognizedTargetError(target, eligible_system_dict.keys())

            # If it is, then also check that the port index is correct
            if target.ports is None:
                continue  # No need to check things if ports is None

            num_ports_in_target = eligible_system_dict[target.name].num_output_ports()
            for port_index in target.ports:
                if port_index < 0 or port_index >= num_ports_in_target:
                    raise ValueError(f"Port index {port_index} is out of bounds for system {target.name} (only {num_ports_in_target} ports exist)")

        # All checks passed!
        pass

    def find_eligible_systems(
        self,
        builder: DiagramBuilder
    ) -> List[Union[MultibodyPlant, AffineSystem, LeafSystem]]:
        """
        Description:
            Finds all the systems that are eligible for logging.
            We want to ignore all systems that are:
            - Scene Graphs
            - Loggers
        """

        # Find all the systems that are eligible for logging
        eligible_systems = []

        loguru.logger.info("Finding all eligible systems for logging...")
        for system in builder.GetSystems():
            if type(system) in INELIGIBLE_SYSTEM_TYPES:
                loguru.logger.warning(
                    f"System {system.get_name()} (of type {type(system)}) is not eligible for logging! Skipping..."
                )
                continue

            # Otherwise add to list
            eligible_systems.append(system)

        return eligible_systems

    def get_smart_targets(
        self,
        subject: DiagramBuilder,
        targets: List[DiagramTarget],
    ) -> List[DiagramTarget]:
        """
        Description:
            For each target with None ports, we will try to
            "smartly" create the targets that we want to monitor.
        :param subject:
        :param targets:
        :return:
        """
        # Setup
        smart_targets = []

        for target in targets:
            if target.ports is not None:
                smart_targets.append(target)
                continue

            # If ports is None, then we will try to "smartly" create the ports
            system = subject.GetSubsystemByName(target.name)

            # TODO: Add support for investigating output ports
            num_ports = system.num_output_ports()
            if num_ports == 0:
                loguru.logger.warning(f"System {target.name} has no output ports! Skipping...")
                continue

            output_ports_to_watch = [port_index for port_index in range(num_ports)]

            # Add a target with the connected ports to the smart_targets list
            smart_targets.append(
                DiagramTarget(target.name, output_ports_to_watch),
            )

        # Log the list of inferred targets
        loguru.logger.info(f"Found {len(smart_targets)} inferred targets:")
        for idx, target in enumerate(smart_targets):
            loguru.logger.info(f"{idx}: {target.name} - {target.ports}")

        return smart_targets

    def save_figures(self):
        """
        Description:
            Saves all the figures from the port watchers.
        :return:
        """
        # Announce Saving Figures has started
        loguru.logger.info("Saving figures...")

        # Algorithm
        for system_name in self.port_watchers:
            system_ii = self.diagram.GetSubsystemByName(system_name)
            ports_on_ii = self.port_watchers[system_name]

            loguru.logger.info(f"Saving figures for system {system_name}...")

            for port_name in ports_on_ii:
                temp_port_watcher = ports_on_ii[port_name]
                temp_plotting_options = temp_port_watcher.options.plotting
                if temp_plotting_options.save_to_file: # Plot only if the PortWatcher flag is set
                    temp_port_watcher.plotter.save_figures(self.diagram_context)
                    loguru.logger.info(f"Saved figures for port {port_name} on system {system_name}")

    def save_raw_data(self):
        """
        Description:
            Saves all the raw data from the port watchers.
        :return:
        """
        # Announce Saving Raw Data has started
        loguru.logger.info("Saving raw data...")

        # Algorithm
        for system_name in self.port_watchers:
            system_ii = self.diagram.GetSubsystemByName(system_name)
            ports_on_ii = self.port_watchers[system_name]

            loguru.logger.info(f"Saving raw data for system {system_name}...")

            for port_name in ports_on_ii:
                temp_port_watcher = ports_on_ii[port_name]
                temp_raw_data_options = temp_port_watcher.options.raw_data
                if temp_raw_data_options.save_to_file:
                    temp_port_watcher.save_raw_data(self.diagram_context)

                    loguru.logger.info(f"Saved raw data for port {port_name} on system {system_name}")

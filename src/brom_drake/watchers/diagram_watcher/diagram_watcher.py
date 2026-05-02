"""
DiagramWatcher.py
Description:

    Creates a DiagramWatcher class that can be used to monitor the state of a Diagram
    and automatically log signals of interest.
"""

import os
from typing import Dict, List, Union
import matplotlib.pyplot as plt
import numpy as np
from pathlib import Path
import logging
import warnings

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder, LeafSystem, PortDataType
from pydrake.systems.primitives import (
    VectorLogSink,
    ConstantVectorSource,
    AffineSystem,
    LogVectorOutput,
)

# Internal Imports
from brom_drake.watchers.diagram_target import DiagramTarget
from brom_drake.watchers.port_watcher.port_watcher import PortWatcher
from brom_drake.watchers.port_watcher.port_watcher_options import (
    PortWatcherOptions,
    PortWatcherPlottingOptions,
    PortWatcherRawDataOptions,
)
from brom_drake.watchers.diagram_watcher.diagram_watcher_options import (
    DiagramWatcherOptions,
)
from brom_drake.watchers.diagram_watcher import constants
from brom_drake.watchers.diagram_watcher.errors import (
    PortIsNotFoundInDiagramError,
    PortIsNotBeingWatchedError,
    SystemIsNotFoundInDiagramError,
    SystemIsNotBeingWatchedError,
)


class DiagramWatcher:
    """
    **Description**

    An object that will iterate through all elements of a partially built
    Drake Diagram (via the DiagramBuilder) and add PortWatchers to the specified targets.

    **Parameters**

    subject: DiagramBuilder
        We will search through the subject (a diagram builder)
        to find all the systems that we want to monitor.

    targets: List[DiagramTarget], optional
        The targets that we want to monitor, by default this tries to monitor
        all systems in the diagram. (i.e., when this value is None, we will monitor all systems).

    options: DiagramWatcherOptions, optional
        The options that configure the DiagramWatcher, by default DiagramWatcherOptions()
    """

    def __init__(
        self,
        subject: DiagramBuilder,
        targets: List[DiagramTarget] = None,
        options: DiagramWatcherOptions = DiagramWatcherOptions(),
    ):
        """
        **Description**

        Initializes the DiagramWatcher class.

        **Parameters**

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
        if self.options.base_directory.exists():
            import shutil
            shutil.rmtree(self.options.base_directory)

        # Create directory to plot in
        self.options.base_directory.mkdir(parents=True, exist_ok=True)
        self.logger = self._create_logging_logger()  # Create an "activity summary" log
        # which details what the
        # DiagramWatcher is doing.

        # Collect All the Connections and Systems
        self.eligible_systems = self._find_eligible_systems(subject)
        if targets is None:
            targets = [
                DiagramTarget(system.get_name()) for system in self.eligible_systems
            ]
        else:
            self._check_targets(targets, self.eligible_systems)

        # Log the list of eligible systems
        self.logger.info(
            f"Found {len(self.eligible_systems)} systems in diagram are eligible for targeting:"
        )
        for idx, system in enumerate(self.eligible_systems):
            self.logger.info(f"{idx}: {system.get_name()}")

        # For Each Target with None ports, we will try to
        # "smartly" create the targets that we want to monitor
        inferred_targets = self._get_smart_targets(subject, targets)
        self.inferred_targets = inferred_targets

        # For each target's port, we will add a logger
        self._port_watchers: Dict[str, Dict[str, PortWatcher]] = {
            target.name: {} for target in inferred_targets
        }
        self.logger.info("Adding loggers to the diagram... (via PortWatcher objects)")
        for target in inferred_targets:
            system = subject.GetSubsystemByName(target.name)
            for port_index in target.ports:
                target_port = system.get_output_port(port_index)
                options_for_target_port = options.to_port_watcher_options()

                try:
                    # Configure PortWatcher
                    self._port_watchers[target.name][target_port.get_name()] = (
                        PortWatcher(
                            target_port,
                            subject,
                            self.logger,
                            logger_name=f"{target.name}_logger_{port_index}",
                            base_watcher_dir=options.base_directory,
                            options=options_for_target_port,
                        )
                    )

                except Exception as e:
                    if self.options.hide_messages.during_port_watcher_connection:
                        continue

                    # If we have an error, we will log it
                    self.logger.debug(
                        f"Failed to add a watcher to port {target_port.get_name()} of system {target.name}: {e}",
                        exc_info=False,
                    )

                if target_port.get_name() in self._port_watchers[target.name]:
                    # Announce that we successfully added logger
                    self.logger.info(
                        f"Added logger to port {target_port.get_name()} of system {target.name}"
                    )
                else:
                    # If we did not add the logger, then we will send a small note.
                    self.logger.info(
                        f"Unable to add logger to port {target_port.get_name()} of system {target.name}",
                    )

    def __del__(self):
        """
        **Description**

        During destruction of the DiagramWatcher object, we will try to:
        - Save all the figures from the PortWatchers
        - Save all the raw data from the PortWatchers
        - Close all logging handlers in the logger and the remove them
        """
        # Setup

        is_ready_to_plot = self.diagram is not None
        is_ready_to_plot = is_ready_to_plot and self.diagram_context is not None

        if not is_ready_to_plot:
            return  # Return early if we don't have access to the diagram context

        # Upon deletion, we will PLOT the data from all of our loggers
        # if we have access to the diagram context
        self._save_figures()
        self._save_raw_data()

        # Close all logging handlers in the logger and the remove them
        for handler in self.logger.handlers:
            handler.close()
            self.logger.removeHandler(handler)

    def _create_logging_logger(self) -> logging.Logger:
        """
        **Description**

        Configures the "activity summary" a log of brom's activity.

        **Returns**

        logger: logging.Logger
            The configured logger for LOG MESSAGES.
            In other words, this is not a logger of signals from the diagram.
        """
        # Setup
        options = self.options

        # Create the basic logger
        logger = logging.getLogger("brom_drake.DiagramWatcher")
        for handler in logger.handlers:
            # Remove all existing handlers
            logger.removeHandler(handler)

        # Create a file handler, if none exists

        # Create a logging directory if it does not exist
        options.base_directory.mkdir(parents=True, exist_ok=True)

        # Create a file handler
        file_handler = logging.FileHandler(
            filename=options.base_directory / "activity_summary.log",
            mode="w",  # Append mode
        )
        file_handler.setLevel(logging.DEBUG)

        # Create a formatter and set it for the file handler
        formatter = logging.Formatter(
            "%(asctime)s | %(levelname)s | %(name)s | %(message)s",
            datefmt="%Y-%m-%d %H:%M:%S",
        )
        file_handler.setFormatter(formatter)

        # Add the file handler to the logger
        logger.addHandler(file_handler)

        # Create a terminal handler
        terminal_handler = logging.StreamHandler()
        terminal_handler.setLevel(
            logging.WARNING
        )  # Set to WARNING to avoid cluttering terminal with INFO messages
        terminal_handler.setFormatter(formatter)
        logger.addHandler(terminal_handler)

        # Avoid duplicate logs
        logger.propagate = False

        # Make sure the logger responds to all messages of level DEBUG and above
        logger.setLevel(logging.DEBUG)  # Set to DEBUG to capture all messages

        return logger

    def _check_targets(
        self,
        targets: List[DiagramTarget],
        eligible_systems: List[Union[MultibodyPlant, AffineSystem, LeafSystem]],
    ) -> List[Union[MultibodyPlant, AffineSystem, LeafSystem]]:
        """
        **Description**

        Finds the systems specified by the targets list (i.e., what we want to watch/monitor)
        in the list of eligible systems (i.e., the systems that are in the diagram and are of eligible types).

        We will try to ignore all systems that are:
        - Scene Graphs
        - Loggers
        and raise an error if the target is not found in the eligible systems.

        **Parameters**

        targets : List[DiagramTarget]
            The targets that we want to monitor.

        eligible_systems : List[Union[MultibodyPlant, AffineSystem, LeafSystem]]
            The systems that are eligible for monitoring.

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
                raise SystemIsNotFoundInDiagramError(
                    target, eligible_system_dict.keys()
                )

            # If it is, then also check that the port index is correct
            if target.ports is None:
                continue  # No need to check things if ports is None

            num_ports_in_target = eligible_system_dict[target.name].num_output_ports()
            for port_index in target.ports:
                if port_index < 0 or port_index >= num_ports_in_target:
                    raise PortIsNotFoundInDiagramError(
                        target=target,
                        port_reference=port_index,
                        port_names=[
                            eligible_system_dict[target.name]
                            .get_output_port(port_idx)
                            .get_name()
                            for port_idx in range(
                                eligible_system_dict[target.name].num_output_ports()
                            )
                        ],
                    )

        # All checks passed!
        pass

    def _find_eligible_systems(
        self, builder: DiagramBuilder
    ) -> List[Union[MultibodyPlant, AffineSystem, LeafSystem]]:
        """
        **Description**

        Finds all the systems that are eligible for logging with either:
        - Drake's VectorLog object, or
        - Brom_drake's special logging utilities.

        We want to ignore all systems that are:

        - Scene Graphs
        - Loggers

        """

        # Find all the systems that are eligible for logging
        eligible_systems = []

        self.logger.info("Finding all eligible systems for logging...")
        for system in builder.GetSystems():
            if type(system) in constants.INELIGIBLE_SYSTEM_TYPES:
                self.logger.warning(
                    f"System {system.get_name()} (of type {type(system)}) is not eligible for logging! Skipping..."
                )
                continue

            # Otherwise add to list
            eligible_systems.append(system)
            self.logger.info(
                f"System {system.get_name()} (of type {type(system)}) is eligible for logging with the watcher."
            )

        return eligible_systems

    def get_all_port_watchers_for_system(
        self, system_name: str
    ) -> Dict[str, PortWatcher]:
        """
        **Description**

        Gets all the PortWatcher objects for a given system name.

        **Parameters**

        system_name : str
            The name of the system.

        **Returns**
        port_watchers : Dict[str, PortWatcher]
            A dictionary of port name to PortWatcher object for the given system name.
        """
        if system_name not in self._port_watchers:
            raise SystemIsNotBeingWatchedError(
                target=DiagramTarget(system_name),
                system_names=[system_name for system_name in self._port_watchers],
            )

        return self._port_watchers[system_name]

    def get_port_watcher(self, system_name: str, port_name: str) -> PortWatcher:
        """
        **Description**

        Gets the PortWatcher object for a given system name and port name.

        **Parameters**

        system_name : str
            The name of the system.

        port_name : str
            The name of the port.

        **Returns**

        port_watcher : PortWatcher
            The PortWatcher object for the given system name and port name.
        """
        if system_name not in self._port_watchers:
            raise SystemIsNotBeingWatchedError(
                target=DiagramTarget(system_name),
                system_names=[system_name for system_name in self._port_watchers],
            )

        if port_name not in self._port_watchers[system_name]:
            raise PortIsNotBeingWatchedError(
                target=DiagramTarget(system_name, ports=[port_name]),
                port_reference=port_name,
                port_names=[
                    port_name for port_name in self._port_watchers[system_name]
                ],
            )

        return self._port_watchers[system_name][port_name]

    @property
    def port_watchers(self) -> Dict[str, Dict[str, PortWatcher]]:
        """
        **Description**

        Returns the internal dictionary of port watchers.

        .. deprecated::
            Use `get_all_port_watchers_for_system()` or `get_port_watcher()` instead.

        **Returns**

        port_watchers : Dict[str, Dict[str, PortWatcher]]
            A nested dictionary of system names to port names to PortWatcher objects.
        """
        warnings.warn(
            "The 'port_watchers' property is deprecated. "
            "Use 'get_all_port_watchers_for_system()' or 'get_port_watcher()' instead.",
            DeprecationWarning,
            stacklevel=2,
        )
        return self._port_watchers

    def _get_smart_targets(
        self,
        subject: DiagramBuilder,
        targets: List[DiagramTarget],
    ) -> List[DiagramTarget]:
        """
        **Description**

        For each target with None ports, we will try to
        "smartly" create the targets that we want to monitor.

        **Parameters**

        subject : DiagramBuilder
            The diagram builder that contains the systems.

        targets : List[DiagramTarget]
            The targets that we want to monitor.

        **Returns**

        List[DiagramTarget]
            The list of targets with inferred ports.
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
                self.logger.warning(
                    f"System {target.name} has no output ports! Skipping..."
                )
                continue

            output_ports_to_watch = [port_index for port_index in range(num_ports)]

            # Add a target with the connected ports to the smart_targets list
            smart_targets.append(
                DiagramTarget(target.name, output_ports_to_watch),
            )

        # Log the list of inferred targets
        self.logger.info(f"Found {len(smart_targets)} inferred targets:")
        for idx, target in enumerate(smart_targets):
            self.logger.info(f"{idx}: {target.name} - {target.ports}")

        return smart_targets

    def _save_figures(self):
        """
        **Description**

        Saves all the figures made from plotting data
        currently saved in the known port watchers.
        """
        # Announce Saving Figures has started
        self.logger.info("Saving figures...")

        # Algorithm
        for system_name in self._port_watchers:
            system_ii = self.diagram.GetSubsystemByName(system_name)
            ports_on_ii = self._port_watchers[system_name]

            self.logger.info(f"Saving figures for system {system_name}...")

            for port_name in ports_on_ii:
                temp_port_watcher = ports_on_ii[port_name]
                temp_plotting_options = temp_port_watcher.options.plotting

                # Plot only if the PortWatcher flag is set
                if temp_plotting_options.save_to_file:
                    try:
                        temp_port_watcher.save_all_figures(self.diagram_context)
                        self.logger.info(
                            f"Saved figures for port {port_name} on system {system_name}"
                        )
                    except Exception as e:
                        self.logger.error(
                            f"Failed to save figures for port {port_name} on system {system_name}: {e}"
                        )
                else:
                    self.logger.info(
                        f"Skipped plotting for port {port_name} on system {system_name} (flag not set)"
                    )

    def _save_raw_data(self):
        """
        **Description**

        Saves all the raw data from the port watchers.
        """
        # Announce Saving Raw Data has started
        self.logger.info("Saving raw data...")

        # Algorithm
        for system_name in self._port_watchers:
            system_ii = self.diagram.GetSubsystemByName(system_name)
            ports_on_ii = self._port_watchers[system_name]

            self.logger.info(f"Saving raw data for system {system_name}...")

            for port_name in ports_on_ii:
                temp_port_watcher = ports_on_ii[port_name]
                temp_raw_data_options = temp_port_watcher.options.raw_data

                # Save raw data only if the PortWatcher flag is set
                if temp_raw_data_options.save_to_file:
                    try:
                        temp_port_watcher.save_raw_data(self.diagram_context)

                        self.logger.info(
                            f"Saved raw data for port {port_name} on system {system_name}"
                        )

                    except Exception as e:
                        self.logger.error(
                            f"Failed to save raw data for port {port_name} on system {system_name}: {e}"
                        )

                else:
                    self.logger.info(
                        f"Skipped saving raw data for port {port_name} on system {system_name} (flag not set)"
                    )

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

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder, LeafSystem, PortDataType
from pydrake.systems.primitives import (
    VectorLogSink, ConstantVectorSource, AffineSystem, LogVectorOutput,
)

from brom_drake.all import DiagramTarget
from .constants import INELIGIBLE_SYSTEM_TYPES
from .errors import UnrecognizedTargetError

DiagramLike = Union[DiagramBuilder, Diagram]


class DiagramWatcher:
    def __init__(
        self,
        subject: DiagramLike,
        targets: List[DiagramTarget] = None,
        plot_dir: str = "./.brom",
    ):
        # Setup
        self.diagram_context = None
        self.diagram = None

        # Save the inputs
        self.subject = subject
        self.plot_dir = plot_dir

        # Collect All the Connections and Systems
        self.eligible_systems = self.find_eligible_systems(subject)
        if targets is None:
            targets = [DiagramTarget(system.get_name()) for system in self.eligible_systems]
        else:
            self.check_targets(targets, self.eligible_systems)

        # TODO: Add support for giving a Diagram object as input
        if isinstance(subject, Diagram):
            raise NotImplementedError("Diagram object as input is not supported yet.")

        if isinstance(subject, DiagramBuilder):
            # For Each Target with None ports, we will try to
            # "smartly" create the targets that we want to monitor
            inferred_targets = self.get_smart_targets(subject, targets)
            self.inferred_targets = inferred_targets

            # For each target's port, we will add a logger
            self.loggers = {target.name: {} for target in inferred_targets}
            for target in inferred_targets:
                system = subject.GetSubsystemByName(target.name)
                for port_index in target.ports:
                    target_port = system.get_output_port(port_index)

                    if target_port.get_data_type() != PortDataType.kVectorValued:
                        print(f"Port {port_index} of system {target.name} is not vector-valued! Skipping...")
                        continue

                    logger = LogVectorOutput(system.get_output_port(port_index), subject)
                    logger.set_name(f"{target.name}_logger_{port_index}")
                    self.loggers[target.name][port_index] = logger

    def __del__(self):
        """
        Description:

            Destructor for the Diagram Watcher.
            Will plot the data from all of our loggers if we have access to the diagram context.
        :return:
        """
        # Setup
        plot_dir = self.plot_dir

        is_ready_to_plot = self.diagram_context is not None
        is_ready_to_plot = is_ready_to_plot and (self.diagram is not None)

        # Upon deletion, we will PLOT the data from all of our loggers
        # if we have access to the diagram context

        if is_ready_to_plot:
            # Delete the directory if it exists
            if os.path.exists(plot_dir):
                os.system(f"rm -r {plot_dir}")

            # Create directory to plot in
            os.makedirs(plot_dir, exist_ok=True)

            for system_name in self.loggers:
                system_ii = self.diagram.GetSubsystemByName(system_name)
                ports_on_ii = self.loggers[system_name]
                for port_index in ports_on_ii:
                    system_ii_port = system_ii.get_output_port(port_index)

                    system_ii_port_pi_logger = ports_on_ii[port_index]
                    fig_ii_pi, ax_list_ii_pi = self.plot_logger_data(
                        system_ii, port_index
                    )
                    # Save the figure, if plot was successful
                    if fig_ii_pi is None:
                        continue

                    fig_ii_pi.savefig(
                        f"{plot_dir}/{self.safe_system_name(system_name)}_port_{system_ii_port.get_name()}.png"
                    )

                # data and sampling times from the logger

            # self.logger = LogOutput(diagram.get_context())


    def safe_system_name(self, name: str) -> str:
        """
        Description:
            Returns a safe name for the system.
        :param name: System's name.
        :return:
        """
        out = name
        # First, let's check to see how many "/" exist in the name
        slash_occurences = [i for i, letter in enumerate(name) if letter == "/"]
        if len(slash_occurences) > 0:
            out = out[slash_occurences[-1] + 1:]  # truncrate string based on the last slash

        # Second, replace all spaces with underscores
        out = out.replace(" ", "_")

        return out

    def plot_logger_data(
        self,
        system_ii: LeafSystem,
        port_index: int,
    ) -> (plt.Figure, List[plt.Axes]):
        """
        Description:

            Plots the data from the logger for the specified system and port.
        :param system_ii:
        :param port_jj:
        :return:
        """
        # Setup
        port_jj = system_ii.get_output_port(port_index)
        logger = self.loggers[system_ii.get_name()][port_index]

        # Get the log from the logger
        temp_log = logger.FindLog(self.diagram_context)  # Extract the log from the logger

        log_times = temp_log.sample_times()
        data = temp_log.data()

        if data.shape[0] == 0:
            return None, None

        # Plot the data
        fig = plt.figure()
        ax_list = []

        n_rows, n_cols = self.compute_plot_shape(data.shape[0])

        for dim_index in range(port_jj.size()):
            ax_list.append(
                fig.add_subplot(n_rows, n_cols, 1 + dim_index)
            )
            plt.plot(log_times, data[dim_index, :])
            # TODO: Create a flag for how verbose title will be
            # plt.title(
            #     system_ii.get_name() +
            #     " - Port " + str(port_index) +
            #     'Dim #' + str(dim_index)
            # )
            plt.title(f"Dim #{dim_index}")

        return fig, ax_list


    def compute_plot_shape(self, n_dims: int) -> tuple:
        """
        Description:
            Computes the shape of the plot based on the data.
        :param data: The data to be plotted.
        :return:
        """
        if n_dims == 1:
            return 1, 1

        if n_dims > 1:
            return 2, int(np.ceil(n_dims / 2.0))


        raise NotImplementedError(f"n_dims should be greater than 0! (received {n_dims})")

    def check_targets(
        self,
        targets: List[DiagramTarget],
        eligible_systems: List[Union[MultibodyPlant, AffineSystem, LeafSystem]],
    ) -> List[Union[MultibodyPlant, AffineSystem, LeafSystem]]:
        """
        Description:
            Finds the systems specified by the targets list that
            we want to watch/monitor.
            We will try to ignore all systems that are:
            - Scene Graphs
            - Loggers
            and raise an error if the target is not found in the eligible systems.
        :param targets:
        :param eligible_systems:
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
                continue # No need to check things if ports is None

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

        for system in builder.GetSystems():
            for system_type in INELIGIBLE_SYSTEM_TYPES:
                if isinstance(system, system_type):
                    break

            # Otherwise add to list
            eligible_systems.append(system)

        return eligible_systems

    def get_smart_targets(
        self,
        subject: DiagramLike,
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
            output_ports_to_watch = [port_index for port_index in range(num_ports)]

            # Add a target with the connected ports to the smart_targets list
            smart_targets.append(
                DiagramTarget(target.name, output_ports_to_watch),
            )

        return smart_targets

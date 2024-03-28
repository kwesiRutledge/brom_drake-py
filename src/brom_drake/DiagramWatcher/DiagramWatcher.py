"""
DiagramWatcher.py
Description:

    Creates a DiagramWatcher class that can be used to monitor the state of a Diagram
    and automatically log signals of interest.
"""
from typing import List, Union

from pydrake.multibody.plant import MultibodyPlant
from pydrake.systems.framework import Diagram, DiagramBuilder, LeafSystem
from pydrake.systems.primitives import (
    VectorLogSink, ConstantVectorSource, AffineSystem,
)

from brom_drake import DiagramTarget
from .constants import INELIGIBLE_SYSTEM_TYPES
from .errors import UnrecognizedTargetError


class DiagramWatcher:
    def __init__(self, builder: DiagramBuilder, targets: List[DiagramTarget] = None):
        self.builder = builder

        # Collect All the Connections and Systems
        self.systems_to_watch = self.find_eligible_systems(builder)

        # Print out the connections
        for elt, connection in builder.connection_map():
            print(f"connection w/ key {elt}:", connection)

        # self.logger = LogOutput(diagram.get_context())

    def get_targeted_systems(
        self,
        builder: DiagramBuilder,
        targets: List[DiagramTarget] = None,
    ) -> List[Union[MultibodyPlant, AffineSystem, LeafSystem]]:
        """
        Description:
            Finds the systems specified by the targets list that
            we want to watch/monitor.
            We will try to ignore all systems that are:
            - Scene Graphs
            - Loggers
            and
        :param builder:
        :param targets:
        :return:
        """

        # Find all the systems that are eligible for logging
        eligible_systems = self.find_eligible_systems(builder)
        eligible_system_names = [system.get_name() for system in eligible_systems]

        # Check if all targets are valid
        for target in targets:
            if target.name not in eligible_systems:
                raise UnrecognizedTargetError(target)

        # Find the systems that are targeted
        targeted_systems = []




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

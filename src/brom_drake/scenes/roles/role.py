"""
Description:
    This file defines a Role class that is used to populate the
    scene with ONLY systems that match the appropriate structure/signature.
"""
from dataclasses import dataclass
from typing import List

from pydrake.systems.framework import DiagramBuilder

from brom_drake.scenes.roles.role_port_assignment import RolePortAssignment
# Internal Imports
from brom_drake.utils import (
    find_all_systems_with_output_port, find_all_systems_with_input_port,
    Performer,
)

# Class

@dataclass
class Role:
    name: str
    description: str
    port_assignments: List[RolePortAssignment]

    def __str__(self):
        return self.name

    def connect_performer_ports_to(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        # Setup

        # Add performer to builder
        builder.AddSystem(performer)

        # Connect performer to the rest of the scene (stored in Builder)
        for assignment in self.port_assignments:
            assignment.create_connections_in(builder, performer)
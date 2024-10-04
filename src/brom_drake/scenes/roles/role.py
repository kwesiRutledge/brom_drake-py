"""
Description:
    This file defines a Role class that is used to populate the
    scene with ONLY systems that match the appropriate structure/signature.
"""
from dataclasses import dataclass

from pydrake.systems.framework import DiagramBuilder

# Internal Imports
from brom_drake.utils import (
    find_all_systems_with_output_port, find_all_systems_with_input_port,
    Performer,
)

@dataclass
class PortPairing:
    external_port_name: str
    performer_port_name: str

@dataclass
class Role:
    name: str
    description: str
    required_input_definitions: list[PortPairing]
    required_output_definitions: list[PortPairing]

    def __str__(self):
        return self.name

    def connect_performer_to_system(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        """
        Description
        -----------
        This method connects the performer to the system.
        :param builder: A DiagramBuilder object.
        :param performer: A system or diagram that should have the appropriate ports.
        :return:
        """
        # Setup

        # Check that the performer has the required input and output ports
        for port_pairing in self.required_input_definitions:
            assert performer.HasInputPort(port_pairing.performer_port_name), \
                f"Performer does not have input port \"{port_pairing.performer_port_name}\""

        for port_pairing in self.required_output_definitions:
            assert performer.HasOutputPort(port_pairing.performer_port_name), \
                f"Performer does not have output port \"{port_pairing.performer_port_name}\""

        # Connect the performer's inputs to other systems in the builder
        for port_pairing in self.required_input_definitions:
            performer_port = performer.GetInputPort(port_pairing.performer_port_name)

            # Get the external system that has the output port with the correct name
            systems_list = find_all_systems_with_output_port(builder, port_pairing.external_port_name)
            assert len(systems_list) == 1, \
                f"Expected 1 system to have port \"{port_pairing.external_port_name}\"," + \
                f" but found {len(systems_list)} systems with that output port."

            builder.Connect(
                systems_list[0].GetOutputPort(port_pairing.external_port_name),
                performer_port,
            )

        # Connect the performer's outputs to other systems in the builder
        for port_pairing in self.required_output_definitions:
            performer_port = performer.GetOutputPort(port_pairing.external_port_name)

            # Get the external system that has the input port with the correct name
            systems_list = find_all_systems_with_input_port(builder, port_pairing.external_port_name)
            assert len(systems_list) == 1, \
                f"Expected 1 system to have port \"{port_pairing.external_port_name}\"," + \
                f" but found {len(systems_list)} systems with that output port."

            builder.Connect(
                performer_port,
                systems_list[0].GetInputPort(port_pairing.external_port_name),
            )

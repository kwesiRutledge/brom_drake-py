"""
Description:
    This file defines a Role class that is used to populate the
    scene with ONLY systems that match the appropriate structure/signature.
"""
from dataclasses import dataclass
from multiprocessing.managers import Value

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
    is_required: bool = True # By default, a port pairing is required

@dataclass
class Role:
    name: str
    description: str
    input_definitions: list[PortPairing]
    output_definitions: list[PortPairing]

    def __str__(self):
        return self.name

    def connect_performers_inputs_to_diagram(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        """
        Description
        -----------
        This method iterates through all input definitions in the
        role and attempts to connect the performer's input ports
        to ports in the diagram matching the corresponding external port name.
        :param builder: Builder containing a partially initialized diagram or scene.
        :param performer: The system or diagram that we would like to insert into the
        partially initialized diagram or scene.
        :return: Nothing, but builder is modified.
        """

        # Check that the performer has the required input and output ports
        for port_pairing in self.input_definitions:
            performer_has_input_port = performer.HasInputPort(port_pairing.performer_port_name)

            if (not performer_has_input_port) and port_pairing.is_required:
                raise ValueError( # TODO(kwesi): Define custom error type for this?
                    f"Performer does not have required input port \"{port_pairing.performer_port_name}\""
                )

            if (not performer_has_input_port) and (not port_pairing.is_required):
                raise Warning(
                    f"Performer does not have OPTIONAL input port \"{port_pairing.performer_port_name}\"" +
                    "\n Will skip its connection."
                )

        # Connect the performer's inputs to other systems in the builder
        for port_pairing in self.input_definitions:
            # Check to see if performer port exists
            performer_has_input_port = performer.HasInputPort(port_pairing.performer_port_name)
            if not performer_has_input_port:
                continue # Skip this port if it doesn't exist

            # Otherwise, port exists.
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

    def connect_performers_outputs_to_diagram(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        """
        Description
        -----------
        This method iterates through all OUTPUT definitions in the
        role and attempts to connect the performer's output ports
        to ports in the diagram matching the corresponding external port name.
        :param builder: Builder containing a partially initialized diagram or scene.
        :param performer: The system or diagram that we would like to insert into the
        partially initialized diagram or scene.
        :return: Nothing, but builder is modified.
        """
        # Setup

        # Check to see if all required output ports exist

        for port_pairing in self.output_definitions:
            performer_has_output_port = performer.HasOutputPort(port_pairing.performer_port_name)

            if (not performer_has_output_port) and port_pairing.is_required:
                raise ValueError(
                    f"Performer does not have REQUIRED output port \"{port_pairing.performer_port_name}\""
                )

            if (not performer_has_output_port) and (not port_pairing.is_required):
                raise Warning(
                    f"Performer does not have OPTIONAL output port \"{port_pairing.performer_port_name}\"" +
                    "\n Will skip its connection."
                )



        # Connect the performer's outputs to other systems in the builder
        for port_pairing in self.output_definitions:
            # Check that port exists on performer before trying to do any connection
            performer_has_output_port = performer.HasOutputPort(port_pairing.performer_port_name)
            if not performer_has_output_port:
                continue # Skip this pair if port doesn't exist on performer

            # Port must exist on performer now
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

    def connect_performer_to_diagram(
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

        # Connect inputs and outputs according to the recipes that we've defined in our member variables
        self.connect_performers_inputs_to_diagram(builder, performer)
        self.connect_performers_outputs_to_diagram(builder, performer)


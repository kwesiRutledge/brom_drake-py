from dataclasses import dataclass
from distutils.command.build import build
from enum import IntEnum
from platform import system
from typing import Union, List

from pydrake.systems.framework import DiagramBuilder, LeafSystem

from brom_drake.utils import (
    Performer, find_all_systems_with_output_port, find_all_systems_with_input_port,
)
from brom_drake.utils.search import find_all_systems_with_name

PortName = str
SystemName = str
TargetName = Union[PortName, SystemName]


class PairingType(IntEnum):
    kInput = 1
    kOutput = 2

@dataclass
class RolePortAssignment:
    external_target_name: TargetName  # The name of the port (or system) that we want to connect this port to
    performer_port_name: PortName
    pairing_type: PairingType
    is_required: bool = True  # By default, a port pairing is required

    def create_connections_in(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        """
        Description
        -----------

        :param builder:
        :param performer:
        :return:
        """

        if self.pairing_type == PairingType.kInput:
            self.create_input_port_connections_in(
                builder=builder,
                performer=performer,
            )
        elif self.pairing_type == PairingType.kOutput:
            self.create_output_port_connections_in(
                builder=builder,
                performer=performer,
            )
        else:
            raise ValueError(
                f"Unknown pairing type: {self.pairing_type}"
            )

    def create_input_port_connections_in(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        # Setup

        # Check to see if the performer has the given input port
        performer_has_input_port = performer.HasInputPort(
            self.performer_port_name
        )
        if (not performer_has_input_port) and self.is_required:
            raise ValueError(  # TODO(kwesi): Define custom error type for this?
                f"Performer does not have required input port \"{self.performer_port_name}\""
            )
        elif (not performer_has_input_port) and (not self.is_required):
            print(
                f"Performer {performer.get_name()} does not have OPTIONAL input port named {self.performer_port_name}." +
                "\nWill skip its connection."
            )
            return # Do not do anything if the port does not exist

        # Otherwise, port exists.
        performer_port = performer.GetInputPort(self.performer_port_name)

        # Get the external system that has the output port with the correct name
        systems_list = self.find_any_matching_output_targets(builder)
        assert len(systems_list) == 1, \
            f"Expected 1 system to have port \"{self.external_target_name}\"," + \
            f" but found {len(systems_list)} systems with that output port."

        # Connect system
        system_is_target = systems_list[0].get_name() == self.external_target_name
        if system_is_target:
            builder.Connect(
                systems_list[0].get_output_port(),
                performer_port,
            )
        else:
            builder.Connect(
                systems_list[0].GetOutputPort(self.external_target_name),
                performer_port,
            )

    def find_any_matching_output_targets(
        self,
        builder: DiagramBuilder,
    ) -> List[LeafSystem]:
        """
        Description
        -----------
        This function attempts to find all targets that match the output
        target definition, i.e.:
        - A system with the output port with given name
        - A system with the given name.
        :param builder:
        :return:
        """
        # Setup
        external_target_name = self.external_target_name

        # Retrieve all systems that have that output port name external_target_name
        systems_list = find_all_systems_with_output_port(
            builder, external_target_name,
        )

        if len(systems_list) > 0:
            return systems_list

        # Retrieve all systems that have the name of external_target_name
        systems_list = find_all_systems_with_name(builder, external_target_name)

        if len(systems_list) > 0:
            return systems_list

        # Otherwise, raise an error
        raise ValueError(
            f"Failed to find an external system with the name {external_target_name}" +
            f" or an external system with OUTPUT port name {external_target_name}." +
            f"\nCheck your RolePortAssignment definition."
        )


    def create_output_port_connections_in(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        """
        Description
        -----------
        Creates the connections from the performer's output port to
        the external system's input ports.
        :param builder:
        :param performer:
        :return:
        """
        # Setup

        # Check to see if the performer has the given input port
        performer_has_output_port = performer.HasOutputPort(
            self.performer_port_name
        )
        if (not performer_has_output_port) and self.is_required:
            raise ValueError(  # TODO(kwesi): Define custom error type for this?
                f"Performer does not have required output port \"{self.performer_port_name}\""
            )
        elif (not performer_has_output_port) and (not self.is_required):
            print(
                f"Performer {performer.get_name()} does not have OPTIONAL output port named {self.performer_port_name}." +
                "\nWill skip its connection."
            )
            return  # Do not do anything if the port does not exist

        # Otherwise, port exists.
        performer_port = performer.GetOutputPort(self.performer_port_name)

        # Get the external system that has the input port with the correct name
        systems_list = self.find_any_matching_input_targets(builder)
        assert len(systems_list) == 1, \
            f"Expected 1 system to have port \"{self.external_target_name}\"," + \
            f" but found {len(systems_list)} systems with that output port."

        # TODO(kwesi): Create if-else statement to handle the cases of when systems_list
        # contains either:
        # 1. A system with the name given by external_target_name
        # 2. A system that has a port with name given by external_target_name
        builder.Connect(
            performer_port,
            systems_list[0].GetInputPort(self.external_target_name),
        )

    def find_any_matching_input_targets(
        self,
        builder: DiagramBuilder,
    ) -> List[LeafSystem]:
        """
        Description
        -----------
        This function attempts to find all targets that match the input
        target definition, i.e.:
        - A system with the input port with given name
        - A system with the given name.
        :param builder:
        :return:
        """
        # Setup
        external_target_name = self.external_target_name

        # Retrieve all systems that have that output port name external_target_name
        systems_list = find_all_systems_with_input_port(
            builder, external_target_name,
        )

        if len(systems_list) > 0:
            return systems_list

        # Retrieve all systems that have the name of external_target_name
        systems_list = find_all_systems_with_name(builder, external_target_name)

        if len(systems_list) > 0:
            return systems_list

        # Otherwise, raise an error
        raise ValueError(
            f"Failed to find an external system with the name \"{external_target_name}\"" +
            f" or an external system with INPUT port name \"{external_target_name}\"." +
            f"\nCheck your RolePortAssignment definition."
        )
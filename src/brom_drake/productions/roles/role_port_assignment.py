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
    """
    *Description*

    Describes the type of connection defined by a RolePortAssignment.

    kInput => Connection is between an external target and a performer's INPUT port.

    kOutput => Connection is between an external target and a performer's OUTPUT port.
    """
    kInput = 1
    kOutput = 2

@dataclass
class RolePortAssignment:
    """
    *Description*

    A potential connection between an object that "fills a role" in a Drake diagram,
    and the rest of the diagram. Some connections are required, but they need not be.

    TODO(Kwesi): Refactor this class to use the ``DiagramTarget`` object.
    
    *Parameters*

    external_target_name: TargetName
        The name of the port (or system) that we want to connect this port to

    performer_port_name: PortName
        The name of the port that should exist on the LeafSystem (or Diagram) that "fills the role".

    pairing_type: PairingType
        Describes whether or not `performer_port_name` is an input or output port.
        This determines whether or not external_target_name should be an output or input port, respectively.

    is_required: bool, optional
        Default is True.
    """
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
        *Description*
        
        Creates the connection between the performer and the appropriate
        port in the incomplete Drake diagram specified by `builder`.

        *Parameters*

        builder: DiagramBuilder
            Contains the partially built Diagram.

        performer: Performer
            The LeafSystem (or Diagram) that we wish to connect to the incomplete
            Diagram in ``builder``.
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
        """
        *Description*

        Assuming that the assignment defines an input port connection, this function
        finds either:

        - A system with the target name, and connects the first output port of the target to the performer's input OR
        - The first system with the port given by the constraints, and connects that output port to the performer's input port.

        *Parameters*

        builder: DiagramBuilder
            Contains the partially built Diagram.

        performer: Performer
            The LeafSystem (or Diagram) that we wish to connect to the incomplete
            Diagram in ``builder``.
        """
        # Setup

        # Check to see if the performer has the given input port
        performer_has_input_port = performer.HasInputPort(
            self.performer_port_name
        )
        if (not performer_has_input_port) and self.is_required:
            raise self.create_assignment_port_unavailable_error()
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
        assert len(systems_list) == 1, self.create_not_enough_systems_error(systems_list)

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
        *Description*
        
        This function attempts to find all targets that match the output
        target definition, i.e.:
        
        - A system with the output port with given name
        - A system with the given name.
        
        *Parameters*

        builder: DiagramBuilder
            Contains the partially built Diagram.

        *Returns*

        matching_systems_list: List[LeafSystem]
            A list of all systems that match the constraints/target definition.
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
        raise self.create_no_target_found_error()


    def create_output_port_connections_in(
        self,
        builder: DiagramBuilder,
        performer: Performer,
    ):
        """
        *Description*

        Assuming that the assignment defines an OUTPUT port connection, this function
        finds either:

        - A system with the target name, and connects the first input port of the target to the performer's output OR
        - The first system with the input port given by the constraints, and connects that input port to the performer's output port.

        *Parameters*

        builder: DiagramBuilder
            Contains the partially built Diagram.

        performer: Performer
            The LeafSystem (or Diagram) that we wish to connect to the incomplete
            Diagram in ``builder``.
        """
        # Setup

        # Check to see if the performer has the given input port
        performer_has_output_port = performer.HasOutputPort(
            self.performer_port_name
        )
        if (not performer_has_output_port) and self.is_required:
            raise self.create_assignment_port_unavailable_error()
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
        assert len(systems_list) == 1, self.create_not_enough_systems_error(systems_list)

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
        *Description*
        
        This function attempts to find all targets that match the input
        target definition, i.e.:

        - A system with the input port with given name
        - A system with the given name.
        
        TODO(Kwesi): Compress this method and the output targets method?

        builder: DiagramBuilder
            Contains the partially built Diagram.

        *Returns*

        matching_systems_list: List[LeafSystem]
            A list of all systems that match the constraints/target definition.
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
        raise self.create_no_target_found_error()

    def create_assignment_port_unavailable_error(self) -> ValueError:
        """
        *Description*

        Creates the appropriate version of "Performer does not have required input port" error message.

        TODO(Kwesi): Should we create an error type for this?
        """
        # Setup
        port_type_str = "UNKNOWN"
        if self.pairing_type == PairingType.kInput:
            port_type_str = "INPUT"
        elif self.pairing_type == PairingType.kOutput:
            port_type_str = "OUTPUT"

        # Create the error message
        return ValueError(
            f"Performer does not have required {port_type_str} port \"{self.performer_port_name}\""
        )

    def create_not_enough_systems_error(self, systems_list: List[LeafSystem]) -> str:
        # Setup
        port_type_str = "UNKNOWN"
        if self.pairing_type == PairingType.kInput:
            port_type_str = "INPUT"
        elif self.pairing_type == PairingType.kOutput:
            port_type_str = "OUTPUT"

        # Create message depending on the pairing type
        return f"Expected 1 system to have {port_type_str} port \"{self.external_target_name}\"," + \
            f" but found {len(systems_list)} systems with that {port_type_str} port."

    def create_no_target_found_error(self):
        # Setup
        external_target_name = self.external_target_name

        # Determine if this is an input or output target
        external_target_type = "UNKNOWN"
        if self.pairing_type == PairingType.kInput:
            external_target_type = "OUTPUT"
        elif self.pairing_type == PairingType.kOutput:
            external_target_type = "INPUT"

        # Create error
        return ValueError(
            f"Failed to find an external system with the name \"{external_target_name}\"" +
            f" or an external system with {external_target_type} port name \"{external_target_name}\"." +
            f"\nCheck your RolePortAssignment definition."
        )
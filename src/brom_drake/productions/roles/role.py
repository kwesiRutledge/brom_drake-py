"""
Description:
    This file defines a Role class that is used to populate the
    production with ONLY systems that match the appropriate structure/signature.
"""
from dataclasses import dataclass
from typing import List

from pydrake.systems.framework import DiagramBuilder

from brom_drake.productions.roles.role_port_assignment import RolePortAssignment
# Internal Imports
from brom_drake.utils import Performer

@dataclass
class Role:
    """
    *Description*

    This class defines a "slot" to be filled in a Drake Diagram.
    A name is used for unique identification and a description is (optionally)
    used to describe what is expected of a drake LeafSystem (or Diagram) that
    properly "fills the role" (i.e., fills the slot) in a Drake Diagram.

    *Parameters*

    name: str
        Unique identifier of the role.

    description: str
        Non-binding description of the role.

    port_assignments: List[RolePortAssigment]
        Each element of this list describes:
        
        - an external target (e.g., `LeafSystems`) that the role expects to connect to
        - the port on the ``LeafSystem`` that fills this role that should connect to that target.

    *Usage*

    Consider the following pieces of code: ::

        from brom_drake.productions.roles import Role, RolePortAssignment
        from brom_drake.productions.roles.role_port_assignment import PairingType

        kGraspConfirmer = Role(
            name="Confirm-When-Grasp-Worked",
            description="Attempts to define with a boolean whether or not the object we're interested in is currently in our grasp.",
            port_assignments=[
                # Required inputs for Role
                RolePortAssignment(
                    external_target_name="target_cube_pose",
                    performer_port_name="current_manipuland_pose",
                    pairing_type=PairingType.kInput
                ),
                RolePortAssignment(
                    external_target_name="cube_model_idx",
                    performer_port_name="object_model_index",
                    pairing_type=PairingType.kInput
                ),
                RolePortAssignment(
                    external_target_name="gripper_pose",
                    performer_port_name="gripper_pose",
                    pairing_type=PairingType.kInput,
                ),
                RolePortAssignment(
                    external_target_name="production_gripper_model_idx",
                    performer_port_name="gripper_model_index",
                    pairing_type=PairingType.kInput
                ),
                RolePortAssignment(
                    external_target_name="gripper_configuration",
                    performer_port_name="gripper_configuration",
                    pairing_type=PairingType.kInput,
                ),
                # Output Of Role
                RolePortAssignment(
                    external_target_name="grasp_complete",
                    performer_port_name="grasp_success_flag",
                    pairing_type=PairingType.kOutput,
                )
            ]
        )

    
    The above example defines a Role with:

    - name: ``"Confirm-When-Grasp-Worked"``
    - Required Input Ports:
        + ``"current_manipuland_pose"``
        + ``"object_model_index"``
        + ``"gripper_pose"``
        + ``"gripper_model_index"``
        + ``"gripper_configuration"``
    - Required Output Port:
        + ``"grasp_success_flag"``
    - Required Ports to exist *somewhere else in the Diagram*:
        + ``"target_cube_pose"``
        + ``"cube_model_idx"``
        + ``"gripper_pose"``
        + ``"production_gripper_model_idx"``
        + ``"gripper_configuration"``
        + ``"grasp_complete"``

    Without all of these satisfied, we can not successfully place
    a `LeafSystem` into a Production that expects the ``"Confirm-When-Grasp-Worked"``
    role.

    """
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
        """
        *Description*

        Connect each inputs and output port mentioned in `self.port_assignments`
        on `performer` to the first port found in `builder` with the correct name.

        .. warning ::

            The ``.Build()`` method should not have been called on ``builder``
            before invoking this method.
        
        *Parameters*

        builder: DiagramBuilder
            The ``DiagramBuilder`` containing a partially built diagram.

        performer: Performer
            The object that we think can fill the role defined by self.
        """
        # Setup

        # Add performer to builder
        builder.AddSystem(performer)

        # Connect performer to the rest of the production (stored in Builder)
        for assignment in self.port_assignments:
            assignment.create_connections_in(builder, performer)
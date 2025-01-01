"""
Description:
    This file defines an OfflineMotionPlanner class that is used to define the
    offline motion planning role that can be used in Brom Productions.
"""
from brom_drake.productions.roles import Role, RolePortAssignment
from brom_drake.productions.roles.role_port_assignment import PairingType

kKinematicMotionPlanner = Role(
    name="OfflineMotionPlanner",
    description="This role is used to define the offline motion planning role.",
    port_assignments=[
        RolePortAssignment(
            "start_configuration", "start_configuration",
            pairing_type=PairingType.kInput,
        ),
        RolePortAssignment(
            "goal_configuration", "goal_configuration",
            pairing_type=PairingType.kInput,
        ),
        RolePortAssignment(
            external_target_name="robot_model_index_source",
            performer_port_name="robot_model_index",
            pairing_type=PairingType.kInput,
        ),
        RolePortAssignment(
            "id", "production_id",
            is_required=False,
            pairing_type=PairingType.kInput,
        ),
        RolePortAssignment(
            external_target_name="plan",
            performer_port_name="motion_plan",
            pairing_type=PairingType.kOutput,
        ),
        RolePortAssignment(
            external_target_name="plan_ready",
            performer_port_name="plan_is_ready",
            is_required=False,
            pairing_type=PairingType.kOutput,
        ),
    ],
)

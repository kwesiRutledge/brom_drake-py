"""
Description:
    This file defines the grasp selection role that can be used in Brom Productions.
"""

from brom_drake.productions.roles.role import Role
from brom_drake.productions.roles.role_port_assignment import (
    PairingType,
    RolePortAssignment,
)

# A single performer selects grasps for every object in the scene. Productions choose
# which object it should work on by changing the value of "target_object_model_index".
#
# Performer ports:
# - "target_object_model_index" (input): The ModelInstanceIndex of the object to grasp.
# - "X_ObjectGripper_trajectory" (output): A List[RigidTransform] containing the poses of
#   the Gripper Frame with respect to the target object. The last pose is the grasp pose.
# - "grasp_gripper_command" (output): A vector containing the normalized gripper command
#   (not a joint angle) to close the gripper to when grasping the target object.
# - "grasp_is_ready" (output, optional): A boolean that is True once the other outputs
#   describe a grasp of the current target object. If a performer has this port, then the
#   diagram must contain a system with a "grasp_ready" input port to consume it; otherwise,
#   connecting the role raises an error. Performers may omit this port when nothing consumes it.
kGraspSelector = Role(
    name="GraspSelector",
    description="Selects a gripper trajectory and closing command for a target object.",
    port_assignments=[
        RolePortAssignment(
            external_target_name="target_object_model_index",
            performer_port_name="target_object_model_index",
            pairing_type=PairingType.kInput,
        ),
        RolePortAssignment(
            external_target_name="X_ObjectGripper_trajectory",
            performer_port_name="X_ObjectGripper_trajectory",
            pairing_type=PairingType.kOutput,
        ),
        RolePortAssignment(
            external_target_name="grasp_gripper_command",
            performer_port_name="grasp_gripper_command",
            pairing_type=PairingType.kOutput,
        ),
        RolePortAssignment(
            external_target_name="grasp_ready",
            performer_port_name="grasp_is_ready",
            is_required=False,
            pairing_type=PairingType.kOutput,
        ),
    ],
)

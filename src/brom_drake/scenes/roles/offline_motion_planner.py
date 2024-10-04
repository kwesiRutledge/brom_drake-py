"""
Description:
    This file defines an OfflineMotionPlanner class that is used to define the
    offline motion planning role that can be used in Brom scenes.
"""
from brom_drake.scenes.roles import Role, PortPairing

kOfflineMotionPlanner = Role(
    name="OfflineMotionPlanner",
    description="This role is used to define the offline motion planning role.",
    required_input_definitions=[
        PortPairing("goal_position", "goal_position"),
        PortPairing("joint_positions", "joint_positions"),
        PortPairing("joint_position_limits", "joint_position_limits"),
        PortPairing("id", "scene_id"),
    ],
    required_output_definitions=[
        PortPairing("motion_plan", "plan"),
    ],
)

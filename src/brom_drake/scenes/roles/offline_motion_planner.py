"""
Description:
    This file defines an OfflineMotionPlanner class that is used to define the
    offline motion planning role that can be used in Brom scenes.
"""
from brom_drake.scenes.roles import Role

kOfflineMotionPlanner = Role(
    name="OfflineMotionPlanner",
    description="This role is used to define the offline motion planning role.",
    required_input_ports=[
        "goal_position",
        "joint_positions",
        "joint_position_limits",
        "scene_id",
    ],
    required_output_ports=[
        "plan",
    ],
)

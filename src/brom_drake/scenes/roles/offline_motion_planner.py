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
        PortPairing("start_pose", "start_pose"),
        PortPairing("goal_pose", "goal_pose"),
        PortPairing("id", "scene_id"),
    ],
    required_output_definitions=[
        PortPairing("motion_plan", "plan"),
    ],
)

"""
Description:
    This file defines an OfflineMotionPlanner class that is used to define the
    offline motion planning role that can be used in Brom scenes.
"""
from brom_drake.scenes.roles import Role, PortPairing

kOfflineMotionPlanner = Role(
    name="OfflineMotionPlanner",
    description="This role is used to define the offline motion planning role.",
    input_definitions=[
        PortPairing("start_pose", "start_pose"),
        PortPairing("goal_pose", "goal_pose"),
        PortPairing("id", "scene_id", is_required=False),
    ],
    output_definitions=[
        PortPairing("motion_plan", "plan"),
        PortPairing("plan_is_ready", "plan_ready", is_required=False)
    ],
)

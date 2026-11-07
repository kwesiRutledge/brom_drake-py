"""
Description
-----------
This script records a motion planning production with the production's
DEFAULT cameras, using a neutral "studio" look, and combines all cameras into
one grid video.

- `add_cameras()` with no camera list uses `production.default_cameras()`.
  For ShelfPlanning1 those are an overview, a top-down view, and a "wrist"
  camera attached to the robot's tool0 frame (it moves with the arm).
- `RenderSettings.studio()` gives a grey background, world-fixed lights and
  shadows.
- `save_videos(grid=True)` also writes all_cameras.mp4, which shows every
  camera side by side, aligned by simulation time.

For more info, refer to the README or the docs:
https://kwesirutledge.github.io/brom_drake-py/
"""

from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake.all import RenderSettings
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline import ShelfPlanning1


def main(meshcat_port_number: int = 7001, max_duration: float = 20.0):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None  # Use None for CI

    production = ShelfPlanning1(meshcat_port_number=meshcat_port_number)

    # Use the production's default cameras, with the studio look
    production.add_cameras(
        output_dir="brom/videos/default_cameras_and_grid1",
        render_settings=RenderSettings.studio(),
    )

    # Add a motion planner and build
    planner = BidirectionalRRTConnectPlanner(
        production.arm,
        production.plant,
        production.scene_graph,
        config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
    )
    diagram, diagram_context = production.easy_cast_and_build(
        planner.plan,
        with_watcher=False,
    )

    # Simulate until the plan has been executed (capped at max_duration)
    simulator = Simulator(diagram, diagram_context)
    simulator.Initialize()
    simulator.AdvanceTo(0.1)

    planned_trajectory = production.plan_dispenser.planned_trajectory
    end_time = min(planned_trajectory.end_time() + 1.0, max_duration)
    print(f"Recording until t = {end_time:.1f} s of simulation time...")
    simulator.AdvanceTo(end_time)

    # Finish the per-camera videos and build the grid video
    video_paths = production.save_videos(grid=True)
    for name, path in video_paths.items():
        print(f"Saved {name}: {path}")


if __name__ == "__main__":
    main(meshcat_port_number=-1)

"""
example.py
Description:

    In this script, we use the CornellTech1 production (a Kinova Gen3 arm mounted on
    a robot cart) to test a basic motion planning algorithm.
    This shows one how to use the `easy_cast_and_build` method to simplify how to build
    a Production.

    Pass `--record` to also record videos of the simulation from the production's
    default cameras (written to ./brom/videos/cornell_tech1).
"""

import sys

from pydrake.all import Simulator

# Internal imports
from brom_drake.all import RenderSettings
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.all import CornellTech1


def main(meshcat_port_number: int = 7001, record_videos: bool = False):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None  # Use None for CI

    # Create the production
    production = CornellTech1(
        meshcat_port_number=meshcat_port_number,  # Use None for CI
    )

    if record_videos:
        production.add_cameras(
            output_dir="brom/videos/cornell_tech1",
            render_settings=RenderSettings.studio(),
        )

    # Create a planner object which will be used to plan the motion
    config = BidirectionalRRTConnectPlannerConfig(
        steering_step_size=0.2,
    )
    planner = BidirectionalRRTConnectPlanner(
        production.arm,
        production.plant,
        production.scene_graph,
        config=config,
    )

    # To build the production, we only need to provide a planning function
    # (can come from anywhere, not just a BaseRRTPlanner object)
    diagram, diagram_context = production.easy_cast_and_build(
        planner.plan,
        with_watcher=True,
    )

    print("Simulating...")

    # Simulate the diagram
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(2.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(0.05)
    planned_trajectory = production.plan_dispenser.planned_trajectory
    print(f"Expected end time of trajectory: {planned_trajectory.end_time()}")
    simulator.AdvanceTo(planned_trajectory.end_time() + 1.0)

    if record_videos:
        print(production.save_videos(grid=True))


if __name__ == "__main__":
    main(record_videos="--record" in sys.argv)

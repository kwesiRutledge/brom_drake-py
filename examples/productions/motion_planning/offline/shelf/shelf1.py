"""
shelf1.py
Description:

    In this script, we use the Shelf Production to test a basic motion planning algorithms.
    This shows one how to use the `easy_cast_and_build` method to simplify how to build
    a production.
"""
import ipdb
import numpy as np
from pydrake.all import (
    Simulator,
    RollPitchYaw, RigidTransform,
)
import typer

# Internal imports
from brom_drake.motion_planning.algorithms.rrt import RRTConnectPlannerConfig, RRTConnectPlanner
from brom_drake.productions.motion_planning.offline import ShelfPlanning1

def main(meshcat_port_number: int = 7001):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None # Use None for CI

    # Define the goal pose
    easy_goal_position = np.array([+0.0, 1.0, 1.05])
    goal_orientation = RollPitchYaw(np.pi / 2.0, np.pi / 2.0, 0.0).ToQuaternion()
    goal_pose = RigidTransform(goal_orientation, easy_goal_position)

    # Create the production
    production = ShelfPlanning1(
        meshcat_port_number=meshcat_port_number, # Use None for CI
        goal_pose=goal_pose,
    )

    # Create a planner object which will be used to plan the motion
    config = RRTConnectPlannerConfig(
        steering_step_size=0.01,
        prob_sample_goal=0.05,
        max_iterations=int(1e5),
        convergence_threshold=1e-3,
    )
    planner2 = RRTConnectPlanner(
        production.arm,
        production.plant,
        production.scene_graph,
        config=config,
    )

    # To build the production, we only need to provide a planning function
    # (can come from anywhere, not just a BaseRRTPlanner object)
    diagram, diagram_context = production.easy_cast_and_build(
        planner2.plan,
        with_watcher=True,
    )

    print("Simulating...")

    # Simulate the diagram
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(0.1)
    planned_trajectory = production.plan_dispenser.planned_trajectory
    print(f"Expected end time of the planned trajectory: {planned_trajectory.end_time()}")
    
    # Simulate the full plan
    simulator.AdvanceTo(planned_trajectory.end_time()+1.0)

if __name__ == "__main__":
    main()
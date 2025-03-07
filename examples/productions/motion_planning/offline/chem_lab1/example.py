"""
example.py
Description:

    In this script, we use the ChemLab1 Production to test a basic motion planning algorithms.
    This shows one how to use the `easy_cast_and_build` method to simplify how to build
    a Production.
"""
import ipdb
import numpy as np
from pydrake.all import Simulator
import typer

# Internal imports
from brom_drake.motion_planning.algorithms.rrt.connect import RRTConnectPlannerConfig, RRTConnectPlanner
from brom_drake.productions.motion_planning.offline import ChemLab1

def main(meshcat_port_number: int = 7001):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None # Use None for CI

    # Create the production
    production = ChemLab1(
        meshcat_port_number=meshcat_port_number, # Use None for CI
    )

    # Create a planner object which will be used to plan the motion
    config = RRTConnectPlannerConfig(
        steering_step_size=0.1,
        prob_sample_goal=0.30,
        max_iterations=int(1e5),
        convergence_threshold=1e-3,
        debug_flag=True,
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

    raise NotImplementedError(
        "This example is not working yet. " +
        "Please wait for future versions of brom_drake to fix this."
    )

    print("Simulating...")

    # Simulate the diagram
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(0.1)
    planned_trajectory = production.plan_dispenser.planned_trajectory
    print(f"Expected end time of trajectory: {planned_trajectory.end_time()}")

    # return
    simulator.AdvanceTo(planned_trajectory.end_time()+1.0)

if __name__ == "__main__":
    main()
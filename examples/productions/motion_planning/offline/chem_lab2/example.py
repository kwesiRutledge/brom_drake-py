"""
example.py
Description:

    In this script, we use the Shelf Production to test a basic motion planning algorithms.
    This shows one how to use the `easy_cast_and_build` method to simplify how to build
    a Production.
"""
import ipdb
import numpy as np
from pydrake.all import Simulator, RigidTransform, RollPitchYaw
import typer

# Internal imports
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
    BiRRTConnectSamplingProbabilities,
)
from brom_drake.productions import ChemLab2

def main(meshcat_port_number: int = 7001):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None # Use None for CI

    # Create the production
    production = ChemLab2(
        meshcat_port_number=meshcat_port_number, # Use None for CI
        # pose_WorldBeaker=RigidTransform(
        #     RollPitchYaw(np.pi/2.0, 0.0, 0.0).ToQuaternion(),
        #     np.array([2.0*0.5*0.7+0.2, 0.6+0.6/4.-0.2, 0.1-0.015+0.1]),
        # )
    )

    # Create a planner object which will be used to plan the motion
    config = BidirectionalRRTConnectPlannerConfig(
        steering_step_size=0.2,
    )
    planner2 = BidirectionalRRTConnectPlanner(
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
    simulator.set_target_realtime_rate(2.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(0.05)
    planned_trajectory = production.plan_dispenser.planned_trajectory
    print(f"Expected end time of trajectory: {planned_trajectory.end_time()}")
    simulator.AdvanceTo(0.1)
    # return
    simulator.AdvanceTo(planned_trajectory.end_time()+1.0)

if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
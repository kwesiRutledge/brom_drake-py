"""
shelf1.py
Description:

    In this script, we use the Shelf scene to test a basic motion planning algorithms.
    This shows one how to use the `easy_cast_and_build` method to simplify how to build
    a scene.
"""
import ipdb
from pydrake.systems.analysis import Simulator
import typer


# Internal imports
from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlannerConfig, BaseRRTPlanner
from brom_drake.scenes.motion_planning.offline import ShelfPlanningScene

def main(meshcat_port_number: int = 7001):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None # Use None for CI

    scene = ShelfPlanningScene(
        meshcat_port_number=meshcat_port_number, # Use None for CI
    )

    # Create a planner object which will be used to plan the motion
    planner2 = BaseRRTPlanner(
        scene.arm,
        scene.plant,
        scene.scene_graph,
    )

    # To build the scene, we only need to provide a planning function
    # (can come from anywhere, not just a BaseRRTPlanner object)
    diagram, diagram_context = scene.easy_cast_and_build(
        planner2.plan,
    )

    # Simulate the diagram
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(0.1)
    planned_trajectory = scene.plan_dispenser.planned_trajectory
    print(planned_trajectory.end_time())
    # return
    simulator.AdvanceTo(planned_trajectory.end_time()+1.0)

if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
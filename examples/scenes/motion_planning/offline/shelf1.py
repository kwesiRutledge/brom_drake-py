"""
shelf1.py
Description:

    In this script, we use the Shelf scene to test some basic motion planning algorithms.
"""
import ipdb
from pydrake.all import Parser
from pydrake.systems.analysis import Simulator
import typer
from pydrake.systems.framework import DiagramBuilder
from pydrake.trajectories import PiecewisePolynomial

from brom_drake.motion_planning.algorithms.rrt.base import BaseRRTPlannerConfig
from brom_drake.motion_planning.systems.rrt_plan_generator import RRTPlanGenerator
# Internal imports
from brom_drake.scenes.motion_planning.offline import ShelfPlanningScene
from brom_drake.scenes.roles import kOfflineMotionPlanner


def main(use_meshcat: bool = True):
    # Setup
    scene = ShelfPlanningScene(use_meshcat=use_meshcat)

    # Create dummy cast
    planner = RRTPlanGenerator(
        plant=scene.plant,
        scene_graph=scene.scene_graph,
        rrt_config=BaseRRTPlannerConfig(
            prob_sample_goal=0.3,
            steering_step_size=0.05,
            max_iterations=int(1e4),
        ),
    )
    cast = [
        (kOfflineMotionPlanner, planner)
    ]

    # Build and simulate
    diagram, diagram_context = scene.cast_scene_and_build(cast)

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
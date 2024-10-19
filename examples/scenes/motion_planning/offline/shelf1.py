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

# Internal imports
from brom_drake.scenes.motion_planning.offline import ShelfPlanningScene

def main(use_meshcat: bool = True):
    # Setup
    scene = ShelfPlanningScene(use_meshcat=use_meshcat)

    # Add all secondary cast members to the builder
    scene.add_all_secondary_cast_members_to_builder()

    # Build and simulate
    diagram = scene.builder.Build()

    # Simulate the diagram
    diagram_context = diagram.CreateDefaultContext()
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)

if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
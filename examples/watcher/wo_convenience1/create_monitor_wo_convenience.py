"""
create_monitor_wo_convenience.py
Description:

    In this example, we illustrate how you can create a DiagramWatcher WITHOUT the convenience
    function.
    Note, that you must set the "diagram_context" and "diagram" variables in the watcher after building
    the diagram yourself.
"""

import ipdb
import numpy as np
import typer

# Drake imports
from pydrake.all import (
    AddMultibodyPlantSceneGraph, DiagramBuilder,
    AffineSystem, ConstantVectorSource,
    Meshcat, MeshcatVisualizer, Simulator,
)

from brom_drake.all import diagram_watcher, DiagramWatcherOptions, PortWatcherPlottingOptions
from brom_drake.example_helpers import BlockHandlerSystem

#######################
## Class Definitions ##
#######################

def main():

    # Building Diagram
    time_step = 1e-3

    builder = DiagramBuilder()

    # Add Multibody Plant and the "Block" System
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    block_handler_system = builder.AddSystem(BlockHandlerSystem(plant, scene_graph))

    # Create system that outputs the slowly updating value of the pose of the block.
    A = np.zeros((6, 6))
    B = np.zeros((6, 1))
    f0 = np.array([0.0, 0.1, 0.1, 0.0, 0.0, 0.0])
    C = np.eye(6)
    D = np.zeros((6,1))
    y0 = np.zeros((6,1))
    x0 = np.array([0.0,0.0,0.0,0.0,0.2,0.5])
    target_source2 = builder.AddSystem(
        AffineSystem(A, B, f0, C, D, y0)
        )
    target_source2.configure_default_state(x0)

    # Connect the state of the block to the output of a slowly changing system.
    builder.Connect(
        target_source2.get_output_port(),
        block_handler_system.GetInputPort("desired_pose"))

    u0 = np.array([0.2])
    affine_system_input = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        affine_system_input.get_output_port(),
        target_source2.get_input_port()
    )

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    m_visualizer = MeshcatVisualizer(meshcat0)
    m_visualizer.AddToBuilder(builder, scene_graph, meshcat0)

    # Add Watcher Before
    print("adding watcher before building...")
    watcher = diagram_watcher(
        builder,
        options=DiagramWatcherOptions(
            base_directory="brom",
        ),
    )

    diagram = builder.Build()

    # IMPORTANT: Set the watchers diagram_context and diagram after building the diagram
    diagram_context = diagram.CreateDefaultContext()

    watcher.diagram_context = diagram_context
    watcher.diagram = diagram

    # Set initial pose and vectors
    block_handler_system.SetInitialBlockState(diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    block_handler_system.context = block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)


if __name__ == '__main__':
    main()

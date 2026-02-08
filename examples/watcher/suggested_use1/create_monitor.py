"""
Description:

    In this script, we use the convenience function to create a DiagramWatcher.
    The watcher will automatically monitor the multiple parts of the diagram
    which contains a spinning block whose slowly changing pose is controlled
    by an affine system.
"""

import ipdb
import numpy as np
import typer

# Drake imports
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    AffineSystem,
    ConstantVectorSource,
    Meshcat,
    MeshcatVisualizer,
    Simulator,
)

from brom_drake.all import add_watcher_and_build
from brom_drake.example_helpers import BlockHandlerSystem


def main():

    # Building Diagram
    time_step = 1e-3
    builder = DiagramBuilder()

    # Create Plant and the "Block + Ground system"
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    block_handler_system = builder.AddSystem(BlockHandlerSystem(plant, scene_graph))

    # Connect System To Handler
    # Create system that outputs the slowly updating value of the pose of the block.
    A = np.zeros((6, 6))
    B = np.zeros((6, 1))
    f0 = np.array([0.0, 0.1, 0.1, 0.0, 0.0, 0.0])
    C = np.eye(6)
    D = np.zeros((6, 1))
    y0 = np.zeros((6, 1))
    x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.2, 0.5])
    target_source2 = builder.AddSystem(AffineSystem(A, B, f0, C, D, y0))
    target_source2.configure_default_state(x0)

    # Connect the state of the block to the output of a slowly changing system.
    builder.Connect(
        target_source2.get_output_port(),
        block_handler_system.GetInputPort("desired_pose"),
    )

    u0 = np.array([0.2])
    affine_system_input = builder.AddSystem(ConstantVectorSource(u0))
    builder.Connect(
        affine_system_input.get_output_port(), target_source2.get_input_port()
    )

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    m_visualizer = MeshcatVisualizer(meshcat0)
    m_visualizer.AddToBuilder(builder, scene_graph, meshcat0)

    # Add Watcher and Build
    watcher, diagram, diagram_context = add_watcher_and_build(builder)

    # Set initial pose and vectors
    block_handler_system.SetInitialBlockState(diagram_context)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    block_handler_system.context = (
        block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)
    )
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)


if __name__ == "__main__":
    main()

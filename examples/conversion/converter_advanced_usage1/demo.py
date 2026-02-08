"""
demo.py
Description
-----------
This script contains an example of how to convert a URDF file into a
"Drake-ready" URDF file using Brom's new utility.
"""

from importlib import resources as impresources
from pydrake.geometry import Meshcat, MeshcatVisualizer
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# Internal imports
from brom_drake import robots
from brom_drake.all import (
    DrakeReadyURDFConverter,
    add_watcher_and_build,
    DrakeReadyURDFConverterConfig,
)
from brom_drake.utils import AddGround


def main():
    # Setup
    urdf_file_path = str(impresources.files(robots) / "models/ur/ur10e.urdf")

    # Use converter
    converter = DrakeReadyURDFConverter(
        urdf_file_path,
        config=DrakeReadyURDFConverterConfig(
            overwrite_old_logs=True,
        ),
    )

    # Convert the URDF
    new_urdf_path = converter.convert_urdf()

    # Build Diagram with URDF in it
    time_step = 1e-3
    builder = DiagramBuilder()

    # Create Plant and the "Block + Ground system"
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
    added_models = Parser(plant=plant).AddModels(str(new_urdf_path))
    print(f"Added the following models to the plant: {added_models}")
    AddGround(plant)  # Add ground to plant
    plant.Finalize()

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    m_visualizer = MeshcatVisualizer(meshcat0)
    m_visualizer.AddToBuilder(builder, scene_graph, meshcat0)

    # Add Watcher and Build
    watcher, diagram, diagram_context = add_watcher_and_build(builder)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)


if __name__ == "__main__":
    main()

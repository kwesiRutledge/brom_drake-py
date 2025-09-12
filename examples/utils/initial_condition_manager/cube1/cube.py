import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Meshcat,
    MeshcatVisualizer,
    ModelInstanceIndex,
    MultibodyPlant,
    Parser,
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import typer

# Internal Imports
from brom_drake.all import add_watcher_and_build
from brom_drake.file_manipulation.urdf.shapes import BoxDefinition
from brom_drake.file_manipulation.urdf import SimpleShapeURDFDefinition
from brom_drake.utils.initial_condition_manager import InitialConditionManager

def add_cube_model_to_plant(
    plant: MultibodyPlant,
    name: str = "tutorial-cube"
) -> ModelInstanceIndex:
    # Create Cube 3d model
    simple_cube = BoxDefinition(size=[0.05,0.05,0.05])
    cube_urdf_defn = SimpleShapeURDFDefinition(
        name=name,
        shape=simple_cube,
        color=[0.1,0.1,0.5,1.0],
    )
    cube_urdf_path = cube_urdf_defn.write_to_file()

    # Add cube urdf to plant
    cube_model = Parser(plant=plant).AddModels(cube_urdf_path)[0]

    return cube_model

def add_floor_model_to_plant(
    plant: MultibodyPlant,
    name: str = "tutorial-floor"
) -> ModelInstanceIndex:
    # Create Floor 3d model
    simple_floor = BoxDefinition(size=[5.0,5.0,0.1])
    floor_urdf_defn = SimpleShapeURDFDefinition(
        name=name,
        shape=simple_floor,
        color=[0.5,0.5,0.5,1.0],
    )
    floor_urdf_path = floor_urdf_defn.write_to_file()

    # Add floor urdf to plant
    floor_model = Parser(plant=plant).AddModels(floor_urdf_path)[0]

    # Weld the floor to the world frame
    floor_body_indicies = plant.GetBodyIndices(floor_model)
    floor_body = plant.get_body(floor_body_indicies[0])
    plant.WeldFrames(
        plant.world_frame(),
        floor_body.body_frame(),
        RigidTransform(
            rpy=RollPitchYaw(0, 0, 0),
            p=np.array([0, 0, -0.05]),
        )
    )

    return floor_model

def main(t_final: float = 15.0):

    # Building Diagram
    time_step = 0.002

    builder = DiagramBuilder()

    # Add floor and cube to plant
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)
    floor_model = add_floor_model_to_plant(plant=plant)
    cube_model = add_cube_model_to_plant(plant=plant)

    # Create the initial condition manager
    ic_manager = InitialConditionManager()
    ic_manager.add_initial_pose(
        cube_model, 
        pose_wrt_parent=RigidTransform(
            p=np.array([0, 0, 0.1]),
            rpy=RollPitchYaw(roll=np.pi/4.0, pitch=np.pi/4.0, yaw=np.pi/4.0)
        )
    )

    # Finalize the plant
    plant.Finalize()

    # Set the initial conditions in the plant
    ic_manager.set_all_initial_conditions(plant=plant)

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    mCpp = MeshcatVisualizer(meshcat0)
    mCpp.AddToBuilder(builder,scene_graph,meshcat0)

    # Build the diagram
    _, diagram, diagram_context = add_watcher_and_build(builder)

    # Start the simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.Initialize()
    simulator.AdvanceTo(t_final)

if __name__ == '__main__':
    typer.run(main)
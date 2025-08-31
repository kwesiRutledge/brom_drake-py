import ipdb
import numpy as np
import matplotlib.pyplot as plt
from pydrake.all import (
    AbstractValue,
    RotationMatrix, RigidTransform, CoulombFriction, HalfSpace, RollPitchYaw,
    AddMultibodyPlantSceneGraph, 
    ConstantValueSource,
    Parser, AddModel,
    Meshcat,
    MeshcatVisualizer,
    MultibodyPlant, DiagramBuilder,
    PiecewisePolynomial,
    PiecewiseQuaternionSlerp,
    PiecewisePose,
    Simulator,
    TrajectorySource,
)
import typer

# Internal imports
from brom_drake.utils import AddMultibodyTriad
from brom_drake.file_manipulation.urdf.shapes import (
    BoxDefinition,
)
from brom_drake.file_manipulation.urdf import (
    SimpleShapeURDFDefinition,
)
from brom_drake.utils.puppetmaker import Puppetmaker, PuppetmakerConfiguration
from brom_drake.motion_planning.systems.open_loop_dispensers import OpenLoopPosePlanDispenser
from brom_drake.all import add_watcher_and_build

def create_piecewise_pose_trajectory(t_final: float = 15.0):
    # Setup
    times = np.array([0.0, t_final])

    # Create position trajectory
    x0 = np.array([0.0, 0.1, 0.1])
    xT = x0 + np.array([0.0, 0.0, 0.4])

    # Create rotation trajectory
    rpy0 = RollPitchYaw(0.0, 0.0, 0.0)
    rpyT = RollPitchYaw(0.0, np.pi/4.0, np.pi/2.0)

    # Return the pose trajectory
    return [
        RigidTransform(rpy=rpy0, p=x0),
        RigidTransform(rpy=rpyT, p=xT),
    ]

def create_trajectory_source(
    builder: DiagramBuilder,
    pose_trajectory: list[RigidTransform]
) -> OpenLoopPosePlanDispenser:
    # Setup

    # Create dispenser of trajectory 
    trajectory_dispenser = builder.AddSystem(
        OpenLoopPosePlanDispenser(speed=0.025)
    )

    # Connect a source to "ALWAYS enable" the trajectory source
    enable_trajectory_source = builder.AddSystem(
        ConstantValueSource(value=AbstractValue.Make(True))
    )

    builder.Connect(
        enable_trajectory_source.get_output_port(),
        trajectory_dispenser.GetInputPort("plan_ready")
    )

    # Create source for trajectory
    trajectory_value = builder.AddSystem(
        ConstantValueSource(
            AbstractValue.Make(pose_trajectory)
        )
    )

    builder.Connect(
        trajectory_value.get_output_port(),
        trajectory_dispenser.GetInputPort("plan")
    )

    return trajectory_dispenser

def main(t_final: float = 15.0):

    # Building Diagram
    time_step = 0.002

    builder = DiagramBuilder()

    # plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

    # Create Cube 3d model
    simple_cube = BoxDefinition(size=[0.05,0.05,0.05])
    cube_urdf_defn = SimpleShapeURDFDefinition(
        name="tutorial-cube",
        shape=simple_cube,
        color=[0.1,0.1,0.5,1.0],
    )
    cube_urdf_path =cube_urdf_defn.write_to_file()

    # Add cube urdf to plant
    cube_model = Parser(plant=plant).AddModels(cube_urdf_path)[0]

    # Add "puppet strings" for the cube
    puppetmaker0 = Puppetmaker(plant=plant)
    cube_signature = puppetmaker0.add_strings_for(cube_model)

    # Finalize the plant
    plant.Finalize()

    # Add the puppet controller to the builder
    pose_converter_system = puppetmaker0.add_puppet_controller_for(
        cube_signature,
        builder,
    )

    # Connect a trajectory source system to this
    pose_trajectory = create_piecewise_pose_trajectory(t_final)
    trajectory_source = create_trajectory_source(builder, pose_trajectory)

    builder.Connect(
        trajectory_source.GetOutputPort("pose_in_plan"),
        pose_converter_system.get_input_port()
    )

    # Connect to Meshcat
    meshcat0 = Meshcat(port=7001)  # Object provides an interface to Meshcat
    m_visualizer = MeshcatVisualizer(meshcat0)
    m_visualizer.AddToBuilder(builder, scene_graph, meshcat0)

    watcher, diagram, diagram_context = add_watcher_and_build(builder)

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(t_final)

if __name__ == '__main__':
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
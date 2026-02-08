import ipdb
import importlib.resources as impresources
import numpy as np
from pydrake.all import (
    AbstractValue,
    RigidTransform,
    RollPitchYaw,
    AddMultibodyPlantSceneGraph,
    ConstantValueSource,
    Parser,
    Meshcat,
    MeshcatVisualizer,
    MultibodyPlant,
    DiagramBuilder,
    Simulator,
)
import typer

# Internal imports
from brom_drake import robots
from brom_drake.file_manipulation.urdf.shapes import (
    BoxDefinition,
)
from brom_drake.file_manipulation.urdf import (
    SimpleShapeURDFDefinition,
)
from brom_drake.motion_planning.systems.open_loop_dispensers import (
    OpenLoopPosePlanDispenser,
)
from brom_drake.all import (
    add_watcher_and_build,
    drakeify_my_urdf,
    MeshReplacementStrategy,
    Puppetmaker,
    PuppetmakerConfiguration,
)


def create_piecewise_pose_trajectory(t_final: float = 15.0):
    # Setup

    # Create position trajectory

    # Return the pose trajectory
    return [
        RigidTransform(
            rpy=RollPitchYaw(ii * np.pi / 4.0, 0.0, ii * np.pi / 4.0),
            p=np.array([-ii * 0.1, -ii * 0.1, ii * 0.2]),
        )
        for ii in range(8)
    ]


def create_trajectory_source(
    builder: DiagramBuilder, pose_trajectory: list[RigidTransform]
) -> OpenLoopPosePlanDispenser:
    # Setup

    # Create dispenser of trajectory
    trajectory_dispenser = builder.AddSystem(OpenLoopPosePlanDispenser(speed=0.06))

    # Connect a source to "ALWAYS enable" the trajectory source
    enable_trajectory_source = builder.AddSystem(
        ConstantValueSource(value=AbstractValue.Make(True))
    )

    builder.Connect(
        enable_trajectory_source.get_output_port(),
        trajectory_dispenser.GetInputPort("plan_ready"),
    )

    # Create source for trajectory
    trajectory_value = builder.AddSystem(
        ConstantValueSource(AbstractValue.Make(pose_trajectory))
    )

    builder.Connect(
        trajectory_value.get_output_port(), trajectory_dispenser.GetInputPort("plan")
    )

    return trajectory_dispenser


def main(t_final: float = 15.0):

    # Building Diagram
    time_step = 0.002

    builder = DiagramBuilder()

    # plant = builder.AddSystem(MultibodyPlant(time_step=time_step)) #Add plant to diagram builder
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

    # Load the wrench model
    wrench_file = impresources.files(robots) / "models/wrenches/wrench1/wrench.urdf"

    drakeified_wrench_urdf = drakeify_my_urdf(
        wrench_file,
        overwrite_old_logs=True,
        log_file_name="DemonstrateStaticGripTest_AddManipulandToPlant_flask.log",
        # # For you (yes, you!): Comment out the line below, to see what the default collision mesh looks like
        # collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithConvexDecomposition,
    )

    # Add cube urdf to plant
    cube_model = Parser(plant=plant).AddModels(str(drakeified_wrench_urdf))[0]

    # Add "puppet strings" for the cube
    pm_config0 = PuppetmakerConfiguration(
        frame_on_parent=plant.world_frame(),
        name="cube_puppet",
        sphere_radius=0.02,
        sphere_mass=1e-4,  # Make the puppet links very light
        sphere_color=np.array(
            [0.11372549, 0.12890625, 0.34375, 0.0]
        ),  # Change the color of the puppet links to red
    )
    puppetmaker0 = Puppetmaker(
        plant=plant,
        config=pm_config0,
    )
    cube_signature = puppetmaker0.add_strings_for(cube_model)

    # Finalize the plant
    plant.Finalize()

    # Add the puppet controller to the builder
    pose_converter_system, _ = puppetmaker0.add_puppet_controller_for(
        cube_signature,
        builder,
        Kp=np.array([150.0, 150.0, 150.0, 1e1, 1e1, 3e0]),
    )

    # Connect a trajectory source system to this
    pose_trajectory = create_piecewise_pose_trajectory(t_final)
    trajectory_source = create_trajectory_source(builder, pose_trajectory)

    builder.Connect(
        trajectory_source.GetOutputPort("pose_in_plan"),
        pose_converter_system.get_input_port(),
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


if __name__ == "__main__":
    main()

from importlib import resources as impresources
import numpy as np
from pathlib import Path
from pydrake.all import (
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import subprocess
import trimesh

# Internal Imports
from brom_drake.PortWatcher.port_watcher_options import FigureNamingConvention
from brom_drake.all import GripperType
from brom_drake import robots
from brom_drake.productions import AttemptGraspWithPuppeteerWrist, AttemptGraspWithPuppeteerWristConfiguration

def create_block_m_sdf(path_to_block_m_stl: Path, scaling_factor: float = 0.003, model_mass: float = 2e-3) -> Path:
    """
    *Description*

    Uses the drake internal tools to create an STL file using the STL for the
    robot.
    """
    # Convert mesh to an obj file using trimesh
    block_as_trimesh = trimesh.load_mesh(path_to_block_m_stl)
    
    obj_file_name = path_to_block_m_stl.parent / path_to_block_m_stl.name.replace(".stl", ".obj")
    block_as_trimesh.export(obj_file_name)

    # Use the pydrake utility to create an SDF of the new .obj file
    subprocess.run([
        "python3",
        "-m", "pydrake.multibody.mesh_to_model", str(obj_file_name), 
        "--scale", str(scaling_factor),
        "--origin-at-com",
        "--mass", str(model_mass)
    ])    

    # Return the new sdf's name
    return path_to_block_m_stl.parent / (path_to_block_m_stl.stem + ".sdf")

def main(scaling_factor: float = 3e-3):
    """
    *Description*
    
    In this example, we will demonstrate how to:
    - Load a model into the `AttempptGraspWithPuppeteerWrist` production
    - Specify a sequence of target poses for the gripper wrist to follow
      in order to attempt a grasp on the object.
    - Run a simulation of the grasp attempt, visualizing the results in Meshcat.
    """

    # Setup

    # Create michigan block M sdf
    block_m_stl = impresources.files(robots) / "models/michigan_block_m/University_of_Michigan_M.stl"
    block_m_sdf = create_block_m_sdf(block_m_stl, scaling_factor=scaling_factor)

    # Create the following poses (transforms):
    # - The Grasp Pose (i.e. the target pose of the gripper wrist when grasping the object)
    X_ObjectTarget = RigidTransform(
        p=np.array([-0.13, 0.0, 0.15]),
        rpy=RollPitchYaw(0.1, (6.0/8.0)*np.pi, 0.1),
    )

    # - The Pre-Grasp Pose (i.e. the pose of the gripper wrist just before reaching to grasp the object)
    X_ObjectPreGrasp = X_ObjectTarget.multiply(
        RigidTransform(
            p=np.array([0.0, 0.0, -0.2]),
            rpy=RollPitchYaw(0.0, 0.0, 0.0),
        )
    )
    # X_ObjectPreAlignment = RigidTransform(
    #     p=np.array([-0.1, 0.0, -0.4]),
    #     rpy=RollPitchYaw(0.01, 0.01, 0.01),
    # )

    # Create the production
    production_config = AttemptGraspWithPuppeteerWristConfiguration()
    production_config.script.gripper_approach_time = 20.0
    production = AttemptGraspWithPuppeteerWrist(
        path_to_object=str(block_m_sdf),
        gripper_choice=GripperType.Robotiq_2f_85,
        grasp_joint_positions=np.array([0.7]),
        X_ObjectGripper_trajectory=[X_ObjectPreGrasp, X_ObjectTarget],
        meshcat_port_number=7001, # Use None for CI
        config=production_config,
    )

    # Build with watcher (so we can view the simulation's data in `brom/watcher/plots` which is helpful for debugging)
    diagram, diagram_context = production.add_cast_and_build(
        figure_naming_convention=FigureNamingConvention.kHierarchical
    )

    script = production.config.script

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    simulator.Initialize()
    simulator.AdvanceTo(script.total_time())

if __name__ == "__main__":
    main()
from importlib import resources as impresources
import ipdb
import numpy as np
from pathlib import Path
from pydrake.all import (
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import typer

# Internal Imports
from brom_drake.PortWatcher.port_watcher_options import FigureNamingConvention
from brom_drake.all import drakeify_my_urdf, GripperType, MeshReplacementStrategy
from brom_drake import robots
from brom_drake.productions import AttemptGraspWithPuppeteerWrist

def main():
    """
    *Description*
    
    In this example, we will demonstrate how to:
    - Load a model into the `AttempptGraspWithPuppeteerWrist` production
    - Specify a sequence of target poses for the gripper wrist to follow
      in order to attempt a grasp on the object.
    - Run a simulation of the grasp attempt, visualizing the results in Meshcat.
    """

    # Setup

    # Create erlenmeyer flask urdf
    erlenmeyer_flask_file = str(
        impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
    )

    drakeified_flask_urdf = Path("brom/models/500ml/500ml.drake.urdf")
    if not drakeified_flask_urdf.exists():
        drakeified_flask_urdf = drakeify_my_urdf(
            erlenmeyer_flask_file,
            overwrite_old_logs=True,
            log_file_name="DemonstrateStaticGripTest_AddManipulandToPlant_flask.log",
            # For you (yes, you!): Comment out the line below, to see what the default collision mesh looks like
            collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithConvexDecomposition,
        )

    # Create the following poses (transforms):
    # - The Grasp Pose (i.e. the target pose of the gripper wrist when grasping the object)
    X_ObjectTarget = RigidTransform(
        p=np.array([-0.08, 0.05, 0.15]),
        rpy=RollPitchYaw(0.1, (11.0/20.0)*np.pi, 0.1),
    )

    # - The Pre-Grasp Pose (i.e. the pose of the gripper wrist just before reaching to grasp the object)
    X_WorldPreGrasp = X_ObjectTarget.multiply(
        RigidTransform(
            p=np.array([0.0, 0.0, -0.2]),
            rpy=RollPitchYaw(0.0, 0.0, 0.0),
        )
    )

    # Create the production
    production = AttemptGraspWithPuppeteerWrist(
        path_to_object=str(drakeified_flask_urdf),
        gripper_choice=GripperType.Robotiq_2f_85,
        grasp_joint_positions=np.array([0.7]),
        X_ObjectGripper_trajectory=[X_WorldPreGrasp, X_ObjectTarget],
        meshcat_port_number=7001, # Use None for CI
    )

    # Build with watcher (so we can view the simulation's data in `brom/watcher/plots` which is helpful for debugging)
    diagram, diagram_context = production.add_cast_and_build(
        figure_naming_convention=FigureNamingConvention.kHierarchical
    )

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    simulator.Initialize()
    simulator.AdvanceTo(60.0)

if __name__ == "__main__":
    main()
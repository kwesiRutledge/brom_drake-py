from importlib import resources as impresources
from pathlib import Path
import ipdb
import numpy as np
from pydrake.all import (
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import typer

# Internal Imports
from brom_drake.all import drakeify_my_urdf, GripperType, MeshReplacementStrategy
from brom_drake import robots
from brom_drake.productions.all import AttemptGraspWithStaticWrist


def main():
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

    # Create the transform representing the target (i.e. gripper) frame
    # relative to the object frame
    X_ObjectTarget = RigidTransform(
        p=np.array([-0.08, 0.05, 0.15]),
        rpy=RollPitchYaw(0.0, np.pi / 2.0, 0.0),
    )

    # Create the production
    production = AttemptGraspWithStaticWrist(
        path_to_object=str(drakeified_flask_urdf),
        gripper_choice=GripperType.Robotiq_2f_85,
        grasp_joint_positions=np.array([0.7]),
        X_ObjectTarget=X_ObjectTarget,
        meshcat_port_number=7001,  # Use None for CI
    )

    # Build with watcher
    diagram, diagram_context = production.add_cast_and_build()

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    simulator.Initialize()
    simulator.AdvanceTo(60.0)


if __name__ == "__main__":
    main()

from importlib import resources as impresources
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
from brom_drake.productions import AttemptGrasp, AttemptGraspConfiguration

def main():
    # Setup
    robots_dir = impresources.files(robots)

    # Create erlenmeyer flask urdf
    wrench_file = robots_dir / "../../../examples/productions/helpful_for_debugging/grasping/attempt_grasp/wrench_with_nonconvex_geometries2/wrench.urdf"

    drakeified_flask_urdf = drakeify_my_urdf(
        wrench_file,
        overwrite_old_logs=True,
        log_file_name="DemonstrateStaticGripTest_AddManipulandToPlant_flask.log",
        # For you (yes, you!): Comment out the line below, to see what the default collision mesh looks like
        collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithConvexDecomposition,
    )

    # Create the transform representing the target (i.e. gripper) frame
    # relative to the object frame
    X_ObjectTarget = RigidTransform(
        p=np.array([-0.08, 0.05, 0.15]),
        rpy=RollPitchYaw(0.0, np.pi, 0.0),
    )

    # Create the production
    config = AttemptGraspConfiguration()
    # config.meshcat_port_number = None # Use None for CI

    production = AttemptGrasp(
        path_to_object=str(drakeified_flask_urdf),
        gripper_choice=GripperType.Robotiq_2f_85,
        grasp_joint_positions=np.array([0.7]),
        X_ObjectTarget=X_ObjectTarget,
        config=config
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
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
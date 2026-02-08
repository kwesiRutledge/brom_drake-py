from importlib import resources as impresources
import numpy as np
from pydrake.all import (
    Simulator,
    RollPitchYaw,
    RigidTransform,
)

# Internal Imports
from brom_drake.all import drakeify_my_urdf
from brom_drake import robots
from brom_drake.productions.all import (
    ShowMeThisStaticGrasp,
    ShowMeThisStaticGraspConfiguration,
)


def main(meshcat_port_number: int = 7001):
    """
    Description
    -----------
    This test verifies how we can use the add_cast_and_build method
    to build a DemonstrateStaticGrasp production.
    """
    # Setup

    # Create erlenmeyer flask urdf
    erlenmeyer_flask_file = str(
        impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
    )

    drakeified_flask_urdf = drakeify_my_urdf(
        erlenmeyer_flask_file,
        overwrite_old_logs=True,
        log_file_name="DemonstrateStaticGripTest_AddManipulandToPlant_flask.log",
    )

    # Create the gripper urdf
    gripper_urdf = str(
        impresources.files(robots)
        / "models/robotiq/2f_85_gripper-no-mimic/urdf/robotiq_2f_85.urdf"
    )

    X_ObjectTarget = RigidTransform(
        p=np.array([-0.08, 0.05, 0.15]),
        rpy=RollPitchYaw(0.0, np.pi / 2.0, 0.0),
    )

    # Create the production
    config = ShowMeThisStaticGraspConfiguration()
    config.meshcat_port_number = meshcat_port_number
    config.time_step = 1e-3

    production = ShowMeThisStaticGrasp(
        path_to_object=str(drakeified_flask_urdf),
        path_to_gripper=gripper_urdf,
        X_ObjectGripper=X_ObjectTarget,
        config=config,
    )

    # Call the method
    diagram, diagram_context = production.add_cast_and_build()

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)
    simulator.Initialize()
    simulator.AdvanceTo(10.0)


if __name__ == "__main__":
    main()

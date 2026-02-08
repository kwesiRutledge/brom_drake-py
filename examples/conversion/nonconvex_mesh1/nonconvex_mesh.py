"""
Description
-----------
This script contains an example of how to convert a URDF file containing a non-convex
mesh into a "Drake-ready" URDF file with only convex collision elements
(making collision detection a bit more precise).
"""

from importlib import resources as impresources
from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake import robots
from brom_drake.all import drakeify_my_urdf, MeshReplacementStrategy
from brom_drake.productions.all import ShowMeThisModel


def main():
    # Setup
    urdf_file_path = str(
        impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
    )

    # Convert the URDF
    new_urdf_path = drakeify_my_urdf(
        urdf_file_path,
        overwrite_old_logs=True,
        log_file_name="drakeify-my-urdf1.log",
        # For you (yes, you!): Comment out the line below, to see what the default collision mesh looks like
        collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithConvexDecomposition,
    )

    # Use ShowMeThisModel to visualize the URDF
    show_me_production = ShowMeThisModel(
        path_to_model=str(new_urdf_path),
        show_collision_geometries=True,
    )

    diagram, context = show_me_production.add_cast_and_build()

    # Run simulation
    simulator = Simulator(diagram, context)
    simulator.set_target_realtime_rate(1.0)

    simulator.Initialize()
    simulator.AdvanceTo(15.0)


if __name__ == "__main__":
    main()

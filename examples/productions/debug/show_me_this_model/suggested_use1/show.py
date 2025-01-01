"""
Description
-----------
This script contains an example of how to show
a given urdf at some (optional) specified joint positions.
For more info, refer to the README or the Wiki:
https://github.com/kwesiRutledge/brom_drake-py/wiki/Productions-%E2%80%90-ShowMeThisModel
"""

import ipdb
from importlib import resources as impresources
import numpy as np
import typer
from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake import robots
from brom_drake.all import drakeify_my_urdf
from brom_drake.productions import ShowMeThisModel

def main():
    # Setup
    urdf_file_path = str(
        impresources.files(robots) / "models/ur/ur10e.urdf"
    )

    # Convert the URDF into a format that Drake likes
    new_urdf_path = drakeify_my_urdf(
        urdf_file_path,
        overwrite_old_logs=True,
        log_file_name="drakeify-my-urdf1.log",
    )

    # Visualize the URDF using the "show-me-this-model" feature
    time_step = 1e-3
    production = ShowMeThisModel(
        str(new_urdf_path),
        with_these_joint_positions=[0.0, 0.0, -np.pi/4.0, 0.0, 0.0, 0.0],
        time_step=time_step,
    )

    # Build Diagram
    diagram, diagram_context = production.add_cast_and_build()

    # Set up simulation
    simulator = Simulator(diagram, diagram_context)
    simulator.set_target_realtime_rate(1.0)
    simulator.set_publish_every_time_step(False)

    # Run simulation
    simulator.Initialize()
    simulator.AdvanceTo(15.0)


if __name__ == "__main__":
    with ipdb.launch_ipdb_on_exception():
        typer.run(main)
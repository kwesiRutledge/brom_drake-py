"""
Description
-----------
This script records an MP4 video of a production from a single camera.

Recording is opt-in: a production only renders images if you call
`add_cameras()` before building it. After simulating, `save_videos()` finishes
the files and returns where they were written.

For more info, refer to the README or the docs:
https://kwesirutledge.github.io/brom_drake-py/
"""

from importlib import resources as impresources

import numpy as np
from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake import robots
from brom_drake.all import CameraSpec, drakeify_my_urdf
from brom_drake.productions.all import ShowMeThisModel


def main(meshcat_port_number: int = 7001):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None  # Use None for CI

    # Convert the UR10e URDF into a format that Drake likes
    urdf_file_path = str(impresources.files(robots) / "models/ur/ur10e.urdf")
    new_urdf_path = drakeify_my_urdf(
        urdf_file_path,
        overwrite_old_logs=True,
        log_file_name="record-single-camera1.log",
    )

    # Create the production
    production = ShowMeThisModel(
        str(new_urdf_path),
        with_these_joint_positions=[
            0.3,
            -np.pi / 3,
            np.pi / 2,
            -np.pi / 4,
            -np.pi / 2,
            0.0,
        ],
        meshcat_port_number=meshcat_port_number,
    )

    # Ask for one camera BEFORE building the production.
    # looking_at() places the camera at `position`, pointed at `target`.
    production.add_cameras(
        [
            CameraSpec.looking_at(
                "front",
                position=[1.6, 0.9, 1.1],
                target=[0.0, 0.0, 0.55],
                width=640,
                height=480,
                fps=30,
            ),
        ],
        output_dir="brom/videos/single_camera1",
    )

    # Build and simulate as usual
    diagram, diagram_context = production.add_cast_and_build()
    simulator = Simulator(diagram, diagram_context)
    simulator.Initialize()
    simulator.AdvanceTo(3.0)

    # Finish the video file(s)
    video_paths = production.save_videos()
    for name, path in video_paths.items():
        print(f"Saved the {name} camera's video to {path}")


if __name__ == "__main__":
    main(meshcat_port_number=-1)

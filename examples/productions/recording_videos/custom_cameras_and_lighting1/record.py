"""
Description
-----------
This script records a dynamic motion planning production (ChemLab2) with
custom cameras and custom lighting.

It shows how to:
- give each camera its own resolution, field of view and frame rate,
- place a camera with an explicit pose (RigidTransform) instead of looking_at,
- choose the background color and lights with RenderSettings,
- lay out the combined grid video (number of columns, size of each tile).

The grid video stays in sync even though the cameras record at different
frame rates: each grid frame shows every camera's latest image at that
simulation time.

For more info, refer to the README or the docs:
https://kwesirutledge.github.io/brom_drake-py/
"""

import numpy as np
from pydrake.geometry import LightParameter, Rgba
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.systems.analysis import Simulator

# Internal imports
from brom_drake.all import CameraSpec, RenderSettings
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2


def main(meshcat_port_number: int = 7001, duration: float = 8.0):
    # Setup
    if meshcat_port_number < 0:
        meshcat_port_number = None  # Use None for CI

    production = ChemLab2(meshcat_port_number=meshcat_port_number)

    # Cameras with different resolutions and frame rates
    cameras = [
        # A wide, smooth overview
        CameraSpec.looking_at(
            "overview",
            position=[2.0, -2.0, 1.7],
            target=[0.3, 0.1, 0.4],
            fov_y=np.deg2rad(40),
            width=854,
            height=480,
            fps=30,
        ),
        # A cheaper, lower-rate side view
        CameraSpec.looking_at(
            "side",
            position=[0.3, -2.4, 0.8],
            target=[0.3, 0.0, 0.5],
            fov_y=np.deg2rad(40),
            width=640,
            height=360,
            fps=10,
        ),
        # A camera given by an explicit pose: 2.5 m above the scene, looking
        # straight down. (Drake cameras look along their +Z axis, so this
        # rotation flips +Z to point at -Z_world.)
        CameraSpec(
            "top",
            X_PC=RigidTransform(
                RotationMatrix(
                    np.array([[0.0, -1.0, 0.0], [-1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
                ),
                [0.3, 0.0, 2.5],
            ),
            width=480,
            height=480,
            fps=15,
        ),
    ]

    # A dark background with a warm key light and a cool rim light
    render_settings = RenderSettings(
        background_rgb=(0.12, 0.13, 0.16),
        lights=[
            LightParameter(
                type="directional",
                frame="world",
                direction=[-0.4, 0.5, -1.0],
                intensity=1.0,
                color=Rgba(1.0, 0.95, 0.88),
            ),
            LightParameter(
                type="directional",
                frame="world",
                direction=[0.7, -0.3, -0.2],
                intensity=0.45,
                color=Rgba(0.75, 0.85, 1.0),
            ),
        ],
        cast_shadows=True,
    )

    production.add_cameras(
        cameras,
        output_dir="brom/videos/custom_cameras_and_lighting1",
        render_settings=render_settings,
    )

    # Add a motion planner and build
    planner = BidirectionalRRTConnectPlanner(
        production.arm,
        production.plant,
        production.scene_graph,
        config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
    )
    diagram, diagram_context = production.easy_cast_and_build(
        planner.plan,
        with_watcher=False,
    )

    # Simulate
    simulator = Simulator(diagram, diagram_context)
    simulator.Initialize()
    simulator.AdvanceTo(duration)

    # Two columns, with every camera scaled to fit a 640x360 tile
    video_paths = production.save_videos(
        grid=True,
        grid_columns=2,
        grid_tile_size=(640, 360),
    )
    for name, path in video_paths.items():
        print(f"Saved {name}: {path}")


if __name__ == "__main__":
    main(meshcat_port_number=-1)

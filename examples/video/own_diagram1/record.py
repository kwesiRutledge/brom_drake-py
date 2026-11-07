"""
Description
-----------
This script records a video of YOUR OWN Drake diagram (no Brom production
needed) using `add_cameras_to_builder`.

The scene: a red box falls onto the ground. One camera is fixed in the world
and a second camera is attached to the box itself (so it tumbles with it).

For more info, refer to the README or the docs:
https://kwesirutledge.github.io/brom_drake-py/
"""

import numpy as np
from pydrake.geometry import Box, HalfSpace
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction
from pydrake.multibody.tree import SpatialInertia, UnitInertia
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder

# Internal imports
from brom_drake.all import CameraSpec, RenderSettings, add_cameras_to_builder


def build_falling_box_scene(builder: DiagramBuilder):
    plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

    # Ground
    friction = CoulombFriction(0.8, 0.6)
    X_WG = RigidTransform()
    plant.RegisterCollisionGeometry(
        plant.world_body(), X_WG, HalfSpace(), "ground_collision", friction
    )
    plant.RegisterVisualGeometry(
        plant.world_body(),
        RigidTransform([0, 0, -0.01]),
        Box(4.0, 4.0, 0.02),
        "ground_visual",
        [0.55, 0.57, 0.6, 1.0],
    )

    # A box that starts in the air, tilted
    box = plant.AddRigidBody(
        "box",
        SpatialInertia(
            mass=1.0,
            p_PScm_E=np.zeros(3),
            G_SP_E=UnitInertia.SolidBox(0.2, 0.2, 0.2),
        ),
    )
    plant.RegisterCollisionGeometry(
        box, RigidTransform(), Box(0.2, 0.2, 0.2), "box_collision", friction
    )
    plant.RegisterVisualGeometry(
        box, RigidTransform(), Box(0.2, 0.2, 0.2), "box_visual", [0.85, 0.2, 0.15, 1.0]
    )
    plant.Finalize()
    plant.SetDefaultFloatingBaseBodyPose(
        box, RigidTransform(RollPitchYaw(0.4, 0.6, 0.0), [0.0, 0.0, 1.0])
    )
    return plant, scene_graph


def main():
    builder = DiagramBuilder()
    plant, scene_graph = build_falling_box_scene(builder)

    # Add cameras (and their recorders) to YOUR builder, before Build()
    recorders = add_cameras_to_builder(
        builder,
        scene_graph,
        [
            CameraSpec.looking_at(
                "world_view", position=[1.7, -1.7, 1.2], target=[0.0, 0.0, 0.45]
            ),
            # Attached to the box: 0.5 m "behind" it along the box's -Y axis,
            # looking back at it along +Y (camera +Z axis = box +Y axis)
            CameraSpec(
                "box_view",
                X_PC=RigidTransform(
                    RollPitchYaw(-np.pi / 2, 0.0, 0.0), [0.0, -0.5, 0.0]
                ),
                parent_body_name="box",
            ),
        ],
        plant=plant,  # Needed for cameras attached to a body
        output_dir="brom/videos/own_diagram1",
        render_settings=RenderSettings.studio(),
    )

    diagram = builder.Build()
    simulator = Simulator(diagram)
    simulator.AdvanceTo(2.0)

    # Finish each video file
    for recorder in recorders:
        print(f"Saved {recorder.get_name()}: {recorder.save()}")


if __name__ == "__main__":
    main()

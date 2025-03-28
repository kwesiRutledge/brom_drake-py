import numpy as np
from pydrake.all import (
    Cylinder,
    Frame,
    GeometryInstance,
    MakePhongIllustrationProperties,
    RigidTransform,
    RotationMatrix,
    SceneGraph,
)

def AddTriad(
    source_id,
    frame_id,
    scene_graph: SceneGraph,
    length: float = 0.25,
    radius: float = 0.01,
    opacity: float = 1.0,
    X_FT: RigidTransform = RigidTransform(),
    name: str= "frame",
):
    """
    Adds illustration geometry representing the coordinate frame, with the
    x-axis drawn in red, the y-axis in green and the z-axis in blue. The axes
    point in +x, +y and +z directions, respectively.

    Args:
      source_id: The source registered with SceneGraph.
      frame_id: A geometry::frame_id registered with scene_graph.
      scene_graph: The SceneGraph with which we will register the geometry.
      length: the length of each axis in meters.
      radius: the radius of each axis in meters.
      opacity: the opacity of the coordinate axes, between 0 and 1.
      X_FT: a RigidTransform from the triad frame T to the frame_id frame F
      name: the added geometry will have names name + " x-axis", etc.
    """
    # x-axis
    X_TG = RigidTransform(RotationMatrix.MakeYRotation(np.pi / 2), [length / 2.0, 0, 0])
    geom = GeometryInstance(
        X_FT.multiply(X_TG), Cylinder(radius, length), name + " x-axis"
    )
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([1, 0, 0, opacity])
    )
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # y-axis
    X_TG = RigidTransform(RotationMatrix.MakeXRotation(np.pi / 2), [0, length / 2.0, 0])
    geom = GeometryInstance(
        X_FT.multiply(X_TG), Cylinder(radius, length), name + " y-axis"
    )
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 1, 0, opacity])
    )
    scene_graph.RegisterGeometry(source_id, frame_id, geom)

    # z-axis
    X_TG = RigidTransform([0, 0, length / 2.0])
    geom = GeometryInstance(
        X_FT.multiply(X_TG), Cylinder(radius, length), name + " z-axis"
    )
    geom.set_illustration_properties(
        MakePhongIllustrationProperties([0, 0, 1, opacity])
    )
    scene_graph.RegisterGeometry(source_id, frame_id, geom)


def AddMultibodyTriad(frame: Frame, scene_graph: SceneGraph, length: float =0.25, radius: float =0.01, opacity: float =1.0):
    plant = frame.GetParentPlant()
    AddTriad(
        plant.get_source_id(),
        plant.GetBodyFrameIdOrThrow(frame.body().index()),
        scene_graph,
        length,
        radius,
        opacity,
        frame.GetFixedPoseInBodyFrame(),
        name=frame.name(),
    )
from enum import IntEnum
import numpy as np
from pydrake.all import (
    MultibodyPlant,
    RotationMatrix, RigidTransform,
    CoulombFriction,
    HalfSpace, Box,
)

# Class Definition
class GroundShape(IntEnum):
    """
    Description:
        This class defines the different shapes that the ground can have.
    """
    kHalfSpace = 0
    kBox = 1

def AddGround(
    plant: MultibodyPlant,
    shape: GroundShape = GroundShape.kBox,
):
    """
    Add a flat ground with friction
    """

    # Constants
    transparent_color = np.array([0.5, 0.5, 0.5, 0])
    nontransparent_color = np.array([0.5, 0.5, 0.5, 0.1])

    p_GroundOrigin = [0, 0.0, 0.0]
    R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
    X_GroundOrigin = RigidTransform(R_GroundOrigin,p_GroundOrigin)

    # Input Processing
    if shape == GroundShape.kHalfSpace:
        shape = HalfSpace()
    elif shape == GroundShape.kBox:
        shape = Box(10, 10, 0.1)
    else:
        raise ValueError(f"Invalid shape: {shape}!")  

    # Set Up Ground on Plant
    surface_friction = CoulombFriction(
            static_friction = 0.7,
            dynamic_friction = 0.5)
    plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            shape,
            "ground_collision",
            surface_friction)
    plant.RegisterVisualGeometry(
            plant.world_body(),
            X_GroundOrigin,
            shape,
            "ground_visual",
            transparent_color)  # transparent
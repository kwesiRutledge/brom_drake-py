"""
Parameter specs shared by the productions' ``Scenario`` declarations.

Joint limits are copied by hand from the arm models in ``brom_drake/robots/models``
(reading them from the plant would mean building it). Workspace boxes are in
the world frame, whose origin is at the arm's base in every production that
uses them.
"""

import math
from typing import Tuple

from brom_drake.productions.types.base.scenario import (
    FloatParam,
    PoseParam,
    VectorParam,
)

TWO_PI = 2.0 * math.pi

# Tilt allowed for objects that should stand (or lie) as placed, in radians.
UPRIGHT_TOLERANCE = 0.05

# ---------------------------------------------------------------------------
# UR10e (ChemLab1, ChemLab2, ChemLab3, ShelfPlanning1)
# ---------------------------------------------------------------------------

# From robots/models/ur/ur10e.urdf: every joint is +/-2*pi except the elbow.
UR10E_JOINT_LIMITS = (TWO_PI, TWO_PI, math.pi, TWO_PI, TWO_PI, TWO_PI)

UR10E_CONFIGURATION = VectorParam(
    6,
    tuple(-limit for limit in UR10E_JOINT_LIMITS),
    UR10E_JOINT_LIMITS,
    "rad",
)

# The UR10e reaches about 1.3 m from its base.
UR10E_TARGET_POSE = PoseParam(
    xyz_min=(-1.3, -1.3, -0.2),
    xyz_max=(1.3, 1.3, 1.5),
)

# ---------------------------------------------------------------------------
# Kinova Gen3 (CornellTech1)
# ---------------------------------------------------------------------------

# From robots/models/kinova/gen3_{6,7}dof: every joint lies within +/-pi.
# The joint count depends on the production's arm_variant, which is not a
# scenario field, so the length is left open.
KINOVA_GEN3_CONFIGURATION = VectorParam(None, -math.pi, math.pi, "rad")

# The Gen3 reaches about 0.9 m from its base.
KINOVA_GEN3_TARGET_POSE = PoseParam(
    xyz_min=(-0.9, -0.9, -0.2),
    xyz_max=(0.9, 0.9, 1.2),
)

# ---------------------------------------------------------------------------
# Scene objects
# ---------------------------------------------------------------------------


def placed_object_pose(
    xyz_min: Tuple[float, float, float],
    xyz_max: Tuple[float, float, float],
    roll: float = 0.0,
) -> PoseParam:
    """
    *Description*

    A pose for an object that keeps its placed orientation (``roll`` about x,
    no pitch) up to :data:`UPRIGHT_TOLERANCE`, and may turn freely about the
    vertical axis (any yaw).

    *Parameters*

    xyz_min, xyz_max: tuple[float, float, float]
        Bounds on the object's position, in metres.

    roll: float, optional
        The object's nominal roll, for example ``pi / 2`` for the beaker
        meshes, whose "up" direction is +y.
    """
    return PoseParam(
        xyz_min=xyz_min,
        xyz_max=xyz_max,
        rpy_min=(roll - UPRIGHT_TOLERANCE, -UPRIGHT_TOLERANCE, -math.pi),
        rpy_max=(roll + UPRIGHT_TOLERANCE, UPRIGHT_TOLERANCE, math.pi),
    )


# The shelf stands on the floor in front of the arm.
SHELF_POSE = placed_object_pose(xyz_min=(-1.0, 0.5, 0.3), xyz_max=(1.0, 1.5, 1.0))

TABLE_LENGTH = FloatParam(0.3, 1.5, "m")
TABLE_WIDTH = FloatParam(1.0, 3.0, "m")
TABLE_HEIGHT = FloatParam(0.05, 0.5, "m")

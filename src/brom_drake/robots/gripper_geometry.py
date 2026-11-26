"""
Plain data about the grippers that brom_drake ships, and the grasp point
computed from it.

The table in this module is data on purpose (names and numbers only), so that
other languages (for example a C++ port) can read the same values.
"""

from dataclasses import dataclass
from typing import Dict, Tuple

import numpy as np
from pydrake.all import Context, ModelInstanceIndex, MultibodyPlant

from .gripper_type import GripperType


@dataclass(frozen=True)
class GripperFingerPads:
    """
    *Description*

    Where a gripper's finger pads are.

    The *grasp point* of a gripper is the midpoint of its two finger pad
    frames: the point an object should be at to be held between the pads.

    *Parameters*

    base_link: str
        The name of the gripper's base link (the body a wrist or puppet
        attaches to).

    pad_bodies: Tuple[str, str]
        The names of the two finger pad bodies. Each pad's frame is at the
        centre of its contact surface's box.

    p_BaseGraspPoint_open: Tuple[float, float, float]
        The grasp point in the base link's frame when the gripper is fully
        open, in meters. A grasp selection algorithm can use it to place the
        pads (rather than the base link) on the object.

    closing_axis: str
        The axis of the base link's frame (``"x"``, ``"y"`` or ``"z"``) along
        which the pads move toward each other.
    """

    base_link: str
    pad_bodies: Tuple[str, str]
    p_BaseGraspPoint_open: Tuple[float, float, float]
    closing_axis: str


GRIPPER_FINGER_PADS: Dict[GripperType, GripperFingerPads] = {
    GripperType.Robotiq_2f_85: GripperFingerPads(
        base_link="robotiq_arg2f_base_link",
        pad_bodies=("left_inner_finger_pad", "right_inner_finger_pad"),
        # The pad frames are at (0, -/+0.0462, 0.1303) m in the base link when open
        p_BaseGraspPoint_open=(0.0, 0.0, 0.130),
        closing_axis="y",
    ),
}
"""
The finger pads of each gripper that has them, by :py:class:`GripperType`.
:py:attr:`GripperType.NoGripper` has no entry.
"""


def get_gripper_finger_pads(gripper_type: GripperType) -> GripperFingerPads:
    """
    *Description*

    Looks up a gripper's entry in :py:data:`GRIPPER_FINGER_PADS`.

    *Parameters*

    gripper_type: GripperType
        The gripper.

    *Returns*

    finger_pads: GripperFingerPads
        Where the gripper's finger pads are.

    *Raises*

    ValueError
        If the gripper has no finger pads in the table (for example
        ``GripperType.NoGripper``).
    """
    if gripper_type not in GRIPPER_FINGER_PADS:
        raise ValueError(
            f"Gripper type {gripper_type} has no finger pads in GRIPPER_FINGER_PADS; "
            f"the grippers with finger pads are {[g.name for g in GRIPPER_FINGER_PADS]}."
        )
    return GRIPPER_FINGER_PADS[gripper_type]


def calc_grasp_point_in_world(
    plant: MultibodyPlant,
    plant_context: Context,
    gripper_model_instance: ModelInstanceIndex,
    gripper_type: GripperType,
) -> np.ndarray:
    """
    *Description*

    The gripper's grasp point (the midpoint of its two finger pad frames)
    in the world frame, at the state in ``plant_context``.

    The pads follow the gripper's linkage as it closes, so this point is more
    accurate than ``p_BaseGraspPoint_open`` once the gripper has moved.

    *Parameters*

    plant: MultibodyPlant
        The plant that contains the gripper.

    plant_context: Context
        The plant's context (not the root diagram's context).

    gripper_model_instance: ModelInstanceIndex
        The gripper's model instance in ``plant``.

    gripper_type: GripperType
        Which gripper the model instance is.

    *Returns*

    p_WorldGraspPoint: np.ndarray
        The grasp point in the world frame, shape (3,), in meters.

    *Raises*

    ValueError
        If the gripper has no finger pads in :py:data:`GRIPPER_FINGER_PADS`.
    """
    finger_pads = get_gripper_finger_pads(gripper_type)
    pad_positions = [
        plant.EvalBodyPoseInWorld(
            plant_context, plant.GetBodyByName(pad_name, gripper_model_instance)
        ).translation()
        for pad_name in finger_pads.pad_bodies
    ]
    return 0.5 * (pad_positions[0] + pad_positions[1])

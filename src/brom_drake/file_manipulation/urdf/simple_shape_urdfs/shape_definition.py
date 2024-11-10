from dataclasses import dataclass
from enum import IntEnum
from typing import Union


class ShapeEnum(IntEnum):
    """
    The different simple shapes that can be created.
    """
    kBox = 1
    kSphere = 2
    kCylinder = 3
    kCapsule = 4

@dataclass
class SphereDefinition:
    radius: float
    type: ShapeEnum = ShapeEnum.kSphere


# Shape Definition Can be any one of these
ShapeDefinition = Union[SphereDefinition]
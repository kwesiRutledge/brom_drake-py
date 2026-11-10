from dataclasses import dataclass, field
import numpy as np
from typing import List, Union

# Internal Imports
from brom_drake.productions.types.base.configuration import (
    Configuration as BaseConfiguration,
)
from .script import Script as GraspSelection1Script


@dataclass
class Configuration:
    """
    *Description*

    A dataclass that specifies the configuration for the
    "GraspSelection1" production.

    *Attributes*

    base: BaseConfiguration, optional
        The base configuration for the production.

    show_collision_geometries: bool, optional
        A boolean flag that indicates whether or not to show collision geometries.
        Default is False.
        (TODO(Kwesi): Implement this feature)

    initial_gripper_command: List[float] or np.ndarray
        The gripper command that the gripper starts at and holds until it begins closing.
        This is a normalized command (0 is fully open), not a joint angle.
        Must have the same shape as the production's grasp_gripper_command.
        See the GraspSelection1 documentation for the definition of a gripper command.
        Default is None (the gripper starts fully open).

    target_body_on_gripper: str, optional
        The name of the body on the gripper that we use to define X_GripperObject.
        Default is None.
        (TODO(Kwesi): Implement this feature)

    gripper_color: List[float], optional
        A list of 4 floats representing the RGBA color of the gripper.
        Default is None.
        (TODO(Kwesi): Implement this feature)

    show_gripper_base_frame: bool, optional
        A boolean flag that indicates whether or not to show the gripper base frame.
        Default is False.
        (TODO(Kwesi): Consider deleting this feature)

    script: GraspSelection1Script, optional
        The script configuration for the production.
        Default is the standard GraspSelection1Script.

    """

    base: BaseConfiguration = field(
        default_factory=BaseConfiguration
    )  # By default, use the standard script
    show_collision_geometries: bool = False
    initial_gripper_command: Union[List[float], np.ndarray] = None
    target_body_on_gripper: str = (
        None  # The "Gripper" Frame that we use to define X_GripperObject
    )
    gripper_color: List[float] = None
    show_gripper_base_frame: bool = False
    script: GraspSelection1Script = field(
        default_factory=GraspSelection1Script
    )  # By default, use the standard script

    @property
    def meshcat_port_number(self):
        """
        *Description*

        A property that gets the meshcat port number from the base configuration.
        """
        return self.base.meshcat_port_number

    @meshcat_port_number.setter
    def meshcat_port_number(self, new_value: int):
        """
        *Description*

        A property setter that sets the meshcat port number in the base configuration.
        """
        self.base.meshcat_port_number = new_value

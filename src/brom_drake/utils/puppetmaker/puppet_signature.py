from dataclasses import dataclass
from pydrake.all import (
    ModelInstanceIndex,
    Joint,
    JointActuator
)
from typing import List

@dataclass
class PuppeteerJointSignature:
    joint: Joint
    joint_actuator: JointActuator

@dataclass
class AllJointSignatures:
    prismatic: List[PuppeteerJointSignature]
    revolute: List[PuppeteerJointSignature]

    @property
    def n_joints(self) -> int:
        return len(self.prismatic) + len(self.revolute)

@dataclass
class PuppetSignature:
    name: str
    model_instance_index: ModelInstanceIndex
    prismatic_ghost_bodies: List[ModelInstanceIndex]
    revolute_ghost_bodies: List[ModelInstanceIndex]
    joints: AllJointSignatures

    @property
    def all_joint_actuators(self) -> List[JointActuator]:
        return [sig.joint_actuator for sig in self.joints.prismatic + self.joints.revolute]

    @property
    def all_models(self) -> List[ModelInstanceIndex]:
        return self.prismatic_ghost_bodies + self.revolute_ghost_bodies + [self.model_instance_index]

    @property
    def n_joints(self) -> int:
        return self.joints.n_joints

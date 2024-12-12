from dataclasses import dataclass
import numpy as np
from pydrake.all import (
    RigidTransform,
)

@dataclass
class ProximityPosePlanDispenserConfig:
    proximity_limit: float = 1e-1
    translation_weight: float = 1.0
    orientation_weight: float = 0.0

    def distance_between_poses(
        self,
        pose1: RigidTransform,
        pose2: RigidTransform,
    ) -> float:
        """
        Description
        -----------
        This method computes the distance between two poses.
        """
        # Setup
        Q_translation = self.translation_weight
        Q_orientation = self.orientation_weight
        # 
        translation_distance = np.linalg.norm(pose1.translation() - pose2.translation())
        orientation_distance = np.linalg.norm(pose1.rotation().matrix() - pose2.rotation().matrix())
        return Q_translation * translation_distance + Q_orientation * orientation_distance

    def in_proximity(
        self,
        pose1: RigidTransform,
        pose2: RigidTransform,
    ) -> bool:
        """
        Description
        -----------
        This method checks if two poses are within the proximity of each other.
        """
        # Setup
        return self.distance_between_poses(pose1, pose2) < self.proximity_limit
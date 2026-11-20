import math
import unittest

import numpy as np
from pydrake.math import RigidTransform, RollPitchYaw

# Internal Imports
from brom_drake.productions.motion_planning.offline.kinematic.shelf import (
    ShelfPlanning1,
)
from brom_drake.productions.types.base.scenario import ScenarioError


class ShelfScenarioSmokeTest(unittest.TestCase):
    """
    Description
    -----------
    End to end: a non-default shelf pose, given in the scenario wire format,
    is validated, turned into constructor arguments, and ends up where the
    shelf is placed in the built production.
    """

    # 30 degrees about z, as a [w, x, y, z] quaternion, plus a new position.
    yaw = math.pi / 6.0
    shelf_pose_wire = {
        "xyz": [0.4, 1.2, 0.6],
        "wxyz": [math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)],
    }

    def build_shelf_world_pose(self, production: ShelfPlanning1) -> RigidTransform:
        """Build the production's plant and return the shelf body's world pose."""
        production.add_supporting_cast()
        diagram = production.builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        plant_context = diagram.GetSubsystemContext(production.plant, diagram_context)
        shelf_body = production.plant.GetBodyByName(
            "cupboard_body", production.shelf_model_index
        )
        return production.plant.EvalBodyPoseInWorld(plant_context, shelf_body)

    def test_shelf_is_placed_at_validated_pose1(self):
        # Validate and convert, as a caller outside Brom would
        overrides = ShelfPlanning1.Scenario.validate(
            {"shelf_pose": self.shelf_pose_wire}
        )
        np.testing.assert_allclose(overrides["shelf_pose"]["rpy"], [0, 0, self.yaw])

        kwargs = ShelfPlanning1.Scenario.constructor_kwargs(
            {"shelf_pose": self.shelf_pose_wire}
        )
        self.assertIsInstance(kwargs["shelf_pose"], RigidTransform)

        # Build the production with it
        production = ShelfPlanning1(meshcat_port_number=None, **kwargs)
        X_WorldShelf = self.build_shelf_world_pose(production)

        expected = RigidTransform(
            RollPitchYaw(0.0, 0.0, self.yaw), np.array([0.4, 1.2, 0.6])
        )
        self.assertTrue(
            X_WorldShelf.IsNearlyEqualTo(expected, 1e-12),
            f"shelf is at {X_WorldShelf}, expected {expected}",
        )

    def test_default_shelf_pose_differs1(self):
        """
        Description
        -----------
        Guards the test above: the default shelf is somewhere else, so the
        override really moved it.
        """
        production = ShelfPlanning1(meshcat_port_number=None)
        X_WorldShelf = self.build_shelf_world_pose(production)
        np.testing.assert_allclose(X_WorldShelf.translation(), [0.0, 1.0, 0.65])

    def test_invalid_pose_never_reaches_the_production1(self):
        with self.assertRaises(ScenarioError):
            ShelfPlanning1.Scenario.constructor_kwargs(
                {"shelf_pose": {"xyz": [0.4, 1.2, 0.6], "rpy": [0.0, 0.0, 4.0]}}
            )


if __name__ == "__main__":
    unittest.main()

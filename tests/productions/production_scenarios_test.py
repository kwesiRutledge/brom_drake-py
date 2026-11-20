import math
import unittest

from pydrake.math import RigidTransform, RollPitchYaw

# Internal Imports
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2
from brom_drake.productions.motion_planning.offline.dynamic.cornell_tech1 import (
    CornellTech1,
)
from brom_drake.productions.motion_planning.offline.kinematic.chem_lab1 import ChemLab1
from brom_drake.productions.motion_planning.offline.kinematic.shelf import (
    ShelfPlanning1,
)
from brom_drake.productions.pick_and_place.chem_lab3 import ChemLab3
from brom_drake.productions.types.base.scenario import ScenarioError

UR10E_PRODUCTIONS = [ChemLab1, ShelfPlanning1, ChemLab2, ChemLab3]
ALL_PRODUCTIONS = UR10E_PRODUCTIONS + [CornellTech1]

EXPECTED_FIELDS = {
    ChemLab1: [
        "start_configuration",
        "start_pose",
        "goal_configuration",
        "goal_pose",
        "table_length",
        "table_width",
        "table_height",
        "shelf_pose",
    ],
    ShelfPlanning1: [
        "start_configuration",
        "start_pose",
        "goal_configuration",
        "goal_pose",
        "shelf_pose",
    ],
    ChemLab2: [
        "start_configuration",
        "start_pose",
        "goal_configuration",
        "goal_pose",
        "pose_WorldBeaker",
        "table_length",
        "table_width",
        "table_height",
        "shelf_pose",
    ],
    ChemLab3: [
        "start_configuration",
        "start_pose",
        "table_length",
        "table_width",
        "table_height",
        "shelf_pose",
    ],
    CornellTech1: [
        "start_configuration",
        "start_pose",
        "goal_configuration",
        "goal_pose",
        "pose_WorldBeaker",
    ],
}


def pose_to_wire(X: RigidTransform) -> dict:
    """A RigidTransform in the scenario wire format."""
    return {
        "xyz": X.translation().tolist(),
        "rpy": RollPitchYaw(X.rotation()).vector().tolist(),
    }


class ProductionScenarioTest(unittest.TestCase):
    def test_each_production_declares_its_own_scenario1(self):
        """
        Description
        -----------
        Each production declares its own Scenario (not just its type's), which
        extends its production type's Scenario.
        """
        for production in ALL_PRODUCTIONS:
            with self.subTest(production=production.__name__):
                self.assertIn("Scenario", production.__dict__)
                base_type = production.__mro__[1]
                self.assertTrue(
                    issubclass(production.Scenario, base_type.Scenario),
                    f"{production.__name__}.Scenario should extend "
                    f"{base_type.__name__}.Scenario",
                )
                names = [f["name"] for f in production.Scenario.schema()["fields"]]
                self.assertEqual(names, EXPECTED_FIELDS[production])

    def test_ur10e_joint_vectors1(self):
        """
        Description
        -----------
        UR10e productions take exactly 6 joints, within the arm's limits
        (+/-2*pi, elbow +/-pi).
        """
        for production in UR10E_PRODUCTIONS:
            with self.subTest(production=production.__name__):
                scenario = production.Scenario
                q = [2.0 * math.pi, 0.0, -math.pi, 0.0, 0.0, -2.0 * math.pi]
                self.assertEqual(
                    scenario.validate({"start_configuration": q}),
                    {"start_configuration": q},
                )
                with self.assertRaises(ScenarioError) as ctx:
                    scenario.validate({"start_configuration": [0.0] * 7})
                self.assertIn("expected 6 numbers", ctx.exception.problems[0])
                with self.assertRaises(ScenarioError) as ctx:
                    scenario.validate(
                        {"start_configuration": [0.0, 0.0, 3.5, 0.0, 0.0, 0.0]}
                    )
                self.assertIn("start_configuration[2]", ctx.exception.problems[0])

    def test_cornell_tech1_joint_vectors1(self):
        """
        Description
        -----------
        CornellTech1's joint count follows arm_variant (6 or 7), so both
        lengths validate, within the Gen3's +/-pi.
        """
        for n_joints in (6, 7):
            with self.subTest(n_joints=n_joints):
                q = [0.5] * n_joints
                self.assertEqual(
                    CornellTech1.Scenario.validate({"goal_configuration": q}),
                    {"goal_configuration": q},
                )
        with self.assertRaises(ScenarioError):
            CornellTech1.Scenario.validate({"goal_configuration": [3.5] + [0.0] * 6})

    def test_target_pose_reach1(self):
        reachable = {"xyz": [0.8, 0.0, 0.6], "rpy": [0.0, 0.0, 0.0]}
        too_far = {"xyz": [1.2, 0.0, 0.6], "rpy": [0.0, 0.0, 0.0]}
        self.assertIn("goal_pose", ChemLab1.Scenario.validate({"goal_pose": too_far}))
        self.assertIn(
            "goal_pose", CornellTech1.Scenario.validate({"goal_pose": reachable})
        )
        with self.assertRaises(ScenarioError):
            CornellTech1.Scenario.validate({"goal_pose": too_far})

    def test_default_scene_poses_are_within_bounds1(self):
        """
        Description
        -----------
        The poses each production uses when a field is left as None lie
        inside that field's bounds, so the default scene is a valid scenario.
        """
        cases = [
            (ChemLab1, "shelf_pose", "pose_WorldShelf"),
            (ShelfPlanning1, "shelf_pose", "shelf_pose"),
            (ChemLab2, "shelf_pose", "pose_WorldShelf"),
            (ChemLab2, "pose_WorldBeaker", "pose_WorldBeaker"),
            (ChemLab3, "shelf_pose", "pose_WorldShelf"),
            (CornellTech1, "pose_WorldBeaker", "pose_WorldBeaker"),
        ]
        for production_class, field, attribute in cases:
            with self.subTest(production=production_class.__name__, field=field):
                production = production_class(meshcat_port_number=None)
                X = getattr(production, attribute)
                production_class.Scenario.validate({field: pose_to_wire(X)})

    def test_placed_objects_stay_upright1(self):
        """
        Description
        -----------
        Scene objects may turn about the vertical axis but not tip over.
        """
        turned = {"xyz": [0.0, 0.75, 0.66], "rpy": [0.0, 0.0, 2.5]}
        tipped = {"xyz": [0.0, 0.75, 0.66], "rpy": [0.3, 0.0, 0.0]}
        self.assertIn("shelf_pose", ChemLab1.Scenario.validate({"shelf_pose": turned}))
        with self.assertRaises(ScenarioError):
            ChemLab1.Scenario.validate({"shelf_pose": tipped})

        # Beakers lie with roll pi/2 (their mesh's "up" is +y).
        beaker = {"xyz": [-0.6, 0.45, 0.175], "rpy": [math.pi / 2, 0.0, 1.0]}
        self.assertIn(
            "pose_WorldBeaker", ChemLab2.Scenario.validate({"pose_WorldBeaker": beaker})
        )
        with self.assertRaises(ScenarioError):
            ChemLab2.Scenario.validate(
                {"pose_WorldBeaker": {"xyz": beaker["xyz"], "rpy": [0.0, 0.0, 1.0]}}
            )

    def test_table_dimensions1(self):
        overrides = ChemLab3.Scenario.validate(
            {"table_length": 1.2, "table_width": 2.5, "table_height": 0.2}
        )
        self.assertEqual(
            overrides, {"table_length": 1.2, "table_width": 2.5, "table_height": 0.2}
        )
        with self.assertRaises(ScenarioError):
            ChemLab3.Scenario.validate({"table_height": 0.0})


if __name__ == "__main__":
    unittest.main()

import inspect
import math
import unittest

# Internal Imports
from brom_drake.productions.motion_planning.offline import ChemLab1, ShelfPlanning1
from brom_drake.productions.motion_planning.offline.dynamic.chem_lab2 import ChemLab2
from brom_drake.productions.pick_and_place.chem_lab3 import ChemLab3
from brom_drake.productions.types import (
    KinematicMotionPlanningProduction,
    MotionPlanningAndGraspingProduction,
    OfflineDynamicMotionPlanningProduction,
)
from brom_drake.productions.types.base.scenario import Scenario, ScenarioError

START_GOAL_FIELDS = [
    "start_configuration",
    "start_pose",
    "goal_configuration",
    "goal_pose",
]
EXPECTED_FIELDS = {
    KinematicMotionPlanningProduction: START_GOAL_FIELDS,
    OfflineDynamicMotionPlanningProduction: START_GOAL_FIELDS,
    MotionPlanningAndGraspingProduction: ["start_configuration", "start_pose"],
}
POSE = {"xyz": [0.5, 0.0, 0.5], "rpy": [0.0, math.pi / 2, 0.0]}


class ProductionTypeScenarioTest(unittest.TestCase):
    """
    Description
    -----------
    The three base production types declare a Scenario with their start (and,
    where the constructor has them, goal) arguments.
    """

    def test_fields_and_exclusive_pairs1(self):
        for production_type, expected in EXPECTED_FIELDS.items():
            with self.subTest(production_type=production_type.__name__):
                scenario = production_type.Scenario
                self.assertTrue(issubclass(scenario, Scenario))
                schema = scenario.schema()
                self.assertEqual([f["name"] for f in schema["fields"]], expected)
                self.assertEqual(
                    schema["mutually_exclusive"],
                    [expected[i : i + 2] for i in range(0, len(expected), 2)],
                )
                # Defaults are None: "use the constructor's own default".
                self.assertTrue(all(f["default"] is None for f in schema["fields"]))

    def test_fields_are_constructor_arguments1(self):
        """
        Description
        -----------
        Each field name is a keyword argument of the production type's own
        constructor, with the same default (None).
        """
        for production_type in EXPECTED_FIELDS:
            parameters = inspect.signature(production_type.__init__).parameters
            for field in production_type.Scenario.schema()["fields"]:
                with self.subTest(
                    production_type=production_type.__name__, field=field["name"]
                ):
                    self.assertIn(field["name"], parameters)
                    self.assertIsNone(parameters[field["name"]].default)

    def test_joint_vectors_of_any_arm_are_accepted1(self):
        """
        Description
        -----------
        The base types cannot know the arm, so 6-joint (UR10e) and 7-joint
        (Kinova Gen3) configurations both validate.
        """
        for production_type in EXPECTED_FIELDS:
            for n_joints in (6, 7):
                with self.subTest(
                    production_type=production_type.__name__, n_joints=n_joints
                ):
                    overrides = production_type.Scenario.validate(
                        {"start_configuration": [0.1] * n_joints}
                    )
                    self.assertEqual(
                        overrides, {"start_configuration": [0.1] * n_joints}
                    )

    def test_joint_limits1(self):
        with self.assertRaises(ScenarioError) as ctx:
            KinematicMotionPlanningProduction.Scenario.validate(
                {"goal_configuration": [0.0, 0.0, 7.0, 0.0, 0.0, 0.0]}
            )
        self.assertIn("goal_configuration[2]", ctx.exception.problems[0])

    def test_poses_are_accepted1(self):
        overrides = OfflineDynamicMotionPlanningProduction.Scenario.validate(
            {"start_pose": POSE, "goal_pose": POSE}
        )
        self.assertEqual(overrides["goal_pose"]["xyz"], [0.5, 0.0, 0.5])

    def test_configuration_and_pose_together_are_refused1(self):
        with self.assertRaises(ScenarioError) as ctx:
            KinematicMotionPlanningProduction.Scenario.validate(
                {
                    "start_configuration": [0.0] * 6,
                    "start_pose": POSE,
                    "goal_configuration": [0.0] * 6,
                    "goal_pose": POSE,
                }
            )
        self.assertEqual(len(ctx.exception.problems), 2)

    def test_no_goal_fields_for_grasping_type1(self):
        with self.assertRaises(ScenarioError) as ctx:
            MotionPlanningAndGraspingProduction.Scenario.validate({"goal_pose": POSE})
        self.assertIn("unknown parameter 'goal_pose'", ctx.exception.problems[0])

    def test_productions_inherit_their_type_scenario1(self):
        """
        Description
        -----------
        Until a production declares its own Scenario, it inherits its type's.
        """
        self.assertIs(ChemLab1.Scenario, KinematicMotionPlanningProduction.Scenario)
        self.assertIs(
            ShelfPlanning1.Scenario, KinematicMotionPlanningProduction.Scenario
        )
        self.assertIs(
            ChemLab2.Scenario, OfflineDynamicMotionPlanningProduction.Scenario
        )
        self.assertIs(ChemLab3.Scenario, MotionPlanningAndGraspingProduction.Scenario)


if __name__ == "__main__":
    unittest.main()

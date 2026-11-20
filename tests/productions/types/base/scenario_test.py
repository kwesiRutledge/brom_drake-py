import json
import math
import unittest
from dataclasses import dataclass

import numpy as np
from pydrake.common.eigen_geometry import Quaternion
from pydrake.math import RollPitchYaw

# Internal Imports
from brom_drake.productions.types.base import (
    FloatParam,
    PoseParam,
    Scenario,
    ScenarioError,
    VectorParam,
    param,
)
from brom_drake.productions.types.base.scenario import POSE_CONVENTION_DOCS_URL

JOINT_LIMIT = 2.0 * math.pi
WORKSPACE = PoseParam(xyz_min=(-1.0, -1.0, 0.0), xyz_max=(1.0, 1.0, 2.0))


@dataclass
class ExampleScenario(Scenario):
    """A scenario shaped like a motion planning production's."""

    mutually_exclusive = (("start_configuration", "start_pose"),)

    start_configuration: list = param(
        VectorParam(6, (-JOINT_LIMIT,) * 6, (JOINT_LIMIT,) * 6, "rad")
    )
    start_pose: dict = param(WORKSPACE)
    table_height: float = param(FloatParam(0.05, 0.5, "m"), default=0.1)
    shelf_pose: dict = param(
        WORKSPACE,
        default={"xyz": [0.0, 0.75, 0.66], "rpy": [0.0, 0.0, -math.pi / 2]},
    )


@dataclass
class NarrowYawScenario(Scenario):
    """A pose whose yaw may only be in [0, 0.1]."""

    pose: dict = param(
        PoseParam(
            xyz_min=(-1.0, -1.0, 0.0),
            xyz_max=(1.0, 1.0, 2.0),
            rpy_min=(-0.1, -0.1, 0.0),
            rpy_max=(0.1, 0.1, 0.1),
        )
    )


def valid_pose(**orientation):
    pose = {"xyz": [0.0, 0.5, 0.5]}
    pose.update(orientation or {"rpy": [0.0, 0.0, 0.0]})
    return pose


class ScenarioValidateTest(unittest.TestCase):
    def assertProblems(self, values, *fragments, scenario=ExampleScenario):
        """
        Description
        -----------
        Assert that validate() raises a ScenarioError and that, for each
        fragment, some reported problem contains it.
        """
        with self.assertRaises(ScenarioError) as ctx:
            scenario.validate(values)
        problems = ctx.exception.problems
        for fragment in fragments:
            self.assertTrue(
                any(fragment in p for p in problems),
                f"no problem mentions {fragment!r}; problems: {problems}",
            )
        return problems

    # Accepted values and normalization

    def test_empty_values_return_no_overrides1(self):
        self.assertEqual(ExampleScenario.validate({}), {})

    def test_returns_only_given_keys_normalized1(self):
        """
        Description
        -----------
        Only the keys passed in come back, as plain JSON: ints become floats,
        numpy arrays become lists, poses carry both "xyz" and "rpy".
        """
        overrides = ExampleScenario.validate(
            {"table_height": 0.25, "start_configuration": np.zeros(6, dtype=int)}
        )
        self.assertEqual(
            overrides,
            {"table_height": 0.25, "start_configuration": [0.0] * 6},
        )
        self.assertIs(type(overrides["start_configuration"]), list)
        self.assertIs(type(overrides["start_configuration"][0]), float)
        json.dumps(overrides)  # plain JSON

    def test_pose_ints_are_returned_as_floats1(self):
        overrides = ExampleScenario.validate({"shelf_pose": valid_pose(rpy=[0, 0, 0])})
        self.assertEqual(
            overrides["shelf_pose"], {"xyz": [0.0, 0.5, 0.5], "rpy": [0.0, 0.0, 0.0]}
        )
        self.assertIs(type(overrides["shelf_pose"]["xyz"][0]), float)

    def test_bounds_are_inclusive1(self):
        overrides = ExampleScenario.validate({"table_height": 0.5})
        self.assertEqual(overrides, {"table_height": 0.5})

    def test_none_allowed_when_default_is_none1(self):
        """
        Description
        -----------
        None means "use the constructor's default" and is only accepted for
        fields whose own default is None.
        """
        self.assertEqual(
            ExampleScenario.validate({"start_pose": None}), {"start_pose": None}
        )
        self.assertProblems({"table_height": None}, "table_height: None")

    def test_values_must_be_a_dict1(self):
        self.assertProblems([("table_height", 0.2)], "expected a dict")

    # Refused values

    def test_unknown_name1(self):
        problems = self.assertProblems(
            {"table_hieght": 0.2}, "unknown parameter 'table_hieght'"
        )
        self.assertIn("table_height", problems[0])  # lists the known names

    def test_wrong_type1(self):
        self.assertProblems(
            {"table_height": "0.2"}, "table_height: expected a number, got str"
        )
        self.assertProblems(
            {"start_configuration": "zeros"},
            "start_configuration: expected a list of 6 numbers",
        )
        self.assertProblems({"shelf_pose": [0, 0, 0]}, "shelf_pose: expected a pose")

    def test_bool_is_not_a_number1(self):
        self.assertProblems({"table_height": True}, "expected a number, got bool")
        self.assertProblems(
            {"start_configuration": [0.0] * 5 + [False]},
            "start_configuration[5]: expected a number, got bool",
        )
        self.assertProblems(
            {"table_height": np.bool_(True)}, "expected a number, got bool"
        )

    def test_nan_and_infinity1(self):
        for bad in (math.nan, math.inf, -math.inf):
            with self.subTest(value=bad):
                self.assertProblems(
                    {"table_height": bad}, "table_height: expected a finite number"
                )
                self.assertProblems(
                    {"shelf_pose": {"xyz": [0.0, bad, 0.5], "rpy": [0, 0, 0]}},
                    "shelf_pose.xyz[1]: expected a finite number",
                )

    def test_wrong_vector_length1(self):
        self.assertProblems(
            {"start_configuration": [0.0] * 7},
            "start_configuration: expected 6 numbers, got 7",
        )
        self.assertProblems(
            {"shelf_pose": {"xyz": [0.0, 0.5], "rpy": [0, 0, 0]}},
            "shelf_pose.xyz: expected 3 numbers, got 2",
        )

    def test_out_of_range1(self):
        self.assertProblems(
            {"table_height": 0.6},
            "table_height: 0.6 is outside the allowed range [0.05, 0.5]",
        )
        self.assertProblems(
            {"start_configuration": [0.0, 0.0, 7.0, 0.0, 0.0, 0.0]},
            "start_configuration[2]: 7.0 is outside the allowed range",
        )
        self.assertProblems(
            {"shelf_pose": {"xyz": [0.0, 0.5, 3.0], "rpy": [0, 0, 0]}},
            "shelf_pose.xyz[2]: 3.0 is outside the allowed range [0.0, 2.0]",
        )

    def test_start_configuration_and_start_pose_both_set1(self):
        self.assertProblems(
            {"start_configuration": [0.0] * 6, "start_pose": valid_pose()},
            "at most one of ['start_configuration', 'start_pose']",
        )

    def test_one_of_exclusive_pair_with_other_none_is_fine1(self):
        overrides = ExampleScenario.validate(
            {"start_configuration": [0.0] * 6, "start_pose": None}
        )
        self.assertEqual(
            overrides, {"start_configuration": [0.0] * 6, "start_pose": None}
        )

    def test_several_problems_reported_together1(self):
        problems = self.assertProblems(
            {
                "bogus": 1,
                "table_height": True,
                "start_configuration": [0.0] * 5,
                "start_pose": {"xyz": [0.0, 0.0, math.nan]},
            },
            "unknown parameter 'bogus'",
            "table_height: expected a number, got bool",
            "start_configuration: expected 6 numbers, got 5",
            "start_pose.xyz[2]: expected a finite number",
            "start_pose: missing orientation",
            "at most one of",
        )
        self.assertEqual(len(problems), 6)

    def test_error_message_lists_every_problem1(self):
        with self.assertRaises(ScenarioError) as ctx:
            ExampleScenario.validate({"table_height": 9.0, "bogus": 1})
        self.assertIsInstance(ctx.exception, ValueError)
        for problem in ctx.exception.problems:
            self.assertIn(problem, str(ctx.exception))


class PoseOrientationTest(unittest.TestCase):
    def test_quaternion_is_returned_as_rpy1(self):
        """
        Description
        -----------
        A "wxyz" quaternion (scalar first, as in Drake) comes back as "rpy",
        describing the same rotation.
        """
        wxyz = [math.cos(0.25), 0.0, 0.0, math.sin(0.25)]  # 0.5 rad about z
        overrides = ExampleScenario.validate({"shelf_pose": valid_pose(wxyz=wxyz)})
        pose = overrides["shelf_pose"]
        self.assertEqual(set(pose), {"xyz", "rpy"})
        np.testing.assert_allclose(pose["rpy"], [0.0, 0.0, 0.5], atol=1e-12)

    def test_rpy_and_quaternion_give_same_rotation1(self):
        wxyz = np.array([0.3, -0.5, 0.7, 0.2])
        wxyz /= np.linalg.norm(wxyz)
        overrides = ExampleScenario.validate(
            {"shelf_pose": valid_pose(wxyz=wxyz.tolist())}
        )
        R_from_rpy = RollPitchYaw(overrides["shelf_pose"]["rpy"]).ToRotationMatrix()
        R_from_q = Quaternion(wxyz).rotation()
        np.testing.assert_allclose(R_from_rpy.matrix(), R_from_q, atol=1e-12)

    def test_hand_typed_quaternion_is_renormalized1(self):
        overrides = ExampleScenario.validate(
            {"shelf_pose": valid_pose(wxyz=[0.7071, 0.0, 0.0, 0.7071])}
        )
        self.assertAlmostEqual(overrides["shelf_pose"]["rpy"][2], math.pi / 2)

    def test_missing_orientation1(self):
        with self.assertRaises(ScenarioError) as ctx:
            ExampleScenario.validate({"shelf_pose": {"xyz": [0.0, 0.5, 0.5]}})
        (problem,) = ctx.exception.problems
        self.assertIn("missing orientation", problem)
        self.assertIn(POSE_CONVENTION_DOCS_URL, problem)

    def test_both_orientations_given1(self):
        with self.assertRaises(ScenarioError) as ctx:
            ExampleScenario.validate(
                {
                    "shelf_pose": valid_pose(
                        rpy=[0.0, 0.0, 0.0], wxyz=[1.0, 0.0, 0.0, 0.0]
                    )
                }
            )
        (problem,) = ctx.exception.problems
        self.assertIn('both "rpy" and "wxyz"', problem)
        self.assertIn(POSE_CONVENTION_DOCS_URL, problem)

    def test_unknown_pose_key1(self):
        with self.assertRaises(ScenarioError) as ctx:
            ExampleScenario.validate(
                {"shelf_pose": {"xyz": [0, 0.5, 0.5], "quaternion": [1, 0, 0, 0]}}
            )
        problems = ctx.exception.problems
        self.assertTrue(any("unknown pose key 'quaternion'" in p for p in problems))
        self.assertTrue(all(POSE_CONVENTION_DOCS_URL in p for p in problems))

    def test_non_unit_quaternion1(self):
        """
        Description
        -----------
        A quaternion far from unit length is refused with an explanation of the
        [w, x, y, z] order, rather than quietly normalized.
        """
        for wxyz in ([1.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 0.0], [2.0, 0.0, 0.0, 0.0]):
            with self.subTest(wxyz=wxyz):
                with self.assertRaises(ScenarioError) as ctx:
                    ExampleScenario.validate({"shelf_pose": valid_pose(wxyz=wxyz)})
                (problem,) = ctx.exception.problems
                self.assertIn("is not a unit quaternion", problem)
                self.assertIn("[w, x, y, z]", problem)
                self.assertIn(POSE_CONVENTION_DOCS_URL, problem)

    def test_quaternion_wrong_length1(self):
        with self.assertRaises(ScenarioError) as ctx:
            ExampleScenario.validate({"shelf_pose": valid_pose(wxyz=[1.0, 0.0, 0.0])})
        self.assertIn("shelf_pose.wxyz: expected 4 numbers, got 3", str(ctx.exception))

    def test_quaternion_outside_rpy_bounds1(self):
        """
        Description
        -----------
        A quaternion is checked against the rpy bounds after conversion, and
        the error says which rotation it was and why.
        """
        wxyz = [math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4)]  # yaw 90 deg
        with self.assertRaises(ScenarioError) as ctx:
            NarrowYawScenario.validate({"pose": valid_pose(wxyz=wxyz)})
        (problem,) = ctx.exception.problems
        self.assertIn("is the rotation rpy = (0, ", problem)
        self.assertIn("yaw = 1.5708 rad (allowed [0, 0.1])", problem)
        self.assertIn("RollPitchYaw", problem)
        self.assertIn(POSE_CONVENTION_DOCS_URL, problem)

    def test_quaternion_inside_rpy_bounds1(self):
        wxyz = [math.cos(0.025), 0.0, 0.0, math.sin(0.025)]  # yaw 0.05 rad
        overrides = NarrowYawScenario.validate({"pose": valid_pose(wxyz=wxyz)})
        self.assertAlmostEqual(overrides["pose"]["rpy"][2], 0.05)

    def test_rpy_angle_outside_convention1(self):
        """
        Description
        -----------
        An rpy angle outside [-pi, pi] is refused with the equivalent angle
        inside [-pi, pi] and a link to the convention.
        """
        with self.assertRaises(ScenarioError) as ctx:
            ExampleScenario.validate({"shelf_pose": valid_pose(rpy=[0.0, 0.0, 4.71])})
        (problem,) = ctx.exception.problems
        self.assertIn("yaw = 4.71 rad is outside [-pi, pi]", problem)
        self.assertIn("the same angle in [-pi, pi] is -1.57319", problem)
        self.assertIn(POSE_CONVENTION_DOCS_URL, problem)

    def test_rpy_angle_outside_declared_bounds1(self):
        with self.assertRaises(ScenarioError) as ctx:
            NarrowYawScenario.validate({"pose": valid_pose(rpy=[0.0, 0.0, 0.5])})
        (problem,) = ctx.exception.problems
        self.assertIn(
            "pose.rpy[2]: yaw = 0.5 rad is outside the allowed range", problem
        )


class PoseParamDeclarationTest(unittest.TestCase):
    def test_rpy_bounds_outside_convention_are_refused1(self):
        with self.assertRaises(ValueError) as ctx:
            PoseParam(
                (-1, -1, 0),
                (1, 1, 2),
                rpy_min=(0.0, 0.0, 3 * math.pi / 2),
                rpy_max=(0.0, 0.0, 5 * math.pi / 2),
            )
        message = str(ctx.exception)
        self.assertIn("must lie inside [-pi, pi]", message)
        self.assertIn("rpy_min[2] (yaw)", message)
        # The same range, wrapped into [-pi, pi], is suggested.
        self.assertIn("rpy_min=(0, 0, -1.5708), rpy_max=(0, 0, 1.5708)", message)
        self.assertIn(POSE_CONVENTION_DOCS_URL, message)

    def test_range_crossing_pi_is_explained1(self):
        with self.assertRaises(ValueError) as ctx:
            PoseParam(
                (-1, -1, 0),
                (1, 1, 2),
                rpy_min=(0.0, 0.0, math.pi / 2),
                rpy_max=(0.0, 0.0, 3 * math.pi / 2),
            )
        message = str(ctx.exception)
        self.assertIn("cannot be written as one interval", message)
        self.assertIn("crosses +/-pi", message)

    def test_rounded_pi_bounds_are_accepted1(self):
        PoseParam((-1, -1, 0), (1, 1, 2), rpy_min=(-3.14159265,) * 3)
        PoseParam((-1, -1, 0), (1, 1, 2), rpy_max=(np.pi,) * 3)

    def test_min_greater_than_max_is_refused1(self):
        with self.assertRaises(ValueError):
            PoseParam((1, 0, 0), (0, 1, 1))
        with self.assertRaises(ValueError):
            FloatParam(1.0, 0.0)
        with self.assertRaises(ValueError):
            VectorParam(2, (0.0, 1.0), (1.0, 0.0))

    def test_vector_bounds_must_match_length1(self):
        with self.assertRaises(ValueError):
            VectorParam(3, (0.0, 0.0), (1.0, 1.0))


class VectorParamAnyLengthTest(unittest.TestCase):
    """
    Description
    -----------
    VectorParam(length=None) accepts any non-empty length, with one shared
    min and max for every entry. Base production types use it for joint
    vectors, whose length depends on the arm.
    """

    spec = VectorParam(None, -1.0, 1.0, "rad")

    def check(self, value):
        problems = []
        return self.spec.check(value, "q", problems), problems

    def test_any_length_is_accepted1(self):
        for n in (1, 6, 7):
            with self.subTest(length=n):
                self.assertEqual(self.check([0.5] * n), ([0.5] * n, []))

    def test_shared_bounds_apply_to_every_entry1(self):
        value, problems = self.check([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5])
        self.assertIsNone(value)
        self.assertEqual(
            problems, ["q[6]: 1.5 is outside the allowed range [-1.0, 1.0]"]
        )

    def test_empty_and_non_list_are_refused1(self):
        self.assertEqual(self.check([])[1], ["q: expected a non-empty list of numbers"])
        self.assertEqual(
            self.check(0.5)[1], ["q: expected a list of numbers, got float"]
        )

    def test_bounds_must_be_single_numbers1(self):
        with self.assertRaises(TypeError):
            VectorParam(None, (-1.0, -1.0), (1.0, 1.0))
        with self.assertRaises(ValueError):
            VectorParam(None, 1.0, -1.0)

    def test_schema_bounds1(self):
        self.assertEqual(self.spec.bounds(), {"length": None, "min": -1.0, "max": 1.0})


class ScenarioDeclarationTest(unittest.TestCase):
    def test_subclass_without_dataclass_is_refused1(self):
        class Forgotten(ExampleScenario):
            extra: float = param(FloatParam(0.0, 1.0))

        with self.assertRaises(TypeError) as ctx:
            Forgotten.validate({})
        self.assertIn("must be decorated with @dataclass", str(ctx.exception))

    def test_field_without_param_is_refused1(self):
        @dataclass
        class Plain(Scenario):
            x: float = 0.1

        with self.assertRaises(TypeError) as ctx:
            Plain.schema()
        self.assertIn("field 'x' must be declared with param", str(ctx.exception))

    def test_default_outside_range_is_refused1(self):
        @dataclass
        class BadDefault(Scenario):
            x: float = param(FloatParam(0.0, 1.0), default=5.0)

        with self.assertRaises(TypeError) as ctx:
            BadDefault.validate({})
        self.assertIn("default of 'x'", str(ctx.exception))

    def test_mutually_exclusive_unknown_field_is_refused1(self):
        @dataclass
        class BadGroup(Scenario):
            mutually_exclusive = (("x", "missing"),)
            x: float = param(FloatParam(0.0, 1.0))

        with self.assertRaises(TypeError) as ctx:
            BadGroup.schema()
        self.assertIn("unknown field 'missing'", str(ctx.exception))

    def test_param_rejects_non_spec1(self):
        with self.assertRaises(TypeError):
            param((0.0, 1.0))

    def test_mutable_defaults_are_not_shared1(self):
        a, b = ExampleScenario(), ExampleScenario()
        a.shelf_pose["xyz"][0] = 99.0
        self.assertEqual(b.shelf_pose["xyz"][0], 0.0)
        self.assertEqual(
            ExampleScenario.schema()["fields"][3]["default"]["xyz"][0], 0.0
        )

    def test_subclass_inherits_and_extends_fields1(self):
        @dataclass
        class Child(ExampleScenario):
            beaker_height: float = param(FloatParam(0.0, 0.3, "m"), default=0.1)

        names = [f["name"] for f in Child.schema()["fields"]]
        self.assertEqual(
            names,
            [
                "start_configuration",
                "start_pose",
                "table_height",
                "shelf_pose",
                "beaker_height",
            ],
        )
        # The parent's rules still apply.
        with self.assertRaises(ScenarioError):
            Child.validate(
                {"start_configuration": [0.0] * 6, "start_pose": valid_pose()}
            )


class ScenarioSchemaTest(unittest.TestCase):
    def test_schema_is_json_and_describes_each_field1(self):
        schema = ExampleScenario.schema()
        json.dumps(schema)

        self.assertEqual(
            schema["mutually_exclusive"], [["start_configuration", "start_pose"]]
        )
        fields = {f["name"]: f for f in schema["fields"]}
        self.assertEqual(
            fields["table_height"],
            {
                "name": "table_height",
                "kind": "float",
                "default": 0.1,
                "units": "m",
                "description": "",
                "min": 0.05,
                "max": 0.5,
            },
        )
        self.assertEqual(fields["start_configuration"]["kind"], "vector")
        self.assertEqual(fields["start_configuration"]["length"], 6)
        self.assertIsNone(fields["start_configuration"]["default"])

        pose = fields["shelf_pose"]
        self.assertEqual(pose["kind"], "pose")
        self.assertEqual(pose["units"], "m, rad")
        self.assertEqual(pose["orientation_keys"], ["rpy", "wxyz"])
        self.assertEqual(pose["xyz_min"], [-1.0, -1.0, 0.0])
        self.assertEqual(pose["rpy_max"], [math.pi] * 3)

    def test_description_is_included1(self):
        @dataclass
        class Described(Scenario):
            x: float = param(FloatParam(0.0, 1.0), description="How far along")

        self.assertEqual(
            Described.schema()["fields"][0]["description"], "How far along"
        )


if __name__ == "__main__":
    unittest.main()

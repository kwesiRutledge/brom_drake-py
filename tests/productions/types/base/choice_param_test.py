import json
import unittest
from dataclasses import dataclass

# Internal Imports
from brom_drake.productions.types.base import (
    ChoiceParam,
    Scenario,
    ScenarioError,
    param,
)
from brom_drake.productions.types.base.scenario import CHOICE_ID_PATTERN, is_choice_id

FRUITS = {
    "apple": "package://fruit/apple.sdf",
    "banana_2": "package://fruit/banana.sdf",
}


@dataclass
class FruitScenario(Scenario):
    fruit: str = param(ChoiceParam(FRUITS), description="Which fruit.")


@dataclass
class FruitWithDefaultScenario(Scenario):
    fruit: str = param(ChoiceParam(FRUITS), default="apple")


class ChoiceIdPatternTest(unittest.TestCase):
    def test_pattern_is_the_one_saphira_and_wrench_check1(self):
        # Saphira and the Wrench API repeat this pattern; change it there too.
        self.assertEqual(CHOICE_ID_PATTERN, r"^[a-z0-9][a-z0-9_]{0,63}$")

    def test_valid_ids1(self):
        for value in ("a", "0", "ycb_006_mustard_bottle", "a" * 64):
            with self.subTest(value=value):
                self.assertTrue(is_choice_id(value))

    def test_invalid_ids1(self):
        for value in (
            "",
            "_a",
            "A",
            "a-b",
            "a b",
            "a" * 65,
            "apple\n",
            "package://drake_models/ycb/006_mustard_bottle.sdf",
            "../x",
            1,
            None,
            True,
        ):
            with self.subTest(value=value):
                self.assertFalse(is_choice_id(value))


class ChoiceParamDeclarationTest(unittest.TestCase):
    def test_ids_must_match_pattern1(self):
        with self.assertRaisesRegex(ValueError, "Bad_ID"):
            ChoiceParam({"ok": 1, "Bad_ID": 2})

    def test_needs_a_choice1(self):
        with self.assertRaises(ValueError):
            ChoiceParam({})

    def test_needs_a_mapping1(self):
        with self.assertRaises(TypeError):
            ChoiceParam(["apple"])

    def test_choices_cannot_change_after_declaration1(self):
        source = dict(FRUITS)
        spec = ChoiceParam(source)
        source["cherry"] = "package://fruit/cherry.sdf"
        spec.choices["cherry"] = "package://fruit/cherry.sdf"
        self.assertEqual(spec.ids(), ["apple", "banana_2"])

    def test_default_must_be_a_choice1(self):
        @dataclass
        class BadDefault(Scenario):
            fruit: str = param(ChoiceParam(FRUITS), default="cherry")

        with self.assertRaisesRegex(TypeError, "cherry"):
            BadDefault.schema()


class ChoiceParamValidateTest(unittest.TestCase):
    def test_valid_id_is_returned1(self):
        self.assertEqual(FruitScenario.validate({"fruit": "apple"}), {"fruit": "apple"})

    def test_none_is_allowed_when_default_is_none1(self):
        self.assertEqual(FruitScenario.validate({"fruit": None}), {"fruit": None})

    def test_unknown_id_lists_the_choices1(self):
        with self.assertRaises(ScenarioError) as cm:
            FruitScenario.validate({"fruit": "cherry"})
        self.assertIn("'cherry'", cm.exception.problems[0])
        self.assertIn("['apple', 'banana_2']", cm.exception.problems[0])

    def test_the_value_behind_an_id_is_not_accepted1(self):
        # A caller must send the ID, never the path or URL it stands for.
        with self.assertRaises(ScenarioError):
            FruitScenario.validate({"fruit": "package://fruit/apple.sdf"})

    def test_non_strings_are_refused1(self):
        for value in (0, 1.0, True, ["apple"], {"id": "apple"}):
            with self.subTest(value=value):
                with self.assertRaises(ScenarioError):
                    FruitScenario.validate({"fruit": value})

    def test_long_values_are_shortened_in_errors1(self):
        with self.assertRaises(ScenarioError) as cm:
            FruitScenario.validate({"fruit": "x" * 1000})
        self.assertLess(len(cm.exception.problems[0]), 300)


class ChoiceParamConversionTest(unittest.TestCase):
    def test_constructor_kwargs_maps_id_to_value1(self):
        self.assertEqual(
            FruitScenario.constructor_kwargs({"fruit": "banana_2"}),
            {"fruit": "package://fruit/banana.sdf"},
        )

    def test_constructor_kwargs_keeps_none1(self):
        self.assertEqual(
            FruitScenario.constructor_kwargs({"fruit": None}), {"fruit": None}
        )


class ChoiceParamSchemaTest(unittest.TestCase):
    def test_schema_lists_sorted_ids_but_not_values1(self):
        schema = FruitScenario.schema()
        self.assertEqual(
            schema["fields"],
            [
                {
                    "name": "fruit",
                    "kind": "choice",
                    "default": None,
                    "units": "",
                    "description": "Which fruit.",
                    "choices": ["apple", "banana_2"],
                }
            ],
        )
        text = json.dumps(schema)
        self.assertNotIn("package://", text)

    def test_schema_default1(self):
        self.assertEqual(
            FruitWithDefaultScenario.schema()["fields"][0]["default"], "apple"
        )


if __name__ == "__main__":
    unittest.main()

import importlib
import inspect
import pkgutil
import unittest

# Internal Imports
import brom_drake.productions
from brom_drake.productions.types.base import BaseProduction
from brom_drake.productions.types.base.scenario import Scenario


def all_production_classes():
    """
    Description
    -----------
    Import every module under brom_drake.productions and return every
    BaseProduction subclass found, so new productions are covered without
    being listed here.
    """
    for module in pkgutil.walk_packages(
        brom_drake.productions.__path__, prefix="brom_drake.productions."
    ):
        importlib.import_module(module.name)

    found, stack = [], [BaseProduction]
    while stack:
        klass = stack.pop()
        for subclass in klass.__subclasses__():
            if subclass not in found:
                found.append(subclass)
                stack.append(subclass)
    return sorted(found, key=lambda k: f"{k.__module__}.{k.__qualname__}")


def constructor_default(production_class, name):
    """
    The default of the keyword argument ``name``, from the first ``__init__``
    up the class chain that declares it.
    """
    for klass in production_class.__mro__:
        init = klass.__dict__.get("__init__")
        if init is None:
            continue
        parameter = inspect.signature(init).parameters.get(name)
        if parameter is not None:
            return parameter.default
    raise LookupError(name)


class ScenarioContractTest(unittest.TestCase):
    """
    Description
    -----------
    The contract callers such as Saphira rely on: a value for a Scenario
    field is applied by passing it as the constructor keyword argument of the
    same name, so every field must be one, with the same default.
    """

    @classmethod
    def setUpClass(cls):
        cls.productions = [
            p for p in all_production_classes() if getattr(p, "Scenario", None)
        ]

    def test_productions_with_scenarios_are_found1(self):
        names = {p.__name__ for p in self.productions}
        for expected in (
            "KinematicMotionPlanningProduction",
            "OfflineDynamicMotionPlanningProduction",
            "MotionPlanningAndGraspingProduction",
            "ChemLab1",
            "ChemLab2",
            "ChemLab3",
            "ShelfPlanning1",
            "CornellTech1",
        ):
            self.assertIn(expected, names)

    def test_scenario_is_a_valid_declaration1(self):
        for production in self.productions:
            with self.subTest(production=production.__name__):
                self.assertTrue(issubclass(production.Scenario, Scenario))
                # Raises TypeError for a missing @dataclass, a field without
                # param(), a default outside its range, or a bad
                # mutually_exclusive group.
                production.Scenario.schema()

    def test_fields_are_constructor_arguments1(self):
        for production in self.productions:
            accepted = production.accepted_arguments()
            for field in production.Scenario.schema()["fields"]:
                with self.subTest(production=production.__name__, field=field["name"]):
                    self.assertIn(
                        field["name"],
                        accepted,
                        f"{production.__name__}.Scenario field {field['name']!r} "
                        "is not a keyword argument of any __init__ in "
                        f"{production.__name__}'s class chain",
                    )

    def test_defaults_match_constructor1(self):
        """
        Description
        -----------
        A field's default is the constructor's default for that argument
        (None for start, goal and scene poses, numbers for dimensions).
        """
        for production in self.productions:
            for field in production.Scenario.schema()["fields"]:
                with self.subTest(production=production.__name__, field=field["name"]):
                    expected = constructor_default(production, field["name"])
                    self.assertIsNot(
                        expected,
                        inspect.Parameter.empty,
                        f"{field['name']!r} has no default in the constructor",
                    )
                    self.assertEqual(field["default"], expected)


if __name__ == "__main__":
    unittest.main()

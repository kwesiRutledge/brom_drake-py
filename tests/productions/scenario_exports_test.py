import unittest


class ScenarioExportsTest(unittest.TestCase):
    """
    Description
    -----------
    The Scenario API is importable from the package's public entry points,
    which is where callers such as Saphira import it from.
    """

    names = [
        "Scenario",
        "ScenarioError",
        "FloatParam",
        "VectorParam",
        "PoseParam",
        "param",
    ]

    def test_exported_from_public_modules1(self):
        import brom_drake.all
        import brom_drake.productions.all
        import brom_drake.productions.types
        from brom_drake.productions.types.base import scenario

        for module in (
            brom_drake.all,
            brom_drake.productions.all,
            brom_drake.productions.types,
        ):
            for name in self.names:
                with self.subTest(module=module.__name__, name=name):
                    self.assertIs(getattr(module, name), getattr(scenario, name))

    def test_star_import1(self):
        namespace = {}
        exec("from brom_drake.productions.all import *", namespace)
        for name in self.names:
            self.assertIn(name, namespace)

    def test_productions_scenarios_reachable_from_all1(self):
        from brom_drake.all import ChemLab1, ScenarioError

        with self.assertRaises(ScenarioError):
            ChemLab1.Scenario.validate({"not_a_field": 1})


if __name__ == "__main__":
    unittest.main()

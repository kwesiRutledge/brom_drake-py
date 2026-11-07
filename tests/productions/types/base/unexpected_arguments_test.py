import unittest

# Internal imports
from brom_drake.all import BaseProduction
from brom_drake.productions.motion_planning.offline import ChemLab1, ShelfPlanning1


class UnexpectedArgumentsTest(unittest.TestCase):
    def test_base_production_rejects_unknown1(self):
        """
        Description
        -----------
        BaseProduction itself accepts no keyword arguments.
        """
        BaseProduction()  # No arguments is fine
        with self.assertRaises(TypeError) as ctx:
            BaseProduction(not_a_real_argument=1)
        self.assertIn("not_a_real_argument", str(ctx.exception))

    def test_typo_suggests_correct_name1(self):
        """
        Description
        -----------
        A misspelled argument is rejected (it used to be silently ignored) and
        the error suggests the argument that was probably meant.
        """
        with self.assertRaises(TypeError) as ctx:
            ChemLab1(meshcat_port_number=None, start_config=[0.0] * 6)
        message = str(ctx.exception)
        self.assertIn("ChemLab1", message)
        self.assertIn("'start_config'", message)
        self.assertIn("Did you mean 'start_configuration'", message)

    def test_accepted_arguments_cover_hierarchy1(self):
        """
        Description
        -----------
        accepted_arguments lists the parameters of every __init__ in the class
        hierarchy (the production's own and its production type's).
        """
        accepted = ShelfPlanning1.accepted_arguments()
        self.assertIn("shelf_pose", accepted)  # ShelfPlanning1
        self.assertIn("meshcat_port_number", accepted)  # ShelfPlanning1
        self.assertIn("start_configuration", accepted)  # Kinematic production type
        self.assertNotIn("self", accepted)
        self.assertNotIn("kwargs", accepted)

    def test_valid_arguments_still_work1(self):
        """
        Description
        -----------
        Arguments defined on a parent production type (passed up through
        **kwargs) are still accepted.
        """
        production = ShelfPlanning1(
            meshcat_port_number=None,
            start_configuration=[-0.323, -1.566, 0.283, -0.287, -1.571, -1.893],
        )
        self.assertIsNotNone(production)


if __name__ == "__main__":
    unittest.main()

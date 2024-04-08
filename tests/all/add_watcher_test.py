"""
add_watcher_test.py
Description:

    Tests the add_watcher convenience functions.
"""

import unittest

from pydrake.all import DiagramBuilder, MultibodyPlant
from pydrake.systems.framework import LeafSystem

from brom_drake.all import DiagramTarget, parse_list_of_simplified_targets


class AddWatcherTest(unittest.TestCase):
    def test_parse_list_of_simplified_targets1(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when an empty list is passed in.
        :return:
        """

        builder = DiagramBuilder()
        try:
            parse_list_of_simplified_targets(builder, [])
            self.assertTrue(False)
        except ValueError as e:
            print(e)
            self.assertTrue(True)
        except Exception as e:
            print(e)
            self.assertTrue(False)

    def test_parse_list_of_simplified_targets2(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when the input is not a list
        but something else (maybe a string?).
        :return:
        """
        builder = DiagramBuilder()
        try:
            parse_list_of_simplified_targets(builder, "test")
            self.assertTrue(False)
        except ValueError as e:
            self.assertTrue(True)

    def test_parse_list_of_simplified_targets3(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function produces a valid list of DiagramTargets when
        given a list of strings.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        controller = builder.AddNamedSystem("my_controller", LeafSystem())

        # Test
        targets = parse_list_of_simplified_targets(builder, ["my_plant", "my_controller"])
        self.assertEqual(len(targets), 2)

        for ii, dt_ii in enumerate(targets):
            self.assertTrue(isinstance(dt_ii, DiagramTarget))
            self.assertEqual(dt_ii.name, ["my_plant", "my_controller"][ii])



if __name__ == "__main__":
    unittest.main()
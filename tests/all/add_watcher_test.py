"""
add_watcher_test.py
Description:

    Tests the add_watcher convenience functions.
"""

import unittest

from pydrake.all import DiagramBuilder, MultibodyPlant
from pydrake.systems.framework import LeafSystem

from brom_drake.all import DiagramTarget, parse_list_of_simplified_targets, add_watcher


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

    def test_parse_list_of_simplified_targets4(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function produces a valid list of DiagramTargets when
        given a list of tuples (most of the tuples contain list of ints for the ports_list).
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))

        # Test
        targets = parse_list_of_simplified_targets(
            builder, [("my_plant",[0,1]), ("my_controller",[0,1,2])])
        self.assertEqual(len(targets), 2)

        self.assertEqual(targets[0].name, "my_plant")
        self.assertEqual(targets[0].ports, [0, 1])

        self.assertEqual(targets[1].name, "my_controller")
        self.assertEqual(targets[1].ports, [0, 1, 2])


    def test_add_watcher1(self):
        """
        Description:

            Tests the add_watcher convenience function.
            We will make sure that new loggers are created (one for each port) and that
            the logger is connected to the port.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))

        # Add Watcher
        watcher = add_watcher(builder, targets=[("my_plant", 0), ("my_controller", 0)])

        # Verify that the watcher is connected to the correct ports
        self.assertIn(plant.get_output_port(0).get_name(), "geometry_pose")
        self.assertIn(controller.get_output_port(0).get_name(), "geometry_pose")


if __name__ == "__main__":
    unittest.main()

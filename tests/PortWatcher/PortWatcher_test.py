"""
PortWatcher_test.py
Description:

    This
"""
from importlib import resources as impresources
import unittest
import os

import numpy as np
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.systems.framework import DiagramBuilder, PortDataType, Diagram, Context

# Internal Imports
from brom_drake.all import (
    PortWatcher,
)

class PortWatcherTest(unittest.TestCase):
    def setUp(self):
        """
        Description:

            This method is used to setup each test.
        :return:
        """
        # Delete the .brom directory if it exists when the test is done.
        self.delete_test_brom_directory_on_teardown = True
        self.T_sim1 = 5.0  # Time to simulate

    def get_brom_drake_dir(self):
        """
        Description:

            This function returns the path to the brom_drake directory.
        :return:
        """
        if "tests" in os.getcwd():
            return os.path.abspath(os.path.join(os.getcwd(), "..", ".."))
        else:
            return os.getcwd()

    def test_init1(self):
        """
        Description:

            Tests that the PortWatcher object correctly
            is created if given a valid system, output port, and
            builder.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        self.assertTrue(True)
        self.assertIn(
            "PortWatcher", pw0.logger.get_name(),
        )
        self.assertIn(
            plant.get_name(), pw0.logger.get_name(),
        )
        self.assertIn(
            "state", pw0.logger.get_name(),
        )

    def test_init2(self):
        """
        Description:

            This test verifies that if a port is provided to the
            PortWatcher that is not kVectorValued, then an error is
            raised.
        :return:
        """

        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        plant_test_port = plant.get_output_port(0)

        try:
            pw0 = PortWatcher(
                plant, plant_test_port, builder,
            )
        except ValueError as e:
            expected_error = ValueError(
                f"This watcher only supports vector valued ports (i.e., of type {PortDataType.kVectorValued}.\n" +
                f"Received port of type {plant_test_port.get_data_type()}."
            )

            self.assertEqual(
                str(e), str(expected_error),
            )
        else:
            self.assertTrue(False)

if __name__ == '__main__':
    unittest.main()

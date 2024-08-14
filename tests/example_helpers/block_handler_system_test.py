"""
block_handler_system_test.py
Description:

    This file defines tests for the BlockHandlerSystem class.
    Or any other helper class that is used in the examples.
"""

from importlib import resources as impresources
import unittest
from pydrake.all import (
    DiagramBuilder, AddMultibodyPlantSceneGraph
)

# Internal Imports
from brom_drake.example_helpers import BlockHandlerSystem, AddGround
import brom_drake.example_helpers as eh


class TestBlockHandlerSystem(unittest.TestCase):
    def test_init1(self):
        """
        Description:

            This test checks that the BlockHandlerSystem can be initialized.
        :return:
        """
        # Setup
        time_step = 1e-3
        urdf_path = impresources.files(eh) / "models/slider-block.urdf"  # TODO(Kwesi): Create a directory to hold all models
        builder = DiagramBuilder()

        # Create Plant and the "Block + Ground system"
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
        block_handler_system = builder.AddSystem(
            BlockHandlerSystem(plant, scene_graph, model_urdf_path=urdf_path)
        )

        # Check that the plant is finalized after building the system.
        self.assertTrue(plant.is_finalized())

    def test_AddGround1(self):
        """
        Description:

            This test checks that the AddGround function can be called.
        :return:
        """
        # Setup
        time_step = 1e-3
        builder = DiagramBuilder()

        # Create Plant and the "Block + Ground system"
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
        AddGround(plant)

        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()


"""
DiagramWatcher_test.py
Description:

    This file defines the tests for the DiagramWatcher module.
"""

import unittest
import numpy as np

from pydrake.all import (
    DiagramBuilder,
    LeafSystem,
    PortDataType,
    BasicVector,
    Context,
    Simulator,
)

from brom_drake.watchers.diagram_watcher.diagram_watcher import DiagramWatcher

class TestDiagramWatcher(unittest.TestCase):
    def test_init1(self):
        """
        Description:

            This test checks that an exception is raised
            when we try to create a DiagramWatcher using a
            subject that is not a DiagramBuilder.
        :return:
        """
        # Create a simple system
        system = LeafSystem()

        # Try to create a DiagramWatcher with the system
        try:
            watcher = DiagramWatcher(system)
            self.fail("Should have raised an exception!")
        except ValueError as e:
            expectedError = ValueError("subject must be a DiagramBuilder!")
            self.assertEqual(str(e), str(expectedError))
        else:
            self.fail("Should have raised an exception!")
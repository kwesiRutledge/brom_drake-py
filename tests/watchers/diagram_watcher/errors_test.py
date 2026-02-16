"""
errors_test.py
Description

        Tests the custom errors for the DiagramWatcher module.
"""

import unittest

from brom_drake.all import DiagramTarget
from brom_drake.watchers.diagram_watcher.errors import (
    PortIsNotFoundInDiagramError,
    PortIsNotBeingWatchedError,
    SystemIsNotFoundInDiagramError, 
    SystemIsNotBeingWatchedError,
)


class TestErrors(unittest.TestCase):
    def test_PortIsNotFoundInDiagramError1(self):
        target = DiagramTarget("test", ports=[0])
        error = PortIsNotFoundInDiagramError(target, port_reference=0)
        self.assertEqual(error.target, target)
        self.assertEqual(error.port_reference, 0)
        self.assertIn(
            "Port \"0\" on system with name test was not found in the Diagram.\n",
            error.message,
        )

    def test_PortNotBeingWatchedError1(self):
        target = DiagramTarget("test", ports=[0])
        error = PortIsNotBeingWatchedError(target, port_reference=0)
        self.assertEqual(error.target, target)
        self.assertIn(
            "Port \"0\" on system with name test is not being watched by the DiagramWatcher.\n",
            error.message,
        )

    def test_SystemIsNotFoundInDiagramError1(self):
        target = DiagramTarget("test")
        error = SystemIsNotFoundInDiagramError(target)
        self.assertEqual(error.target, target)
        self.assertIn(
            "System with name \"test\" was not found in the Diagram.\n",
            error.message,
        )

    def test_SystemNotBeingWatchedError1(self):
        target = DiagramTarget("test")
        error = SystemIsNotBeingWatchedError(target)
        self.assertEqual(error.target, target)
        self.assertIn(
            "System with name \"test\" is not being watched by the DiagramWatcher.\n",
            error.message,
        )
    


if __name__ == "__main__":
    unittest.main()

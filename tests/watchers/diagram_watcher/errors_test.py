"""
errors_test.py
Description

        Tests the custom errors for the DiagramWatcher module.
"""

import unittest

from brom_drake.all import DiagramTarget
from brom_drake.watchers.diagram_watcher.errors import UnrecognizedTargetError


class TestErrors(unittest.TestCase):
    def test_UnrecognizedTargetError(self):
        target = DiagramTarget("test")
        error = UnrecognizedTargetError(target)
        self.assertEqual(error.target, target)
        self.assertIn(
            "Target with name test is not recognized by the DiagramWatcher.\n",
            error.message,
        )

        target = DiagramTarget("test")
        error = UnrecognizedTargetError(target, ["test1", "test2"])
        self.assertEqual(error.target, target)
        self.assertIn(
            "Target with name test is not recognized by the DiagramWatcher.\n(Available systems are ['test1', 'test2'])\n",
            error.message,
        )

        print(error)


if __name__ == "__main__":
    unittest.main()

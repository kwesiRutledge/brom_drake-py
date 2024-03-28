"""
DiagramTarget_test.py
Description:

    Tests the DiagramTarget class.
"""

import unittest

from brom_drake import DiagramTarget


class TestDiagramTarget(unittest.TestCase):
    def test_init(self):
        target = DiagramTarget("test")
        self.assertEqual(target.name, "test")
        self.assertIsNone(target.ports)

        target = DiagramTarget("test", [1, 2, 3])
        self.assertEqual(target.name, "test")
        self.assertEqual(target.ports, [1, 2, 3])


if __name__ == "__main__":
    unittest.main()

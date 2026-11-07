from brom_drake.productions.grasping.phases import GraspSelection1Phase

import unittest


class TestGraspSelection1Phase(unittest.TestCase):
    def test_order1(self):
        """
        Description
        -----------
        This test verifies that the phases of the GraspSelection1 production
        are ordered in the sequence in which they occur in the script.
        """
        # Setup
        expected_order = [
            GraspSelection1Phase.kObjectSettlingOnFloor,
            GraspSelection1Phase.kGripperApproach,
            GraspSelection1Phase.kGripperClosing,
            GraspSelection1Phase.kObjectSettlingInGrasp,
            GraspSelection1Phase.kFloorDrop,
        ]

        # Verify the order
        self.assertEqual(sorted(GraspSelection1Phase), expected_order)


if __name__ == "__main__":
    unittest.main()

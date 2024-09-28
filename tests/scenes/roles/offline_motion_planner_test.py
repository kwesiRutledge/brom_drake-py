import unittest

from brom_drake.scenes.roles.offline_motion_planner import kOfflineMotionPlanner


class OfflineMotionPlannerTest(unittest.TestCase):
    def test_required_input_ports1(self):
        """
        Description
        -----------
        This test checks the required_input_ports method of the OfflineMotionPlanner class.
        :return:
        """
        # Setup
        omp_role = kOfflineMotionPlanner

        # Check the required input ports
        for port_name in omp_role.required_input_ports:
            self.assertTrue(port_name in ["joint_positions", "joint_position_limits", "goal_position"])

    def test_required_output_ports1(self):
        """
        Description
        -----------
        This test checks the required_output_ports method of the OfflineMotionPlanner class.

        :return:
        """
        # Check
        omp_role = kOfflineMotionPlanner

        # Assertion
        self.assertEqual(omp_role.required_output_ports, ["plan"])

if __name__ == "__main__":
    unittest.main()
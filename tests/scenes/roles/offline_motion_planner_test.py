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
        expected_port_names = [
            "start_pose",
            "goal_pose",
            "scene_id",
        ]

        # Check the required input ports
        required_preformer_ports = [
            port.performer_port_name for port in omp_role.input_definitions
        ]
        for port_name in expected_port_names:
            self.assertTrue(port_name in required_preformer_ports)

    def test_required_output_ports1(self):
        """
        Description
        -----------
        This test checks the required_output_ports method of the OfflineMotionPlanner class.

        :return:
        """
        # Check
        omp_role = kOfflineMotionPlanner
        expected_port_names = ["plan"]

        # Assertion
        required_performer_outputs = [
            port.performer_port_name for port in omp_role.output_definitions
        ]
        for port_name in expected_port_names:
            self.assertTrue(port_name in required_performer_outputs)

if __name__ == "__main__":
    unittest.main()
import unittest

from brom_drake.productions.roles.motion_planners.kinematic import kKinematicMotionPlanner
from brom_drake.productions.roles.role_port_assignment import PairingType


class OfflineMotionPlannerTest(unittest.TestCase):
    def test_required_input_ports1(self):
        """
        Description
        -----------
        This test checks the required_input_ports method of the OfflineMotionPlanner class.
        :return:
        """
        # Setup
        omp_role = kKinematicMotionPlanner
        expected_port_names = [
            "start_configuration",
            "goal_configuration",
            "scene_id",
        ]

        # Check the required input ports
        required_preformer_ports = [
            port.performer_port_name for port in omp_role.port_assignments
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
        omp_role = kKinematicMotionPlanner
        expected_port_names = ["motion_plan", "plan_is_ready"]

        # Assertion
        required_performer_outputs = [
            assignment.performer_port_name
                for assignment in omp_role.port_assignments
                if assignment.pairing_type == PairingType.kOutput
        ]
        for port_name in expected_port_names:
            self.assertTrue(port_name in required_performer_outputs)

if __name__ == "__main__":
    unittest.main()
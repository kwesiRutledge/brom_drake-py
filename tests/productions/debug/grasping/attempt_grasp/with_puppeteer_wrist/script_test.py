from brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.script import Script as AttemptGraspWithPuppeteerWristScript

import unittest

class TestAttemptGraspWithPuppeteerWristScript(unittest.TestCase):
    def test_to_networkx_graph1(self):
        """
        Description
        -----------
        This test verifies that the method (with a default Script)
        creates a NetworkX DiGraph with four nodes.
        """
        # Setup
        script = AttemptGraspWithPuppeteerWristScript()

        # Create graph
        graph = script.to_networkx_graph()

        # Check the number of nodes
        self.assertEqual(len(graph.nodes), 4)

    def test_total_time1(self):
        """
        Creates a Script with known time durations and
        verifies that the total_time method returns the expected sum.
        """
        # Setup
        script = AttemptGraspWithPuppeteerWristScript(
            settling_time_on_floor=8.0,
            gripper_approach_time=4.0,
            grasp_closing_time=3.0,
            post_grasp_settling_time=1.0,
            drop_time=6.0,
        )

        # Calculate total time
        total_time = script.total_time()

        # Verify the total time
        expected_total_time = 8.0 + 4.0 + 3.0 + 1.0 + 6.0
        self.assertEqual(total_time, expected_total_time)


if __name__ == "__main__":
    unittest.main()

        
from brom_drake.productions.grasping.phases import GraspSelection1Phase
from brom_drake.productions.grasping.script import Script as GraspSelection1Script
from brom_drake.systems.network_fsm import NetworkXFSM

import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw
import unittest


class TestGraspSelection1Script(unittest.TestCase):
    def setUp(self):
        # Create a script with known time durations
        self.script = GraspSelection1Script(
            settling_time_on_floor=8.0,
            gripper_approach_time=4.0,
            grasp_closing_time=3.0,
            post_grasp_settling_time=1.0,
            drop_time=6.0,
        )

    def test_to_networkx_graph1(self):
        """
        Description
        -----------
        This test verifies that the method (with a default Script)
        creates a NetworkX DiGraph with five nodes (one for each phase).
        """
        # Setup
        script = GraspSelection1Script()

        # Create graph
        graph = script.to_networkx_graph()

        # Check the nodes
        self.assertEqual(set(graph.nodes), set(GraspSelection1Phase))

    def test_to_networkx_graph2(self):
        """
        Description
        -----------
        This test verifies that when the post-grasp settling time is zero,
        the kObjectSettlingInGrasp phase is skipped (i.e., the graph contains four nodes
        and the gripper closing phase transitions directly to the floor drop).
        """
        # Setup
        script = GraspSelection1Script(post_grasp_settling_time=0.0)

        # Create graph
        graph = script.to_networkx_graph()

        # Check the nodes and edges
        self.assertEqual(len(graph.nodes), 4)
        self.assertNotIn(GraspSelection1Phase.kObjectSettlingInGrasp, graph.nodes)
        self.assertTrue(
            graph.has_edge(
                GraspSelection1Phase.kGripperClosing,
                GraspSelection1Phase.kFloorDrop,
            )
        )

    def test_to_fsm1(self):
        """
        Description
        -----------
        This test verifies that the to_fsm method creates a NetworkXFSM
        with the output ports that the GraspSelection1 production connects to.
        """
        # Setup
        fsm = self.script.to_fsm()

        # Verify the type and the output ports
        self.assertIsInstance(fsm, NetworkXFSM)
        for port_name in ["start_floor", "switch_to_memory"]:
            self.assertTrue(
                fsm.HasOutputPort(port_name),
                f"Expected FSM to have output port {port_name}",
            )

    def test_start_time_of_phase1(self):
        """
        Description
        -----------
        This test verifies that the start time of each phase is the sum of the
        durations of all phases that come before it.
        """
        # Setup
        expected_start_times = {
            GraspSelection1Phase.kObjectSettlingOnFloor: 0.0,
            GraspSelection1Phase.kGripperApproach: 8.0,
            GraspSelection1Phase.kGripperClosing: 8.0 + 4.0,
            GraspSelection1Phase.kObjectSettlingInGrasp: 8.0 + 4.0 + 3.0,
            GraspSelection1Phase.kFloorDrop: 8.0 + 4.0 + 3.0 + 1.0,
        }

        # Verify the start times
        for phase, expected_start_time in expected_start_times.items():
            self.assertAlmostEqual(
                self.script.start_time_of_phase(phase),
                expected_start_time,
            )

    def test_start_time_of_phase2(self):
        """
        Description
        -----------
        This test verifies that the method raises a ValueError
        when given something that is not a phase.
        """
        with self.assertRaises(ValueError):
            self.script.start_time_of_phase(-1)

    def test_total_time1(self):
        """
        Description
        -----------
        Creates a Script with known time durations and
        verifies that the total_time method returns the expected sum.
        """
        # Calculate total time
        total_time = self.script.total_time()

        # Verify the total time
        expected_total_time = 8.0 + 4.0 + 3.0 + 1.0 + 6.0
        self.assertEqual(total_time, expected_total_time)

    def test_build_gripper_approach_trajectory_from_waypoints1(self):
        """
        Description
        -----------
        This test verifies that the trajectory created from a list of waypoints:
        - holds the first waypoint until the gripper approach begins,
        - reaches the last waypoint when the gripper closing begins, and
        - holds the last waypoint afterwards.
        """
        # Setup
        X_ObjectPreGrasp = RigidTransform(p=np.array([0.0, 0.0, 0.5]))
        X_ObjectMidpoint = RigidTransform(
            p=np.array([0.0, 0.1, 0.3]),
            rpy=RollPitchYaw(0.0, np.pi / 4.0, 0.0),
        )
        X_ObjectGrasp = RigidTransform(
            p=np.array([0.0, 0.2, 0.1]),
            rpy=RollPitchYaw(0.0, np.pi / 2.0, 0.0),
        )
        waypoints = [X_ObjectPreGrasp, X_ObjectMidpoint, X_ObjectGrasp]

        t_approach = self.script.start_time_of_phase(
            GraspSelection1Phase.kGripperApproach
        )
        t_closing = self.script.start_time_of_phase(
            GraspSelection1Phase.kGripperClosing
        )

        # Create trajectory
        trajectory = self.script.build_gripper_approach_trajectory_from_waypoints(
            waypoints
        )

        # Verify the poses at key times
        test_cases = [
            (0.0, X_ObjectPreGrasp),
            (t_approach, X_ObjectPreGrasp),
            ((t_approach + t_closing) / 2.0, X_ObjectMidpoint),
            (t_closing, X_ObjectGrasp),
        ]
        for t, X_expected in test_cases:
            X_actual = trajectory.GetPose(t)
            self.assertTrue(
                X_actual.IsNearlyEqualTo(X_expected, 1e-6),
                f"At time {t}, expected pose\n{X_expected}\nbut received\n{X_actual}",
            )

        self.assertAlmostEqual(trajectory.end_time(), t_closing)

    def test_build_gripper_approach_trajectory_from_waypoints2(self):
        """
        Description
        -----------
        This test verifies that a single waypoint creates a trajectory
        that stays at that waypoint for the entire approach.
        """
        # Setup
        X_ObjectGrasp = RigidTransform(p=np.array([0.0, 0.2, 0.1]))

        # Create trajectory
        trajectory = self.script.build_gripper_approach_trajectory_from_waypoints(
            [X_ObjectGrasp]
        )

        # Verify the poses
        for t in np.linspace(0.0, trajectory.end_time(), 5):
            self.assertTrue(trajectory.GetPose(t).IsNearlyEqualTo(X_ObjectGrasp, 1e-6))


if __name__ == "__main__":
    unittest.main()

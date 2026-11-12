import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw, Solve
import unittest

# Internal Imports
from brom_drake.motion_planning.algorithms.rrt.bidirectional_connect import (
    BidirectionalRRTConnectPlanner,
    BidirectionalRRTConnectPlannerConfig,
)
from brom_drake.productions.motion_planning.offline.dynamic.cornell_tech1 import (
    CornellTech1,
)
from brom_drake.productions.ids import ProductionID
from brom_drake.stations.classical import KinovaGen3Variant


class CornellTech1Test(unittest.TestCase):
    def build_production(
        self, arm_variant: KinovaGen3Variant = KinovaGen3Variant.kSevenDof
    ) -> CornellTech1:
        """Builds the production (with an RRT planner) and returns it."""
        production = CornellTech1(meshcat_port_number=None, arm_variant=arm_variant)
        planner = BidirectionalRRTConnectPlanner(
            production.arm,
            production.plant,
            production.scene_graph,
            config=BidirectionalRRTConnectPlannerConfig(steering_step_size=0.2),
        )
        production.easy_cast_and_build(planner.plan, with_watcher=False)
        return production

    def test_init1(self):
        """
        Description
        -----------
        This test verifies that we can initialize CornellTech1 without any errors
        and that the arm has 7 degrees of freedom.
        """
        production = CornellTech1(meshcat_port_number=None)
        production.station.Finalize()
        self.assertEqual(production.plant.num_positions(production.arm), 7)

    def test_easy_cast_and_build1(self):
        """
        Description
        -----------
        This test verifies that we can build CornellTech1 and that the
        start and goal configurations are collision free.
        """
        np.random.seed(0)
        production = self.build_production()
        planner = production.performers[0]

        self.assertFalse(
            planner.check_collision_in_config(production.start_configuration)
        )
        self.assertFalse(
            planner.check_collision_in_config(production.goal_configuration)
        )

    def test_easy_cast_and_build_six_dof1(self):
        """
        Description
        -----------
        This test verifies that we can build CornellTech1 with the 6-DoF arm and
        that its start and goal configurations have 6 entries and are collision free.
        """
        np.random.seed(0)
        production = self.build_production(arm_variant=KinovaGen3Variant.kSixDof)
        planner = production.performers[0]

        self.assertEqual(production.plant.num_positions(production.arm), 6)
        self.assertEqual(production.start_configuration.shape, (6,))
        self.assertEqual(production.goal_configuration.shape, (6,))
        self.assertFalse(
            planner.check_collision_in_config(production.start_configuration)
        )
        self.assertFalse(
            planner.check_collision_in_config(production.goal_configuration)
        )

    def test_start_and_goal_configurations1(self):
        """
        Description
        -----------
        This test verifies that the start and goal configurations
        (computed with inverse kinematics) put the end effector at the
        start and goal poses.
        """
        for variant in KinovaGen3Variant:
            with self.subTest(variant=variant):
                self.check_start_and_goal_configurations(variant)

    def check_start_and_goal_configurations(self, variant: KinovaGen3Variant):
        """Checks the start/goal configurations of the production with this variant."""
        production = CornellTech1(meshcat_port_number=None, arm_variant=variant)
        production.station.Finalize()

        for q, pose in [
            (production.start_configuration, production.start_pose),
            (production.goal_configuration, production.goal_pose),
        ]:
            pose_from_q = production.solve_forward_kinematics_problem_for_arm(q)
            # (The IK position constraint is a +/- 2.5 cm box in the end effector's frame)
            self.assertLess(
                np.linalg.norm(pose_from_q.translation() - pose.translation()),
                0.025 * np.sqrt(3.0) + 1e-3,
            )
            # End effector should point (approximately) down
            z_axis = pose_from_q.rotation().matrix()[:, 2]
            self.assertLess(z_axis[2], -np.cos(np.pi / 8.0) + 1e-3)

    def test_beaker_on_desk1(self):
        """
        Description
        -----------
        This test verifies that, by default, the beaker sits on the desk right
        behind the arm, and that the goal pose is above the beaker.
        """
        production = CornellTech1(meshcat_port_number=None)
        p_beaker = production.pose_WorldBeaker.translation()

        # On top of the desk (desk frame is on the floor), slightly above to avoid penetration
        desk_top_z = production.pose_WorldDesk.translation()[2] + production.desk_height
        self.assertAlmostEqual(p_beaker[2], desk_top_z + 0.012, places=9)

        # Directly behind the arm (whose base is at the origin), over the desktop
        self.assertAlmostEqual(p_beaker[0], 0.0, places=9)
        self.assertGreater(p_beaker[1], 0.1)  # the desk's front edge is at y = 0.1

        # The goal is 0.2 m above the beaker, pointing down
        p_goal = production.goal_pose.translation()
        np.testing.assert_allclose(p_goal, p_beaker + np.array([0.0, 0.0, 0.2]))
        z_axis = production.goal_pose.rotation().matrix()[:, 2]
        np.testing.assert_allclose(z_axis, [0.0, 0.0, -1.0], atol=1e-12)

    def test_beaker_follows_desk1(self):
        """
        Description
        -----------
        This test verifies that the default beaker pose moves with the desk.
        """
        default = CornellTech1(meshcat_port_number=None)
        shift = np.array([-0.2, 0.1, 0.0])
        moved = CornellTech1(
            meshcat_port_number=None,
            pose_WorldDesk=RigidTransform(default.pose_WorldDesk.translation() + shift),
        )
        np.testing.assert_allclose(
            moved.pose_WorldBeaker.translation(),
            default.pose_WorldBeaker.translation() + shift,
        )

    def test_ik_needs_more_than_default_guess1(self):
        """
        Description
        -----------
        This test verifies that solve_pose_ik_problem finds the goal configuration
        even though a single solve from the default configuration fails
        (which is why it tries several initial guesses).
        """
        for variant in KinovaGen3Variant:
            with self.subTest(variant=variant):
                production = CornellTech1(meshcat_port_number=None, arm_variant=variant)
                production.station.Finalize()

                # A single solve from the default configuration fails...
                ik_problem = production.define_pose_ik_problem(
                    production.goal_pose, "end_effector"
                )
                default_positions = production.plant.GetPositions(
                    production.plant.CreateDefaultContext()
                )
                self.assertFalse(
                    Solve(ik_problem.prog(), default_positions).is_success()
                )

                # ... but solve_pose_ik_problem succeeds, deterministically
                q1 = production.solve_pose_ik_problem(production.goal_pose)
                q2 = production.solve_pose_ik_problem(production.goal_pose)
                self.assertEqual(q1.shape, (variant.num_dofs,))
                np.testing.assert_array_equal(q1, q2)

    def test_ik_unreachable_pose1(self):
        """
        Description
        -----------
        This test verifies that solve_pose_ik_problem raises a helpful error
        for a pose that the arm cannot reach.
        """
        production = CornellTech1(meshcat_port_number=None)
        production.n_ik_initial_guesses = 3
        production.station.Finalize()

        unreachable = RigidTransform(RollPitchYaw(np.pi, 0.0, 0.0), [3.0, 0.0, 0.5])
        with self.assertRaisesRegex(RuntimeError, "after 3 initial guesses"):
            production.solve_pose_ik_problem(unreachable)

    def test_goal_pose1(self):
        """
        Description
        -----------
        This test verifies that goal_pose does not raise an error when the
        production is given a goal configuration (instead of a goal pose).
        """
        production = CornellTech1(
            meshcat_port_number=None,
            goal_configuration=np.zeros((7,)),
        )
        goal_pose = production.goal_pose

        # At the zero configuration, the Gen3 points straight up
        self.assertGreater(goal_pose.translation()[2], 1.0)

    def test_start_pose1(self):
        """
        Description
        -----------
        This test verifies that start_pose does not raise an error when the
        production is given a start configuration (instead of a start pose).
        """
        production = CornellTech1(
            meshcat_port_number=None,
            start_configuration=np.zeros((7,)),
        )
        start_pose = production.start_pose
        self.assertGreater(start_pose.translation()[2], 1.0)

    def test_id1(self):
        """
        Description
        -----------
        This test verifies that CornellTech1 is using the correct ID.
        """
        production = CornellTech1(meshcat_port_number=None)
        self.assertEqual(production.id, ProductionID.kCornellTech1)


if __name__ == "__main__":
    unittest.main()

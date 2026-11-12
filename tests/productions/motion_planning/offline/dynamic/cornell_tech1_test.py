import numpy as np
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

"""
Tests for brom_drake.robots.gripper_geometry: the finger pad table and the
grasp point computed from it.
"""

from importlib import resources as impresources
import unittest

import numpy as np
from pydrake.all import MultibodyPlant, Parser

from brom_drake import robots
from brom_drake.all import (
    GRIPPER_FINGER_PADS,
    GraspSelection1,
    GripperController,
    GripperFingerPads,
    GripperType,
)
from brom_drake.robots import calc_grasp_point_in_world, get_gripper_finger_pads

ROBOTIQ_2F_85_URDF = str(
    impresources.files(robots) / "models/robotiq/2f_85_gripper/urdf/robotiq_2f_85.urdf"
)


def robotiq_2f_85_welded_at_origin():
    """A plant with the 2F-85's base link welded to the world at the identity."""
    plant = MultibodyPlant(time_step=0.0)
    gripper = Parser(plant).AddModels(ROBOTIQ_2F_85_URDF)[0]
    pads = GRIPPER_FINGER_PADS[GripperType.Robotiq_2f_85]
    plant.WeldFrames(plant.world_frame(), plant.GetFrameByName(pads.base_link, gripper))
    plant.Finalize()
    return plant, plant.CreateDefaultContext(), gripper


class GripperFingerPadsTableTest(unittest.TestCase):
    def test_robotiq_2f_85_entry1(self):
        pads = GRIPPER_FINGER_PADS[GripperType.Robotiq_2f_85]
        self.assertIsInstance(pads, GripperFingerPads)
        self.assertEqual(pads.base_link, "robotiq_arg2f_base_link")
        self.assertEqual(
            pads.pad_bodies, ("left_inner_finger_pad", "right_inner_finger_pad")
        )
        self.assertEqual(pads.closing_axis, "y")

    def test_names_exist_in_the_model1(self):
        plant, _, gripper = robotiq_2f_85_welded_at_origin()
        pads = GRIPPER_FINGER_PADS[GripperType.Robotiq_2f_85]
        for name in (pads.base_link, *pads.pad_bodies):
            with self.subTest(name=name):
                self.assertTrue(plant.HasBodyNamed(name, gripper))

    def test_no_gripper_has_no_entry1(self):
        with self.assertRaisesRegex(ValueError, "NoGripper"):
            get_gripper_finger_pads(GripperType.NoGripper)


class CalcGraspPointInWorldTest(unittest.TestCase):
    def test_open_gripper_matches_the_table1(self):
        plant, context, gripper = robotiq_2f_85_welded_at_origin()
        p = calc_grasp_point_in_world(
            plant, context, gripper, GripperType.Robotiq_2f_85
        )
        expected = GRIPPER_FINGER_PADS[GripperType.Robotiq_2f_85].p_BaseGraspPoint_open
        np.testing.assert_allclose(p, expected, atol=1e-3)

    def test_closed_gripper_stays_on_the_approach_axis1(self):
        plant, context, gripper = robotiq_2f_85_welded_at_origin()
        p_open = calc_grasp_point_in_world(
            plant, context, gripper, GripperType.Robotiq_2f_85
        )

        controller = GripperController(GripperType.Robotiq_2f_85)
        q_closed = controller.find_joint_positions_for_position_target(0.9)
        plant.SetPositions(context, gripper, q_closed)
        p_closed = calc_grasp_point_in_world(
            plant, context, gripper, GripperType.Robotiq_2f_85
        )

        # The pads really moved...
        left = plant.GetBodyByName("left_inner_finger_pad", gripper)
        self.assertGreater(
            plant.EvalBodyPoseInWorld(context, left).translation()[1], -0.04
        )
        # ...but their midpoint stays on the base's z axis. The linkage swings the
        # pads away from the base as they close: about 14 mm at command 0.9
        # (z = 0.1444 m), which is why the grasp point is read from the live pads.
        np.testing.assert_allclose(p_closed[:2], [0.0, 0.0], atol=1e-6)
        self.assertGreater(p_closed[2], p_open[2])
        self.assertLess(p_closed[2] - p_open[2], 0.015)

    def test_no_gripper_raises1(self):
        plant, context, gripper = robotiq_2f_85_welded_at_origin()
        with self.assertRaisesRegex(ValueError, "NoGripper"):
            calc_grasp_point_in_world(plant, context, gripper, GripperType.NoGripper)


class GraspSelection1AccessorsBeforeBuildTest(unittest.TestCase):
    def test_accessors_raise_before_build1(self):
        production = GraspSelection1(
            path_to_object=str(
                impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
            ),
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
        )
        self.assertEqual(production.gripper_choice, GripperType.Robotiq_2f_85)
        for method in (
            production.calc_grasp_point_in_world,
            production.calc_object_center_of_mass_in_world,
            production.calc_object_spatial_velocity_in_world,
        ):
            with self.subTest(method=method.__name__):
                with self.assertRaisesRegex(RuntimeError, "easy_cast_and_build"):
                    method()


if __name__ == "__main__":
    unittest.main()

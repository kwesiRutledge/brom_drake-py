import unittest

import numpy as np
from pydrake.all import RigidTransform, RollPitchYaw, RotationMatrix

from brom_drake.utils.puppetmaker import PuppetPoseController
from brom_drake.utils.puppetmaker.pose_controller import (
    rotation_error_vector,
    yaw_pitch_roll_jacobian,
)

KP = np.array([100.0, 100.0, 100.0, 10.0, 10.0, 6.0])
KD = np.array([10.0, 10.0, 10.0, 3.0, 3.0, 2.0])


def generalized_force(
    controller: PuppetPoseController,
    X_desired: RigidTransform,
    q: np.ndarray,
    v: np.ndarray = np.zeros(6),
) -> np.ndarray:
    context = controller.CreateDefaultContext()
    controller.get_input_port_desired_pose().FixValue(context, X_desired)
    controller.get_input_port_estimated_state().FixValue(
        context, np.concatenate([q, v])
    )
    return controller.get_output_port().Eval(context)


def ypr_state(roll: float, pitch: float, yaw: float, xyz=(0.0, 0.0, 0.0)):
    """Joint positions [x, y, z, yaw, pitch, roll]."""
    return np.array([*xyz, yaw, pitch, roll])


class YawPitchRollJacobianTest(unittest.TestCase):
    def test_matches_drake1(self):
        rng = np.random.default_rng(0)
        for _ in range(20):
            roll, pitch, yaw = rng.uniform(-np.pi, np.pi, 3)
            yaw_dot, pitch_dot, roll_dot = rng.normal(size=3)
            expected = RollPitchYaw(
                roll, pitch, yaw
            ).CalcAngularVelocityInParentFromRpyDt([roll_dot, pitch_dot, yaw_dot])
            np.testing.assert_allclose(
                yaw_pitch_roll_jacobian(yaw, pitch)
                @ np.array([yaw_dot, pitch_dot, roll_dot]),
                expected,
                atol=1e-12,
            )

    def test_loses_rank_at_gimbal_lock1(self):
        self.assertEqual(
            np.linalg.matrix_rank(yaw_pitch_roll_jacobian(0.3, np.pi / 2)), 2
        )


class RotationErrorVectorTest(unittest.TestCase):
    def test_small_rotation1(self):
        R = RotationMatrix.MakeZRotation(0.1)
        np.testing.assert_allclose(
            rotation_error_vector(R, RotationMatrix()), [0.0, 0.0, 0.1], atol=1e-12
        )

    def test_is_the_short_way_round1(self):
        # 350 degrees one way is 10 degrees the other way.
        R = RotationMatrix.MakeXRotation(np.deg2rad(350.0))
        np.testing.assert_allclose(
            rotation_error_vector(R, RotationMatrix()),
            [np.deg2rad(-10.0), 0.0, 0.0],
            atol=1e-12,
        )


class PuppetPoseControllerTest(unittest.TestCase):
    def setUp(self):
        self.controller = PuppetPoseController(Kp=KP, Kd=KD)

    def test_gains_must_have_six_entries1(self):
        with self.assertRaises(ValueError):
            PuppetPoseController(Kp=np.ones(5), Kd=np.ones(6))
        with self.assertRaises(ValueError):
            PuppetPoseController(Kp=KP, Kd=KD, singularity_damping=0.0)

    def test_zero_at_target1(self):
        X = RigidTransform(RollPitchYaw(0.3, -0.4, 0.7), [0.1, 0.2, 0.3])
        u = generalized_force(
            self.controller, X, ypr_state(0.3, -0.4, 0.7, (0.1, 0.2, 0.3))
        )
        np.testing.assert_allclose(u, np.zeros(6), atol=1e-12)

    def test_matches_joint_space_pd_for_small_errors1(self):
        """
        Away from gimbal lock, a small orientation error gives (almost) the same
        torques as a PD controller on the roll-pitch-yaw angles, so gains tuned
        for that controller keep their meaning.
        """
        q = ypr_state(0.3, -0.4, 0.7)
        delta = np.array([0.0, 0.0, 0.0, 0.002, -0.001, 0.003])  # yaw, pitch, roll
        target = q + delta
        X = RigidTransform(RollPitchYaw(target[5], target[4], target[3]), [0, 0, 0])
        u = generalized_force(self.controller, X, q)
        np.testing.assert_allclose(u[3:], KP[3:] * delta[3:], rtol=0.02, atol=1e-6)

    def test_roll_wrap_does_not_matter1(self):
        """
        A gripper pointing straight down has roll = +/-pi. The same rotation
        written with +pi or -pi, or reached by joints wound a full turn,
        gives the same torques.
        """
        q = ypr_state(np.pi - 0.01, 0.0, 0.0)
        X_plus = RigidTransform(RollPitchYaw(np.pi, 0.0, 0.0), [0, 0, 0])
        X_minus = RigidTransform(RollPitchYaw(-np.pi, 0.0, 0.0), [0, 0, 0])
        u_plus = generalized_force(self.controller, X_plus, q)
        u_minus = generalized_force(self.controller, X_minus, q)
        np.testing.assert_allclose(u_plus, u_minus, atol=1e-9)
        # The torque turns roll up by 0.01 rad (not down by 2*pi - 0.01).
        self.assertAlmostEqual(u_plus[5], KP[5] * 0.01, delta=1e-2 * KP[5] * 0.01)

        q_wound = q + np.array([0, 0, 0, 0, 0, 2 * np.pi])
        np.testing.assert_allclose(
            generalized_force(self.controller, X_plus, q_wound), u_plus, atol=1e-9
        )

    def test_both_rpy_descriptions_give_the_same_torques1(self):
        """
        (r, p, y) and (r + pi, pi - p, y + pi) are the same orientation, so the
        controller must not care which one the joints are at.
        """
        r, p, y = 0.1, 0.3 * np.pi, 0.1
        X = RigidTransform(RollPitchYaw(r, p + 0.01, y), [0, 0, 0])
        u1 = generalized_force(self.controller, X, ypr_state(r, p, y))
        u2 = generalized_force(
            self.controller, X, ypr_state(r + np.pi, np.pi - p, y + np.pi)
        )
        # Same orientation error, mapped through each branch's joints
        R1 = RollPitchYaw(r, p, y).ToRotationMatrix()
        R2 = RollPitchYaw(r + np.pi, np.pi - p, y + np.pi).ToRotationMatrix()
        self.assertTrue(R1.IsNearlyEqualTo(R2, 1e-12))
        J1 = yaw_pitch_roll_jacobian(y, p)
        J2 = yaw_pitch_roll_jacobian(y + np.pi, np.pi - p)
        # Both move the puppet toward the target with the same angular velocity
        np.testing.assert_allclose(
            J1 @ (u1[3:] / KP[3:]), J2 @ (u2[3:] / KP[3:]), atol=1e-6
        )

    def test_bounded_at_gimbal_lock1(self):
        q = ypr_state(0.2, np.pi / 2, -0.4)
        X = RigidTransform(RollPitchYaw(0.5, np.pi / 2 - 0.05, 0.1), [0, 0, 0])
        u = generalized_force(self.controller, X, q)
        self.assertTrue(np.all(np.isfinite(u)))
        self.assertLess(np.max(np.abs(u[3:] / KP[3:])), 1.0 / 0.05)

    def test_damping_uses_joint_velocities1(self):
        q = ypr_state(0.0, 0.0, 0.0)
        v = np.array([0.1, 0.0, 0.0, 0.2, 0.0, -0.3])
        u = generalized_force(self.controller, RigidTransform(), q, v)
        np.testing.assert_allclose(u, -KD * v, atol=1e-12)


if __name__ == "__main__":
    unittest.main()

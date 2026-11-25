"""
A pose controller for a Puppetmaker puppet that never compares Euler angles.

A puppet is moved by six actuated joints in a chain: translations along x, y
and z, then rotations about yaw (z), pitch (y') and roll (x''), so its
orientation is ``RollPitchYaw(roll, pitch, yaw)``. Comparing the desired and
current roll-pitch-yaw angles directly goes wrong in three ways:

1. Roll and yaw wrap at +/-pi, so a target near +/-pi can jump by 2*pi.
2. Every orientation has two roll-pitch-yaw descriptions, and the conversion
   switches between them when pitch crosses +/-pi/2.
3. At pitch = +/-pi/2 (gimbal lock), roll and yaw describe the same motion.

This controller compares orientations as rotations instead. Its orientation
error is the rotation that takes the current orientation to the desired one,
as an angle-axis vector, which is continuous everywhere and always the short
way round. The error is converted into joint-angle corrections with a damped
least-squares inverse of the chain's Jacobian, so per-joint gains keep the
meaning they had with a joint-space PID (for small errors away from gimbal
lock the two are the same), and the corrections stay bounded near gimbal lock.
"""

from typing import Sequence

import numpy as np
from pydrake.all import (
    AbstractValue,
    BasicVector,
    Context,
    InputPort,
    LeafSystem,
    OutputPort,
    RigidTransform,
    RollPitchYaw,
)

# Order of the puppet's joints in the controller's state, gain and output vectors.
PUPPET_JOINT_ORDER = ("x", "y", "z", "yaw", "pitch", "roll")

# Default damping (rad) of the least-squares inverse used near gimbal lock.
DEFAULT_SINGULARITY_DAMPING = 0.05


def yaw_pitch_roll_jacobian(yaw: float, pitch: float) -> np.ndarray:
    """
    *Description*

    The matrix ``J`` with ``w = J @ [yaw_dot, pitch_dot, roll_dot]``, where ``w``
    is the angular velocity of ``RollPitchYaw(roll, pitch, yaw)`` expressed in
    the parent frame. Its columns are the joint axes in the parent frame: z, then
    y rotated by yaw, then x rotated by yaw and pitch. It loses rank at
    ``pitch = +/-pi/2``.
    """
    cy, sy = np.cos(yaw), np.sin(yaw)
    cp, sp = np.cos(pitch), np.sin(pitch)
    return np.array(
        [
            [0.0, -sy, cy * cp],
            [0.0, cy, sy * cp],
            [1.0, 0.0, -sp],
        ]
    )


def rotation_error_vector(R_desired, R_current) -> np.ndarray:
    """
    *Description*

    The rotation that takes ``R_current`` to ``R_desired``, as an angle-axis
    vector (``angle * axis``, angle in ``[0, pi]``) expressed in the parent
    frame.
    """
    angle_axis = R_desired.multiply(R_current.inverse()).ToAngleAxis()
    return angle_axis.angle() * angle_axis.axis()


class PuppetPoseController(LeafSystem):
    """
    *Description*

    Computes the generalized forces for a puppet's six joints that move the
    puppet toward a desired pose, with zero desired velocity.

    - Translation: ``Kp * (p_desired - p) - Kd * v`` per axis.
    - Rotation: ``Kp * dq - Kd * q_dot`` per joint, where ``dq`` is the
      orientation error mapped to joint angles by a damped least-squares inverse
      of :func:`yaw_pitch_roll_jacobian`. ``dq`` depends only on the desired and
      current rotations, never on how their angles are written, so it does not
      wrap.

    *Input ports*

    - ``desired_pose``: an abstract ``RigidTransform``, the desired pose of the
      puppet's frame in the frame its joints are anchored to.
    - ``estimated_state``: the puppet's joint state,
      ``[x, y, z, yaw, pitch, roll]`` positions then velocities (12 values).

    *Output ports*

    - ``generalized_force``: forces and torques for the joints, in the order
      ``[x, y, z, yaw, pitch, roll]``.

    *Parameters*

    Kp, Kd: Sequence[float]
        Six gains each, in the order ``[x, y, z, yaw, pitch, roll]``
        (N/m and N*s/m for translation, N*m/rad and N*m*s/rad for rotation).

    singularity_damping: float, optional
        The damping (in rad) of the least-squares inverse. Larger values keep the
        rotational corrections smaller near gimbal lock at the cost of tracking
        accuracy there. Default is 0.05.
    """

    def __init__(
        self,
        Kp: Sequence[float],
        Kd: Sequence[float],
        singularity_damping: float = DEFAULT_SINGULARITY_DAMPING,
    ):
        LeafSystem.__init__(self)

        self.Kp = np.array(Kp, dtype=float).reshape(-1)
        self.Kd = np.array(Kd, dtype=float).reshape(-1)
        if self.Kp.shape != (6,) or self.Kd.shape != (6,):
            raise ValueError(
                "Expected Kp and Kd to have 6 entries each "
                + f"(ordered {list(PUPPET_JOINT_ORDER)}); received shapes "
                + f"{self.Kp.shape} and {self.Kd.shape}."
            )
        if singularity_damping <= 0.0:
            raise ValueError(
                f"Expected singularity_damping to be positive; received {singularity_damping}."
            )
        self.singularity_damping = float(singularity_damping)

        self._desired_pose_port: InputPort = self.DeclareAbstractInputPort(
            "desired_pose", AbstractValue.Make(RigidTransform())
        )
        self._estimated_state_port: InputPort = self.DeclareVectorInputPort(
            "estimated_state", BasicVector(12)
        )
        self._output_port: OutputPort = self.DeclareVectorOutputPort(
            "generalized_force", BasicVector(6), self.CalcGeneralizedForce
        )

    def get_input_port_desired_pose(self) -> InputPort:
        return self._desired_pose_port

    def get_input_port_estimated_state(self) -> InputPort:
        return self._estimated_state_port

    def get_output_port(self, index: int = 0) -> OutputPort:
        return self._output_port

    def orientation_error_in_joint_angles(
        self, R_desired, yaw: float, pitch: float, roll: float
    ) -> np.ndarray:
        """
        *Description*

        The orientation error as corrections to ``[yaw, pitch, roll]``:
        ``J^T (J J^T + damping^2 I)^-1 e``, where ``e`` is
        :func:`rotation_error_vector` and ``J`` is
        :func:`yaw_pitch_roll_jacobian`.
        """
        R_current = RollPitchYaw(roll, pitch, yaw).ToRotationMatrix()
        error = rotation_error_vector(R_desired, R_current)
        J = yaw_pitch_roll_jacobian(yaw, pitch)
        damped = J @ J.T + (self.singularity_damping**2) * np.eye(3)
        return J.T @ np.linalg.solve(damped, error)

    def CalcGeneralizedForce(self, context: Context, output: BasicVector):
        X_desired: RigidTransform = self._desired_pose_port.Eval(context)
        state = self._estimated_state_port.Eval(context)
        position, velocity = state[:6], state[6:]
        yaw, pitch, roll = position[3:6]

        position_error = np.zeros(6)
        position_error[:3] = X_desired.translation() - position[:3]
        position_error[3:] = self.orientation_error_in_joint_angles(
            X_desired.rotation(), yaw, pitch, roll
        )

        output.SetFromVector(self.Kp * position_error - self.Kd * velocity)

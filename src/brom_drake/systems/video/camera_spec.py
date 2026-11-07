from dataclasses import dataclass, field
from typing import Optional, Sequence, Tuple

import numpy as np
from pydrake.math import RigidTransform, RotationMatrix


@dataclass
class CameraSpec:
    """
    **Description**

    A lightweight description of a camera that should record a video of a simulation.

    The pose ``X_PC`` follows Drake's camera convention: the camera looks along its
    +Z axis, +X points to the right of the image and +Y points down the image.
    If you would rather specify where the camera is and what it should look at,
    use :py:meth:`CameraSpec.looking_at`.

    **Parameters**

    name: str
        A unique name for the camera. It is used to name the systems added to the
        diagram and the output video file (``<name>.mp4``).

    X_PC: RigidTransform, optional
        The pose of the camera in its parent frame (the world frame, unless
        ``parent_body_name`` is given). Default is the identity.

    width: int, optional
        Image width in pixels. Must be even (required by H.264). Default is 640.

    height: int, optional
        Image height in pixels. Must be even (required by H.264). Default is 480.

    fov_y: float, optional
        Vertical field of view in radians. Default is pi/4.

    fps: float, optional
        Frames per second of *simulation time* to capture. Default is 30.0.

    parent_body_name: str, optional
        If given, the camera is rigidly attached to this body of the production's
        plant (e.g., a wrist camera). Default is None (camera fixed in the world).

    clipping: Tuple[float, float], optional
        The (near, far) clipping distances in meters. Default is (0.01, 10.0).

    **Usage**

    ::

        from brom_drake.all import CameraSpec

        front = CameraSpec.looking_at(
            "front",
            position=[1.5, 0.0, 1.0],
            target=[0.0, 0.0, 0.3],
        )
    """

    name: str
    X_PC: RigidTransform = field(default_factory=RigidTransform)
    width: int = 640
    height: int = 480
    fov_y: float = np.pi / 4.0
    fps: float = 30.0
    parent_body_name: Optional[str] = None
    clipping: Tuple[float, float] = (0.01, 10.0)

    def __post_init__(self):
        # Validate inputs early so that errors appear before the diagram is built
        if not self.name:
            raise ValueError("CameraSpec.name must be a non-empty string.")

        if self.width <= 0 or self.height <= 0:
            raise ValueError(
                f"CameraSpec '{self.name}' must have a positive width and height; "
                f"received ({self.width}, {self.height})."
            )

        if self.width % 2 != 0 or self.height % 2 != 0:
            raise ValueError(
                f"CameraSpec '{self.name}' must have an even width and height "
                f"(required by the H.264 encoder); received ({self.width}, {self.height})."
            )

        if self.fps <= 0:
            raise ValueError(
                f"CameraSpec '{self.name}' must have a positive fps; received {self.fps}."
            )

        if not (0.0 < self.fov_y < np.pi):
            raise ValueError(
                f"CameraSpec '{self.name}' must have fov_y in (0, pi); received {self.fov_y}."
            )

        near, far = self.clipping
        if not (0.0 < near < far):
            raise ValueError(
                f"CameraSpec '{self.name}' must satisfy 0 < near < far for its clipping "
                f"distances; received {self.clipping}."
            )

    @property
    def frame_period(self) -> float:
        """
        **Description**

        The simulation time (in seconds) between two captured frames.
        """
        return 1.0 / self.fps

    @classmethod
    def looking_at(
        cls,
        name: str,
        position: Sequence[float],
        target: Sequence[float],
        up: Sequence[float] = (0.0, 0.0, 1.0),
        **kwargs,
    ) -> "CameraSpec":
        """
        **Description**

        Creates a CameraSpec for a camera placed at ``position`` that looks at
        ``target``, with ``up`` pointing (approximately) toward the top of the image.
        All vectors are expressed in the parent frame (the world, by default).

        **Parameters**

        name: str
            The name of the camera.

        position: Sequence[float]
            The 3D position of the camera.

        target: Sequence[float]
            The 3D point that the camera should look at.

        up: Sequence[float], optional
            The direction that should appear "up" in the image. Default is +Z.

        kwargs:
            Any other CameraSpec fields (e.g., ``width``, ``fps``).

        **Returns**

        spec: CameraSpec
            The camera specification.
        """
        X_PC = cls.pose_looking_at(position, target, up)
        return cls(name=name, X_PC=X_PC, **kwargs)

    @staticmethod
    def pose_looking_at(
        position: Sequence[float],
        target: Sequence[float],
        up: Sequence[float] = (0.0, 0.0, 1.0),
    ) -> RigidTransform:
        """
        **Description**

        Computes the pose of a Drake camera (+Z forward, +X right, +Y down)
        located at ``position`` and looking at ``target``.
        """
        p = np.asarray(position, dtype=float).reshape(3)
        t = np.asarray(target, dtype=float).reshape(3)
        u = np.asarray(up, dtype=float).reshape(3)

        forward = t - p
        if np.linalg.norm(forward) < 1e-9:
            raise ValueError("Camera position and target must be different points.")
        forward /= np.linalg.norm(forward)

        right = np.cross(forward, u)
        if np.linalg.norm(right) < 1e-9:
            raise ValueError(
                "The camera's 'up' direction must not be parallel to its viewing direction."
            )
        right /= np.linalg.norm(right)

        down = np.cross(forward, right)

        R = RotationMatrix(np.column_stack([right, down, forward]))
        return RigidTransform(R, p)

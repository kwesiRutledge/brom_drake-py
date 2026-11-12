from importlib import resources as impresources
from pydrake.all import RigidTransform

# Local imports
from brom_drake import robots
from brom_drake.file_manipulation.urdf import DrakeReadyURDFConverter
from brom_drake.stations.classical.ur10e import UR10eStation


class KinovaGen3Station(UR10eStation):
    """
    *Description*

    A template system diagram for controlling a 7-DoF Kinova Gen3 arm
    in a simulated environment. It is identical to the ``UR10eStation``
    (same inverse dynamics arm controller, same optional Robotiq 2F-85 gripper),
    but uses the Kinova Gen3 model that ships with brom_drake.

    The arm's base (``base_link``) is welded to the world origin and the
    gripper (if any) is welded to the arm's tool flange (``end_effector_link``).

    *Diagram*

    The diagram's inputs and outputs can be represented as: ::

                                            -----------------
                                            |               |
        gripper_target -------------------->|               |
        (Vector, np.array)                  |   Kinova      |
                                            |   Gen3        |
        gripper_target_type --------------->|   Station     |
        (Abstract, GripperTarget Enum)      |               |
                                            |               |
        desired_joint_positions ----------->|               |
        (Vector, np.array)                  |               |
                                            |               |
                                            -----------------

    The arm-related output ports are prefixed with ``kinova_gen3``
    (e.g., ``kinova_gen3_arm_position_measured``).
    """

    station_name: str = "KinovaGen3_Station"
    arm_port_prefix: str = "kinova_gen3"

    def AddArm(self):
        """
        *Description*

        Add the Kinova Gen3 arm to the station's plant and to its controller plant.
        """
        # Setup
        arm_urdf_path = str(
            impresources.files(robots) / "models/kinova/gen3_7dof/gen3_7dof.urdf",
        )
        arm_urdf = str(DrakeReadyURDFConverter(arm_urdf_path).convert_urdf())

        # The tool flange of the Gen3 (z-axis points out of the flange)
        self.end_effector_frame_name = "end_effector_link"

        # The "end_effector" frame sits roughly between the gripper's fingertips
        self.X_ee = RigidTransform()
        self.X_ee.set_translation([0, 0, 0.13])

        # Add the arm to the plant and to the controller plant
        self.arm, self.arm_ee_frame = self.add_arm_to_plant_with_ee_frame(
            arm_urdf,
            self.plant,
            X_ee=self.X_ee,
        )
        self.controller_arm, self.controller_arm_ee_frame = (
            self.add_arm_to_plant_with_ee_frame(
                arm_urdf,
                self.controller_plant,
                X_ee=self.X_ee,
            )
        )

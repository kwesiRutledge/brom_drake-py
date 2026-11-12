from enum import Enum
from importlib import resources as impresources
from pydrake.all import RigidTransform

# Local imports
from brom_drake import robots
from brom_drake.file_manipulation.urdf import DrakeReadyURDFConverter
from brom_drake.robots.gripper_type import GripperType
from brom_drake.stations.classical.ur10e import UR10eStation


class KinovaGen3Variant(Enum):
    """
    *Description*

    The variants of the Kinova Gen3 arm that brom_drake has models for.
    """

    #: The 7 degree-of-freedom Gen3 (the most common variant)
    kSevenDof = "gen3_7dof"

    #: The 6 degree-of-freedom Gen3
    kSixDof = "gen3_6dof"

    @property
    def urdf_path(self) -> str:
        """
        *Description*

        The path to this variant's URDF file (in brom_drake's robot models).
        """
        return str(
            impresources.files(robots) / f"models/kinova/{self.value}/{self.value}.urdf"
        )

    @property
    def num_dofs(self) -> int:
        """
        *Description*

        The number of joints (and actuators) in this variant of the arm.
        """
        return {KinovaGen3Variant.kSevenDof: 7, KinovaGen3Variant.kSixDof: 6}[self]


class KinovaGen3Station(UR10eStation):
    """
    *Description*

    A template system diagram for controlling a Kinova Gen3 arm (6-DoF or 7-DoF)
    in a simulated environment. It is identical to the ``UR10eStation``
    (same inverse dynamics arm controller, same optional Robotiq 2F-85 gripper),
    but uses the Kinova Gen3 models that ship with brom_drake.

    The arm's base (``base_link``) is welded to the world origin and the
    gripper (if any) is welded to the arm's tool flange (``end_effector_link``).

    *Parameters*

    time_step: float, optional
        The time step of the station's plants, by default 0.002.

    meshcat_port_number: int, optional
        The port number for meshcat visualization, by default 7001.
        Use None to disable meshcat.

    gripper_type: GripperType, optional
        The type of gripper to use, by default GripperType.Robotiq_2f_85.

    variant: KinovaGen3Variant, optional
        Which Gen3 arm to use, by default KinovaGen3Variant.kSevenDof.

    model_gripper_in_controller: bool, optional
        Whether the arm controller's model of the robot includes the gripper
        (as a rigid load on the arm's flange), by default True.

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
    (e.g., ``kinova_gen3_arm_position_measured``), for both variants.
    The size of ``desired_joint_positions`` (and of the arm's measured
    positions and velocities) is ``variant.num_dofs``.
    """

    station_name: str = "KinovaGen3_Station"
    arm_port_prefix: str = "kinova_gen3"

    def __init__(
        self,
        time_step: float = 0.002,
        meshcat_port_number: int = 7001,
        gripper_type: GripperType = GripperType.Robotiq_2f_85,
        variant: KinovaGen3Variant = KinovaGen3Variant.kSevenDof,
        model_gripper_in_controller: bool = True,
    ):
        # The variant must be known before the parent constructor calls AddArm()
        self.variant = variant

        super().__init__(
            time_step=time_step,
            meshcat_port_number=meshcat_port_number,
            gripper_type=gripper_type,
            model_gripper_in_controller=model_gripper_in_controller,
        )

    def AddArm(self):
        """
        *Description*

        Add the Kinova Gen3 arm (of the station's variant) to the station's plant
        and to its controller plant.
        """
        # Setup
        arm_urdf = str(DrakeReadyURDFConverter(self.variant.urdf_path).convert_urdf())

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

"""
Description
-----------
Tests that the classical stations (UR10e and Kinova Gen3) include the gripper
in their arm controller's model of the robot, and that doing so makes the
controller compensate for the gripper's weight.
"""

from typing import Callable, List, Tuple

import numpy as np
from pydrake.all import (
    AbstractValue,
    ConstantValueSource,
    ConstantVectorSource,
    DiagramBuilder,
    Simulator,
)
import unittest

# Internal imports
from brom_drake.control import GripperTarget
from brom_drake.robots.gripper_type import GripperType
from brom_drake.stations.classical import (
    KinovaGen3Station,
    KinovaGen3Variant,
    UR10eStation,
)

GRIPPER_BASE_FRAME = "robotiq_arg2f_base_link"


def station_factories() -> List[Tuple[str, Callable[..., UR10eStation], np.ndarray]]:
    """
    Returns (name, factory, q_hold) for each classical station, where factory(**kwargs)
    creates the station (without meshcat) and q_hold is a configuration in which the
    gripper's weight loads the arm's joints.
    """
    return [
        (
            "UR10e",
            lambda **kw: UR10eStation(meshcat_port_number=None, **kw),
            np.array([0.0, -1.2, 1.2, -1.0, -1.57, 0.0]),
        ),
        (
            "Gen3-7dof",
            lambda **kw: KinovaGen3Station(
                meshcat_port_number=None, variant=KinovaGen3Variant.kSevenDof, **kw
            ),
            np.array([0.0, 0.5, 0.0, 1.5, 0.0, 1.0, 0.0]),
        ),
        (
            "Gen3-6dof",
            lambda **kw: KinovaGen3Station(
                meshcat_port_number=None, variant=KinovaGen3Variant.kSixDof, **kw
            ),
            np.array([0.0, 0.5, 1.5, 0.0, 1.0, 0.0]),
        ),
    ]


def random_arm_configurations(
    station: UR10eStation, n: int, seed: int = 0
) -> List[np.ndarray]:
    """Samples n arm configurations within the arm's joint limits."""
    rng = np.random.default_rng(seed)
    lower = station.plant.GetPositionLowerLimits()
    upper = station.plant.GetPositionUpperLimits()
    lower = station.plant.GetPositionsFromArray(station.arm, lower)
    upper = station.plant.GetPositionsFromArray(station.arm, upper)
    return [rng.uniform(lower, upper) for _ in range(n)]


def simulate_holding(
    station: UR10eStation, q_hold: np.ndarray, duration: float = 2.0
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Commands the (finalized) station to hold q_hold with an open gripper,
    starting at q_hold, and returns the arm's measured position and the
    measured end effector wrench after `duration` seconds.
    """
    builder = DiagramBuilder()
    builder.AddSystem(station)

    q_source = builder.AddSystem(ConstantVectorSource(q_hold))
    builder.Connect(
        q_source.get_output_port(), station.GetInputPort("desired_joint_positions")
    )
    type_source = builder.AddSystem(
        ConstantValueSource(AbstractValue.Make(GripperTarget.kPosition))
    )
    builder.Connect(
        type_source.get_output_port(), station.GetInputPort("gripper_target_type")
    )
    target_source = builder.AddSystem(ConstantVectorSource(np.array([0.0])))
    builder.Connect(
        target_source.get_output_port(), station.GetInputPort("gripper_target")
    )

    diagram = builder.Build()
    diagram_context = diagram.CreateDefaultContext()
    station.plant.SetPositions(
        station.plant.GetMyMutableContextFromRoot(diagram_context),
        station.arm,
        q_hold,
    )

    simulator = Simulator(diagram, diagram_context)
    simulator.AdvanceTo(duration)

    station_context = station.GetMyContextFromRoot(simulator.get_context())
    q = station.GetOutputPort(f"{station.arm_port_prefix}_arm_position_measured").Eval(
        station_context
    )
    wrench = station.GetOutputPort("measured_ee_wrench").Eval(station_context)
    return q, wrench


class TestControllerGripperModelStructure(unittest.TestCase):
    """Tests of the contents of the controller plant (no simulation)."""

    def test_controller_plant_contains_gripper1(self):
        """
        With the default settings, the controller plant contains a static gripper,
        and its degrees of freedom and actuators are still exactly the arm's.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()

                self.assertIsNotNone(station.controller_gripper)
                self.assertTrue(
                    station.controller_plant.HasFrameNamed(GRIPPER_BASE_FRAME)
                )
                self.assertEqual(
                    station.controller_plant.num_positions(station.controller_gripper),
                    0,
                )

                n_arm = station.plant.num_positions(station.arm)
                self.assertEqual(station.controller_plant.num_positions(), n_arm)
                self.assertEqual(station.controller_plant.num_velocities(), n_arm)
                self.assertEqual(station.controller_plant.num_actuated_dofs(), n_arm)

    def test_flag_disables_controller_gripper1(self):
        """
        With model_gripper_in_controller=False, the controller plant only contains the arm.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make(model_gripper_in_controller=False)
                station.Finalize()

                self.assertFalse(station.model_gripper_in_controller)
                self.assertIsNone(station.controller_gripper)
                self.assertFalse(
                    station.controller_plant.HasFrameNamed(GRIPPER_BASE_FRAME)
                )

    def test_no_gripper1(self):
        """
        Without a gripper, the controller plant does not contain a gripper either,
        and its mass is the arm's mass.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make(gripper_type=GripperType.NoGripper)
                station.Finalize()

                self.assertIsNone(station.controller_gripper)
                self.assertFalse(
                    station.controller_plant.HasFrameNamed(GRIPPER_BASE_FRAME)
                )

                plant_mass = station.plant.CalcTotalMass(
                    station.plant.CreateDefaultContext(), [station.arm]
                )
                controller_mass = station.controller_plant.CalcTotalMass(
                    station.controller_plant.CreateDefaultContext()
                )
                self.assertAlmostEqual(controller_mass, plant_mass, places=9)

    def test_total_mass_matches1(self):
        """
        The controller plant's total mass is the mass of the arm and the gripper
        in the simulated plant.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()

                plant_mass = station.plant.CalcTotalMass(
                    station.plant.CreateDefaultContext(),
                    [station.arm, station.gripper],
                )
                controller_mass = station.controller_plant.CalcTotalMass(
                    station.controller_plant.CreateDefaultContext()
                )
                self.assertAlmostEqual(controller_mass, plant_mass, places=9)

                # The gripper should be a noticeable part of that mass
                gripper_mass = station.plant.CalcTotalMass(
                    station.plant.CreateDefaultContext(), [station.gripper]
                )
                self.assertGreater(gripper_mass, 0.3)

    def test_gripper_pose_matches1(self):
        """
        For random arm configurations, the gripper's base is at the same pose
        in the controller plant and in the simulated plant.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()
                plant, controller_plant = station.plant, station.controller_plant
                plant_context = plant.CreateDefaultContext()
                controller_context = controller_plant.CreateDefaultContext()

                for q in random_arm_configurations(station, n=10):
                    plant.SetPositions(plant_context, station.arm, q)
                    controller_plant.SetPositions(controller_context, q)

                    X_plant = plant.GetFrameByName(
                        GRIPPER_BASE_FRAME, station.gripper
                    ).CalcPoseInWorld(plant_context)
                    X_controller = controller_plant.GetFrameByName(
                        GRIPPER_BASE_FRAME, station.controller_gripper
                    ).CalcPoseInWorld(controller_context)

                    self.assertTrue(X_plant.IsNearlyEqualTo(X_controller, 1e-12))

    def test_end_effector_frame_matches1(self):
        """
        The controller plant's "end_effector" frame (used by the wrench calculator)
        is at the same pose as the simulated plant's.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()
                plant, controller_plant = station.plant, station.controller_plant
                plant_context = plant.CreateDefaultContext()
                controller_context = controller_plant.CreateDefaultContext()

                for q in random_arm_configurations(station, n=5, seed=1):
                    plant.SetPositions(plant_context, station.arm, q)
                    controller_plant.SetPositions(controller_context, q)

                    X_plant = plant.GetFrameByName(
                        "end_effector", station.arm
                    ).CalcPoseInWorld(plant_context)
                    X_controller = controller_plant.GetFrameByName(
                        "end_effector", station.controller_arm
                    ).CalcPoseInWorld(controller_context)

                    self.assertTrue(X_plant.IsNearlyEqualTo(X_controller, 1e-12))

    def test_gravity_matches1(self):
        """
        For random arm configurations (with the gripper open), the gravity forces
        on the arm's joints are the same in the controller plant and in the
        simulated plant. This is what lets the controller cancel the gripper's weight.
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()
                plant, controller_plant = station.plant, station.controller_plant
                plant_context = plant.CreateDefaultContext()
                controller_context = controller_plant.CreateDefaultContext()

                for q in random_arm_configurations(station, n=10, seed=2):
                    plant.SetPositions(plant_context, station.arm, q)
                    controller_plant.SetPositions(controller_context, q)

                    tau_g_plant = plant.GetVelocitiesFromArray(
                        station.arm,
                        plant.CalcGravityGeneralizedForces(plant_context),
                    )
                    tau_g_controller = controller_plant.CalcGravityGeneralizedForces(
                        controller_context
                    )

                    np.testing.assert_allclose(
                        tau_g_controller, tau_g_plant, rtol=1e-9, atol=1e-9
                    )

    def test_gravity_differs_without_gripper_model1(self):
        """
        Sanity check for the test above: when the controller plant does not
        contain the gripper, its gravity forces differ from the simulated plant's.
        """
        for name, make, q_hold in station_factories():
            with self.subTest(station=name):
                station = make(model_gripper_in_controller=False)
                station.Finalize()
                plant, controller_plant = station.plant, station.controller_plant
                plant_context = plant.CreateDefaultContext()
                controller_context = controller_plant.CreateDefaultContext()

                plant.SetPositions(plant_context, station.arm, q_hold)
                controller_plant.SetPositions(controller_context, q_hold)

                tau_g_plant = plant.GetVelocitiesFromArray(
                    station.arm, plant.CalcGravityGeneralizedForces(plant_context)
                )
                tau_g_controller = controller_plant.CalcGravityGeneralizedForces(
                    controller_context
                )
                self.assertGreater(np.max(np.abs(tau_g_controller - tau_g_plant)), 0.1)

    def test_arm_mass_matrix_matches1(self):
        """
        For random arm configurations, the controller plant's mass matrix equals
        the arm block of the simulated plant's mass matrix (i.e., the arm's inertia
        with the gripper's joints held still).
        """
        for name, make, _ in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()
                plant, controller_plant = station.plant, station.controller_plant
                plant_context = plant.CreateDefaultContext()
                controller_context = controller_plant.CreateDefaultContext()

                # Indices of the arm's velocities in the simulated plant
                arm_v_indices = plant.GetVelocitiesFromArray(
                    station.arm, np.arange(plant.num_velocities())
                ).astype(int)

                for q in random_arm_configurations(station, n=5, seed=3):
                    plant.SetPositions(plant_context, station.arm, q)
                    controller_plant.SetPositions(controller_context, q)

                    M_plant = plant.CalcMassMatrix(plant_context)
                    M_arm = M_plant[np.ix_(arm_v_indices, arm_v_indices)]
                    M_controller = controller_plant.CalcMassMatrix(controller_context)

                    np.testing.assert_allclose(
                        M_controller, M_arm, rtol=1e-9, atol=1e-9
                    )


class TestControllerGripperModelBehavior(unittest.TestCase):
    """Tests of the closed-loop behavior of the stations (with simulation)."""

    def test_holds_configuration1(self):
        """
        With the gripper in the controller's model, every station holds a
        configuration that loads the arm with the gripper's weight (almost) exactly.
        """
        for name, make, q_hold in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()
                q, _ = simulate_holding(station, q_hold)
                np.testing.assert_allclose(q, q_hold, atol=1e-3)

    def test_sags_without_gripper_model1(self):
        """
        Without the gripper in the controller's model, the 6-DoF Gen3's wrist
        sags noticeably under the gripper's weight (this is the behavior the
        gripper model fixes).
        """
        name, make, q_hold = station_factories()[2]
        station = make(model_gripper_in_controller=False)
        station.Finalize()
        q, _ = simulate_holding(station, q_hold)
        self.assertGreater(np.max(np.abs(q - q_hold)), 0.03)

    def test_wrench_at_rest1(self):
        """
        With the gripper in the controller's model, the measured end effector
        wrench is (almost) zero when nothing touches the gripper.
        """
        for name, make, q_hold in station_factories():
            with self.subTest(station=name):
                station = make()
                station.Finalize()
                _, wrench = simulate_holding(station, q_hold)
                self.assertLess(np.linalg.norm(wrench[3:]), 0.05)  # force (N)
                self.assertLess(np.linalg.norm(wrench[:3]), 0.01)  # torque (Nm)

    def test_wrench_at_rest_without_gripper_model1(self):
        """
        Without the gripper in the controller's model, the measured end effector
        force at rest is the gripper's weight.
        """
        for name, make, q_hold in station_factories():
            with self.subTest(station=name):
                station = make(model_gripper_in_controller=False)
                station.Finalize()
                gripper_weight = 9.81 * station.plant.CalcTotalMass(
                    station.plant.CreateDefaultContext(), [station.gripper]
                )
                _, wrench = simulate_holding(station, q_hold)
                self.assertAlmostEqual(
                    np.linalg.norm(wrench[3:]), gripper_weight, delta=0.05
                )

    def test_holds_configuration_without_gripper1(self):
        """
        Without a gripper, every station still holds its configuration.
        """
        for name, make, q_hold in station_factories():
            with self.subTest(station=name):
                station = make(gripper_type=GripperType.NoGripper)
                station.Finalize()

                builder = DiagramBuilder()
                builder.AddSystem(station)
                q_source = builder.AddSystem(ConstantVectorSource(q_hold))
                builder.Connect(
                    q_source.get_output_port(),
                    station.GetInputPort("desired_joint_positions"),
                )
                diagram = builder.Build()
                diagram_context = diagram.CreateDefaultContext()
                station.plant.SetPositions(
                    station.plant.GetMyMutableContextFromRoot(diagram_context),
                    station.arm,
                    q_hold,
                )
                simulator = Simulator(diagram, diagram_context)
                simulator.AdvanceTo(1.0)

                q = station.GetOutputPort(
                    f"{station.arm_port_prefix}_arm_position_measured"
                ).Eval(station.GetMyContextFromRoot(simulator.get_context()))
                np.testing.assert_allclose(q, q_hold, atol=1e-3)


if __name__ == "__main__":
    unittest.main()

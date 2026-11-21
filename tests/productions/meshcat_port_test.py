import unittest
from unittest.mock import Mock, patch

from brom_drake.productions.debug.show_me.show_me_this_model import ShowMeThisModel
from brom_drake.productions.motion_planning.offline import ChemLab1
from brom_drake.productions.types.base.base import BaseProduction
from brom_drake.productions.types.debug.basic_grasping import (
    BasicGraspingDebuggingProduction,
)
from brom_drake.stations.classical.kinova_gen3 import KinovaGen3Station
from brom_drake.stations.classical.ur10e import UR10eStation as ClassicalUR10eStation
from brom_drake.stations.kinematic.ur10e_station import (
    UR10eStation as KinematicUR10eStation,
)


class MeshcatPortTest(unittest.TestCase):
    def assert_negative_port_error(self, function):
        with self.assertRaises(ValueError) as ctx:
            function()
        message = str(ctx.exception)
        self.assertIn("meshcat_port_number", message)
        self.assertIn("-1", message)
        self.assertIn("None", message)

    def test_negative_constructor_ports(self):
        constructors = [
            lambda: ShowMeThisModel("unused.urdf", meshcat_port_number=-1),
            lambda: BasicGraspingDebuggingProduction(
                "unused_object.urdf",
                "unused_gripper.urdf",
                None,
                meshcat_port_number=-1,
            ),
            lambda: ChemLab1(meshcat_port_number=-1),
            lambda: ClassicalUR10eStation(meshcat_port_number=-1),
            lambda: KinematicUR10eStation(meshcat_port_number=-1),
            lambda: KinovaGen3Station(meshcat_port_number=-1),
        ]
        for constructor in constructors:
            with self.subTest(constructor=constructor):
                self.assert_negative_port_error(constructor)

    def test_production_port_assignment(self):
        production = BaseProduction()
        for port in (None, 0, 7001):
            production.meshcat_port_number = port
            self.assertEqual(production.meshcat_port_number, port)
        self.assert_negative_port_error(
            lambda: setattr(production, "meshcat_port_number", -1)
        )
        self.assertEqual(production.meshcat_port_number, 7001)

    def test_station_connect_rejects_negative_port_before_meshcat(self):
        stations = [
            (
                ClassicalUR10eStation,
                "brom_drake.stations.classical.ur10e.Meshcat",
            ),
            (
                KinematicUR10eStation,
                "brom_drake.stations.kinematic.ur10e_station.Meshcat",
            ),
        ]
        for station, meshcat_path in stations:
            with self.subTest(station=station), patch(meshcat_path) as meshcat:
                self.assert_negative_port_error(
                    lambda: station.ConnectToMeshcatVisualizer(Mock(), port=-1)
                )
                meshcat.assert_not_called()

    def test_station_connect_preserves_nonnegative_and_none_ports(self):
        stations = [
            (ClassicalUR10eStation, "brom_drake.stations.classical.ur10e"),
            (KinematicUR10eStation, "brom_drake.stations.kinematic.ur10e_station"),
        ]
        for station, module in stations:
            for port in (None, 0, 7001):
                with (
                    self.subTest(station=station, port=port),
                    patch(f"{module}.Meshcat") as meshcat,
                    patch(f"{module}.MeshcatVisualizer"),
                ):
                    station.ConnectToMeshcatVisualizer(Mock(), port=port)
                    meshcat.assert_called_once_with(port)


if __name__ == "__main__":
    unittest.main()

import logging
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

import networkx as nx
from pydrake.geometry import SceneGraph
from pydrake.systems.framework import DiagramBuilder

from brom_drake.file_manipulation.urdf.drake_ready_urdf_converter.config import (
    DrakeReadyURDFConverterConfig,
)
from brom_drake.file_manipulation.urdf.drake_ready_urdf_converter.converter import (
    DrakeReadyURDFConverter,
)
from brom_drake.file_manipulation.urdf.drake_ready_urdf_converter.file_manager import (
    DrakeReadyURDFConverterFileManager,
)
from brom_drake.file_manipulation.urdf.shapes.sphere import SphereDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import (
    SimpleShapeURDFDefinition,
)
from brom_drake.systems.network_fsm.config import NetworkXFSMConfig
from brom_drake.systems.network_fsm.networkx_fsm import NetworkXFSM
from brom_drake.systems.video.add_cameras import add_cameras_to_builder
from brom_drake.systems.video.camera_spec import CameraSpec
from brom_drake.stations.kinematic.ur10e_station import UR10eStation


class RuntimeOutputTest(unittest.TestCase):
    def setUp(self):
        directory = tempfile.TemporaryDirectory()
        self.addCleanup(directory.cleanup)
        self.root = Path(directory.name).resolve()

    def close_logger(self, logger):
        for handler in list(logger.handlers):
            handler.close()
            logger.removeHandler(handler)

    def test_file_manager_defaults_resolve_at_creation(self):
        for value, expected in [
            (None, Path("brom")),
            ("", Path("brom")),
            ("relative-output", Path("relative-output")),
            (str(self.root), self.root),
        ]:
            with self.subTest(value=value), patch.dict(os.environ):
                if value is None:
                    os.environ.pop("BROM_DIR", None)
                else:
                    os.environ["BROM_DIR"] = value
                manager = DrakeReadyURDFConverterFileManager(Path("robot.urdf"))
                os.environ["BROM_DIR"] = "changed"
                self.assertEqual(manager.models_directory, expected / "models")
                self.assertEqual(
                    manager.output_urdf_path,
                    expected / "models/robot/robot.drake.urdf",
                )

        explicit = self.root / "explicit"
        with patch.dict(os.environ, {"BROM_DIR": "ignored"}):
            manager = DrakeReadyURDFConverterFileManager(
                Path("robot.urdf"), models_directory=explicit
            )
            self.assertEqual(manager.models_directory, explicit)

    def test_converter_preserves_existing_output_and_appends_logs(self):
        original = self.root / "robot.urdf"
        original.write_text('<robot name="robot"><link name="base"/></robot>')
        model_dir = self.root / "models/robot"
        model_dir.mkdir(parents=True)
        output = model_dir / "robot.drake.urdf"
        output.write_text("existing model")
        log = model_dir / "conversion.log"
        log.write_text("existing log\n")
        with patch.dict(os.environ, {"BROM_DIR": str(self.root)}):
            first = DrakeReadyURDFConverter(
                str(original),
                DrakeReadyURDFConverterConfig(overwrite_old_models=True),
            )
            self.addCleanup(self.close_logger, first.logger)
            self.assertEqual(first.file_manager.models_directory, self.root / "models")
            second = DrakeReadyURDFConverter(str(original))
            self.addCleanup(self.close_logger, second.logger)
        self.assertEqual(output.read_text(), "existing model")
        self.assertTrue(log.read_text().startswith("existing log\n"))
        self.assertIsNot(first.logger, second.logger)
        self.assertEqual(len(first.logger.handlers), 1)
        self.assertEqual(len(second.logger.handlers), 1)

    def test_explicit_cleanup_only_removes_target_model(self):
        models_dir = self.root / "models"
        target = models_dir / "robot"
        unrelated = models_dir / "other"
        target.mkdir(parents=True)
        unrelated.mkdir()
        (target / "mesh.obj").write_text("target")
        marker = unrelated / "mesh.obj"
        marker.write_text("keep")
        manager = DrakeReadyURDFConverterFileManager(
            Path("robot.urdf"), models_directory=models_dir
        )
        manager.clean_up_models_dir()
        self.assertFalse(target.exists())
        self.assertEqual(marker.read_text(), "keep")

    def test_converter_resolves_new_root_and_honors_explicit_output(self):
        original = self.root / "robot.urdf"
        original.write_text('<robot name="robot"><link name="base"/></robot>')
        for subdirectory, explicit in [
            ("first", None),
            ("second", None),
            ("third", self.root / "explicit/converted.urdf"),
        ]:
            root = self.root / subdirectory
            expected = (
                explicit
                if explicit is not None
                else root / "models/robot/robot.drake.urdf"
            )
            log = expected.parent / "conversion.log"
            log.parent.mkdir(parents=True)
            log.write_text("old log\n")
            with patch.dict(os.environ, {"BROM_DIR": str(root)}):
                converter = DrakeReadyURDFConverter(
                    str(original),
                    DrakeReadyURDFConverterConfig(
                        output_urdf_file_path=explicit,
                        overwrite_old_logs=True,
                    ),
                )
            self.addCleanup(self.close_logger, converter.logger)
            self.assertEqual(converter.file_manager.output_urdf_path, expected)
            self.assertEqual(converter.file_manager.models_directory, root / "models")
            self.assertNotIn("old log", log.read_text())

    def test_simple_writer_captures_root_and_honors_explicit_path(self):
        with patch.dict(os.environ, {"BROM_DIR": str(self.root / "first")}):
            first = SimpleShapeURDFDefinition("sphere", SphereDefinition(0.1))
        with patch.dict(os.environ, {"BROM_DIR": str(self.root / "second")}):
            second = SimpleShapeURDFDefinition("sphere", SphereDefinition(0.1))
            self.assertEqual(
                first.write_to_file(), self.root / "first/models/shapes/sphere.urdf"
            )
            self.assertEqual(
                second.write_to_file(), self.root / "second/models/shapes/sphere.urdf"
            )
            explicit = self.root / "explicit.urdf"
            self.assertEqual(first.write_to_file(explicit), explicit)
            self.assertTrue(explicit.exists())

    def test_fsm_loggers_use_instance_roots(self):
        systems = []
        for subdirectory in ["first", "second"]:
            graph = nx.DiGraph()
            graph.add_node(0, outputs=[])
            with patch.dict(os.environ, {"BROM_DIR": str(self.root / subdirectory)}):
                system = NetworkXFSM(
                    graph, NetworkXFSMConfig(show_logs_in_terminal=False)
                )
            self.addCleanup(self.close_logger, system.logger)
            systems.append(system)
            handlers = [
                handler
                for handler in system.logger.handlers
                if isinstance(handler, logging.FileHandler)
            ]
            self.assertEqual(len(handlers), 1)
            self.assertEqual(
                Path(handlers[0].baseFilename),
                self.root / subdirectory / "networkx_fsm/network_fsm.log",
            )
        self.assertIsNot(systems[0].logger, systems[1].logger)

    def test_camera_default_resolves_each_call_and_explicit_path_wins(self):
        for subdirectory, explicit in [
            ("first", None),
            ("second", None),
            ("third", self.root / "explicit"),
        ]:
            builder = DiagramBuilder()
            scene_graph = builder.AddSystem(SceneGraph())
            with (
                patch.dict(os.environ, {"BROM_DIR": str(self.root / subdirectory)}),
                patch.object(SceneGraph, "HasRenderer", return_value=True),
            ):
                recorders = add_cameras_to_builder(
                    builder,
                    scene_graph,
                    [CameraSpec("camera", width=64, height=48)],
                    output_dir=explicit,
                )
            expected = (
                explicit
                if explicit is not None
                else self.root / subdirectory / "videos"
            )
            self.assertEqual(recorders[0].output_path, expected / "camera.mp4")

    def test_ur10e_station_cache_uses_runtime_root(self):
        for subdirectory in ["first", "second"]:
            root = self.root / subdirectory
            expected = root / "models/ur10e/ur10e.drake.urdf"
            expected.parent.mkdir(parents=True)
            expected.write_text("cached model")
            station = SimpleNamespace(
                force_conversion_of_original_urdf=False,
                plant=object(),
                controller_plant=object(),
                add_arm_to_plant_with_ee_frame=Mock(return_value=(object(), object())),
            )
            with (
                patch.dict(os.environ, {"BROM_DIR": str(root)}),
                patch(
                    "brom_drake.stations.kinematic.ur10e_station.DrakeReadyURDFConverter"
                ) as converter,
            ):
                UR10eStation.AddArm(station)
            converter.assert_not_called()
            calls = station.add_arm_to_plant_with_ee_frame.call_args_list
            self.assertEqual(len(calls), 2)
            self.assertTrue(all(call.args[0] == str(expected) for call in calls))

    def test_ur10e_station_does_not_reuse_stale_default_cache(self):
        root = self.root / "new-root"
        expected = root / "models/ur10e/ur10e.drake.urdf"
        station = SimpleNamespace(
            force_conversion_of_original_urdf=False,
            plant=object(),
            controller_plant=object(),
            add_arm_to_plant_with_ee_frame=Mock(return_value=(object(), object())),
        )
        with (
            patch.dict(os.environ, {"BROM_DIR": str(root)}),
            patch.object(
                Path,
                "exists",
                lambda path: path == Path("brom/models/ur10e/ur10e.drake.urdf"),
            ),
            patch(
                "brom_drake.stations.kinematic.ur10e_station.DrakeReadyURDFConverter"
            ) as converter,
        ):
            converter.return_value.convert_urdf.return_value = expected
            UR10eStation.AddArm(station)
        converter.assert_called_once()
        calls = station.add_arm_to_plant_with_ee_frame.call_args_list
        self.assertTrue(all(call.args[0] == str(expected) for call in calls))

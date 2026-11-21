import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from pydrake.systems.primitives import Integrator

from brom_drake.productions.types.base import BaseProduction
from brom_drake.watchers.diagram_watcher import DiagramWatcherOptions
from brom_drake.watchers.port_watcher.port_watcher_options import (
    PortWatcherPlottingOptions,
    PortWatcherRawDataOptions,
)


class ProductionOutputDirectoryTest(unittest.TestCase):
    def test_build_forwards_options_and_records_unique_metadata(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            options = DiagramWatcherOptions(
                base_directory=root / "custom-watchers",
                plotting_options=PortWatcherPlottingOptions(
                    file_format="svg", save_to_file=False
                ),
                raw_data_options=PortWatcherRawDataOptions(save_to_file=False),
            )
            with patch.dict(os.environ, {"BROM_DIR": str(root)}):
                productions = [BaseProduction(), BaseProduction()]
                for production in productions:
                    production.builder.AddSystem(Integrator(1))
                    diagram, context = production.build_production(
                        watcher_options=options
                    )
                    self.assertIs(production.watcher.diagram, diagram)
                    self.assertIs(production.watcher.diagram_context, context)
                    self.assertEqual(
                        production.watcher.options.base_directory.parent,
                        root / "custom-watchers",
                    )
                    self.assertEqual(
                        production.watcher.options.plotting_options.file_format, "svg"
                    )
                reports = list((root / "productions").glob("*.json"))
                self.assertEqual(len(reports), 2)
                self.assertEqual(
                    json.loads(reports[0].read_text())["num_performers"], 0
                )
                for production in productions:
                    production.watcher.diagram = None

    def test_add_cast_and_build_forwards_options(self):
        production = BaseProduction()
        options = DiagramWatcherOptions()
        with (
            patch.object(production, "add_supporting_cast"),
            patch.object(production, "add_main_cast"),
            patch.object(production, "build_production") as build,
        ):
            production.add_cast_and_build(watcher_options=options)
        self.assertIs(build.call_args.kwargs["watcher_options"], options)

    def test_production_resolves_output_root_at_construction(self):
        with TemporaryDirectory() as directory:
            root = Path(directory)
            for name in ("first", "second"):
                with patch.dict(os.environ, {"BROM_DIR": str(root / name)}):
                    production = BaseProduction()
                self.assertEqual(production._video_output_dir, root / name / "videos")
                production._record_metadata()
                self.assertEqual(
                    len(list((root / name / "productions").glob("*.json"))), 1
                )

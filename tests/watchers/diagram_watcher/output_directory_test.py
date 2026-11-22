from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import ConstantVectorSource, Integrator
from pydrake.systems.analysis import Simulator

from brom_drake.directories import get_brom_dir
from brom_drake.watchers.diagram_watcher import DiagramWatcher, DiagramWatcherOptions
from brom_drake.watchers.diagram_watcher.add_watcher import (
    add_watcher,
    add_watcher_and_build,
)
from brom_drake.watchers.port_watcher.port_watcher_options import (
    PortWatcherPlottingOptions,
)


def build_watchers_in_shared_directory(directory):
    os.chdir(directory)
    os.environ.pop("BROM_DIR", None)
    paths = []
    for _ in range(20):
        builder = DiagramBuilder()
        source = builder.AddSystem(ConstantVectorSource([1.0]))
        integrator = builder.AddNamedSystem("integrator", Integrator(1))
        builder.Connect(source.get_output_port(), integrator.get_input_port())
        watcher, diagram, context = add_watcher_and_build(
            builder,
            options=DiagramWatcherOptions(
                plotting_options=PortWatcherPlottingOptions(save_to_file=False)
            ),
        )
        Simulator(diagram, context).Initialize()
        watcher._save_raw_data()
        path = watcher.options.base_directory
        watcher.logger.info(f"run={path.name}")
        paths.append(str(path))
        watcher.diagram = None
        del watcher
    return paths


class OutputDirectoryTest(unittest.TestCase):
    def setUp(self):
        self.directory = TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.environment = patch.dict(os.environ, {"BROM_DIR": str(self.root)})
        self.environment.start()
        self.addCleanup(self.environment.stop)

    def test_runtime_environment_and_fallback(self):
        options = DiagramWatcherOptions()
        for name in ("first", "second"):
            root = self.root / name
            with patch.dict(os.environ, {"BROM_DIR": str(root)}):
                watcher = DiagramWatcher(DiagramBuilder(), options=options)
                self.assertEqual(
                    watcher.options.base_directory.parent, root / "watcher"
                )
        self.assertIsNone(options.base_directory)
        with patch.dict(os.environ, {}, clear=True):
            self.assertEqual(get_brom_dir(), Path("./brom"))
        with patch.dict(os.environ, {"BROM_DIR": ""}):
            self.assertEqual(get_brom_dir(), Path("./brom"))

    def test_reused_explicit_options_do_not_delete_or_share_outputs(self):
        parent = self.root / "custom"
        parent.mkdir()
        sentinel = parent / "keep.txt"
        sentinel.write_text("existing data")
        options = DiagramWatcherOptions(base_directory=str(parent))
        first = DiagramWatcher(DiagramBuilder(), options=options)
        second = DiagramWatcher(DiagramBuilder(), options=options)
        self.assertNotEqual(first.options.base_directory, second.options.base_directory)
        self.assertEqual(first.options.base_directory.parent, parent)
        self.assertEqual(options.base_directory, str(parent))
        self.assertEqual(sentinel.read_text(), "existing data")
        self.assertIsNot(first.logger, second.logger)
        first.logger.info("first watcher is still logging")
        second.logger.info("second watcher is logging")
        first_log = first.options.base_directory / "activity_summary.log"
        second_log = second.options.base_directory / "activity_summary.log"
        del second
        first.logger.info("first watcher survives second")
        self.assertIn("first watcher survives second", first_log.read_text())
        self.assertNotIn("second watcher is logging", first_log.read_text())
        self.assertIn("second watcher is logging", second_log.read_text())
        handlers = first.logger.handlers[:]
        del first
        self.assertTrue(all(handler._closed for handler in handlers))

    def test_convenience_functions_forward_options_and_directory(self):
        options = DiagramWatcherOptions(
            base_directory=self.root / "options",
            plotting_options=PortWatcherPlottingOptions(file_format="svg"),
        )
        watcher = add_watcher(
            DiagramBuilder(), watcher_dir=self.root / "override", options=options
        )
        self.assertEqual(watcher.options.base_directory.parent, self.root / "override")
        self.assertEqual(watcher.options.plotting_options.file_format, "svg")
        self.assertEqual(options.base_directory, self.root / "options")
        builder = DiagramBuilder()
        builder.AddSystem(ConstantVectorSource([1.0]))
        watcher, diagram, context = add_watcher_and_build(builder, options=options)
        self.assertEqual(watcher.options.base_directory.parent, self.root / "options")
        self.assertIs(watcher.diagram, diagram)
        self.assertIs(watcher.diagram_context, context)

    def test_convenience_functions_accept_str_watcher_dir(self):
        """
        Description
        -----------
        Regression test for issue #104: passing ``watcher_dir`` as a ``str``
        (rather than a ``pathlib.Path``) to ``add_watcher`` or
        ``add_watcher_and_build`` should work, both with and without an
        explicit ``options`` object.
        """
        options = DiagramWatcherOptions(base_directory=self.root / "options")
        for use_options in (False, True):
            with self.subTest(function="add_watcher", use_options=use_options):
                watcher_dir = self.root / f"add_watcher_{use_options}"
                watcher = add_watcher(
                    DiagramBuilder(),
                    watcher_dir=str(watcher_dir),
                    options=options if use_options else None,
                )
                self.assertIsInstance(watcher.options.base_directory, Path)
                self.assertEqual(watcher.options.base_directory.parent, watcher_dir)
                self.assertTrue(watcher.options.base_directory.is_dir())

            with self.subTest(
                function="add_watcher_and_build", use_options=use_options
            ):
                watcher_dir = self.root / f"add_watcher_and_build_{use_options}"
                builder = DiagramBuilder()
                builder.AddSystem(ConstantVectorSource([1.0]))
                watcher, diagram, context = add_watcher_and_build(
                    builder,
                    watcher_dir=str(watcher_dir),
                    options=options if use_options else None,
                )
                self.assertIsInstance(watcher.options.base_directory, Path)
                self.assertEqual(watcher.options.base_directory.parent, watcher_dir)
                self.assertTrue(watcher.options.base_directory.is_dir())
                self.assertIs(watcher.diagram, diagram)
        self.assertEqual(options.base_directory, self.root / "options")

    def test_concurrent_watchers_in_shared_working_directory(self):
        context = multiprocessing.get_context("spawn")
        barrier = context.Barrier(4)
        with ProcessPoolExecutor(
            max_workers=4,
            mp_context=context,
            initializer=barrier.wait,
            initargs=(30,),
        ) as executor:
            results = list(
                executor.map(build_watchers_in_shared_directory, [str(self.root)] * 4)
            )
        paths = [self.root / path for group in results for path in group]
        self.assertEqual(len(paths), 80)
        self.assertEqual(len(set(paths)), 80)
        self.assertEqual(len({path.name.split("-")[3] for path in paths}), 4)
        self.assertEqual(len(list((self.root / "brom" / "watcher").iterdir())), 80)
        for path in paths:
            self.assertIn(
                f"run={path.name}", (path / "activity_summary.log").read_text()
            )
            self.assertTrue(list((path / "raw_data").rglob("*.npy")))

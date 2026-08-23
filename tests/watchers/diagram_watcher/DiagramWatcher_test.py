"""
DiagramWatcher_test.py
Description:

    This file defines the tests for the DiagramWatcher module.
"""

from pydrake.all import (
    AffineSystem,
    DiagramBuilder,
    Integrator,
    LeafSystem,
    PortDataType,
    BasicVector,
    Context,
    Simulator,
)
from brom_drake.watchers.diagram_target import DiagramTarget
from brom_drake.watchers.diagram_watcher.diagram_watcher import DiagramWatcher
from brom_drake.watchers.diagram_watcher.diagram_watcher_options import (
    DiagramWatcherOptions,
)
from brom_drake.watchers.diagram_watcher.errors import (
    PortIsNotBeingWatchedError,
    SystemIsNotBeingWatchedError,
)
import numpy as np
from pathlib import Path
from typing import Tuple
import unittest


class TestDiagramWatcher(unittest.TestCase):
    def test_output_directories(self):
        """
        The watcher exposes its configured plot and raw-data directories.
        """
        builder, _, _ = self.create_simple_affine_with_integrator_diagram()
        base_directory = Path("./test_diagram_watcher_output")
        watcher = DiagramWatcher(
            builder,
            options=DiagramWatcherOptions(base_directory=base_directory),
        )

        self.assertEqual(watcher.plot_dir, base_directory / "plots")
        self.assertEqual(watcher.raw_data_dir, base_directory / "raw_data")

    def create_simple_affine_with_integrator_diagram(
        self,
    ) -> Tuple[DiagramBuilder, AffineSystem, Integrator]:
        """
        **Description**

        This helper function creates a simple DiagramBuilder with two systems and a connection between them.
        1. The first system is an AffineSystem that outputs a slowly updating value of the pose of the block.
        2. The second system is an Integrator that integrates the output of the first system, just to make things a little more interesting.

        **Returns**

        builder: DiagramBuilder
            The DiagramBuilder containing the two systems and their connection.

        target_source2: AffineSystem
            The first system that outputs the slowly updating value of the pose of the block.

        integrator: Integrator
            The second system that integrates the output of the first system.
        """
        # Create a simple DiagramBuilder with two systems and a connection between them
        builder = DiagramBuilder()

        # Create a system that outputs the slowly updating value of the pose of the block.
        A = np.zeros((6, 6))
        B = np.zeros((6, 1))
        f0 = np.array([0.0, 0.1, 0.1, 0.0, 0.0, 0.0])
        C = np.eye(6)
        D = np.zeros((6, 1))
        y0 = np.zeros((6, 1))
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.2, 0.5])
        target_source2 = builder.AddSystem(AffineSystem(A, B, f0, C, D, y0))

        # Add an integrator to integrate the output of the first system, just to make things a little more interesting
        integrator = builder.AddNamedSystem(
            system=Integrator(size=6), name="test_integrator"
        )

        # Connect the systems
        builder.Connect(target_source2.get_output_port(0), integrator.get_input_port(0))

        return builder, target_source2, integrator

    def test_init1(self):
        """
        Description:

            This test checks that an exception is raised
            when we try to create a DiagramWatcher using a
            subject that is not a DiagramBuilder.
        :return:
        """
        # Create a simple system
        system = LeafSystem()

        # Try to create a DiagramWatcher with the system
        try:
            watcher = DiagramWatcher(system)
            self.fail("Should have raised an exception!")
        except ValueError as e:
            expectedError = ValueError("subject must be a DiagramBuilder!")
            self.assertEqual(str(e), str(expectedError))
        else:
            self.fail("Should have raised an exception!")

    def test_get_port_watcher1(self):
        """
        Description:

            This test verifies that the get_port_watcher() method of the DiagramWatcher correctly raises an error
            when the specified system name is not found in the Diagram.
        """
        # Setup a simple Diagram with one system
        builder, _, _ = self.create_simple_affine_with_integrator_diagram()

        # Create watcher and build the diagram
        watcher = DiagramWatcher(builder)
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        watcher.diagram = diagram
        watcher.diagram_context = diagram_context

        # Try to run the get_port_watcher method with a non-existent system name and check that it raises the correct error
        bad_target_name = "non_existent_system"
        try:
            watcher.get_port_watcher(bad_target_name, 0)
            self.fail("Should have raised an exception!")
        except SystemIsNotBeingWatchedError as e:
            expectedError = SystemIsNotBeingWatchedError(
                target=DiagramTarget(bad_target_name),
                system_names=[system_name for system_name in watcher._port_watchers],
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.fail("Should have raised a SystemIsNotBeingWatchedError exception!")

    def test_get_port_watcher2(self):
        """
        **Description**

        This test verifies that the get_port_watcher() method of the DiagramWatcher correctly raises an error
        when the specified PORT NAME/INDEX is not being watched by the DiagramWatcher.
        """
        # Setup a simple Diagram with one system
        builder, _, integrator = self.create_simple_affine_with_integrator_diagram()

        # Create watcher and build the diagram
        watcher = DiagramWatcher(builder)
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        watcher.diagram = diagram
        watcher.diagram_context = diagram_context

        # Try to run the get_port_watcher method with a non-existent port name and check that it raises the correct error
        bad_port_name = "non_existent_port"
        target_system_name = integrator.get_name()
        try:
            watcher.get_port_watcher(target_system_name, bad_port_name)
            self.fail("Should have raised an exception!")
        except PortIsNotBeingWatchedError as e:
            expectedError = PortIsNotBeingWatchedError(
                target=DiagramTarget(target_system_name, ports=[bad_port_name]),
                port_reference=bad_port_name,
                port_names=[
                    port_name
                    for port_name in watcher._port_watchers[target_system_name]
                ],
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.fail("Should have raised a PortIsNotBeingWatchedError exception!")

    def test_get_port_watcher3(self):
        """
        **Description**

        This test verifies that the get_port_watcher() method of the DiagramWatcher correctly returns the PortWatcher
        object for a valid system name and port name.
        """
        # Setup a simple Diagram with one system
        builder, _, integrator = self.create_simple_affine_with_integrator_diagram()

        # Create watcher and build the diagram
        watcher = DiagramWatcher(builder)
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        watcher.diagram = diagram
        watcher.diagram_context = diagram_context

        # Get the PortWatcher for the integrator's output port and check that it is correct
        target_system_name = integrator.get_name()
        target_port_name = integrator.get_output_port(0).get_name()
        port_watcher = watcher.get_port_watcher(target_system_name, target_port_name)
        self.assertIsNotNone(port_watcher)

    def test_get_all_port_watchers_for_system1(self):
        """
        **Description**

        This test verifies that the get_all_port_watchers_for_system() method of the DiagramWatcher correctly raises an error
        when the specified system name is not found in the Diagram.
        """
        # Setup a simple Diagram with one system
        builder, _, _ = self.create_simple_affine_with_integrator_diagram()

        # Create watcher and build the diagram
        watcher = DiagramWatcher(builder)
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        watcher.diagram = diagram
        watcher.diagram_context = diagram_context

        # Try to run the get_all_port_watchers_for_system method with a non-existent system name and check that it raises the correct error
        bad_target_name = "non_existent_system"
        try:
            watcher.get_all_port_watchers_for_system(bad_target_name)
            self.fail("Should have raised an exception!")
        except SystemIsNotBeingWatchedError as e:
            expectedError = SystemIsNotBeingWatchedError(
                target=DiagramTarget(bad_target_name),
                system_names=[system_name for system_name in watcher._port_watchers],
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.fail("Should have raised a SystemIsNotBeingWatchedError exception!")

    def test_get_all_port_watchers_for_system2(self):
        """
        **Description**

        This test verifies that the get_all_port_watchers_for_system() method of the DiagramWatcher correctly returns the dictionary of port name to PortWatcher object for a valid system name.
        """
        # Setup a simple Diagram with one system
        builder, _, integrator = self.create_simple_affine_with_integrator_diagram()

        # Create watcher and build the diagram
        watcher = DiagramWatcher(builder)
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()
        watcher.diagram = diagram
        watcher.diagram_context = diagram_context

        # Get the dictionary of port name to PortWatcher object for the integrator and check that it is correct
        target_system_name = integrator.get_name()
        port_watchers_dict = watcher.get_all_port_watchers_for_system(
            target_system_name
        )
        self.assertIsNotNone(port_watchers_dict)
        self.assertIn(integrator.get_output_port(0).get_name(), port_watchers_dict)
        self.assertEqual(
            len(list(port_watchers_dict.keys())),
            1,
            f'Expected 1 port to be watched for integrator system "{target_system_name}", but found {len(list(port_watchers_dict.keys()))} ports being watched.',
        )


if __name__ == "__main__":
    unittest.main()

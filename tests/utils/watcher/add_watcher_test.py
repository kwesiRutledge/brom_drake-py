"""
add_watcher_test.py
Description:

    Tests the add_watcher convenience functions.
"""

import unittest

import numpy as np
from pydrake.all import DiagramBuilder, MultibodyPlant, ConstantVectorSource
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import LeafSystem
from pydrake.systems.primitives import AffineSystem

from brom_drake.all import (
    DiagramTarget, parse_list_of_simplified_targets,
    add_watcher, add_watcher_and_build,
    PortFigureArrangement,
)
from brom_drake.example_helpers import BlockHandlerSystem


class AddWatcherTest(unittest.TestCase):
    def test_parse_list_of_simplified_targets1(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when an empty list is passed in.
        :return:
        """

        builder = DiagramBuilder()
        try:
            parse_list_of_simplified_targets(builder, [])
            self.assertTrue(False)
        except ValueError as e:
            print(e)
            self.assertTrue(True)
        except Exception as e:
            print(e)
            self.assertTrue(False)

    def test_parse_list_of_simplified_targets2(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when the input is not a list
        but something else (maybe a string?).
        :return:
        """
        builder = DiagramBuilder()
        try:
            parse_list_of_simplified_targets(builder, "test")
            self.assertTrue(False)
        except ValueError as e:
            self.assertTrue(True)

    def test_parse_list_of_simplified_targets3(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function produces a valid list of DiagramTargets when
        given a list of strings.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        controller = builder.AddNamedSystem("my_controller", LeafSystem())

        # Test
        targets = parse_list_of_simplified_targets(builder, ["my_plant", "my_controller"])
        self.assertEqual(len(targets), 2)

        for ii, dt_ii in enumerate(targets):
            self.assertTrue(isinstance(dt_ii, DiagramTarget))
            self.assertEqual(dt_ii.name, ["my_plant", "my_controller"][ii])

    def test_parse_list_of_simplified_targets4(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function produces a valid list of DiagramTargets when
        given a list of tuples (most of the tuples contain list of ints for the ports_list).
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        targets = parse_list_of_simplified_targets(
            builder, [("my_plant", [0, 1]), ("my_controller", [0, 1, 2])],
        )
        self.assertEqual(len(targets), 2)

        self.assertEqual(targets[0].name, "my_plant")
        self.assertEqual(targets[0].ports, [0, 1])

        self.assertEqual(targets[1].name, "my_controller")
        self.assertEqual(targets[1].ports, [0, 1, 2])

    def test_parse_list_of_simplified_targets5(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function produces a valid list of DiagramTargets when
        given a list of tuples (most of the tuples contain a single ints for the ports_list).
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        targets = parse_list_of_simplified_targets(
            builder, [("my_plant", 0), ("my_controller", 1)],
        )
        self.assertEqual(len(targets), 2)

        self.assertEqual(targets[0].name, "my_plant")
        self.assertEqual(targets[0].ports, [0])

        self.assertEqual(targets[1].name, "my_controller")
        self.assertEqual(targets[1].ports, [1])

    def test_parse_list_of_simplified_targets6(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function produces a valid list of DiagramTargets when
        given a list of tuples (most of the tuples contain a two ints).
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        targets = parse_list_of_simplified_targets(
            builder, [(0, 0), (1, 1)],
        )
        self.assertEqual(len(targets), 2)

        self.assertEqual(targets[0].name, "my_plant")
        self.assertEqual(targets[0].ports, [0])

        self.assertEqual(targets[1].name, "my_controller")
        self.assertEqual(targets[1].ports, [1])

    def test_parse_list_of_simplified_targets7(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when the first element
        of the tuple is an integer that is out of bounds given the
        number of systems in the diagram.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        try:
            targets = parse_list_of_simplified_targets(
                builder, [(3, 0)],
            )
            self.assertTrue(False)
        except ValueError as e:
            expectedError = ValueError(
                f"the index {3} is not a valid index for the list of systems (has length {2})."
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.assertTrue(False)

    def test_parse_list_of_simplified_targets8(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when
        the first element of the tuple is not an integer or a string.
        In this case, it will be a float.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        try:
            targets = parse_list_of_simplified_targets(
                builder, [(3.14, 0)],
            )
            self.assertTrue(False)
        except ValueError as e:
            expectedError = ValueError(
                "the first element of the tuple must be a string or an integer; received {} (type {}).".format(
                    3.14, type(3.14),
                )
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.assertTrue(False)

    def test_parse_list_of_simplified_targets9(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when
        the second element of the tuple is a string
        that doesn't correspond to a valid port name.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        bad_port_name = "made_up_port_name"
        try:
            targets = parse_list_of_simplified_targets(
                builder, [("my_plant", [bad_port_name])]
            )
            self.assertTrue(False)
        except ValueError as e:
            expectedError = ValueError(
                f"the system {plant.get_name()} does not have an output port named {bad_port_name}."
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.assertTrue(False)

    def test_parse_list_of_simplified_targets10(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when
        the second element of the tuple is not a list of strings
        or a list of ints.
        It is a list of floats.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        bad_port_name = 3.14
        try:
            targets = parse_list_of_simplified_targets(
                builder, [("my_plant", [bad_port_name])]
            )
            self.assertTrue(False)
        except ValueError as e:
            expectedError = ValueError(
                f"the target_list[{0}] of the tuple is not an integer or a string! " +
                f"Received {bad_port_name} of type {type(bad_port_name)}."
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.assertTrue(False)

    def test_parse_list_of_simplified_targets11(self):
        """
        Tests the parse_list_of_simplified_targets function.
        Verifies that the function raises an error when
        the second element of the tuple is not a list of strings
        or a list of ints or an into or a string.
        It will be a float.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Test
        bad_port_name = 3.14
        try:
            targets = parse_list_of_simplified_targets(
                builder, [("my_plant", bad_port_name)]
            )
            self.assertTrue(False)
        except ValueError as e:
            expectedError = ValueError(
                "the second element of the tuple must be either a: \n" +
                "- an integer\n" +
                "- a list of integers\n" +
                "- a string\n" +
                "- a list of strings\n" +
                "- None.\n" +
                f"Received type {type(bad_port_name)}"
            )
            self.assertEqual(str(e), str(expectedError))
        else:
            self.assertTrue(False)

    def test_add_watcher1(self):
        """
        Description:

            Tests the add_watcher convenience function.
            We will make sure that new loggers are created (one for each port) and that
            the logger is connected to the port.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Add Watcher
        watcher = add_watcher(builder, targets=[("my_plant", "state"), ("my_controller", "state")])

        # Verify that the watcher is connected to the correct ports
        self.assertEqual(len(watcher.port_watchers), 2)

    def test_add_watcher2(self):
        """
        Description:

            Tests the add_watcher convenience function.
            We will make sure that new loggers are created (one for each port) and that
            the logger is connected to the port.
            This call will happen with targets that are tuples of two strings!
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Add Watcher
        watcher = add_watcher(
            builder,
            targets=[
                ("my_plant", ["state", "generalized_acceleration"]),
                ("my_controller", "state")
            ],
        )

        # Verify that the watcher is connected to the correct ports
        self.assertEqual(
            3,
            len(watcher.port_watchers["my_plant"]) + len(watcher.port_watchers["my_controller"]),
        )

    def test_add_watcher3(self):
        """
        Description:

            Tests the add_watcher convenience function.
            We will make sure that new loggers are created (one for each port) and that
            the logger is connected to the port.
            This call will happen with targets that are a mixture of strings and tuples of two strings!
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Add Watcher
        watcher = add_watcher(
            builder,
            targets=[
                "my_plant",
                ("my_controller", "state"),
            ],
        )

        # Verify that the watcher is connected to the correct ports
        self.assertLessEqual(
            4,
            len(watcher.port_watchers["my_plant"]) + len(watcher.port_watchers["my_controller"]),
        )
        self.assertEqual(1, len(watcher.port_watchers["my_controller"]))
        self.assertLess(
            2,
            len(watcher.port_watchers["my_plant"]),
        )

    def test_add_watcher_and_build1(self):
        """
        Description:

            Tests the add_watcher_and_build() convenience function.
            We will make sure that an exception is raised when you provide a plot_arrangement that is
            invalid.
        :return:
        """
        # Setup
        time_step = 1e-3

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Add Watcher
        invalid_arrangement_value = "invalid"
        try:
            watcher = add_watcher_and_build(
                builder,
                targets=[
                    "my_plant",
                    ("my_controller", "state"),
                ],
                plot_arrangement=invalid_arrangement_value,
            )
        except ValueError as e:
            self.assertEqual(
                str(e),
                f"plot_arrangement must be of type PortFigureArrangement; received invalid of type {type(invalid_arrangement_value)}.",
            )
        except:
            self.assertTrue(False)

    def test_add_watcher_and_build2(self):
        """
        Description:

            Tests the add_watcher_and_build() convenience function.
            We will make sure that the default usage of the function works on a short (1 second) sim.
            So, the diagram should be successfully built and the simulation should run.
        :return:
        """
        # Setup

        time_step = 1e-3
        builder = DiagramBuilder()

        # Create Plant and the "Block + Ground system"
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)
        block_handler_system = builder.AddSystem(BlockHandlerSystem(plant, scene_graph))

        # Connect System To Handler
        # Create system that outputs the slowly updating value of the pose of the block.
        A = np.zeros((6, 6))
        B = np.zeros((6, 1))
        f0 = np.array([0.0, 0.1, 0.1, 0.0, 0.0, 0.0])
        C = np.eye(6)
        D = np.zeros((6, 1))
        y0 = np.zeros((6, 1))
        x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.2, 0.5])
        target_source2 = builder.AddSystem(
            AffineSystem(A, B, f0, C, D, y0)
        )
        target_source2.configure_default_state(x0)

        # Connect the state of the block to the output of a slowly changing system.
        builder.Connect(
            target_source2.get_output_port(),
            block_handler_system.GetInputPort("desired_pose"))

        u0 = np.array([0.2])
        affine_system_input = builder.AddSystem(ConstantVectorSource(u0))
        builder.Connect(
            affine_system_input.get_output_port(),
            target_source2.get_input_port()
        )

        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()
        controller = builder.AddNamedSystem("my_controller", MultibodyPlant(time_step=time_step))
        controller.Finalize()

        # Add Watcher
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            targets=[
                "my_plant",
                ("my_controller", "state"),
            ],
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
        )

        # Set initial pose and vectors
        block_handler_system.SetInitialBlockState(diagram_context)

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        block_handler_system.context = block_handler_system.plant.GetMyMutableContextFromRoot(diagram_context)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(1.0)

        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()

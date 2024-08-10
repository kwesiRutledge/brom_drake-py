"""
PortWatcher_test.py
Description:

    This
"""
import shutil
import unittest
import os

import numpy as np
from pydrake.all import (
    RotationMatrix, RigidTransform, HalfSpace,
    CoulombFriction, SpatialVelocity,
)
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant, AddMultibodyPlantSceneGraph
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import DiagramBuilder, PortDataType

from brom_drake.PortWatcher import PortWatcher
from brom_drake.PortWatcher.PortWatcher import PortFigureArrangement, PortWatcherOptions


class PortWatcherTest(unittest.TestCase):
    def get_brom_drake_dir(self):
        """
        Description:

            This function returns the path to the brom_drake directory.
        :return:
        """
        if "tests" in os.getcwd():
            return os.path.abspath(os.path.join(os.getcwd(), "..", ".."))
        else:
            return os.getcwd()

    def test_init1(self):
        """
        Description:

            Tests that the PortWatcher object correctly
            is created if given a valid system, output port, and
            builder.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        self.assertTrue(True)
        self.assertIn(
            "PortWatcher", pw0.logger.get_name(),
        )
        self.assertIn(
            plant.get_name(), pw0.logger.get_name(),
        )
        self.assertIn(
            "state", pw0.logger.get_name(),
        )

    def test_init2(self):
        """
        Description:

            This test verifies that if a port is provided to the
            PortWatcher that is not kVectorValued, then an error is
            raised.
        :return:
        """

        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        plant_test_port = plant.get_output_port(0)

        try:
            pw0 = PortWatcher(
                plant, plant_test_port, builder,
            )
        except ValueError as e:
            expected_error = ValueError(
                f"This watcher only supports vector valued ports (i.e., of type {PortDataType.kVectorValued}.\n" +
                f"Received port of type {plant_test_port.get_data_type()}."
            )

            self.assertEqual(
                str(e), str(expected_error),
            )
        else:
            self.assertTrue(False)

    def test_compute_plot_shape1(self):
        """
        Description:

            Verifies that if you have a single dimension
            input to compute_plot_shape() you receive a 1,1 tuple.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Test
        shape0 = pw0.compute_plot_shape(1)

        self.assertEqual(
            shape0, (1,1),
        )

    def test_compute_plot_shape2(self):
        """
        Description:

            Verifies that if you have a 2 dimension
            input to compute_plot_shape() you receive a 1,2 tuple.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Test
        shape0 = pw0.compute_plot_shape(2)

        self.assertEqual(
            shape0, (1,2),
        )

    def test_compute_plot_shape3(self):
        """
        Description:

            Verifies that if you have a 6 dimension
            input to compute_plot_shape() you receive a 2,3 tuple.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Test
        shape0 = pw0.compute_plot_shape(6)

        self.assertEqual(
            shape0, (2,3),
        )

    def test_compute_plot_shape4(self):
        """
        Description:

            Verifies that if you have a 12 dimension
            input to compute_plot_shape() you receive a 3,4 tuple.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Test
        shape0 = pw0.compute_plot_shape(12)

        self.assertEqual(
            shape0, (3,4),
        )

    def test_plot_logger_data1(self):
        """
        Description:

            This test verifies that the plot_logger_data method
            properly creates a single figure when the
            options struct contains a plot_arrangement value of
            OnePlotPerPort.
        :return:
        """
        # Setup Diagram
        # - Create Builder
        # - Define Plant
        time_step = 0.01

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/monitor1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0, 0.0, 0.0]
        R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
        X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)
        surface_friction = CoulombFriction(
            static_friction=0.7,
            dynamic_friction=0.5)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Setup simulation
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set initial conditions
        # - Initial pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi / 2.0)  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName(block_body_name),
            X_WBlock)

        # - initial Velocities
        plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName(block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant.GetMyContextFromRoot(diagram_context))

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Try to plot
        figs, axs_list = pw0.plot_logger_data(diagram_context)

        self.assertEqual(1, len(figs))

    def test_plot_logger_data2(self):
        """
        Description:

            This test verifies that the plot_logger_data method
            properly panics if called with a logger that
            does not hold any data.
        :return:
        """
        # Setup Diagram
        # - Create Builder
        # - Define Plant
        time_step = 0.01

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/monitor1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0, 0.0, 0.0]
        R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
        X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)
        surface_friction = CoulombFriction(
            static_friction=0.7,
            dynamic_friction=0.5)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Setup simulation
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set initial conditions
        # - Initial pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi / 2.0)  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName(block_body_name),
            X_WBlock)

        # - initial Velocities
        plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName(block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant.GetMyContextFromRoot(diagram_context))

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        # simulator.AdvanceTo(10.0)

        # Try to plot
        figs, axs_list = pw0.plot_logger_data(diagram_context)

        self.assertIsNone(figs)
        self.assertIsNone(axs_list)

    def test_plot_logger_data3(self):
        """
        Description:

            This test verifies that the plot_logger_data method
            properly creates thirteen figures when the
            options struct contains a plot_arrangement value of
            OnePlotPerDim.
        :return:
        """
        # Setup Diagram
        # - Create Builder
        # - Define Plant
        time_step = 0.01

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/monitor1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0, 0.0, 0.0]
        R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
        X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)
        surface_friction = CoulombFriction(
            static_friction=0.7,
            dynamic_friction=0.5)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
            options=PortWatcherOptions(
                plot_arrangement=PortFigureArrangement.OnePlotPerDim,
            )
        )

        # Setup simulation
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set initial conditions
        # - Initial pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi / 2.0)  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName(block_body_name),
            X_WBlock)

        # - initial Velocities
        plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName(block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant.GetMyContextFromRoot(diagram_context))

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Try to plot
        figs, axs_list = pw0.plot_logger_data(diagram_context)

        self.assertEqual(13, len(figs))
        self.assertEqual(13, len(axs_list))

    def test_plot_logger_data_subplots1(self):
        """
        Description:

            Verifies that if you have a single dimension
            input to compute_plot_shape() you receive a 1,1 tuple.
        :return:
        """
        # Setup Diagram
        # - Create Builder
        # - Define Plant
        time_step = 0.01

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/monitor1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0, 0.0, 0.0]
        R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
        X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)
        surface_friction = CoulombFriction(
            static_friction=0.7,
            dynamic_friction=0.5)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

        plant.Finalize()

        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
        )

        # Setup simulation
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set initial conditions
        # - Initial pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi / 2.0)  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName(block_body_name),
            X_WBlock)

        # - initial Velocities
        plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName(block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant.GetMyContextFromRoot(diagram_context))

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Collect Data
        temp_log = pw0.logger.FindLog(diagram_context)

        log_times = temp_log.sample_times()
        log_data = temp_log.data()

        # Try to plot
        pw0.plot_logger_data_subplots(diagram_context, log_times, log_data)

        self.assertTrue(True)

    def test_savefigs(self):
        """
        Description:

            Verifies that the savefigs method properly saves
            a single figure to the desired directory when we
            use a PortWatcher with the OnePlotPerPort arrangement.
        :return:
        """
        # Setup Diagram
        # - Create Builder
        # - Define Plant
        time_step = 0.01

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/monitor1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0, 0.0, 0.0]
        R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
        X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)
        surface_friction = CoulombFriction(
            static_friction=0.7,
            dynamic_friction=0.5)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

        plant.Finalize()

        plot_dir = "./.brom2"
        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
            plot_dir=plot_dir,
        )

        # Setup simulation
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set initial conditions
        # - Initial pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi / 2.0)  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName(block_body_name),
            X_WBlock)

        # - initial Velocities
        plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName(block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant.GetMyContextFromRoot(diagram_context))

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there is only one png file in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]

        self.assertEqual(1, len(png_files))

        for file in files:
            os.remove(os.path.join(plot_dir, file))
        os.rmdir(plot_dir)

    def test_savefigs2(self):
        """
        Description:

            Verifies that the savefigs method properly saves
            thirteen figures to the desired directory when we
            use a PortWatcher with the OnePlotPerDim arrangement.
            The figures should not be within the main plot_dir,
            but in a directory below iw.
        :return:
        """
        # Setup Diagram
        # - Create Builder
        # - Define Plant
        time_step = 0.01

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/monitor1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0, 0.0, 0.0]
        R_GroundOrigin = RotationMatrix.MakeXRotation(0.0)
        X_GroundOrigin = RigidTransform(R_GroundOrigin, p_GroundOrigin)
        surface_friction = CoulombFriction(
            static_friction=0.7,
            dynamic_friction=0.5)
        plant.RegisterCollisionGeometry(
            plant.world_body(),
            X_GroundOrigin,
            HalfSpace(),
            "ground_collision",
            surface_friction)

        plant.Finalize()

        plot_dir = "./.brom3"
        pw_options = PortWatcherOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
        )
        pw0 = PortWatcher(
            plant, plant.GetOutputPort("state"), builder,
            plot_dir=plot_dir,
            options=pw_options,
        )

        # Setup simulation
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Set initial conditions
        # - Initial pose
        p_WBlock = [0.0, 0.0, 0.2]
        R_WBlock = RotationMatrix.MakeXRotation(np.pi / 2.0)  # RotationMatrix.MakeXRotation(-np.pi/2.0)
        X_WBlock = RigidTransform(R_WBlock, p_WBlock)
        plant.SetFreeBodyPose(
            plant.GetMyContextFromRoot(diagram_context),
            plant.GetBodyByName(block_body_name),
            X_WBlock)

        # - initial Velocities
        plant.SetFreeBodySpatialVelocity(
            plant.GetBodyByName(block_body_name),
            SpatialVelocity(np.zeros(3), np.array([0.0, 0.0, 0.0])),
            plant.GetMyContextFromRoot(diagram_context))

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(10.0)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]
        self.assertEqual(0, len(png_files))

        # check the number of elements in the port subdirectory
        port_dir = os.path.join(plot_dir, "plant_state")
        files = os.listdir(port_dir)
        png_files = [f for f in files if f.endswith(".png")]
        self.assertEqual(13, len(png_files))

        shutil.rmtree(plot_dir)

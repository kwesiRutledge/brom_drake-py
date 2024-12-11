from importlib import resources as impresources
import numpy as np
import os
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    ConstantVectorSource,
    Context, CoulombFriction,
    Diagram, DiagramBuilder, HalfSpace,
    LogVectorOutput,
    MultibodyPlant, Parser,
    RotationMatrix, RigidTransform,
    Simulator, SpatialVelocity,
)
import shutil
from typing import Tuple
import unittest

# Internal Imports
from brom_drake.PortWatcher.plotter import PortWatcherPlotter
from brom_drake.PortWatcher.port_watcher_options import (
    PortWatcherPlottingOptions, PortFigureArrangement,
    FigureNamingConvention,
)
import brom_drake.robots as robots

class PortWatcherPlotterTest(unittest.TestCase):
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
        
    def setUp(self):
        """
        Description:

            This method is used to setup each test.
        :return:
        """
        # Delete the .brom directory if it exists when the test is done.
        self.delete_test_brom_directory_on_teardown = True
        self.T_sim1 = 5.0  # Time to simulate
        
    def test_compute_plot_shape1(self):
        """
        Description:

            Verifies that if you have a single dimension
            input to compute_plot_shape() you receive a 1,1 tuple.
        :return:
        """
        # Setup Diagram
        time_step = 0.01

        # Build Diagram
        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
        )

        # Test
        shape0 = plotter0.compute_plot_shape(1)

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

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
        )

        # Test
        shape0 = plotter0.compute_plot_shape(2)

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

        # Build Diagram
        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
        )

        # Test
        shape0 = plotter0.compute_plot_shape(6)

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

        # Build the Diagram
        builder = DiagramBuilder()
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
        )

        # Test
        shape0 = plotter0.compute_plot_shape(12)

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
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/watcher/suggested_use1/slider-block.urdf",
        )[0]
        block_body_name = "block"

        p_GroundOrigin = [0.0, 0.0, 0.0]
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

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
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
        simulator.AdvanceTo(self.T_sim1)

        # Try to plot
        figs, axs_list = plotter0.plot_logger_data(diagram_context)

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
            self.get_brom_drake_dir() + "/examples/watcher/suggested_use1/slider-block.urdf",
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

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
            plotting_options=PortWatcherPlottingOptions(
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

        # simulator.Initialize()
        # simulator.AdvanceTo(10.0)

        # Try to plot
        figs, axs_list = plotter0.plot_logger_data(diagram_context)

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
            self.get_brom_drake_dir() + "/examples/watcher/suggested_use1/slider-block.urdf",
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

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
            plotting_options=PortWatcherPlottingOptions(
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
        simulator.AdvanceTo(self.T_sim1)

        # Try to plot
        figs, axs_list = plotter0.plot_logger_data(diagram_context)

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
        time_step = 1e-3

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/watcher/suggested_use1/slider-block.urdf",
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

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
        )

        # pw0 = PortWatcher(
        #     plant, plant.GetOutputPort("state"), builder,
        # )

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
        simulator.AdvanceTo(self.T_sim1)

        # Collect Data
        temp_log = plotter0.logger.FindLog(diagram_context)

        log_times = temp_log.sample_times()
        log_data = temp_log.data()

        # Try to plot
        plotter0.plot_logger_data_subplots(log_times, log_data)

        self.assertTrue(True)

    def build_example_diagram(
        self,
        time_step: float = 0.01,
        plotting_options: PortWatcherPlottingOptions = PortWatcherPlottingOptions(),
    )->Tuple[Diagram, Context, PortWatcherPlotter, MultibodyPlant]:
        """
        Description:

                This method creates a simple diagram with a plant
                and a block model. It is meant to be reused for testing our plotting
                feature.
        :param time_step: The time step to use in plant simulation
        :param plot_dir: The directory in which to save the plots
        :param pw_options: The options to use for the PortWatcher
        :return:
        """

        # Setup Diagram
        # - Create Builder
        # - Define Plant

        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=time_step)

        block_model_idx = Parser(plant=plant).AddModels(
            self.get_brom_drake_dir() + "/examples/watcher/suggested_use1/slider-block.urdf",
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

        logger0 = LogVectorOutput(plant.GetOutputPort("state"), builder)

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
            plotting_options=plotting_options,
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

        return diagram, diagram_context, plotter0, plant

    def test_save_figures(self):
        """
        Description:

            Verifies that the savefigs method properly saves
            a single figure to the desired directory when we
            use a PortWatcher with the OnePlotPerPort arrangement.
        :return:
        """
        # Setup

        # Set up a simple diagram with an included watcher
        plot_dir = "./.brom2"
        options = PortWatcherPlottingOptions(
            base_directory=plot_dir
        )
        diagram, diagram_context, pw0, _ = self.build_example_diagram(
            plotting_options=options,
        )

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(self.T_sim1)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there is only one png file in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]

        self.assertEqual(1, len(png_files))

        if self.delete_test_brom_directory_on_teardown:
            for file in files:
                os.remove(os.path.join(plot_dir, file))

            os.rmdir(plot_dir)

    def test_save_figures2(self):
        """
        Description:

            Verifies that the savefigs method properly saves
            thirteen figures to the desired directory when we
            use a PortWatcher with the OnePlotPerDim arrangement.
            The figures should not be within the main plot_dir,
            but in a directory below iw.
        :return:
        """
        # Setup
        plot_dir = "./.brom3"
        pw_options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
            figure_naming_convention=FigureNamingConvention.kHierarchical,
            base_directory=plot_dir,
        )

        # Set up a simple diagram with an included watcher
        diagram, diagram_context, pw0, plant = self.build_example_diagram(
            plotting_options=pw_options,
        )

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(self.T_sim1)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]
        self.assertEqual(0, len(png_files))

        # check the number of elements in the port subdirectory
        port_dir = os.path.join(plot_dir, "system_plant/port_state")
        files = os.listdir(port_dir)
        png_files = [f for f in files if f.endswith(".png")]

        state_dim = plant.get_state_output_port().size()
        self.assertEqual(state_dim, len(png_files)) # TODO(Kwesi): Why is one state dimension not plotted?

        if self.delete_test_brom_directory_on_teardown:
            shutil.rmtree(plot_dir)

    def test_save_figures3(self):
        """
        Description:

            Verifies that the savefigs method properly saves
            a single figure to the desired directory when we
            use a PortWatcher with the OnePlotPerPort arrangement.
        :return:
        """
        # Setup
        plot_dir = "./.brom4"
        pw_options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
            figure_naming_convention=FigureNamingConvention.kHierarchical,
            base_directory=plot_dir,
        )

        # Set up a simple diagram with an included watcher
        diagram, diagram_context, pw0, plant = self.build_example_diagram(
            plotting_options=pw_options,
        )

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(self.T_sim1)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]
        self.assertEqual(0, len(png_files))

        # check the number of elements in the port subdirectory
        port_dir = os.path.join(plot_dir, "system_plant/port_state")
        files = os.listdir(port_dir)
        png_files = [f for f in files if f.endswith(".png")]
        state_dim = plant.get_state_output_port().size()
        self.assertEqual(state_dim, len(png_files)) # TODO(Kwesi): Why is one state dimension not plotted?

        if self.delete_test_brom_directory_on_teardown:
            shutil.rmtree(plot_dir)

    def test_save_figures4(self):
        """
        Description:
            This test verifies that when choosing the plot options:
            - kFlat
            - kOnePlotPerDim
            the correct number of plots is created (should be 13, the number of dimensions of the state)
        :return:
        """
        # Setup
        plot_dir = "./.brom5"
        pw_options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
            figure_naming_convention=FigureNamingConvention.kFlat,
            base_directory=plot_dir,
        )

        # Set up a simple diagram with an included watcher
        diagram, diagram_context, pw0, plant = self.build_example_diagram(
            plotting_options=pw_options,
        )

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(self.T_sim1)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there are n_state - 1 png files in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]
        state_dim = plant.get_state_output_port().size()
        self.assertEqual(len(png_files), state_dim)

        if self.delete_test_brom_directory_on_teardown:
            shutil.rmtree(plot_dir)

    def test_save_figures5(self):
        """
        Description:
            This test verifies that when choosing the plot options:
            - kHierarchical
            - OnePlotPerPort
            the correct number of plots is created (should be greater than 13)
        :return:
        """
        # Setup
        plot_dir = "./.brom6"
        plotting_options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerPort,
            figure_naming_convention=FigureNamingConvention.kHierarchical,
            base_directory=plot_dir,
        )

        # Set up a simple diagram with an included watcher
        diagram, diagram_context, pw0, plant = self.build_example_diagram(
            plotting_options=plotting_options,
        )

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(self.T_sim1)

        # Save figs
        pw0.save_figures(diagram_context)

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(".png")]
        self.assertEqual(0, len(png_files))

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir + "/system_plant")
        png_files = [f for f in files if f.endswith(".png")]

        expected_n_plots = 1 # Because we are targeting only one port in the system
        self.assertEqual(len(png_files), expected_n_plots)

        if self.delete_test_brom_directory_on_teardown:
            shutil.rmtree(plot_dir)

    def test_save_figures7(self):
        """
        Description:
            This test verifies that when choosing the svg file format
            that we can properly produce a figure.
        :return:
        """
        # Setup
        plot_dir = "./.brom7"
        test_file_format = "svg"
        plotting_options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerPort,
            figure_naming_convention=FigureNamingConvention.kHierarchical,
            file_format=test_file_format,
            base_directory=plot_dir,
        )

        # Set up a simple diagram with an included watcher
        diagram, diagram_context, plotter0, plant = self.build_example_diagram(
            plotting_options=plotting_options,
        )

        # Run sim
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)

        # simulator.Initialize()
        simulator.AdvanceTo(self.T_sim1)

        # Save figs
        plotter0.save_figures(diagram_context)

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir)
        png_files = [f for f in files if f.endswith(f".{test_file_format}")]
        self.assertEqual(0, len(png_files))

        # Check that there are 0 png files in the plot_dir
        files = os.listdir(plot_dir + "/system_plant")
        png_files = [f for f in files if f.endswith(f".{test_file_format}")]

        expected_n_plots = 1 # Because we are targeting only one port in the system
        self.assertEqual(len(png_files), expected_n_plots)

        if self.delete_test_brom_directory_on_teardown:
            shutil.rmtree(plot_dir)

    def test_system_is_multibody_plant1(self):
        """
        Description:

            This test provides a multibody plant to the
            PortWatcher and verifies that the system_is_multibody_plant
            method returns True.
        :return:
        """
        # Setup
        time_step = 0.01
        builder = DiagramBuilder()

        # Create Diagram
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
            plotting_options=PortWatcherPlottingOptions(
                plot_arrangement=PortFigureArrangement.OnePlotPerDim,
            )
        )

        # Test
        self.assertTrue(plotter0.system_is_multibody_plant())

    def test_system_is_multibody_plant2(self):
        """
        Description:

            This test provides a non-multibody plant to the
            PortWatcher and verifies that the system_is_multibody_plant
            method returns False.
        :return:
        """
        # Setup
        time_step = 0.01
        builder = DiagramBuilder()

        # Create Diagram
        plant = builder.AddNamedSystem("my_plant", MultibodyPlant(time_step=time_step))
        plant.Finalize()

        constant_vector_source = builder.AddSystem(ConstantVectorSource([1.0, 2.0, 3.0]))

        logger0 = LogVectorOutput(
            constant_vector_source.get_output_port(),
            builder,
        )

        # Create PortWatcherPlotter
        plotter0 = PortWatcherPlotter(
            logger0,
            constant_vector_source.get_output_port(),
        )

        # Test
        self.assertFalse(plotter0.system_is_multibody_plant())

    def test_name_of_data_at_index1(self):
        """
        Description:

            This test verifies that the name_of_data_at_index method
            properly returns the name of the data at the given index
            when handling a plant with some models in it.

            Note: The new name is only active with specific options.
        :return:
        """
        # Setup
        time_step = 0.01
        builder = DiagramBuilder()

        # Add in some models
        plant =  MultibodyPlant(time_step=time_step)
        plant = builder.AddSystem(plant)
        cupboard_sdf_file = str(
            impresources.files(robots) / "models/cupboard/cupboard.sdf"
        )
        Parser(plant).AddModels(cupboard_sdf_file)
        plant.Finalize()

        logger0 = LogVectorOutput(
            plant.GetOutputPort("state"),
            builder,
        )

        # Create PortWatcherPlotter
        options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
        )
        plotter0 = PortWatcherPlotter(
            logger0,
            plant.GetOutputPort("state"),
            plotting_options=options,
        )

        # Test
        name0 = plotter0.name_of_data_at_index(0)

        self.assertIn(
            "cupboard",
            name0,
        )

if __name__ == '__main__':
    unittest.main()
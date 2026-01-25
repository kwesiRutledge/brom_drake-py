from brom_drake import robots
from brom_drake.watchers.port_watcher.file_manager import PortWatcherFileManager
from brom_drake.watchers.port_watcher.port_watcher_options import PortWatcherPlottingOptions
from brom_drake.watchers.port_watcher.port_figure_arrangement import PortFigureArrangement
import importlib.resources as impresources
import os
from pathlib import Path
from pydrake.geometry import HalfSpace
from pydrake.math import RigidTransform, RotationMatrix
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import AddMultibodyPlantSceneGraph, CoulombFriction, MultibodyPlant
from pydrake.systems.framework import DiagramBuilder
from pydrake.systems.primitives import LogVectorOutput
import unittest

class PortWatcherFileManagerTest(unittest.TestCase):
    def get_brom_drake_dir(self):
        """
        *Description*

        This function returns the path to the brom_drake directory.
        
        TODO(Kwesi): Replace this with a more robust method
        """
        if "tests" in os.getcwd():
            return os.path.abspath(os.path.join(os.getcwd(), "..", ".."))
        else:
            return os.getcwd()
        
    def setUp(self):
        # Setup code for the tests
        pass

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
        test_watcher_directory = Path("./.brom_pw_file_manager1/watcher")

        # Create builder
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

        # Create PortWatcherFileManager
        options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
        )
        test_file_manager = PortWatcherFileManager(
            base_directory=test_watcher_directory,
            plotting_options=options,
        )

        # Test
        name0 = test_file_manager.name_of_data_at_index(0, plant.GetOutputPort("state"), logger0)

        self.assertIn("cupboard", name0)

    def test_time_data_names1(self):
        """
        *Description*
        
        This test verifies that the time and raw data names are
        correctly containing the system name and port name.
        In this case, for the state port of a MultibodyPlant system.
        """
        # Setup
        test_watcher_directory = Path("./.brom_pw_file_manager2")

        # Setup Diagram
        # - Create Builder
        # - Define Plant
        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_path = Path(self.get_brom_drake_dir()) / "examples/watcher/suggested_use1/slider-block.urdf"
        Parser(plant=plant).AddModels(
            str(block_model_path)
        )

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

        # Create PortWatcherFileManager
        options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
        )
        test_file_manager = PortWatcherFileManager(
            base_directory=test_watcher_directory,
            plotting_options=options,
        )

        # Verify that the path names for one of the ports
        # is correct.
        time_data_path = test_file_manager.time_data_file_path(
            system_name=plant.get_name(), 
            port_name=plant.GetOutputPort("state").get_name()
        )
        self.assertIn("times", str(time_data_path))

    def test_raw_data_names1(self):
        """
        *Description*
        
        This test verifies that the raw data names are
        correctly containing the system name and port name.
        In this case, for the state port of a MultibodyPlant system.
        """
        # Setup
        test_watcher_directory = Path("./.brom_pw_file_manager2")

        # Setup Diagram
        # - Create Builder
        # - Define Plant
        builder = DiagramBuilder()

        # Define plant with:
        # + add block model
        # + ground
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=1e-3)

        block_model_path = Path(self.get_brom_drake_dir()) / "examples/watcher/suggested_use1/slider-block.urdf"
        Parser(plant=plant).AddModels(
            str(block_model_path)
        )[0]

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

        # Create PortWatcherFileManager
        options = PortWatcherPlottingOptions(
            plot_arrangement=PortFigureArrangement.OnePlotPerDim,
        )
        test_file_manager = PortWatcherFileManager(
            base_directory=test_watcher_directory,
            plotting_options=options,
        )

        # Verify that the path names for one of the ports
        # is correct.
        test_raw_data_path = test_file_manager.raw_data_file_path(
            system_name=plant.get_name(), 
            port_name=plant.GetOutputPort("state").get_name()
        )

        self.assertIn(plant.get_name(), str(test_raw_data_path))
        self.assertIn("state", str(test_raw_data_path))

if __name__ == "__main__":
    unittest.main()
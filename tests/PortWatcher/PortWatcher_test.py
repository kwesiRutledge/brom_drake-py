"""
PortWatcher_test.py
Description:

    This
"""
from importlib import resources as impresources
import logging
import unittest
import os

import numpy as np
from pathlib import Path
from pydrake.all import (
    AbstractValue,
    AddMultibodyPlantSceneGraph,
    ConstantValueSource, CoulombFriction,
    HalfSpace,
    MultibodyPlant,
    Parser,
    RigidTransform, RotationMatrix,
    Simulator,
)
from pydrake.systems.framework import DiagramBuilder, PortDataType, Diagram, Context

# Internal Imports
from brom_drake.PortWatcher.port_watcher_options import PortWatcherOptions, PortWatcherPlottingOptions
from brom_drake.all import (
    PortWatcher,
)
from brom_drake.directories import DEFAULT_BROM_DIR

class PortWatcherTest(unittest.TestCase):
    def setUp(self):
        """
        Description:

            This method is used to setup each test.
        :return:
        """
        # Delete the .brom directory if it exists when the test is done.
        self.delete_test_brom_directory_on_teardown = True
        self.T_sim1 = 5.0  # Time to simulate

    def create_dummy_logger(self, log_file_name: str):
        # Create dummy logger
        python_logger = logging.getLogger("PortWatcher Test Logger")
        for handler in python_logger.handlers:
            python_logger.removeHandler(handler)

        # Add a single file handler to the logger
        parent_dir = Path(
            DEFAULT_BROM_DIR + "/PortWatcherTest/" + log_file_name
        ).parent
        if not parent_dir.exists():
            # Create the parent directory if it does not exist
            parent_dir.mkdir(parents=True, exist_ok=True)

        file_handler = logging.FileHandler(
            filename=parent_dir / log_file_name,
            mode='w'
        )
        file_handler.setLevel(logging.DEBUG)
        # Create a formatter and set it for the handler
        formatter = logging.Formatter('%(asctime)s | %(levelname)s | %(name)s | %(message)s')
        file_handler.setFormatter(formatter)
        python_logger.addHandler(file_handler)

        return python_logger


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
            plant.GetOutputPort("state"), builder,
            python_logger=self.create_dummy_logger("PortWatcherTest_init1.log")
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

        plant_test_port = plant.GetOutputPort("contact_results")
        print(plant_test_port.get_name())

        try:
            pw0 = PortWatcher(
                plant_test_port, builder,
                python_logger=self.create_dummy_logger("PortWatcherTest_init2.log"),
            )
        except ValueError as e:
            expected_error = PortWatcher.create_port_value_type_error(plant_test_port)

            self.assertEqual(
                str(e), str(expected_error),
            )
        else:
            self.assertTrue(False)

    def test_time_and_raw_data_names1(self):
        """
        Description
        -----------
        This test verifies that the time and raw data names are
        correctly containing the system name and port name.
        In this case, for the state port of a MultibodyPlant system.
        """
        # Setup
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

        # Create PortWatcherPlotter
        pw0 = PortWatcher(
            plant.GetOutputPort("state"),
            builder,
            python_logger=self.create_dummy_logger("PortWatcherTest_time_and_raw_data_names1.log"),
        )

        # Verify
        name_tuple = pw0.time_and_raw_data_names()
        self.assertEqual(2, len(name_tuple))

        for name_ii in name_tuple:
            self.assertIn(plant.get_name(), name_ii)
            self.assertIn("state", name_ii)
        self.assertIn("times", name_tuple[0])

    def test_check_port_type1(self):
        """
        Description
        -----------
        This test verifies that the check_port_type method
        does NOT raise an error when we provide a port that contains
        an abstract value that is a RigidTransform.
        """
        # Setup
        builder = DiagramBuilder()

        # Setup Diagram
        pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(RigidTransform()))
        )

        # Create PortWatcher object
        pw0 = PortWatcher(
            pose_source.get_output_port(), builder,
            python_logger=self.create_dummy_logger("PortWatcherTest_check_port_type1.log"),
        )

        # this test will only fail if an error is raised
        self.assertTrue(True)

    def test_save_raw_data1(self):
        """
        Description
        -----------
        This test verifies that the save_raw_data method
        correctly saves the raw data to the correct file.
        """
        # Setup
        builder = DiagramBuilder()

        # Setup Diagram
        pose_source = builder.AddSystem(
            ConstantValueSource(AbstractValue.Make(RigidTransform()))
        )

        # Create PortWatcher object
        pw_options0 = PortWatcherOptions(
            base_directory="./brom/test_save_raw_data1/watcher"
        )
        pw0 = PortWatcher(
            pose_source.get_output_port(),
            builder,
            python_logger=self.create_dummy_logger("PortWatcherTest_save_raw_data1.log"),
            options=pw_options0,
        )

        # Build Diagram
        diagram = builder.Build()
        diagram_context = diagram.CreateDefaultContext()

        # Create simulator and simulate for a few seconds
        simulator = Simulator(diagram, diagram_context)
        simulator.set_publish_every_time_step(False)
        simulator.AdvanceTo(1.0)
        simulator.AdvanceTo(2.0)

        # Save raw data
        pw0.save_raw_data(diagram_context)

        # Verify that the raw_data directory exists
        self.assertTrue(
            os.path.exists(pw0.options.raw_data_dir()),
        )

        # Verify that there is at least one file in the directory
        self.assertTrue(
            len(os.listdir(pw0.options.raw_data_dir())) > 0,
        )


if __name__ == '__main__':
    unittest.main()

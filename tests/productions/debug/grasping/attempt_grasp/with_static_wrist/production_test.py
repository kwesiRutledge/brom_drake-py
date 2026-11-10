from importlib import resources as impresources
from pathlib import Path
import numpy as np
from pydrake.all import (
    DiagramBuilder,
    Parser,
    PrismaticJoint,
    MultibodyPlant,
    RigidTransform,
    Simulator,
)
import unittest

# Internal Imports
from brom_drake.all import drakeify_my_urdf, add_watcher_and_build, GripperType
from brom_drake import robots
from brom_drake.file_manipulation.urdf.shapes.box import BoxDefinition
from brom_drake.file_manipulation.urdf.simple_writer.urdf_definition import (
    SimpleShapeURDFDefinition,
)
from brom_drake.productions.types.base import Configuration as BaseConfiguration
from brom_drake.productions.debug.grasping.attempt_grasp.with_static_wrist.production import (
    AttemptGraspWithStaticWrist,
)
from brom_drake.productions.debug.grasping.attempt_grasp.with_static_wrist.config import (
    Configuration as AttemptGraspWithStaticWristConfiguration,
)
from brom_drake.utils.model_instances import (
    get_name_of_first_body_in_urdf,
)


class AttemptGraspWithStaticWristTest(unittest.TestCase):
    def setUp(self):
        # Create erlenmeyer flask urdf
        erlenmeyer_flask_file = str(
            impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
        )

        drakeified_flask_urdf = drakeify_my_urdf(
            erlenmeyer_flask_file,
            overwrite_old_logs=True,
            log_file_name="DemonstrateStaticGripTest_AddManipulandToPlant_flask.log",
        )
        self.drakeified_flask_urdf = str(drakeified_flask_urdf)

    def create_far_production(self, initial_gripper_command=None):
        """
        Description
        -----------
        Creates a production whose gripper is far from the flask,
        using a new configuration with the given initial gripper command.
        """
        return AttemptGraspWithStaticWrist(
            path_to_object=self.drakeified_flask_urdf,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
            X_ObjectTarget=RigidTransform(p=np.array([-0.2, 0.0, 0.5])),
            config=AttemptGraspWithStaticWristConfiguration(
                base=BaseConfiguration(
                    meshcat_port_number=None,  # Use None for CI
                ),
                initial_gripper_command=initial_gripper_command,
            ),
        )

    def test_initial_gripper_command1(self):
        """
        Description
        -----------
        This test verifies that the gripper starts fully open by default.
        """
        # Setup
        production = self.create_far_production()

        # Build
        _, diagram_context = production.add_cast_and_build()
        plant = production.plant
        plant_context = plant.GetMyContextFromRoot(diagram_context)

        # Verify
        np.testing.assert_allclose(
            plant.GetPositions(plant_context, production.gripper_model_index),
            np.zeros((6,)),
            atol=1e-9,
        )

    def test_initial_gripper_command2(self):
        """
        Description
        -----------
        This test verifies that the gripper starts partially closed when the
        configuration asks it to, with its joints respecting the URDF's mimic signs,
        and that it holds that position before it starts closing.
        """
        # Setup
        production = self.create_far_production(initial_gripper_command=[0.5])

        # Build
        diagram, diagram_context = production.add_cast_and_build()
        plant = production.plant
        plant_context = plant.GetMyContextFromRoot(diagram_context)

        # Verify the start
        q_start = plant.GetPositions(plant_context, production.gripper_model_index)
        self.assertGreater(q_start[0], 0.2)
        np.testing.assert_allclose(
            q_start, q_start[0] * np.array([1.0, -1.0, 1.0, 1.0, -1.0, 1.0])
        )

        # Verify that the joints stay near their initial positions
        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()

        for t in [0.01, 0.1, 1.0]:
            simulator.AdvanceTo(t)
            q_later = plant.GetPositions(plant_context, production.gripper_model_index)
            np.testing.assert_allclose(q_later, q_start, atol=0.02)

    def test_initial_gripper_command3(self):
        """
        Description
        -----------
        This test verifies that the constructor raises a ValueError when the
        initial gripper command does not match the shape of the grasp gripper command.
        """
        with self.assertRaises(ValueError):
            self.create_far_production(initial_gripper_command=[0.3, 0.3])

    def test_default_configuration1(self):
        """
        Description
        -----------
        This test verifies that productions created without a configuration
        do not share one (the constructor writes the Meshcat port into it).
        """
        # Setup
        production1 = AttemptGraspWithStaticWrist(
            path_to_object=self.drakeified_flask_urdf,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
            X_ObjectTarget=RigidTransform(),
            meshcat_port_number=7999,
        )
        production2 = AttemptGraspWithStaticWrist(
            path_to_object=self.drakeified_flask_urdf,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
            X_ObjectTarget=RigidTransform(),
        )

        # Verify
        self.assertIsNot(production1.config, production2.config)
        self.assertEqual(production1.config.base.meshcat_port_number, 7999)
        self.assertEqual(
            production2.config.base.meshcat_port_number,
            BaseConfiguration().meshcat_port_number,
        )

    def test_find_floor_z1(self):
        """
        Description
        -----------
        This test verifies that we can find the X_WorldFloor without any errors.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf

        # Create the production
        config = AttemptGraspWithStaticWristConfiguration(
            base=BaseConfiguration(
                meshcat_port_number=None,  # Use None for CI
            ),
        )
        production = AttemptGraspWithStaticWrist(
            path_to_object=flask_urdf,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
            config=config,
        )
        production.add_supporting_cast()

        # Call the method
        z_floor = production.find_floor_z_via_bounding_box(debug_flag=True)

        self.assertLess(
            z_floor,
            1_000.0,
        )

    def test_demonstrate_controlling_floor1(self):
        """
        Description
        -----------
        This test verifies that we can find the X_WorldFloor without any errors.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf

        # Create the production
        builder = DiagramBuilder()
        plant = builder.AddSystem(MultibodyPlant(time_step=0.0))

        # Create a box urdf for the floor
        floor_geometry_defn = BoxDefinition(size=[10.0, 10.0, 0.1])
        floor_thickness = floor_geometry_defn.size[2]

        # Create a urdf for the floor
        floor_urdf_defn = SimpleShapeURDFDefinition(
            name="floor",
            shape=floor_geometry_defn,
        )
        floor_urdf = str(floor_urdf_defn.write_to_file())

        # Add the floor to the plant
        floor_model_idcs = Parser(plant=plant).AddModels(floor_urdf)
        self.floor_model_index = floor_model_idcs[0]

        # Create joint + actuator for the floor
        floor_joint = plant.AddJoint(
            PrismaticJoint(
                name="floor_joint",
                frame_on_parent=plant.world_frame(),
                frame_on_child=plant.GetFrameByName(
                    get_name_of_first_body_in_urdf(floor_urdf)
                ),
                axis=[0, 0, 1],
            )
        )

        self.floor_actuator = plant.AddJointActuator(
            "floor_elevation_actuator",
            floor_joint,
        )

        plant.Finalize()

        # Build the production and simulate it
        watcher, diagram, diagram_context = add_watcher_and_build(
            builder,
            watcher_dir=Path("./brom/watcher/attempt_grasp_test/controlling_floor1"),
        )

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.Initialize()
        simulator.AdvanceTo(0.1)
        self.assertTrue(True)

    def test_add_floor_to_plant1(self):
        """
        Description
        -----------
        This test verifies that we can add the floor to the plant without any errors.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf

        # Create the production
        config = AttemptGraspWithStaticWristConfiguration(
            base=BaseConfiguration(
                meshcat_port_number=None,  # Use None for CI
            ),
        )
        production = AttemptGraspWithStaticWrist(
            path_to_object=flask_urdf,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
            config=config,
        )

        # Call the method
        production.add_floor_to_plant()

        # Finalize the plant
        production.plant.Finalize()

        # Verift that plant has a single actuator (for the floor)
        self.assertEqual(
            production.plant.num_actuators(),
            1,
        )

    def test_floor_z_creates_no_collisions1(self):
        """
        Description
        -----------
        This test verifies that when the floor value is at an incorrect height,
        the Production can correctly identify that the floor and flask will
        be in collision.

        We will create the beaker + Robotiq gripper and set the floor
        to 0.1 which should certainly cause collision in the scene.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf

        # Create the production
        config = AttemptGraspWithStaticWristConfiguration(
            base=BaseConfiguration(
                meshcat_port_number=None,  # Use None for CI
            ),
        )
        production = AttemptGraspWithStaticWrist(
            path_to_object=flask_urdf,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.7]),
            config=config,
        )

        # Call the method
        tf = production.floor_z_creates_collisions(
            floor_z_translation=0.035,
            debug_flag=True,
        )

        self.assertTrue(
            tf,
            msg="The floor height of 0.1 should cause a collision between the flask and the floor.",
        )


if __name__ == "__main__":
    unittest.main()

from pydrake.all import (
    AddMultibodyPlant,
    DiagramBuilder,
    MultibodyPlant,
    MultibodyPlantConfig,
    Parser,
    SceneGraph
)
import unittest

# Internal Imports
from brom_drake.file_manipulation.urdf.shapes import SphereDefinition
from brom_drake.file_manipulation.urdf import SimpleShapeURDFDefinition
from brom_drake.utils import Puppetmaker

class PuppetmakerTest(unittest.TestCase):
    def setUp(self):
        pass

    def test_init1_all_actuators_are_created(self):
        # Setup
        builder = DiagramBuilder()

        # Create plant with the single object
        plant_sg_pair = AddMultibodyPlant(MultibodyPlantConfig(), builder)
        plant: MultibodyPlant = plant_sg_pair[0]

        # Create a simple shape URDF
        sphere_geometry_defn = SphereDefinition(radius=0.1)
        sphere_urdf_defn = SimpleShapeURDFDefinition(
            name="test_sphere",
            shape=sphere_geometry_defn,
            mass=0.2,
        )
        sphere_urdf = sphere_urdf_defn.write_to_file()

        # Add the test sphere to the plant
        sphere_model_idcs = Parser(plant=plant).AddModels(sphere_urdf)

        # Create the puppetmaker and add strings to the sphere
        puppetmaker0 = Puppetmaker(plant)
        puppetmaker0.add_strings_for(sphere_model_idcs[0])

        # Finalize
        plant.Finalize()

        # Verify that the actuator ports created by the puppetmaker exist
        all_input_port_names = [
            plant.get_input_port(port_index).get_name()
            for port_index in range(plant.num_input_ports())
        ]

        puppetmaker_port_exists = False
        for port_name_ii in all_input_port_names:
            if puppetmaker0.config.name in port_name_ii:
                puppetmaker_port_exists = True

        self.assertTrue(puppetmaker_port_exists)


    def test_init2_fails_if_plant_already_finalized(self):
        # Setup
        builder = DiagramBuilder()

        # Create plant with the single object
        plant_sg_pair = AddMultibodyPlant(MultibodyPlantConfig(), builder)
        plant: MultibodyPlant = plant_sg_pair[0]

        # Create a simple shape URDF
        sphere_geometry_defn = SphereDefinition(radius=0.1)
        sphere_urdf_defn = SimpleShapeURDFDefinition(
            name="test_sphere",
            shape=sphere_geometry_defn,
            mass=0.2,
        )
        sphere_urdf = sphere_urdf_defn.write_to_file()

        # Add the test sphere to the plant
        sphere_model_idcs = Parser(plant=plant).AddModels(sphere_urdf)

        # Finalize
        plant.Finalize()

        # Try to call the puppetmaker
        try:
            puppetmaker0 = Puppetmaker(plant)
            puppetmaker0.add_strings_for(sphere_model_idcs[0])
            self.assertTrue(
                False,
                "The puppetmaker constructor should fail before reaching here!"
            )
        except Exception as e:
            self.assertIn(
                "finalize",
                str(e)
            )

    def test_add_actuators_for1(self):
        """
        Description
        -----------
        This test verifies that the output of add_actuators_for is as expected.
        We should see 6 joints and 6 actuators created by default.
        """
        # Setup
        builder = DiagramBuilder()

        # Create plant with the single object
        plant_sg_pair = AddMultibodyPlant(MultibodyPlantConfig(), builder)
        plant: MultibodyPlant = plant_sg_pair[0]

        # Create a simple shape URDF
        sphere_geometry_defn = SphereDefinition(radius=0.1)
        sphere_urdf_defn = SimpleShapeURDFDefinition(
            name="test_sphere",
            shape=sphere_geometry_defn,
            mass=0.2,
        )
        sphere_urdf = sphere_urdf_defn.write_to_file()

        # Add the test sphere to the plant
        sphere_model_idcs = Parser(plant=plant).AddModels(sphere_urdf)

        # Create the puppetmaker and add strings to the sphere
        puppetmaker0 = Puppetmaker(plant)
        puppet_signature1 = puppetmaker0.add_actuators_for(sphere_model_idcs[0])

        # Check the signature's value
        self.assertEqual(len(puppet_signature1.prismatic_joints), 3)
        self.assertEqual(len(puppet_signature1.revolute_joints), 3)

        # Finalize plant
        plant.Finalize()

        # Check for all of the actuators that the puppetmaker should have added to the plant
        for joint_actuator in puppet_signature1.all_joint_actuators:
            try:
                plant.GetJointActuatorByName(joint_actuator.name())
            except Exception as e:
                self.fail(f"Failed to find joint actuator {joint_actuator.name}: {e}")

    def test_add_puppet_controller_for1(self):
        """
        Description
        -----------
        This test verifies that the add_puppet_controller_for() method correctly
        creates:
        - A Demultiplexer to send the combined control signals to the actuators and
        - A PID controller for the vector of inputs to the puppet.
        """
        """
        Description
        -----------
        This test verifies that the output of add_actuators_for is as expected.
        We should see 6 joints and 6 actuators created by default.
        """
        # Setup
        builder = DiagramBuilder()

        # Create plant with the single object
        plant_sg_pair = AddMultibodyPlant(MultibodyPlantConfig(), builder)
        plant: MultibodyPlant = plant_sg_pair[0]

        # Create a simple shape URDF
        sphere_geometry_defn = SphereDefinition(radius=0.1)
        sphere_urdf_defn = SimpleShapeURDFDefinition(
            name="test_sphere",
            shape=sphere_geometry_defn,
            mass=0.2,
        )
        sphere_urdf = sphere_urdf_defn.write_to_file()

        # Add the test sphere to the plant
        sphere_model_idcs = Parser(plant=plant).AddModels(sphere_urdf)

        # Create the puppetmaker and add strings to the sphere
        puppetmaker0 = Puppetmaker(plant)
        puppet_signature1 = puppetmaker0.add_actuators_for(sphere_model_idcs[0])

        # Finalize plant
        plant.Finalize()

        # Call the method
        puppetmaker0.add_puppet_controller_for(puppet_signature1, builder)

if __name__ == '__main__':
    unittest.main()

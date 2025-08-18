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

        # Try to call the puppetmaker
        puppetmaker0 = Puppetmaker(sphere_model_idcs[0], plant)

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
            puppetmaker0 = Puppetmaker(sphere_model_idcs[0], plant)
            self.assertTrue(False, "The puppetmaker constructor should fail before reaching here!")
        except Exception as e:
            self.assertIn(
                "finalize",
                str(e)
            )

if __name__ == '__main__':
    unittest.main()

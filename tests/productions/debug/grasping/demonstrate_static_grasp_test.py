from importlib import resources as impresources
import numpy as np
from pydrake.all import (
    RigidTransform,
    RollPitchYaw,
    Simulator,
)
import unittest

# Internal Imports
from brom_drake.all import drakeify_my_urdf
from brom_drake.productions.debug.grasping.demonstrate_static_grasp import (
    DemonstrateStaticGrasp,
)
from brom_drake import robots

class DemonstrateStaticGraspTest(unittest.TestCase):
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

        # Create the gripper urdf
        gripper_urdf_path = str(
            impresources.files(robots) / "models/robotiq/2f_85_gripper-no-mimic/urdf/robotiq_2f_85.urdf"
        )
        self.gripper_urdf_path = gripper_urdf_path

    def test_add_manipuland_to_plant1(self):
        """
        Description
        -----------
        This test verifies that we can add the manipuland to the plant without any errors.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf  
        gripper_urdf = self.gripper_urdf_path      

        # Create the production
        production = DemonstrateStaticGrasp(
            path_to_object=flask_urdf,
            path_to_gripper=gripper_urdf,
            meshcat_port_number=None, # Use None for CI
        )

        # Call the method
        production.add_manipuland_to_plant()

        # Finalize the plant
        production.plant.Finalize()
        # production.connect_to_meshcat()
        
        # Build the production and simulate
        diagram, diagram_context = production.build_production()
        simulator = Simulator(diagram, diagram_context)

        # Set up simulation
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)
        simulator.Initialize()
        simulator.AdvanceTo(0.1)

        self.assertTrue(True)

    def test_add_gripper_to_plant1(self):
        """
        Description
        -----------
        This test verifies that we can add the manipuland AND
        the gripper to the plant without any errors.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf  
        gripper_urdf = self.gripper_urdf_path      

        # Create the production
        production = DemonstrateStaticGrasp(
            path_to_object=flask_urdf,
            path_to_gripper=gripper_urdf,
            meshcat_port_number=None, # Use None for CI
        )

        # Call the method
        production.add_manipuland_to_plant()
        production.add_gripper_to_plant()

        # Finalize the plant
        production.plant.Finalize()
        production.connect_to_meshcat()
        
        # Build the production and simulate
        diagram, diagram_context = production.build_production()
        simulator = Simulator(diagram, diagram_context)

        production.show_me_system.mutable_plant_context = production.plant.GetMyMutableContextFromRoot(
            production.diagram_context,
        )


        # Set up simulation
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)
        simulator.Initialize()
        simulator.AdvanceTo(0.1)

        self.assertTrue(True)

    def test_add_cast_and_build1(self):
        """
        Description
        -----------
        This test verifies how we can use the add_cast_and_build method
        to build a DemonstrateStaticGrasp production.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf  
        gripper_urdf = self.gripper_urdf_path      
        X_ObjectTarget = RigidTransform(
            p=np.array([-0.08, 0.05, 0.15]),
            rpy=RollPitchYaw(0.0, np.pi/2.0, 0.0),
        )


        # Create the production
        production = DemonstrateStaticGrasp(
            path_to_object=flask_urdf,
            path_to_gripper=gripper_urdf,
            meshcat_port_number=None, # Use None for CI
            X_ObjectTarget=X_ObjectTarget,
        )

        # Call the method
        diagram, diagram_context = production.add_cast_and_build()

        # Set up simulation
        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(1.0)
        simulator.set_publish_every_time_step(False)
        simulator.Initialize()
        simulator.AdvanceTo(10.1)

        self.assertTrue(True)


if __name__ == "__main__":
    unittest.main()
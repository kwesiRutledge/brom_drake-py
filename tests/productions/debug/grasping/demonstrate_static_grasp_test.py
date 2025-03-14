from importlib import resources as impresources
import numpy as np
from pydrake.all import (
    RigidTransform,
    RollPitchYaw,
    SceneGraph,
    Simulator,
)
from pydrake.all import Role as DrakeRole
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
        simulator.AdvanceTo(0.1)

        self.assertTrue(True)


    def test_add_gripper_to_plant1(self):
        """
        Description
        -----------
        This test verifies that we can add the gripper to the plant
        and that it creates a new RigidBodyFrame for the base when we
        call the vanilla version of add_gripper_to_plant.
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
        production.add_gripper_to_plant()

        # Finalize the plant
        production.plant.Finalize()

        # Verify that the gripper frame is added to the base
        scene_graph: SceneGraph = production.scene_graph
        
        # Search through the geometries in the scene for the ones created by the 
        # AddMultibodyTriad method on the gripper base.
        all_geometries = scene_graph.model_inspector().GetAllGeometryIds()
        foundTriad = False
        for geometry_id in all_geometries:
            frame_id = scene_graph.model_inspector().GetFrameId(geometry_id)
            try:
                frame_geometry_x = scene_graph.model_inspector().GetGeometryIdByName(
                    frame_id,
                    DrakeRole.kIllustration,
                    production.target_frame_name_on_gripper + " x-axis"
                )
                foundTriad = foundTriad or True
            except:
                pass
                        

        self.assertTrue(foundTriad)

    def test_add_gripper_to_plant2(self):
        """
        Description
        -----------
        This test verifies that we can add the gripper to the plant
        and that it creates two new MultibodyTriad objects for the base
        and for the target when we choose a target body that is NOT the
        base.
        """
        # Setup
        flask_urdf = self.drakeified_flask_urdf  
        gripper_urdf = self.gripper_urdf_path      

        # Create the production
        production = DemonstrateStaticGrasp(
            path_to_object=flask_urdf,
            path_to_gripper=gripper_urdf,
            meshcat_port_number=None, # Use None for CI
            target_body_on_gripper="left_inner_finger_pad",
            always_show_gripper_base_frame=True,
        )

        # Call the method
        production.add_gripper_to_plant()

        # Finalize the plant
        production.plant.Finalize()

        # Verify that the gripper frame is added to the base
        scene_graph: SceneGraph = production.scene_graph
        
        # Search through the geometries in the scene for the ones created by the 
        # AddMultibodyTriad method on the gripper target.
        all_geometries = scene_graph.model_inspector().GetAllGeometryIds()
        foundTargetTriad = False
        for geometry_id in all_geometries:
            frame_id = scene_graph.model_inspector().GetFrameId(geometry_id)
            try:
                frame_geometry_x = scene_graph.model_inspector().GetGeometryIdByName(
                    frame_id,
                    DrakeRole.kIllustration,
                    production.target_frame_name_on_gripper + " x-axis"
                )
                foundTargetTriad = foundTargetTriad or True
            except:
                pass

        self.assertTrue(foundTargetTriad)

        # Search through the geometries in the scene for the ones created by the
        # AddMultibodyTriad method on the gripper base.
        foundBaseTriad = False
        gripper_base_frame = production.plant.GetFrameByName(
            production.get_name_of_first_frame_in_gripper()
        )
        for geometry_id in all_geometries:
            frame_id = scene_graph.model_inspector().GetFrameId(geometry_id)
            try:
                frame_geometry_x = scene_graph.model_inspector().GetGeometryIdByName(
                    frame_id,
                    DrakeRole.kIllustration,
                    gripper_base_frame.name() + " x-axis"
                )
                foundBaseTriad = foundBaseTriad or True
            except:
                pass

        self.assertTrue(foundBaseTriad)

if __name__ == "__main__":
    unittest.main()
import importlib.resources as impresources
import numpy as np
from pydrake.all import (
    AddMultibodyPlant,
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    MultibodyPlant,
    Parser,
    RigidTransform,
    RollPitchYaw
)
import unittest

# Internal Imports
from brom_drake.all import drakeify_my_urdf
from brom_drake.file_manipulation.urdf.shapes import BoxDefinition
from brom_drake.file_manipulation.urdf import SimpleShapeURDFDefinition
from brom_drake import robots
from brom_drake.utils.initial_condition_manager import InitialCondition

class InitialConditionTest(unittest.TestCase):
    def setUp(self):
        # Setup
        self.builder = DiagramBuilder()

        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=1e-3)

    def test_set_initial_pose1(self):
        # Setup
        pose0 = RigidTransform(
            p=np.array([0.0, 0.0, 0.5]),
            rpy=RollPitchYaw(roll=np.pi/2.0, pitch=0.0, yaw=0.0)
        )

        # Create Cube 3d model
        simple_cube = BoxDefinition(size=[0.05,0.05,0.05])
        cube_urdf_defn = SimpleShapeURDFDefinition(
            name="tutorial-cube",
            shape=simple_cube,
            color=[0.1,0.1,0.5,1.0],
        )
        cube_urdf_path = cube_urdf_defn.write_to_file()

        # Add cube urdf to plant
        cube_model = Parser(plant=self.plant).AddModels(cube_urdf_path)[0]

        # Create the initial condition
        ic_condition = InitialCondition(
            model_instance_index=cube_model,
            pose_wrt_parent=pose0
        )
        ic_condition.set_initial_pose(plant=self.plant)

        # Build the diagram
        self.plant.Finalize()
        diagram = self.builder.Build()

        # Get default context and verify that the initial pose is set correctly
        diagram_context = diagram.CreateDefaultContext()
        context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        
        plant: MultibodyPlant = self.plant
        cube_body_indices = plant.GetBodyIndices(cube_model)
        pose_out = plant.GetFreeBodyPose(context, plant.get_body(cube_body_indices[0]))

        self.assertTrue(
            np.allclose(
                pose_out.translation(),
                pose0.translation())
        )
        self.assertTrue(
            np.allclose(
                pose_out.rotation().ToRollPitchYaw().vector(),
                pose0.rotation().ToRollPitchYaw().vector()
            )
        )

    def test_set_initial_configuration1(self):
        """
        Description
        -----------
        Test setting the initial configuration for a model instance.
        We'll import a UR10e model for this test and then put it in a simple
        6-DOF configuration.
        """
        # Setup
        q0 = np.array([0.0, -np.pi/4.0, 0.0, -np.pi/2.0, 0.0, np.pi/3.0])

        # Add the ur10e model
        original_arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        new_arm_urdf_path = drakeify_my_urdf(
            original_arm_urdf_path,
            overwrite_old_logs=True,
            log_file_name="test_set_initial_configuration11.log",
        )
        ur10e_model = Parser(plant=self.plant).AddModels(str(new_arm_urdf_path))[0]

        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link", ur10e_model),
        )

        self.plant.Finalize()

        # Set the initial configuration
        ic_condition = InitialCondition(
            model_instance_index=ur10e_model,
            configuration=q0,
        )
        ic_condition.set_initial_configuration(plant=self.plant)

        # Build the diagram
        diagram = self.builder.Build()

        # Get default context and verify that the initial pose is set correctly
        diagram_context = diagram.CreateDefaultContext()
        context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        pose_out = self.plant.GetPositions(context, ur10e_model)

        print(pose_out)
        print(q0)

        self.assertTrue(
            np.allclose(
                pose_out,
                q0)
        )

if __name__ == '__main__':
    unittest.main()

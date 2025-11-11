import importlib.resources as impresources
import numpy as np
from pydrake.all import (
    AddMultibodyPlant,
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    ModelInstanceIndex,
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
from brom_drake.utils.initial_condition_manager import InitialCondition, InitialConditionManager

class InitialConditionManagerTest(unittest.TestCase):
    def setUp(self):
        # Setup
        self.builder = DiagramBuilder()

        # Create plant and scene graph
        self.plant, self.scene_graph = AddMultibodyPlantSceneGraph(self.builder, time_step=1e-3)

    def add_cube_model_to_plant(self, name: str = "tutorial-cube") -> ModelInstanceIndex:
        # Create Cube 3d model
        simple_cube = BoxDefinition(size=[0.05,0.05,0.05])
        cube_urdf_defn = SimpleShapeURDFDefinition(
            name=name,
            shape=simple_cube,
            color=[0.1,0.1,0.5,1.0],
        )
        cube_urdf_path = cube_urdf_defn.write_to_file()

        # Add cube urdf to plant
        cube_model = Parser(plant=self.plant).AddModels(cube_urdf_path)[0]

        return cube_model
    
    def test_add_initial_condition1(self):
        # Setup
        pose0 = RigidTransform(
            p=np.array([0.0, 0.0, 0.5]),
            rpy=RollPitchYaw(roll=np.pi/2.0, pitch=0.0, yaw=0.0)
        )

        # Add cube model to plant
        cube_model = self.add_cube_model_to_plant()

        # Create the initial condition
        ic_manager = InitialConditionManager()
        ic_manager.add_initial_condition(
            InitialCondition(
                model_instance_index=cube_model,
                pose_wrt_parent=pose0
            )
        )

        # Check that there is one initial condition stored in the internal variable
        self.assertEqual(len(ic_manager._ic_tuples), 1)

    def test_add_initial_pose1(self):
        # Setup
        pose0 = RigidTransform(
            p=np.array([0.0, 0.0, 0.5]),
            rpy=RollPitchYaw(roll=np.pi/2.0, pitch=0.0, yaw=0.0)
        )

        # Add cube model to plant
        cube_model = self.add_cube_model_to_plant()

        # Create the initial condition
        ic_manager = InitialConditionManager()
        ic_manager.add_initial_pose(
            model_instance_index=cube_model,
            pose_wrt_parent=pose0
        )

        # Check that there is one initial condition stored in the internal variable
        self.assertEqual(len(ic_manager._ic_tuples), 1)

    def test_add_initial_configuration1(self):
        # Setup
        configuration0 = np.array([0.5])

        # Add cube model to plant
        cube_model = self.add_cube_model_to_plant()

        # Create the initial condition
        ic_manager = InitialConditionManager()
        ic_manager.add_initial_configuration(
            model_instance_index=cube_model,
            configuration=configuration0
        )

        # Check that there is one initial condition stored in the internal variable
        self.assertEqual(len(ic_manager._ic_tuples), 1)

    def test_set_all_initial_conditions1(self):
        """
        Description
        -----------
        Test setting both the initial pose and configuration for a model instance.
        """
        # Setup
        pose0 = RigidTransform(
            p=np.array([0.0, 0.0, 0.5]),
            rpy=RollPitchYaw(roll=np.pi/2.0, pitch=0.0, yaw=0.0)
        )
        configuration0 = np.array([0.0, -np.pi/4.0, 0.0, -np.pi/2.0, 0.0, np.pi/3.0])

        # Add the ur10e model
        original_arm_urdf_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf",
        )
        new_arm_urdf_path = drakeify_my_urdf(
            original_arm_urdf_path,
            overwrite_old_logs=True,
            log_file_name="test_set_all_initial_conditions1.log",
        )
        ur10e_model = Parser(plant=self.plant).AddModels(str(new_arm_urdf_path))[0]

        self.plant.WeldFrames(
            self.plant.world_frame(),
            self.plant.GetFrameByName("base_link", ur10e_model)
        )

        # Add a cube to the plant
        extra_cube_idx = self.add_cube_model_to_plant(name="extra-cube")

        # Create the initial condition
        ic_manager = InitialConditionManager()
        ic_manager.add_initial_pose(
            model_instance_index=extra_cube_idx,
            pose_wrt_parent=pose0
        )
        ic_manager.add_initial_configuration(
            model_instance_index=ur10e_model,
            configuration=configuration0
        )

        # Finalize the plant
        self.plant.Finalize()

        # Set all initial conditions in the plant
        ic_manager.set_all_initial_conditions(plant=self.plant)

        # Build the diagram
        diagram = self.builder.Build()

        # Get default context and verify that the initial pose is set correctly
        diagram_context = diagram.CreateDefaultContext()
        context = diagram.GetMutableSubsystemContext(self.plant, diagram_context)
        
        plant: MultibodyPlant = self.plant
        extra_cube_body_indices = plant.GetBodyIndices(extra_cube_idx)
        pose_out = plant.GetFreeBodyPose(context, plant.get_body(extra_cube_body_indices[0]))

        # Verify that the initial configuration is set correctly
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

        # Verify that the initial configuration is set correctly
        q_out = plant.GetPositions(context, ur10e_model)
        self.assertTrue(np.allclose(q_out, configuration0))

if __name__ == '__main__':
    unittest.main()
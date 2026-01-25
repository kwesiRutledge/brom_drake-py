from brom_drake import robots
from brom_drake.all import (
    drakeify_my_urdf,
    GripperType, 
    MeshReplacementStrategy,
    PortWatcher,
    ProductionID
)
from brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.production import AttemptGraspWithPuppeteerWrist
from brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.script import Script as AttemptGraspWithPuppeteerWristScript
from brom_drake.productions.debug.grasping.attempt_grasp.with_puppeteer_wrist.phases import AttemptGraspWithPuppeteerWristPhase
import numpy as np
from pathlib import Path
from pydrake.all import RigidTransform, Simulator
import importlib.resources as impresources
import unittest

class TestAttemptGraspWithPuppeteerWristProduction(unittest.TestCase):
    def setUp(self):
        # Create dummy object
        self.raw_erlenmeyer_flask_file = impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
        
        drakeified_flask_urdf = Path("brom/models/500ml/500ml.drake.urdf")
        if not drakeified_flask_urdf.exists():
            drakeified_flask_urdf = drakeify_my_urdf(
                str(self.raw_erlenmeyer_flask_file),
                overwrite_old_logs=True,
                log_file_name="DemonstrateStaticGripTest_AddManipulandToPlant_flask.log",
                # For you (yes, you!): Comment out the line below, to see what the default collision mesh looks like
                collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithConvexDecomposition,
            )
        self.manipuland_file = drakeified_flask_urdf
    
    def test_suggested_roles1(self):
        """
        Description
        -----------
        This test verifies that the suggested_roles method
        returns NO expected roles for the AttemptGraspWithPuppeteerWrist production.
        """
        # Setup
        production = AttemptGraspWithPuppeteerWrist(
            path_to_object=str(self.manipuland_file),
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            X_ObjectGripper_trajectory=[RigidTransform(), RigidTransform()],
        )

        # Get suggested roles
        roles = production.suggested_roles

        # Verify the expected roles are present
        self.assertLessEqual(roles, [])

    def test_id1(self):
        """
        Description
        -----------
        This test verifies that the id() method returns the appropriate 
        Enum value for the AttemptGraspWithPuppeteerWrist production.
        It should not be the undefined id.
        """
        # Setup
        production = AttemptGraspWithPuppeteerWrist(
            path_to_object=str(self.manipuland_file),
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            X_ObjectGripper_trajectory=[RigidTransform(), RigidTransform()],
        )

        # Verify the id
        self.assertEqual(
            production.id,
            ProductionID.kAttemptGraspWithPuppeteer,
        )

    def test_add_cast_and_build1(self):
        """
        Description
        -----------
        This test verifies that the add_cast_and_build method
        successfully builds the production without errors when given:
        - The Erlenmeyer flask model
        - the Robotiq 2f-85 gripper
        - An unsuccessful two-point gripper wrist trajectory

        """
        # Setup
        WorldGripper_far = RigidTransform(
            p=np.array([-0.2, 0.0, 0.5]),
        )
        production = AttemptGraspWithPuppeteerWrist(
            path_to_object=str(self.manipuland_file),
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            X_ObjectGripper_trajectory=[WorldGripper_far, WorldGripper_far],
            meshcat_port_number=None, # Use None for CI
        )

        # Build with watcher
        diagram, diagram_context = production.add_cast_and_build()
        watcher = production.watcher

        # Verify that the diagram and context are created
        self.assertIsNotNone(diagram)
        self.assertIsNotNone(diagram_context)

        # Simulate the production for a short time to ensure no errors occur
        script: AttemptGraspWithPuppeteerWristScript = production.config.script
        t_final = script.total_time()

        simulator = Simulator(diagram, diagram_context)
        simulator.set_target_realtime_rate(2.0)
        simulator.set_publish_every_time_step(False)
        simulator.Initialize()
        simulator.AdvanceTo(t_final)

        # Use the watcher to collect the states of the erlenmeyer flask and verify that they
        # were "falling" after the grasp attempt (i.e., the z velocity should be negative and significant after the grasp time)
        t_drop_start = script.start_time_of_phase(AttemptGraspWithPuppeteerWristPhase.kFloorDrop)
        
        plant_name = production.plant.get_name()
        flask_name = production.manipuland_name
        flask_state_port_name = f"{flask_name}_state"
        
        flask_state_logger: PortWatcher = watcher.port_watchers[plant_name][flask_state_port_name]
        flask_state_log = flask_state_logger.get_vector_log_sink().FindLog(diagram_context)
        flask_state_times = flask_state_log.sample_times()
        flask_state_data = flask_state_log.data()

        # Select the velocity data after the drop start time
        flask_states_after_drop = flask_state_data[:, flask_state_times >= t_drop_start]
        flask_velocities_after_drop = flask_states_after_drop[7:13, :]  # Velocities are in indices 7 to 12
        flask_speeds_after_drop = np.linalg.norm(flask_velocities_after_drop, axis=0)
        print(f"Found {len(flask_speeds_after_drop)} flask-speeds after drop start time of {t_drop_start} seconds.")

        print("Maximum flask speed after drop start time:", np.max(flask_speeds_after_drop))

        # The grasp should have failed, so the flask should be falling significantly after the drop start time
        self.assertTrue(
            np.any(flask_speeds_after_drop > 1.0),
            "Expected to find large flask speeds (due to free falling) for the flask after the drop start time, but none was found."
        )

if __name__ == "__main__":
    unittest.main()
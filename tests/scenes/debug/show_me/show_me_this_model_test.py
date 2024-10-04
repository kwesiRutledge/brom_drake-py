from importlib import resources as impresources
import numpy as np
import unittest

# Internal imports
from brom_drake import robots
from brom_drake.all import drakeify_my_urdf
from brom_drake.scenes.debug import show_me_this_model_in_sim

class ShowMeThisModelTest(unittest.TestCase):
    def test_runs(self):
        """
        Description
        -----------
        This test checks if the scene runs without errors.
        :return:
        """
        # Setup
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )

        # Convert the URDF
        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
        )

        # Visualize the URDF using the "show-me-this-model" feature
        time_step = 1e-3
        scene, diagram, diagram_context, simulator = show_me_this_model_in_sim(
            new_urdf_path,
            desired_joint_positions=[0.0, 0.0, -np.pi / 4.0, 0.0, 0.0, 0.0],
            time_step=time_step,
        )

        # Set up simulation
        simulator.set_target_realtime_rate(2.0)
        simulator.set_publish_every_time_step(False)

        # Run simulation
        simulator.Initialize()
        simulator.AdvanceTo(5.0)

        # Successfully Ran
        self.assertTrue(True)

if __name__ == "__main__":
    unittest.main()
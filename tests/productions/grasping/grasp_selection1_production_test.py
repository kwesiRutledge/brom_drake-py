from brom_drake import robots
from brom_drake.all import (
    drakeify_my_urdf,
    GripperType,
    MeshReplacementStrategy,
    PortWatcher,
    ProductionID,
)
from brom_drake.productions.grasping.config import (
    Configuration as GraspSelection1Configuration,
)
from brom_drake.productions.grasping.grasp_selection import GraspSelection1
from brom_drake.productions.grasping.phases import GraspSelection1Phase
from brom_drake.productions.grasping.script import Script as GraspSelection1Script

import numpy as np
from pathlib import Path
from pydrake.all import RigidTransform, RollPitchYaw, Simulator
import importlib.resources as impresources
from typing import List
import unittest


class TestGraspSelection1Production(unittest.TestCase):
    def setUp(self):
        # Create dummy object
        self.raw_erlenmeyer_flask_file = (
            impresources.files(robots) / "models/erlenmeyer_flask/500ml.urdf"
        )

        drakeified_flask_urdf = Path("brom/models/500ml/500ml.drake.urdf")
        if not drakeified_flask_urdf.exists():
            drakeified_flask_urdf = drakeify_my_urdf(
                str(self.raw_erlenmeyer_flask_file),
                overwrite_old_logs=True,
                log_file_name="GraspSelection1Test_AddManipulandToPlant_flask.log",
                collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithConvexDecomposition,
            )
        self.manipuland_file = drakeified_flask_urdf

        # Create a grasp pose and pre-grasp pose (relative to the object)
        self.X_ObjectGrasp = RigidTransform(
            p=np.array([-0.08, 0.05, 0.2]),
            rpy=RollPitchYaw(0.0, np.pi / 2.0, 0.0),
        )
        self.X_ObjectPreGrasp = self.X_ObjectGrasp.multiply(
            RigidTransform(p=np.array([0.0, 0.0, -0.2]))
        )

    def create_production(
        self, config: GraspSelection1Configuration = None
    ) -> GraspSelection1:
        """
        Description
        -----------
        Creates a GraspSelection1 production for the Erlenmeyer flask
        and the Robotiq 2f-85 gripper.

        A fresh configuration is created for each production
        (to avoid sharing the default configuration between tests).
        """
        if config is None:
            config = GraspSelection1Configuration()

        return GraspSelection1(
            path_to_object=str(self.manipuland_file),
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_joint_positions=np.array([0.7]),
            meshcat_port_number=None,  # Use None for CI
            config=config,
        )

    def two_waypoint_algorithm(self, path_to_object: str) -> List[RigidTransform]:
        """
        Description
        -----------
        A simple grasp selection algorithm that ignores the object
        and returns the pre-grasp and grasp poses.
        """
        return [self.X_ObjectPreGrasp, self.X_ObjectGrasp]

    def test_init1(self):
        """
        Description
        -----------
        This test verifies that the constructor raises a ValueError
        when given a gripper that is not supported.
        """
        with self.assertRaises(ValueError):
            GraspSelection1(
                path_to_object=str(self.manipuland_file),
                gripper_choice=GripperType.NoGripper,
                grasp_joint_positions=np.array([0.7]),
                config=GraspSelection1Configuration(),
            )

    def test_init2(self):
        """
        Description
        -----------
        This test verifies that the constructor does not define
        the gripper trajectory (it is defined by the grasp selection algorithm).
        """
        # Setup
        production = self.create_production()

        # Verify
        self.assertIsNone(production.X_ObjectGripper_trajectory)
        self.assertIsNone(production.grasp_selection_algorithm)

    def test_suggested_roles1(self):
        """
        Description
        -----------
        This test verifies that the suggested_roles method
        returns NO roles for the GraspSelection1 production.
        """
        # Setup
        production = self.create_production()

        # Verify
        self.assertEqual(production.suggested_roles, [])

    def test_id1(self):
        """
        Description
        -----------
        This test verifies that the id property returns the appropriate
        Enum value for the GraspSelection1 production.
        """
        # Setup
        production = self.create_production()

        # Verify the id
        self.assertEqual(production.id, ProductionID.kGraspSelection1)

    def test_cast_grasp_selection_algorithm1(self):
        """
        Description
        -----------
        This test verifies that the method:
        - calls the algorithm with the path to the object,
        - saves the returned list of poses as the gripper trajectory, and
        - saves the last pose as the grasp pose.
        """
        # Setup
        production = self.create_production()
        received_paths = []

        def algorithm(path_to_object: str) -> List[RigidTransform]:
            received_paths.append(path_to_object)
            return [self.X_ObjectPreGrasp, self.X_ObjectGrasp]

        # Cast the algorithm
        trajectory = production.cast_grasp_selection_algorithm(algorithm)

        # Verify
        self.assertEqual(received_paths, [str(self.manipuland_file)])
        self.assertEqual(len(trajectory), 2)
        self.assertIs(production.X_ObjectGripper_trajectory, trajectory)
        self.assertTrue(
            production.X_ObjectGripper.IsExactlyEqualTo(self.X_ObjectGrasp)
        )
        self.assertIs(production.grasp_selection_algorithm, algorithm)

    def test_cast_grasp_selection_algorithm2(self):
        """
        Description
        -----------
        This test verifies that a single pose returned by the algorithm
        is converted into a trajectory with one waypoint.
        """
        # Setup
        production = self.create_production()

        # Cast the algorithm
        trajectory = production.cast_grasp_selection_algorithm(
            lambda path_to_object: self.X_ObjectGrasp
        )

        # Verify
        self.assertEqual(len(trajectory), 1)
        self.assertTrue(trajectory[0].IsExactlyEqualTo(self.X_ObjectGrasp))

    def test_cast_grasp_selection_algorithm3(self):
        """
        Description
        -----------
        This test verifies that the method raises a ValueError
        when the algorithm returns an empty trajectory.
        """
        # Setup
        production = self.create_production()

        # Verify
        with self.assertRaises(ValueError):
            production.cast_grasp_selection_algorithm(lambda path_to_object: [])

        self.assertIsNone(production.X_ObjectGripper_trajectory)

    def test_cast_grasp_selection_algorithm4(self):
        """
        Description
        -----------
        This test verifies that the method raises a TypeError
        when the algorithm returns something that is not a RigidTransform.
        """
        # Setup
        production = self.create_production()

        # Verify
        with self.assertRaises(TypeError):
            production.cast_grasp_selection_algorithm(
                lambda path_to_object: [self.X_ObjectGrasp, np.eye(4)]
            )

        self.assertIsNone(production.X_ObjectGripper_trajectory)

    def test_add_cast_and_build1(self):
        """
        Description
        -----------
        This test verifies that add_cast_and_build raises an error
        if no grasp selection algorithm was cast before it was called.
        """
        # Setup
        production = self.create_production()

        # Verify
        with self.assertRaises(AssertionError):
            production.add_cast_and_build(with_watcher=False)

    def test_add_cast_and_build2(self):
        """
        Description
        -----------
        This test verifies that add_cast_and_build succeeds
        if a grasp selection algorithm was cast before it was called.
        """
        # Setup
        production = self.create_production()
        production.cast_grasp_selection_algorithm(self.two_waypoint_algorithm)

        # Build
        diagram, diagram_context = production.add_cast_and_build(with_watcher=False)

        # Verify
        self.assertIsNotNone(diagram)
        self.assertIsNotNone(diagram_context)

    def test_easy_cast_and_build1(self):
        """
        Description
        -----------
        This test verifies that easy_cast_and_build:
        - builds the production (without a watcher, by default),
        - defines the gripper trajectory using the algorithm, and
        - records the trajectory and the algorithm's name in the metadata.
        """
        # Setup
        production = self.create_production()

        # Build
        diagram, diagram_context = production.easy_cast_and_build(
            self.two_waypoint_algorithm
        )

        # Verify the build
        self.assertIsNotNone(diagram)
        self.assertIsNotNone(diagram_context)
        self.assertIsNone(production.watcher)

        # Verify the metadata
        metadata = production._metadata
        self.assertEqual(
            metadata["grasp_selection_algorithm"], "two_waypoint_algorithm"
        )

        recorded_trajectory = metadata["X_ObjectGripper_trajectory"]
        self.assertEqual(len(recorded_trajectory), 2)
        for recorded_matrix, X_expected in zip(
            recorded_trajectory, [self.X_ObjectPreGrasp, self.X_ObjectGrasp]
        ):
            np.testing.assert_allclose(
                np.array(recorded_matrix), X_expected.GetAsMatrix4()
            )

    def test_easy_cast_and_build2(self):
        """
        Description
        -----------
        This test verifies that easy_cast_and_build
        successfully builds and simulates the production without errors when given:
        - The Erlenmeyer flask model
        - the Robotiq 2f-85 gripper
        - A grasp selection algorithm that keeps the gripper far from the flask

        Because the grasp is unsuccessful, the flask should fall after the floor drops.

        This test also verifies that the gripper's desired pose does not jump when the
        floor drops (i.e., the pose memory holds the world pose from the moment of the switch).
        """
        # Setup
        X_ObjectGripper_far = RigidTransform(p=np.array([-0.2, 0.0, 0.5]))
        production = self.create_production()

        # Build with watcher
        diagram, diagram_context = production.easy_cast_and_build(
            lambda path_to_object: [X_ObjectGripper_far, X_ObjectGripper_far],
            with_watcher=True,
        )
        watcher = production.watcher

        # Simulate the full script
        script: GraspSelection1Script = production.config.script
        t_final = script.total_time()

        t_drop_start = script.start_time_of_phase(GraspSelection1Phase.kFloorDrop)

        gripper_pose_memory = production.gripper_pose_memory
        memory_context = gripper_pose_memory.GetMyContextFromRoot(diagram_context)

        simulator = Simulator(diagram, diagram_context)
        simulator.Initialize()

        # Collect the gripper's desired pose just before the floor drops
        simulator.AdvanceTo(t_drop_start - 0.01)
        X_WorldGripperDesired_before = (
            gripper_pose_memory.get_pose_out_port().Eval(memory_context)
        )

        # Collect the gripper's desired pose just after the floor drops
        simulator.AdvanceTo(t_drop_start + 0.01)
        X_WorldGripperDesired_after = (
            gripper_pose_memory.get_pose_out_port().Eval(memory_context)
        )

        simulator.AdvanceTo(t_final)
        X_WorldGripperDesired_final = (
            gripper_pose_memory.get_pose_out_port().Eval(memory_context)
        )

        # Verify that the desired pose does not jump at the switch and is then held
        self.assertTrue(
            X_WorldGripperDesired_after.IsNearlyEqualTo(
                X_WorldGripperDesired_before, 1e-2
            ),
            f"Expected the gripper's desired pose to stay (nearly) the same when the floor drops, "
            + f"but it went from\n{X_WorldGripperDesired_before}\nto\n{X_WorldGripperDesired_after}",
        )
        self.assertTrue(
            X_WorldGripperDesired_final.IsExactlyEqualTo(X_WorldGripperDesired_after),
            "Expected the gripper's desired pose to be held after the floor drops.",
        )

        # Use the watcher to collect the states of the flask

        plant_name = production.plant.get_name()
        flask_state_port_name = f"{production.manipuland_name}_state"

        flask_state_logger: PortWatcher = watcher._port_watchers[plant_name][
            flask_state_port_name
        ]
        flask_state_log = flask_state_logger.get_vector_log_sink().FindLog(
            diagram_context
        )
        flask_state_times = flask_state_log.sample_times()
        flask_state_data = flask_state_log.data()

        # Compare the height of the flask when the floor starts to drop with its final height
        # (In the flask's state, the position is in indices 4 to 6, so its height is index 6)
        flask_heights = flask_state_data[6, :]
        z_flask_at_drop = flask_heights[np.argmin(np.abs(flask_state_times - t_drop_start))]
        z_flask_final = flask_heights[-1]

        # The grasp should have failed, so the flask should have fallen with the floor
        # (i.e., ended up far below its height when the floor started to drop)
        self.assertLess(
            z_flask_final,
            z_flask_at_drop - 1.0,
            f"Expected the flask to fall with the floor after the drop start time, but its height went from "
            + f"{z_flask_at_drop} to {z_flask_final}.",
        )


if __name__ == "__main__":
    unittest.main()

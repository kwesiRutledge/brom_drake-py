"""
motion_planner_test.py
Description:
    This script tests some of the basic features of the motion_planner base object.
"""
from importlib import resources as impresources

import numpy as np
import unittest

from pydrake.geometry import SceneGraph, CollisionFilterDeclaration, GeometrySet
from pydrake.math import RigidTransform, RollPitchYaw
from pydrake.multibody.parsing import Parser
from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex
from pydrake.systems.analysis import Simulator
from pydrake.systems.framework import Context

# Internal Imports
from brom_drake import robots
from brom_drake.all import (
    drakeify_my_urdf,
)
from brom_drake.productions.debug import ShowMeThisModel
from brom_drake.file_manipulation.urdf.DrakeReadyURDFConverter import MeshReplacementStrategy
from brom_drake.utils import AddGround


class MotionPlannerTest(unittest.TestCase):
    @staticmethod
    def add_shelf1(plant: MultibodyPlant) -> ModelInstanceIndex:
        """
        Add the shelf to the scene.
        :param plant:
        :return:
        """
        # Setup

        # Algorithm
        urdf_file_path = str(
            impresources.files(robots) / "models/cupboard/cupboard.sdf"
        )

        # Add the shelf to the plant
        model_idcs = Parser(plant=plant).AddModels(urdf_file_path)
        return model_idcs[0]

    @staticmethod
    def add_shelf2(plant: MultibodyPlant) -> ModelInstanceIndex:
        # Algorithm
        urdf_file_path = str(
            impresources.files(robots) / "models/bookshelf/bookshelf.urdf"
        )

        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="bookshelf2.log",
        )

        # Add the shelf to the plant
        model_idcs = Parser(plant=plant).AddModels(str(new_urdf_path))
        return model_idcs[0]

    @staticmethod
    def check_collision_in_config1(
        plant: MultibodyPlant,
        root_context: Context,
        scene_graph: SceneGraph,
        robot_model_idx: ModelInstanceIndex,
        q_model: np.ndarray
    ) -> bool:
        plant_context = plant.GetMyMutableContextFromRoot(root_context)
        scene_graph_context = scene_graph.GetMyMutableContextFromRoot(root_context)

        # Set the configuration
        plant.SetPositions(
            plant_context,
            robot_model_idx,
            q_model
        )

        # Check for Collisions (and investigate scene) using query object from SceneGraph
        query = scene_graph.get_query_output_port().Eval(scene_graph_context)
        print("Play with the query object here!")
        print(f"Collision exist in query? {query.HasCollisions()}")
        inspector = scene_graph.model_inspector()
        # print(query.ComputeSignedDistancePairwiseClosestPoints())
        for ii, pair_ii in enumerate(query.ComputeSignedDistancePairwiseClosestPoints()):
            if pair_ii.distance < 0.0:
                print(pair_ii)
                print(f"{inspector.GetName(pair_ii.id_A)} with id {pair_ii.id_A}")
                print(f"{inspector.GetName(pair_ii.id_B)} with id {pair_ii.id_B}")
                print(pair_ii.distance)

                # Apply filtering for each of these geometries
                scene_graph.collision_filter_manager(scene_graph_context).Apply(
                    CollisionFilterDeclaration().ExcludeBetween(
                        GeometrySet(pair_ii.id_A),
                        GeometrySet(pair_ii.id_B)
                    )
                )

                # Add triad for each point
                print(pair_ii.p_ACa)



        print(" ")



        return query.HasCollisions()

    def test_check_collision_in_config1(self):
        """
        Test the collision checking functionality.
        """
        # Setup
        urdf_file_path = str(
            impresources.files(robots) / "models/ur/ur10e.urdf"
        )
        q0 = np.array([
            0.65*(np.pi/2.0), # 0.95*(np.pi/2.0),
            (-6/4.0)*(np.pi/4.0),
            -0.25 + np.pi / 2.0,
            1.5*(-np.pi/2.0),
            0.0,
            0.0,
        ])

        # Convert the URDF
        new_urdf_path = drakeify_my_urdf(
            urdf_file_path,
            overwrite_old_logs=True,
            log_file_name="drakeify-my-urdf1.log",
            collision_mesh_replacement_strategy=MeshReplacementStrategy.kWithMinimalEnclosingCylinder,
        )

        # Visualize the URDF using the "show-me-this-model" feature
        time_step = 1e-3
        scene = ShowMeThisModel(
            str(new_urdf_path),
            with_these_joint_positions=q0,
            time_step=time_step,
            meshcat_port_number=None, # Turn off for CI
        )

        # Add Bookshelf to scene
        shelf_model_index = self.add_shelf1(scene.plant)
        shelf_pose = RigidTransform(
            RollPitchYaw(0.0, 0.0, +np.pi/2.0).ToQuaternion(),
            np.array([0.0, 1.0, 0.4]),
        )
        scene.plant.WeldFrames(
            scene.plant.world_frame(),
            scene.plant.GetFrameByName("cupboard_body", shelf_model_index),
            shelf_pose,
        ) # Weld the bookshelf to the world frame

        # Build Diagram and Simulate
        diagram, diagram_context = scene.cast_scene_and_build()

        # Try to check the collision
        in_collision = self.check_collision_in_config1(
            scene.plant,
            diagram_context,
            scene.scene_graph,
            scene.model_index,
            q0
        )

        self.assertFalse(
            in_collision
        )

        # Simulate
        simulator = Simulator(diagram, diagram_context)

        # Run simulation
        simulator.Initialize()
        simulator.set_target_realtime_rate(1.0)
        simulator.AdvanceTo(1.0)

    # TODO(kwesi): Figure out why the bookcase is colliding with the robot
    # def test_check_collision_in_config2(self):
    #     """
    #     Test the collision checking functionality.
    #     """
    #     # Setup
    #     urdf_file_path = str(
    #         impresources.files(robots) / "models/ur/ur10e.urdf"
    #     )
    #     q0 = np.array([
    #         0.75*(np.pi/2.0),
    #         (-7/4.0)*(np.pi/4.0),
    #         +np.pi / 2.0,
    #         1.5*(-np.pi/2.0),
    #         0.0,
    #         0.0,
    #     ])
    #
    #     # Convert the URDF
    #     new_urdf_path = drakeify_my_urdf(
    #         urdf_file_path,
    #         overwrite_old_logs=True,
    #         log_file_name="drakeify-my-urdf1.log",
    #     )
    #
    #     # Visualize the URDF using the "show-me-this-model" feature
    #     time_step = 1e-3
    #     scene = ShowMeThisModel(
    #         str(new_urdf_path),
    #         with_these_joint_positions=q0,
    #         time_step=time_step,
    #     )
    #
    #     # Add Bookshelf to scene
    #     shelf_model_index = self.add_shelf2(scene.plant)
    #     shelf_pose = RigidTransform(
    #         RollPitchYaw(0.0, 0.0, 0.0).ToQuaternion(),
    #         np.array([-0.4, 0.8, 0.0]),
    #     )
    #     scene.plant.WeldFrames(
    #         scene.plant.world_frame(),
    #         scene.plant.GetFrameByName("base_footprint", shelf_model_index),
    #         shelf_pose,
    #     ) # Weld the bookshelf to the world frame
    #
    #     # Build Diagram and Simulate
    #     robot_diagram, diagram_context = scene.cast_scene_and_build()
    #
    #     # Try to check the collision
    #     in_collision = self.check_collision_in_config1(
    #         scene.plant,
    #         diagram_context,
    #         scene.scene_graph,
    #         scene.model_index,
    #         q0
    #     )
    #
    #     self.assertFalse(
    #         in_collision
    #     )
    #
    #     # Simulate
    #     simulator = Simulator(robot_diagram, diagram_context)
    #
    #     # Run simulation
    #     simulator.Initialize()
    #     simulator.set_target_realtime_rate(1.0)
    #     simulator.AdvanceTo(10.0)

if __name__ == '__main__':
    unittest.main()

import unittest

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    CoulombFriction,
    DiagramBuilder,
    RigidTransform,
    SpatialInertia,
    UnitInertia,
)

from brom_drake.utils.collision_checking import (
    CollisionCheckMethod,
    count_collisions,
    find_collisions,
    using_point_pair_penetration,
)


class CollisionCheckingTest(unittest.TestCase):
    """
    Two 0.1 m cubes, "box_a" fixed at the origin and "box_b" on a free joint,
    so we can control exactly how far apart (or how deep) they are.
    """

    def _build_two_boxes(self, x_offset_of_b: float):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        side = 0.1
        inertia = SpatialInertia(
            mass=1.0,
            p_PScm_E=np.zeros(3),
            G_SP_E=UnitInertia.SolidBox(side, side, side),
        )
        bodies = {}
        for name in ["box_a", "box_b"]:
            body = plant.AddRigidBody(name, inertia)
            plant.RegisterCollisionGeometry(
                body,
                RigidTransform(),
                Box(side, side, side),
                f"{name}_collision",
                CoulombFriction(0.5, 0.5),
            )
            bodies[name] = body
        plant.WeldFrames(plant.world_frame(), bodies["box_a"].body_frame())
        plant.Finalize()

        diagram = builder.Build()
        context = diagram.CreateDefaultContext()
        plant.SetFreeBodyPose(
            plant.GetMyMutableContextFromRoot(context),
            bodies["box_b"],
            RigidTransform([x_offset_of_b, 0.0, 0.0]),
        )
        return plant, scene_graph, context

    def test_overlapping_boxes_are_reported_with_depth_and_body_names(self):
        # 0.1 m cubes 0.08 m apart overlap by 0.02 m
        plant, scene_graph, context = self._build_two_boxes(0.08)

        collisions = find_collisions(scene_graph, context, plant=plant)

        self.assertEqual(len(collisions), 1)
        self.assertAlmostEqual(collisions[0].depth, 0.02, places=6)
        names = {collisions[0].name_A, collisions[0].name_B}
        self.assertTrue(any(n.endswith("box_a") for n in names), names)
        self.assertTrue(any(n.endswith("box_b") for n in names), names)
        self.assertEqual(count_collisions(scene_graph, context, plant=plant), 1)
        self.assertTrue(using_point_pair_penetration(plant, scene_graph, context))

    def test_separated_boxes_have_no_collisions(self):
        plant, scene_graph, context = self._build_two_boxes(0.5)

        self.assertEqual(find_collisions(scene_graph, context, plant=plant), [])
        self.assertEqual(
            count_collisions(
                scene_graph, context, method=CollisionCheckMethod.SIGNED_DISTANCE
            ),
            0,
        )
        self.assertFalse(using_point_pair_penetration(plant, scene_graph, context))

    def test_shallow_contact_is_below_the_penetration_threshold(self):
        # Overlap of 0.5 mm is under the default 1 mm threshold
        plant, scene_graph, context = self._build_two_boxes(0.0995)

        self.assertEqual(find_collisions(scene_graph, context), [])
        self.assertEqual(len(find_collisions(scene_graph, context, threshold=0.0)), 1)

    def test_signed_distance_method_counts_near_misses_inside_the_margin(self):
        # 5 mm gap: not penetrating, but inside a 1 cm safety margin
        plant, scene_graph, context = self._build_two_boxes(0.105)

        self.assertEqual(find_collisions(scene_graph, context), [])
        near_misses = find_collisions(
            scene_graph, context, threshold=1e-2, method="signed_distance"
        )
        self.assertEqual(len(near_misses), 1)
        self.assertAlmostEqual(near_misses[0].depth, -0.005, places=6)


if __name__ == "__main__":
    unittest.main()

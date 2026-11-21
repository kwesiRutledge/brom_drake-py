"""
Checks that the planners' collision checks (which now go through
``brom_drake.utils.collision_checking.find_collisions``) keep their default
thresholds and respect a user-supplied ``collision_threshold``.
"""

import unittest

import numpy as np
from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    Box,
    CoulombFriction,
    DiagramBuilder,
    PrismaticJoint,
    RigidTransform,
    SpatialInertia,
    UnitInertia,
)

from brom_drake.motion_planning.algorithms.motion_planner import MotionPlanner
from brom_drake.motion_planning.systems.async_planner import AsyncPlannerSystem
from brom_drake.motion_planning.systems.prototypical_planner import (
    PrototypicalPlannerSystem,
)
from brom_drake.utils.collision_checking import CollisionCheckMethod, find_collisions

SIDE = 0.1  # Both boxes are 0.1 m cubes


class CollisionThresholdTest(unittest.TestCase):
    """
    "box_a" is welded at the origin. "box_b" slides along x on an actuated
    prismatic joint in its own model instance ("slider"), so q = [x] is the
    distance between the box centers. Boxes overlap when x < 0.1.
    """

    def setUp(self):
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)
        inertia = SpatialInertia(
            mass=1.0,
            p_PScm_E=np.zeros(3),
            G_SP_E=UnitInertia.SolidBox(SIDE, SIDE, SIDE),
        )
        slider = plant.AddModelInstance("slider")
        box_a = plant.AddRigidBody("box_a", plant.AddModelInstance("fixed"), inertia)
        box_b = plant.AddRigidBody("box_b", slider, inertia)
        for body in [box_a, box_b]:
            plant.RegisterCollisionGeometry(
                body,
                RigidTransform(),
                Box(SIDE, SIDE, SIDE),
                f"{body.name()}_collision",
                CoulombFriction(0.5, 0.5),
            )
        plant.WeldFrames(plant.world_frame(), box_a.body_frame())
        joint = plant.AddJoint(
            PrismaticJoint(
                "slide_x",
                plant.world_frame(),
                box_b.body_frame(),
                [1.0, 0.0, 0.0],
                -1.0,
                1.0,
            )
        )
        plant.AddJointActuator("slide_x_actuator", joint)
        plant.Finalize()

        diagram = builder.Build()
        self.plant, self.scene_graph, self.slider = plant, scene_graph, slider
        self.root_context = diagram.CreateDefaultContext()

    def _planner_system(self, cls=PrototypicalPlannerSystem, **kwargs):
        system = cls(
            self.plant,
            self.scene_graph,
            lambda q_start, q_goal, collision_fcn: np.vstack([q_start, q_goal]),
            robot_model_idx=self.slider,
            **kwargs,
        )
        system.set_internal_root_context(self.root_context)
        return system

    def _motion_planner(self, **kwargs):
        planner = MotionPlanner(self.slider, self.plant, self.scene_graph, **kwargs)
        planner.root_context = self.root_context
        return planner

    def test_prototypical_planner_default_threshold1(self):
        system = self._planner_system()
        self.assertEqual(system.collision_threshold, 1e-2)

        # 2 cm overlap is above the 1 cm default; 5 mm overlap is not
        self.assertTrue(system.check_collision_in_config(np.array([0.08])))
        self.assertFalse(system.check_collision_in_config(np.array([0.095])))
        self.assertFalse(system.check_collision_in_config(np.array([0.5])))

    def test_prototypical_planner_custom_threshold1(self):
        system = self._planner_system(collision_threshold=1e-3)

        # With a 1 mm threshold, the 5 mm overlap now counts
        self.assertTrue(system.check_collision_in_config(np.array([0.095])))

    def test_prototypical_planner_restores_configuration1(self):
        system = self._planner_system()
        plant_context = self.plant.GetMyMutableContextFromRoot(self.root_context)
        self.plant.SetPositions(plant_context, self.slider, np.array([0.5]))

        system.check_collision_in_config(np.array([0.08]))

        np.testing.assert_allclose(
            self.plant.GetPositions(plant_context, self.slider), [0.5]
        )

    def test_async_planner_accepts_threshold1(self):
        system = self._planner_system(AsyncPlannerSystem, collision_threshold=1e-3)
        self.assertEqual(system.collision_threshold, 1e-3)
        self.assertTrue(system.check_collision_in_config(np.array([0.095])))

    def test_planner_methods_match_collision_utility(self):
        for cls in (PrototypicalPlannerSystem, AsyncPlannerSystem):
            for method in (CollisionCheckMethod.PENETRATION, "signed_distance"):
                with self.subTest(cls=cls, method=method):
                    system = self._planner_system(
                        cls, collision_threshold=1e-3, collision_check_method=method
                    )
                    self.assertEqual(system.collision_check_method, method)
                    for q in (0.095, 0.1005, 0.12):
                        actual = system.check_collision_in_config(np.array([q]))
                        plant_context = self.plant.GetMyMutableContextFromRoot(
                            self.root_context
                        )
                        self.plant.SetPositions(plant_context, self.slider, [q])
                        expected = bool(
                            find_collisions(
                                self.scene_graph,
                                self.root_context,
                                threshold=system.collision_threshold,
                                method=system.collision_check_method,
                            )
                        )
                        self.assertEqual(actual, expected)
                        if q == 0.1005:
                            self.assertEqual(
                                actual, method == CollisionCheckMethod.SIGNED_DISTANCE
                            )

    def test_planner_rejects_invalid_method(self):
        with self.assertRaises(ValueError):
            self._planner_system(collision_check_method="invalid")

    def test_motion_planner_default_margin1(self):
        planner = self._motion_planner()
        self.assertEqual(planner.collision_threshold, 1e-2)

        # Signed distance: a 5 mm gap is inside the 1 cm margin, a 2 cm gap is not
        self.assertTrue(planner.check_collision_in_config(np.array([0.105])))
        self.assertFalse(planner.check_collision_in_config(np.array([0.12])))

    def test_motion_planner_custom_margin1(self):
        planner = self._motion_planner(collision_threshold=0.0)

        # With no margin, the 5 mm gap is no longer a collision
        self.assertFalse(planner.check_collision_in_config(np.array([0.105])))
        self.assertTrue(planner.check_collision_in_config(np.array([0.08])))


if __name__ == "__main__":
    unittest.main()

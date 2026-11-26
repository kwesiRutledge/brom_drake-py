"""
GraspSelection1 with the YCB objects in its Scenario's object catalog.

Resolving a drake_models URL downloads Drake's models archive the first time
(cached in ~/.cache/drake/package_map). These tests are skipped when it cannot
be downloaded.
"""

import os
from typing import List
import unittest

import numpy as np
from pydrake.all import (
    Box,
    Cylinder,
    RigidTransform,
    RotationMatrix,
    Simulator,
    Sphere,
)

from brom_drake.all import (
    GraspSelection1,
    GraspSelection1Configuration,
    GraspSelection1Phase,
    GraspSelection1Script,
    GripperType,
    ObjectDescription,
    ObjectGeometry,
    ScenarioError,
)
from brom_drake.productions.scenario_presets import DRAKE_YCB_OBJECT_URLS

# Distance from the Robotiq 2F-85's base frame to the center of its open finger pads,
# along the gripper's approach (z) axis. The fingers close along the gripper's y axis.
ROBOTIQ_2F_85_PAD_DEPTH = 0.13


def drake_models_available() -> bool:
    try:
        from brom_drake.utils.model_files import resolve_model_path

        resolve_model_path(next(iter(DRAKE_YCB_OBJECT_URLS.values())))
        return True
    except Exception:
        return False


DRAKE_MODELS_AVAILABLE = drake_models_available()
SKIP_REASON = "Drake's drake_models package could not be downloaded"


def shape_corners(shape) -> np.ndarray:
    """The 8 corners of a shape's bounding box, in the shape's frame."""
    if isinstance(shape, Box):
        half = np.array([shape.width(), shape.depth(), shape.height()]) / 2.0
    elif isinstance(shape, Cylinder):
        half = np.array([shape.radius(), shape.radius(), shape.length() / 2.0])
    elif isinstance(shape, Sphere):
        half = np.full(3, shape.radius())
    else:
        raise TypeError(f"Unexpected shape {shape}")
    signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
    return signs * half


def top_down_grasp(geometry: ObjectGeometry) -> List[RigidTransform]:
    """
    A simple grasp selection algorithm: come straight down from above and close
    across the narrower horizontal side of the object's bounding box.

    Pointing straight down puts the wrist at roll = +/-pi, where roll-pitch-yaw
    angles wrap; the wrist's controller compares rotations, so it does not care.
    """
    points = np.vstack(
        [
            c.X_ObjectShape.multiply(shape_corners(c.shape).T).T
            for c in geometry.collision_shapes
        ]
    )
    lo, hi = points.min(axis=0), points.max(axis=0)
    center = (lo + hi) / 2.0

    # Gripper z (approach) points down; the fingers close along gripper y.
    R_ObjectGripper = RotationMatrix.MakeXRotation(np.pi)
    if hi[0] - lo[0] <= hi[1] - lo[1]:
        # Close along the object's x axis instead.
        R_ObjectGripper = RotationMatrix.MakeZRotation(np.pi / 2.0) @ R_ObjectGripper

    # Put the pads low on the object, but clear of the floor.
    p_ObjectPads = np.array([center[0], center[1], max(center[2], lo[2] + 0.04)])
    approach = R_ObjectGripper.col(2)
    X_ObjectGrasp = RigidTransform(
        R_ObjectGripper, p_ObjectPads - ROBOTIQ_2F_85_PAD_DEPTH * approach
    )
    X_ObjectPreGrasp = RigidTransform(
        R_ObjectGripper, p_ObjectPads - (ROBOTIQ_2F_85_PAD_DEPTH + 0.1) * approach
    )
    return [X_ObjectPreGrasp, X_ObjectGrasp]


def short_script() -> GraspSelection1Script:
    return GraspSelection1Script(
        settling_time_on_floor=2.0,
        gripper_approach_time=2.0,
        grasp_closing_time=1.0,
        post_grasp_settling_time=0.5,
        drop_time=2.0,
    )


class GraspSelection1ScenarioTest(unittest.TestCase):
    """The Scenario declaration; no models are loaded."""

    def test_fields1(self):
        names = [f["name"] for f in GraspSelection1.Scenario.schema()["fields"]]
        self.assertEqual(names, ["path_to_object", "grasp_gripper_command"])

    def test_object_choices_are_the_catalog1(self):
        field = GraspSelection1.Scenario.schema()["fields"][0]
        self.assertEqual(field["kind"], "choice")
        self.assertEqual(field["choices"], sorted(DRAKE_YCB_OBJECT_URLS))
        self.assertIsNone(field["default"])

    def test_constructor_kwargs1(self):
        kwargs = GraspSelection1.Scenario.constructor_kwargs(
            {
                "path_to_object": "ycb_005_tomato_soup_can",
                "grasp_gripper_command": [0.6],
            }
        )
        self.assertEqual(
            kwargs["path_to_object"],
            "package://drake_models/ycb/005_tomato_soup_can.sdf",
        )
        np.testing.assert_array_equal(kwargs["grasp_gripper_command"], [0.6])

    def test_paths_are_refused1(self):
        for value in (
            "package://drake_models/ycb/005_tomato_soup_can.sdf",
            "/etc/passwd",
            "../x.urdf",
            "ycb_003_cracker_box",  # Drake ships it, but it is not in the catalog
        ):
            with self.subTest(value=value):
                with self.assertRaises(ScenarioError):
                    GraspSelection1.Scenario.validate({"path_to_object": value})

    def test_gripper_command_bounds1(self):
        with self.assertRaises(ScenarioError):
            GraspSelection1.Scenario.validate({"grasp_gripper_command": [1.5]})
        with self.assertRaises(ScenarioError):
            GraspSelection1.Scenario.validate({"grasp_gripper_command": [0.5, 0.5]})

    def test_none_reaching_the_constructor_is_explained1(self):
        """
        The Scenario's None means "keep the script's value"; a caller must
        not pass it on to the constructor.
        """
        with self.assertRaisesRegex(ValueError, "path_to_object"):
            GraspSelection1(
                path_to_object=None,
                gripper_choice=GripperType.Robotiq_2f_85,
                grasp_gripper_command=np.array([0.7]),
            )
        with self.assertRaisesRegex(ValueError, "grasp_gripper_command"):
            GraspSelection1(
                path_to_object="model.urdf",
                gripper_choice=GripperType.Robotiq_2f_85,
                grasp_gripper_command=None,
            )


@unittest.skipUnless(DRAKE_MODELS_AVAILABLE, SKIP_REASON)
class GraspSelection1PackageUrlTest(unittest.TestCase):
    def test_package_url_is_resolved_once1(self):
        url = DRAKE_YCB_OBJECT_URLS["ycb_009_gelatin_box"]
        production = GraspSelection1(
            path_to_object=url,
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.9]),
        )
        self.assertEqual(production.object_source, url)
        self.assertTrue(os.path.isfile(production.path_to_object))
        self.assertTrue(production.path_to_object.endswith("009_gelatin_box.sdf"))

    def test_selector_receives_resolved_file1(self):
        received = []

        def algorithm(description: ObjectDescription):
            received.append(description)
            return top_down_grasp(description.geometry)

        production = GraspSelection1(
            path_to_object=DRAKE_YCB_OBJECT_URLS["ycb_009_gelatin_box"],
            gripper_choice=GripperType.Robotiq_2f_85,
            grasp_gripper_command=np.array([0.9]),
        )
        production.cast_grasp_selection_algorithm(algorithm)
        self.assertEqual(received[0].path_to_object, production.path_to_object)

    def test_catalog_objects_have_one_body_and_corner_spheres1(self):
        for object_id, url in DRAKE_YCB_OBJECT_URLS.items():
            with self.subTest(object_id=object_id):
                geometry = ObjectGeometry.from_model_file(url)
                shapes = [c.shape for c in geometry.collision_shapes]
                self.assertEqual(len(shapes), 9)
                self.assertEqual(sum(isinstance(s, Sphere) for s in shapes), 8)


@unittest.skipUnless(DRAKE_MODELS_AVAILABLE, SKIP_REASON)
class GraspSelection1CatalogGraspTest(unittest.TestCase):
    """
    Every object in the catalog can be placed on the floor and grasped by the
    Robotiq 2F-85, and stays in the gripper when the floor drops away.
    """

    def test_each_catalog_object_can_be_grasped1(self):
        for object_id in DRAKE_YCB_OBJECT_URLS:
            with self.subTest(object_id=object_id):
                config = GraspSelection1Configuration()
                config.script = short_script()
                kwargs = GraspSelection1.Scenario.constructor_kwargs(
                    {"path_to_object": object_id}
                )
                production = GraspSelection1(
                    **{
                        "path_to_object": "the script's own object",
                        "gripper_choice": GripperType.Robotiq_2f_85,
                        "grasp_gripper_command": np.array([0.9]),
                        "meshcat_port_number": None,
                        "config": config,
                        **kwargs,
                    }
                )
                diagram, diagram_context = production.easy_cast_and_build(
                    top_down_grasp
                )

                plant = production.plant
                plant_context = plant.GetMyContextFromRoot(diagram_context)
                object_body = plant.get_body(
                    plant.GetBodyIndices(production.manipuland_index)[0]
                )
                gripper_base = plant.GetBodyByName("robotiq_arg2f_base_link")

                def object_height_below_gripper() -> float:
                    return (
                        plant.EvalBodyPoseInWorld(
                            plant_context, gripper_base
                        ).translation()[2]
                        - plant.EvalBodyPoseInWorld(
                            plant_context, object_body
                        ).translation()[2]
                    )

                simulator = Simulator(diagram, diagram_context)
                simulator.Initialize()
                simulator.AdvanceTo(
                    config.script.start_time_of_phase(GraspSelection1Phase.kFloorDrop)
                )
                height_at_drop = object_height_below_gripper()
                simulator.AdvanceTo(config.script.total_time())

                # A dropped object falls about 0.5 m with the floor; a held one
                # slips by at most about 1 cm.
                self.assertLess(object_height_below_gripper() - height_at_drop, 0.03)


if __name__ == "__main__":
    unittest.main()

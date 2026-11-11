from dataclasses import dataclass
from typing import List

from pydrake.all import (
    AddMultibodyPlantSceneGraph,
    DiagramBuilder,
    Parser,
    RigidTransform,
    Role,
    Shape,
)


@dataclass
class ObjectCollisionShape:
    """
    *Description*

    One of the collision shapes of an object.

    *Attributes*

    name: str
        The name of the geometry in the model of the object.

    shape: Shape
        The Drake shape (e.g., Box, Cylinder, Sphere, Mesh or Convex).

    X_ObjectShape: RigidTransform
        The pose of the shape with respect to the object frame. This is the same frame
        that a grasp selection algorithm should express the Gripper Frame's poses in.
    """

    name: str
    shape: Shape
    X_ObjectShape: RigidTransform


@dataclass
class ObjectGeometry:
    """
    *Description*

    The geometry of an object, as seen by the collision engine of the simulation.

    *Attributes*

    collision_shapes: List[ObjectCollisionShape]
        Each of the object's collision shapes and its pose in the object frame.
    """

    collision_shapes: List[ObjectCollisionShape]

    @classmethod
    def from_model_file(cls, path_to_object: str) -> "ObjectGeometry":
        """
        *Description*

        Loads a model into a temporary plant and collects its collision geometry.

        *Parameters*

        path_to_object: str
            The file path to the URDF/SDF of the object.
            The model must contain one body, which defines the object frame.

        *Returns*

        geometry: ObjectGeometry
            The object's geometry.
        """
        # Setup
        builder = DiagramBuilder()
        plant, scene_graph = AddMultibodyPlantSceneGraph(builder, time_step=0.0)

        # Load the model
        model_indices = Parser(plant=plant).AddModels(path_to_object)
        if len(model_indices) != 1:
            raise ValueError(
                f"Expected {path_to_object} to contain one model; received {len(model_indices)}."
            )

        n_bodies = len(plant.GetBodyIndices(model_indices[0]))
        if n_bodies != 1:
            raise ValueError(
                f"Expected {path_to_object} to contain one body; received {n_bodies}."
            )

        plant.Finalize()

        # Collect the shapes. They are cloned because the originals are owned by the temporary SceneGraph.
        inspector = scene_graph.model_inspector()
        collision_shapes = [
            ObjectCollisionShape(
                name=inspector.GetName(geometry_id),
                shape=inspector.GetShape(geometry_id).Clone(),
                X_ObjectShape=inspector.GetPoseInFrame(geometry_id),
            )
            for geometry_id in inspector.GetAllGeometryIds(Role.kProximity)
        ]

        if len(collision_shapes) == 0:
            raise ValueError(f"{path_to_object} does not contain collision geometry.")

        return cls(collision_shapes=collision_shapes)


@dataclass
class ObjectDescription:
    """
    *Description*

    Everything that a grasp selection algorithm can know about the object being grasped.

    *Attributes*

    path_to_object: str
        The file path to the URDF/SDF of the object.

    geometry: ObjectGeometry
        The object's geometry.
    """

    path_to_object: str
    geometry: ObjectGeometry

    @classmethod
    def from_model_file(cls, path_to_object: str) -> "ObjectDescription":
        """
        *Description*

        Creates the description of the object in a model file.

        *Parameters*

        path_to_object: str
            The file path to the URDF/SDF of the object.

        *Returns*

        description: ObjectDescription
            The description of the object.
        """
        return cls(
            path_to_object=path_to_object,
            geometry=ObjectGeometry.from_model_file(path_to_object),
        )

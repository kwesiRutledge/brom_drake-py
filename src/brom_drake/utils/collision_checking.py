from dataclasses import dataclass
from enum import Enum
from typing import List, Optional, Union

from pydrake.all import (
    Context,
    GeometryId,
    MultibodyPlant,
    PenetrationAsPointPair,
    QueryObject,
    SceneGraph,
    SceneGraphInspector,
    SignedDistancePair,
)


class CollisionCheckMethod(str, Enum):
    """
    Description
    -----------
    How ``find_collisions`` decides that two geometries are "in collision".

    - ``PENETRATION``: the pair penetrates by *more* than ``threshold`` (meters).
      Uses ``QueryObject.ComputePointPairPenetration()``. Touching or shallow
      resting contact (depth <= threshold) is not reported.
    - ``SIGNED_DISTANCE``: the pair's signed distance is *less* than ``threshold``.
      Uses ``QueryObject.ComputeSignedDistancePairwiseClosestPoints(max_distance=threshold)``.
      With a positive threshold this acts as a safety margin (near misses count).
    """

    PENETRATION = "penetration"
    SIGNED_DISTANCE = "signed_distance"


@dataclass(frozen=True)
class CollisionPair:
    """
    Description
    -----------
    One pair of geometries that ``find_collisions`` reports as colliding.

    Attributes
    ----------
    id_A, id_B : GeometryId
        The SceneGraph geometry ids of the pair (ordered as Drake returned them).
    name_A, name_B : str
        Human readable names. When a plant is given these are body names
        (e.g. ``"wrist_3_link"``); otherwise they are SceneGraph geometry names.
    depth : float
        Penetration depth in meters (positive means overlapping). For the
        signed-distance method this is ``-distance``, so it is still "how far
        inside each other" the pair is and is negative for near misses.
    """

    id_A: GeometryId
    id_B: GeometryId
    name_A: str
    name_B: str
    depth: float

    @property
    def key(self) -> frozenset:
        """An order-independent key for this pair of names (useful for counting)."""
        return frozenset((self.name_A, self.name_B))


def _name_of(
    geometry_id: GeometryId,
    inspector: SceneGraphInspector,
    plant: Optional[MultibodyPlant],
) -> str:
    if plant is not None:
        try:
            body = plant.GetBodyFromFrameId(inspector.GetFrameId(geometry_id))
            if body is not None:
                model_name = plant.GetModelInstanceName(body.model_instance())
                return f"{model_name}::{body.name()}"
        except RuntimeError:
            pass
    return inspector.GetName(geometry_id)


def find_collisions(
    scene_graph: SceneGraph,
    root_context: Context,
    plant: Optional[MultibodyPlant] = None,
    threshold: float = 1e-3,
    method: Union[CollisionCheckMethod, str] = CollisionCheckMethod.PENETRATION,
) -> List[CollisionPair]:
    """
    Description
    -----------
    Returns every pair of geometries that is in collision in the given context.
    Collision filters configured on the SceneGraph context (e.g. a production's
    ``configure_collision_filter``) are respected.

    Arguments
    ---------
    scene_graph : SceneGraph
        The scene graph to query.
    root_context : Context
        The context of the root diagram that contains ``scene_graph``.
    plant : MultibodyPlant, optional
        If given, colliding geometries are named by ``model_instance::body``.
    threshold : float
        Meters. See ``CollisionCheckMethod`` for how it is interpreted.
        Defaults to 1e-3.
    method : CollisionCheckMethod or str
        ``"penetration"`` (default) or ``"signed_distance"``.

    Returns
    -------
    collisions : List[CollisionPair]
        Empty when the scene is collision free, sorted deepest first.
    """
    method = CollisionCheckMethod(method)

    scene_graph_context = scene_graph.GetMyContextFromRoot(root_context)
    query_object: QueryObject = scene_graph.get_query_output_port().Eval(
        scene_graph_context
    )
    inspector = query_object.inspector()

    collisions: List[CollisionPair] = []
    if method == CollisionCheckMethod.PENETRATION:
        pairs: List[PenetrationAsPointPair] = query_object.ComputePointPairPenetration()
        for pair in pairs:
            if pair.depth > threshold:
                collisions.append(
                    CollisionPair(
                        id_A=pair.id_A,
                        id_B=pair.id_B,
                        name_A=_name_of(pair.id_A, inspector, plant),
                        name_B=_name_of(pair.id_B, inspector, plant),
                        depth=float(pair.depth),
                    )
                )
    else:
        pairs: List[SignedDistancePair] = (
            query_object.ComputeSignedDistancePairwiseClosestPoints(
                max_distance=max(threshold, 0.0)
            )
        )
        for pair in pairs:
            if pair.distance < threshold:
                collisions.append(
                    CollisionPair(
                        id_A=pair.id_A,
                        id_B=pair.id_B,
                        name_A=_name_of(pair.id_A, inspector, plant),
                        name_B=_name_of(pair.id_B, inspector, plant),
                        depth=float(-pair.distance),
                    )
                )

    collisions.sort(key=lambda c: c.depth, reverse=True)
    return collisions


def count_collisions(
    scene_graph: SceneGraph,
    root_context: Context,
    plant: Optional[MultibodyPlant] = None,
    threshold: float = 1e-3,
    method: Union[CollisionCheckMethod, str] = CollisionCheckMethod.PENETRATION,
) -> int:
    """
    Description
    -----------
    The number of colliding geometry pairs in ``root_context``.
    See ``find_collisions`` for the arguments.
    """
    return len(
        find_collisions(
            scene_graph,
            root_context,
            plant=plant,
            threshold=threshold,
            method=method,
        )
    )


def using_point_pair_penetration(
    plant: MultibodyPlant,
    scene_graph: SceneGraph,
    root_context: Context,
    min_acceptable_distance: float = 1e-3,
    debug_flag: bool = False,
) -> bool:
    """
    Description
    -----------
    Check for any collisions using the point pair penetration method.
    Returns True if any pair penetrates deeper than ``min_acceptable_distance``.

    Kept for backwards compatibility; prefer ``find_collisions``.

    Arguments
    ---------
    plant : MultibodyPlant
        The plant to check for collisions (used to name bodies in debug output).
    scene_graph : SceneGraph
        The scene graph associated with the plant.
    root_context : Context
        The context of the root diagram.
    min_acceptable_distance : float
        The penetration depth above which a pair counts as a collision.
        Defaults to 1e-3.
    """
    collisions = find_collisions(
        scene_graph,
        root_context,
        plant=plant,
        threshold=min_acceptable_distance,
        method=CollisionCheckMethod.PENETRATION,
    )
    if debug_flag:
        for c in collisions:
            print(
                f"Collision detected between {c.name_A} and {c.name_B} with depth {c.depth}"
            )
    return len(collisions) > 0

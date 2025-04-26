from pydrake.all import (
    Context,
    Diagram,
    MultibodyPlant,
    PenetrationAsPointPair,
    QueryObject,
    SceneGraph,
)
from typing import List

def using_point_pair_penetration(
    plant: MultibodyPlant,
    scene_graph: SceneGraph,
    root_context: Context,
    min_acceptable_distance: float = 1e-3,
    debug_flag: bool = False
) -> bool:
    """
    Description
    -----------
    Check for any collisions using the point pair penetration method.
    Returns True if any collisions are detected, otherwise False.

    Arguments
    ---------
    plant : MultibodyPlant
        The plant to check for collisions. (Seems unnecessary)
    scene_graph : SceneGraph
        The scene graph associated with the plant.
    root_context : Context
        The context of the root diagram.
    min_acceptable_distance : float
        The minimum distance to consider as a collision. Defaults to 1e-2.
    """
    # Setup
    scene_graph_context = scene_graph.GetMyContextFromRoot(root_context)
    query_object: QueryObject = scene_graph.get_query_output_port().Eval(scene_graph_context)

    # Check for collisions
    closest_points: List[PenetrationAsPointPair] = query_object.ComputePointPairPenetration()
    collision_detected = False
    for pair_ii in closest_points:
        if pair_ii.depth > min_acceptable_distance:
            collision_detected = True
            if debug_flag:
                print(f"Collision detected between {pair_ii.id_A} and {pair_ii.id_B} with depth {pair_ii.depth}")

            break

    return collision_detected
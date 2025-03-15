from pydrake.all import (
    MultibodyPlant,
    ModelInstanceIndex,
    Parser,
    RigidBody,
)

def find_number_of_positions_in_welded_model(
    path_to_model: str,
) -> int:
    """
    Description
    -----------
    This method will return the number of positions in the 
    model described at path_to_model WHEN the base link is welded
    to the world frame.

    This is most useful for finding things like the number of positions in
    a robot.

    Returns
    -------
    int
        The number of positions in the model.
    """
    # Setup
    time_step = 1e-3

    # Create a shadow plant
    shadow_plant = MultibodyPlant(time_step)
    model_idcs = Parser(plant=shadow_plant).AddModels(path_to_model)
    shadow_model_idx = model_idcs[0]

    # Weld the base link to the world frame
    gripper_bodies_indicies = shadow_plant.GetBodyIndices(shadow_model_idx)
    first_body_in_gripper = shadow_plant.get_body(gripper_bodies_indicies[0])
    shadow_plant.WeldFrames(
        shadow_plant.world_frame(),
        first_body_in_gripper.body_frame(),
    )

    # Finalize the shadow plant
    shadow_plant.Finalize()

    return shadow_plant.num_positions()

def get_first_body_in(plant: MultibodyPlant, model_instance: ModelInstanceIndex) -> RigidBody:
    """
    Description
    -----------
    This method returns the first body in the given model instance.
    """
    return get_all_bodies_in(plant, model_instance)[0]

def get_all_bodies_in(
    plant: MultibodyPlant,
    model_instance: ModelInstanceIndex,
) -> list[RigidBody]:
    """
    Description
    -----------
    This method returns all the bodies in the given model instance.
    """
    # Setup

    # Algorithm
    model_body_indices = plant.GetBodyIndices(model_instance)
    return [plant.get_body(body_index) for body_index in model_body_indices]

def get_name_of_first_body_in_urdf(urdf_in: str) -> str:
    """
    Description
    -----------
    This method returns the name of the first body found in the 
    urdf given by urdf_in.
    """
    return get_name_of_all_bodies_in_urdf(urdf_in)[0]

def get_name_of_all_bodies_in_urdf(urdf_in: str) -> list[str]:
    """
    Description
    -----------
    This method returns the name of the first body found in the 
    urdf given by urdf_in.
    """
    # Setup

    # Create Plant with this urdf
    plant = MultibodyPlant(time_step=1e-3)
    try:
        model_instances = Parser(plant).AddModels(urdf_in)
    except Exception as e:
        raise RuntimeError(
            f"Failed to parse the urdf file: {urdf_in}."
        ) from e

    if len(model_instances) > 1:
        raise RuntimeWarning(
            f"More than one model instance found in the urdf file: {urdf_in}."
        )
    elif len(model_instances) == 0:
        raise RuntimeWarning(
            f"No model instance found in the urdf file: {urdf_in}."
        )

    # Algorithm
    model_instance = model_instances[0] # TODO(kwesi): Do we need to finalize the plant?
    all_bodies = get_all_bodies_in(plant, model_instance)
    return [body.name() for body in all_bodies]
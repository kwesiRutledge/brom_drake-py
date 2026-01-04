from pydrake.multibody.plant import MultibodyPlant
from pydrake.multibody.tree import ModelInstanceIndex, BodyIndex
from typing import List

def get_all_associated_body_indices_in_plant(
    plant: MultibodyPlant,
) -> List[BodyIndex]:
    """
    *Description*
    
    This method returns all the bodies in the given model instance.
    """
    # Iterate through all model instances and collect body indices
    body_indices_out = []
    for model_instance_index in range(plant.num_model_instances()):
        model_body_indices = plant.GetBodyIndices(ModelInstanceIndex(model_instance_index))
        body_indices_out.extend(model_body_indices)
    
    return body_indices_out

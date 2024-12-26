from pydrake.all import RigidTransform
from typing import Any

def is_rigid_transform(obj: Any) -> bool:
    """
    Description
    -----------
    This function checks if the object is a RigidTransform.
    We will perform 2 checks.
    1. Check if the object is a RigidTransform object directly, and then
    2. Check if the object has the translation() and rotation() methods.
    """
    # Check if the object is a RigidTransform object directly
    if isinstance(obj, RigidTransform):
        return True
    
    # Check if the object has the translation() and rotation() methods
    if hasattr(obj, "translation") and hasattr(obj, "rotation"):
        # Check to see if the translation and rotation methods are callable
        if callable(obj.translation) and callable(obj.rotation):
            return True
        
    # Otherwise, return False
    return False

import numpy as np
from pydrake.all import (
    AbstractValue, BasicVector,
    Context,
    LeafSystem,
    RigidTransform,
)

from .configuration import Configuration as RigidTransformToVectorSystemConfiguration

class RigidTransformToVectorSystem(LeafSystem):
    """
    Description
    -----------
    This system will take in a RigidTransform and output a vector
    of size 7, where the first 3 elements are the translation
    and the last 4 elements are the quaternion representation
    of the rotation.

    Diagram
    -------
                            |-------------------|
                            | RigidTransform    |
    rigid_transform ---->   | To                | vector_xyz_quat(wxyz) (BasicVector[7])
    (RigidTransform)        | VectorSystem      |
                            |-------------------|
    """
    def __init__(
        self,
        config: RigidTransformToVectorSystemConfiguration = None,
    ):
        # Input Processing
        if config is None:
            config = RigidTransformToVectorSystemConfiguration()
        self.config = config

        # Base class initialization
        LeafSystem.__init__(self)
        self.set_name(self.config.name)

        # Create inputs
        self.create_system_inputs()

    def CalcVectorOutput(self, context: Context, output: BasicVector):
        # Setup
        config: RigidTransformToVectorSystemConfiguration = self.config

        # Collect Transform
        rigid_transform: RigidTransform = self.rigid_transform_input.Eval(context)
        
        # Create output vector
        if config.output_format == "vector_xyz_quat(wxyz)":
            xyz = rigid_transform.translation()
            quat = rigid_transform.rotation().ToQuaternion().wxyz()
            output.SetFromVector(
                np.hstack((xyz, quat))
            )
        elif config.output_format == "vector_xyz_euler(rpy)":
            xyz = rigid_transform.translation()
            euler = rigid_transform.rotation().ToRollPitchYaw().vector()
            output.SetFromVector(
                np.hstack((xyz, euler))
            )

    def create_system_inputs(self):
        # Setup
        config: RigidTransformToVectorSystemConfiguration = self.config

        # Create ports
        self.rigid_transform_input = self.DeclareAbstractInputPort(
            "rigid_transform",
            AbstractValue.Make(RigidTransform()),
        )

        if config.output_format == "vector_xyz_quat(wxyz)":
            self.DeclareVectorOutputPort(
                "vector_xyz_quat(wxyz)",
                7,
                self.CalcVectorOutput,
            )
        elif config.output_format == "vector_xyz_euler(rpy)":
            self.DeclareVectorOutputPort(
                "vector_xyz_euler(rpy)",
                6,
                self.CalcVectorOutput,
            )
        else:
            raise ValueError(f"Unknown output format: {config.output_format}")

from pydrake.all import (
    AbstractValue, BasicVector,
    Context,
    LeafSystem,
    RigidTransform,
)

class RigidTransformToVectorSystem(LeafSystem):
    def __init__(self):
        LeafSystem.__init__(self)
        self.rigid_transform_input = self.DeclareAbstractInputPort(
            "rigid_transform",
            AbstractValue.Make(RigidTransform()),
        )
        self.DeclareVectorOutputPort(
            "vector_xyz_quat(wxyz)",
            7,
            self.CalcVectorOutput,
        )

    def CalcVectorOutput(self, context: Context, output: BasicVector):
        rigid_transform = self.rigid_transform_input.Eval(context)
        xyz_quat = rigid_transform.GetAsVector()
        output.SetFromVector(xyz_quat)
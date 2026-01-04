from pydrake.common.value import AbstractValue
from pydrake.all import RigidTransform
from pydrake.systems.framework import Context, InputPort, LeafSystem

class PoseCompositionSystem(LeafSystem):
    """
    *Description*

    This LeafSystem receives two Pose inputs (i.e., AbstractValue objects
    containing RigidTransform values) and returns the composition of the two
    poses (i.e., an AbstractValue containing a RigidTransform value).

    *Diagram*

    The LeafSystem's input and output ports can be illustrated with the following block: ::

                            |---------------|
        pose_AB ----------->|               |
        (RigidTransform)    | Pose          |
                            | Composition   | ----> pose_AC
        pose_BC ----------->| System        |       (RigidTransform)
        (RigidTransform)    |               |
                            |---------------|
    """

    def __init__(self):
        LeafSystem.__init__(self)

        # Define Input Ports
        self._pose_AB_port = self.DeclareAbstractInputPort(
            "pose_AB",
            AbstractValue.Make(RigidTransform())
        )
        self._pose_BC_port = self.DeclareAbstractInputPort(
            "pose_BC",
            AbstractValue.Make(RigidTransform())
        )

        # Define Output Port
        self.DeclareAbstractOutputPort(
            "pose_AC",
            lambda: AbstractValue.Make(RigidTransform()),
            self.CalcOutputPose,
        )

    def CalcOutputPose(self, context: Context, output: AbstractValue):
        """
        *Description*

        This callback function computes the composed pose, ``pose_AC``,
        from the two inputs ``pose_AB`` and ``pose_BC``.
        """
        # Retrieve current input values
        pose_AB: RigidTransform = self.GetInputPort("pose_AB").Eval(context)
        pose_BC: RigidTransform = self.GetInputPort("pose_BC").Eval(context)
        
        assert type(pose_AB) is RigidTransform, \
            f"Expected pose_AB port to contain \"RigidTransform\" objects, but received type {type(pose_AB)}"
        
        assert type(pose_BC) is RigidTransform, \
            f"Expected pose_BC port to contain \"RigidTransform\" objects, but received type {type(pose_BC)}"

        # Compute Product and return it
        pose_AC = pose_AB.multiply(pose_BC)

        output.SetFrom(
            AbstractValue.Make(pose_AC)
        )

    def get_pose_AB_port(self) -> InputPort:
        return self._pose_AB_port

    def get_pose_BC_port(self) -> InputPort:
        return self._pose_BC_port